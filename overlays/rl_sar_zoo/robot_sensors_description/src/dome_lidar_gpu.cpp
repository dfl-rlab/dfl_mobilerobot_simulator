// SPDX-License-Identifier: Apache-2.0
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/GpuRays.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cstdlib>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <vector>
#include "robot_sensors_description/dome_projection.hpp"
#include "robot_sensors_description/semantic_labels.hpp"

namespace robot_sensors_description
{
// Render range and class in the same native GPU LiDAR pass. The ROS timestamp
// and sensor pose come from that rendering scene, not a later physics tick.
class DomeLidarGpu : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  ~DomeLidarGpu() override
  {
    render_connection_.reset();
    teardown_connection_.reset();
    if (context_) {context_->shutdown("Dome LiDAR stopped");}
  }

  void Configure(const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & config,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & events) override
  {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("dome_lidar", rclcpp::NodeOptions().context(context_));
    try {
      const auto model = gz::sim::Model(entity);
      parent_name_ = model.Name(ecm) + "::" + config->Get<std::string>("parent_link");
      const auto * world = ecm.Component<gz::sim::components::Name>(gz::sim::worldEntity(ecm));
      if (!world) {throw std::runtime_error("Gazebo world is not available");}
      world_name_ = world->Data();
      mount_ = config->Get<gz::math::Pose3d>("mount_pose");
      frame_ = config->Get<std::string>("frame_name");
      resolution_ = config->Get<unsigned>("resolution");
      horizontal_ = config->Get<unsigned>("horizontal_samples");
      const auto horizontal = horizontal_;
      vertical_ = config->Get<unsigned>("vertical_samples");
      const auto vertical = vertical_;
      min_range_ = config->Get<double>("min_range");
      max_range_ = config->Get<double>("max_range");
      rate_ = config->Get<double>("update_rate");
      noise_ = config->Get<double>("noise_stddev", 0.0).first;
      if (resolution_ < 2 || resolution_ > 4096 || horizontal < 2 || vertical < 2 ||
        static_cast<size_t>(horizontal) * vertical > 4000000 ||
        !std::isfinite(min_range_) || !std::isfinite(max_range_) ||
        min_range_ <= 0 || max_range_ <= min_range_ ||
        !std::isfinite(rate_) || rate_ <= 0 || !std::isfinite(noise_) || noise_ < 0 || frame_.empty())
      {
        throw std::runtime_error("Invalid Dome LiDAR configuration");
      }
      samples_ = MakeDomeGrid(horizontal, vertical, resolution_);
      semantics_ = std::make_unique<SemanticLabels>(config->Get<std::string>("semantic_config"));
      publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        config->Get<std::string>("topic"), rclcpp::SensorDataQoS().reliable());
      render_connection_ = events.Connect<gz::sim::events::Render>([this] {Render();});
      teardown_connection_ = events.Connect<gz::sim::events::RenderTeardown>([this] {Cleanup();});
      RCLCPP_INFO(node_->get_logger(), "Harmonic dome configured: %u x %u samples, five %u x %u faces at %.1f Hz",
        horizontal, vertical, resolution_, resolution_, rate_);
    } catch (const std::exception & error) {
      RCLCPP_FATAL(node_->get_logger(), "Dome setup failed: %s", error.what());
    }
  }

private:
  void Initialize()
  {
    // The image includes a narrow renderer patch for this optional override.
    // Native Harmonic otherwise selects a cubemap size from the ray count.
    setenv("DFL_GPU_LIDAR_CUBEMAP_SIZE", std::to_string(resolution_).c_str(), 1);
    rays_ = scene_->CreateGpuRays("dome_gpu_rays");
    rays_->SetRayCount(horizontal_);
    rays_->SetVerticalRayCount(vertical_);
    rays_->SetAngleMin(-kPi);
    rays_->SetAngleMax(kPi - 2*kPi/horizontal_);
    rays_->SetVerticalAngleMin(0);
    rays_->SetVerticalAngleMax(kPi/2);
    rays_->SetNearClipPlane(min_range_);
    rays_->SetFarClipPlane(max_range_);
    rays_->SetClamp(false);
    scene_->RootVisual()->AddChild(rays_);
    connection_ = rays_->ConnectNewGpuRaysFrame(
      [this](const float * data, unsigned width, unsigned height, unsigned channels,
        const std::string &) {
        if (width != horizontal_ || height != vertical_ || channels != 3) {
          throw std::runtime_error("Unexpected native GPU LiDAR frame dimensions");
        }
        ray_data_ = data;
      });
    initialized_ = true;
  }

  void Render()
  {
    if (failed_) {return;}
    try {
      if (!scene_) {scene_ = gz::rendering::sceneFromFirstRenderEngine();}
      if (!scene_) {return;}
      const auto parent = scene_->NodeByName(parent_name_);
      if (!parent) {return;}
      const auto time = scene_->Time();
      // Render events are scheduled by the native clock camera. Quantize to
      // nominal scan periods so small timing jitter does not drop every other
      // frame when a render occurs just under one period after the last one.
      const auto scan = std::llround(std::chrono::duration<double>(time).count()*rate_);
      if (have_frame_ && scan == last_scan_) {return;}
      if (!initialized_) {Initialize();}
      for (unsigned i=0; i<scene_->VisualCount(); ++i) {
        const auto visual = scene_->VisualByIndex(i);
        // The native intensity attachment is 16-bit normalized, so encode the
        // class index across its full 0..2000 retro range. Nearest sampling
        // keeps the class from the same visible surface as the range return.
        const int index = semantics_->IndexForVisual(visual->Name(), world_name_);
        visual->SetUserData("laser_retro", float(index) * (2000.0f/65535.0f));
      }
      const auto pose = parent->WorldPose() * mount_;
      ray_data_ = nullptr;
      rays_->SetWorldPose(pose);
      rays_->Update();
      if (!ray_data_) {throw std::runtime_error("Missing native GPU LiDAR frame");}
      sensor_msgs::msg::PointCloud2 cloud;
      const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(time).count();
      cloud.header.stamp.sec=ns/1000000000LL;
      cloud.header.stamp.nanosec=ns%1000000000LL;
      cloud.header.frame_id=frame_;
      cloud.height=1; cloud.is_bigendian=false; cloud.is_dense=true;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(4, "x",1,sensor_msgs::msg::PointField::FLOAT32,
        "y",1,sensor_msgs::msg::PointField::FLOAT32, "z",1,sensor_msgs::msg::PointField::FLOAT32,
        "intensity",1,sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(samples_.size());
      sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"),y(cloud,"y"),z(cloud,"z"),label(cloud,"intensity");
      size_t valid=0;
      for (size_t i=0; i<samples_.size(); ++i) {
        const auto & sample = samples_[i];
        double range = ray_data_[3*i];
        if (!std::isfinite(range)) {continue;}
        if (range<min_range_ || range>max_range_) {continue;}
        if (noise_>0) {range=std::clamp(range+normal_(random_)*noise_,min_range_,max_range_);}
        *x=range*sample.direction[0]; *y=range*sample.direction[1]; *z=range*sample.direction[2];
        const int index = std::lround(ray_data_[3*i+1] * (65535.0f/2000.0f));
        *label=semantics_->Decode(static_cast<uint16_t>(std::clamp(index, 0, 65535)));
        ++x; ++y; ++z; ++label; ++valid;
      }
      modifier.resize(valid);
      publisher_->publish(cloud);
      last_scan_=scan; have_frame_=true;
    } catch (const std::exception & error) {
      failed_=true;
      RCLCPP_FATAL(node_->get_logger(), "Dome rendering stopped: %s",error.what());
    }
  }

  void Cleanup()
  {
    connection_.reset();
    if (scene_ && rays_) {scene_->DestroySensor(rays_); rays_.reset();}
    scene_.reset(); initialized_=false;
  }

  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  gz::common::ConnectionPtr render_connection_,teardown_connection_;
  gz::common::ConnectionPtr connection_;
  gz::rendering::ScenePtr scene_;
  gz::rendering::GpuRaysPtr rays_;
  const float * ray_data_{nullptr};
  std::unique_ptr<SemanticLabels> semantics_;
  std::vector<DomeSample> samples_;
  gz::math::Pose3d mount_;
  std::string parent_name_,world_name_,frame_;
  unsigned resolution_{},horizontal_{},vertical_{};
  double min_range_{},max_range_{},rate_{},noise_{};
  int64_t last_scan_{};
  bool initialized_{false},have_frame_{false},failed_{false};
  std::mt19937 random_{0};
  std::normal_distribution<double> normal_{0,1};
};
}  // namespace robot_sensors_description
GZ_ADD_PLUGIN(robot_sensors_description::DomeLidarGpu, gz::sim::System, gz::sim::ISystemConfigure)
