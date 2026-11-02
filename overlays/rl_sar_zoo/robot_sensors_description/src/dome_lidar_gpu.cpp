// SPDX-License-Identifier: Apache-2.0
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/sensors/DepthCameraSensor.hh>
#include <gazebo_ros/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "robot_sensors_description/dome_projection.hpp"

namespace robot_sensors_description
{
// One native depth sensor drives all five cube faces. The additional faces are
// rendered in its depth callback, on Gazebo's rendering thread, from the same
// scene and pose. There are no independently timed ROS clouds to stitch together.
class DomeLidarGpu : public gazebo::SensorPlugin
{
public:
  ~DomeLidarGpu() override
  {
    connection_.reset();
    if (cameras_[0]) {
      const auto scene = cameras_[0]->GetScene();
      for (std::size_t face = 1; face < cameras_.size(); ++face) {
        if (scene && cameras_[face]) {
          scene->RemoveCamera(cameras_[face]->Name());
        }
        cameras_[face].reset();
      }
    }
  }

  void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf) override
  {
    node_ = gazebo_ros::Node::Get(sdf);
    if (!node_) {return;}
    auto sensor = std::dynamic_pointer_cast<gazebo::sensors::DepthCameraSensor>(parent);
    if (!sensor || !sensor->DepthCamera()) {
      RCLCPP_ERROR(node_->get_logger(), "GPU dome requires a working Gazebo depth sensor and OpenGL");
      return;
    }
    sensor_ = sensor;
    cameras_[0] = sensor->DepthCamera();
    const unsigned resolution = cameras_[0]->ImageWidth();
    const auto horizontal = sdf->Get<unsigned>("horizontal_samples");
    const auto vertical = sdf->Get<unsigned>("vertical_samples");
    min_range_ = sdf->Get<double>("min_range");
    max_range_ = sdf->Get<double>("max_range");
    noise_stddev_ = sdf->Get<double>("noise_stddev", 0.0).first;
    frame_ = sdf->Get<std::string>("frame_name");
    if (resolution < 2 || resolution > 4096 || cameras_[0]->ImageHeight() != resolution ||
      std::abs(cameras_[0]->HFOV().Radian() - kPi / 2) > 1e-4 ||
      horizontal < 2 || vertical < 2 ||
      static_cast<std::size_t>(horizontal) * vertical > 4000000 ||
      !std::isfinite(min_range_) || !std::isfinite(max_range_) ||
      min_range_ <= 0 || max_range_ <= min_range_ ||
      !std::isfinite(noise_stddev_) || noise_stddev_ < 0 || frame_.empty())
    {
      RCLCPP_ERROR(node_->get_logger(),
        "Invalid GPU dome configuration: image %u x %u, hfov %.9f, samples %u x %u, "
        "range %.4f..%.4f, noise %.4f, frame '%s'",
        resolution, cameras_[0]->ImageHeight(), cameras_[0]->HFOV().Radian(),
        horizontal, vertical, min_range_, max_range_, noise_stddev_, frame_.c_str());
      return;
    }
    // gz sdf -p serializes angles to limited precision. Use the exact cube FOV.
    cameras_[0]->SetHFOV(ignition::math::Angle(kPi / 2));
    samples_ = MakeDomeGrid(horizontal, vertical, resolution);
    rotations_ = {ignition::math::Quaterniond(0, 0, 0),
      ignition::math::Quaterniond(0, 0, kPi),
      ignition::math::Quaterniond(0, 0, kPi / 2),
      ignition::math::Quaterniond(0, 0, -kPi / 2),
      ignition::math::Quaterniond(0, -kPi / 2, 0)};

    const auto scene = cameras_[0]->GetScene();
    sdf::ElementPtr camera_sdf(new sdf::Element);
    sdf::initFile("camera.sdf", camera_sdf);
    camera_sdf->GetElement("horizontal_fov")->Set(kPi / 2);
    auto image = camera_sdf->GetElement("image");
    image->GetElement("width")->Set(resolution);
    image->GetElement("height")->Set(resolution);
    image->GetElement("format")->Set(std::string("R8G8B8"));
    auto clip = camera_sdf->GetElement("clip");
    clip->GetElement("near")->Set(cameras_[0]->NearClip());
    clip->GetElement("far")->Set(cameras_[0]->FarClip());
    camera_sdf->GetElement("depth_camera")->GetElement("output")->Set(std::string("depths"));
    for (std::size_t face = 1; face < cameras_.size(); ++face) {
      const auto name = parent->ScopedName() + "_cube_" + std::to_string(face);
      cameras_[face] = scene->CreateDepthCamera(name, false);
      cameras_[face]->SetCaptureData(true);
      cameras_[face]->Load(camera_sdf->Clone());
      cameras_[face]->Init();
      cameras_[face]->CreateRenderTexture(name + "_image");
      cameras_[face]->CreateDepthTexture(name + "_depth");
    }
    const auto qos = node_->get_qos().get_publisher_qos(
      "~/out", rclcpp::SensorDataQoS().reliable());
    publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/out", qos);
    connection_ = cameras_[0]->ConnectNewDepthFrame(std::bind(
      &DomeLidarGpu::OnDepthFrame, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));
    sensor->SetActive(true);
    RCLCPP_INFO(node_->get_logger(), "GPU dome ready: %u x %u repeat samples, five %u x %u depth faces",
      horizontal, vertical, resolution, resolution);
  }

private:
  void OnDepthFrame(const float * depth, unsigned, unsigned, unsigned, const std::string &)
  {
    const auto sensor = sensor_.lock();
    if (!sensor || !depth) {return;}
    const auto pose = cameras_[0]->WorldPose();
    std::array<const float *, 5> images{};
    images[0] = depth;
    for (std::size_t face = 1; face < cameras_.size(); ++face) {
      cameras_[face]->SetWorldPose(ignition::math::Pose3d(
        pose.Pos(), pose.Rot() * rotations_[face]));
      cameras_[face]->Render(true);
      cameras_[face]->PostRender();
      images[face] = cameras_[face]->DepthData();
      if (!images[face]) {return;}
    }

    sensor_msgs::msg::PointCloud2 cloud;
    const auto stamp = sensor->LastMeasurementTime();
    cloud.header.stamp.sec = stamp.sec;
    cloud.header.stamp.nanosec = stamp.nsec;
    cloud.header.frame_id = frame_;
    cloud.height = 1;
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(samples_.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
      z(cloud, "z"), intensity(cloud, "intensity");
    std::size_t valid = 0;
    for (const auto & sample : samples_) {
      const double axial = images[sample.face][sample.pixel];
      // Background pixels contain FarClip. Convert axial depth to radial range,
      // then enforce the same spherical range limits as the CPU ray sensor.
      if (!std::isfinite(axial) || axial <= cameras_[0]->NearClip() || axial >= max_range_) {
        continue;
      }
      double range = axial * sample.depth_to_range;
      if (range < min_range_ || range > max_range_) {continue;}
      if (noise_stddev_ > 0) {
        range = std::clamp(range + normal_(random_) * noise_stddev_, min_range_, max_range_);
      }
      *x = range * sample.direction[0];
      *y = range * sample.direction[1];
      *z = range * sample.direction[2];
      *intensity = 0;  // No calibrated material reflectivity model.
      ++x; ++y; ++z; ++intensity; ++valid;
    }
    // Match the CPU ROS ray plugin: XYZ/intensity, omit invalid/no-hit points.
    modifier.resize(valid);
    publisher_->publish(cloud);
  }

  gazebo_ros::Node::SharedPtr node_;
  std::weak_ptr<gazebo::sensors::DepthCameraSensor> sensor_;
  std::array<gazebo::rendering::DepthCameraPtr, 5> cameras_;
  std::array<ignition::math::Quaterniond, 5> rotations_;
  gazebo::event::ConnectionPtr connection_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::vector<DomeSample> samples_;
  std::string frame_;
  double min_range_{0.1}, max_range_{60}, noise_stddev_{0};
  std::mt19937 random_{0};
  std::normal_distribution<double> normal_{0, 1};
};

GZ_REGISTER_SENSOR_PLUGIN(DomeLidarGpu)
}  // namespace robot_sensors_description
