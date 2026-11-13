// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocityReset.hh>

namespace robot_sensors_description
{
// Reset the robot without rebuilding Gazebo systems or ROS controllers.
class RobotReset : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & config,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    initial_pose_ = gz::sim::worldPose(entity, ecm);
    const auto service = config->Get<std::string>("service");
    node_.Advertise(service, &RobotReset::Request, this);
  }

  bool Request(const gz::msgs::WorldControl & request, gz::msgs::Boolean & response)
  {
    const bool reset = request.reset().all() || request.reset().model_only();
    if (reset) {pending_.store(true);}
    response.set_data(reset);
    return true;
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager & ecm) override
  {
    // Velocity commands remain active until removed; apply them for one tick.
    if (clear_velocity_commands_) {
      for (const auto link : model_.Links(ecm)) {
        ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(link);
        ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(link);
      }
      clear_velocity_commands_ = false;
    }
    if (!have_initial_pose_) {
      initial_pose_ = gz::sim::worldPose(model_.Entity(), ecm);
      have_initial_pose_ = true;
    }
    if (!pending_.exchange(false)) {return;}
    clear_velocity_commands_ = true;
    model_.SetWorldPoseCmd(ecm, initial_pose_);
    for (const auto entity : model_.Links(ecm)) {
      const gz::sim::Link link(entity);
      link.SetLinearVelocity(ecm, gz::math::Vector3d::Zero);
      link.SetAngularVelocity(ecm, gz::math::Vector3d::Zero);
    }
    for (const auto joint : model_.Joints(ecm)) {
      const auto * type = ecm.Component<gz::sim::components::JointType>(joint);
      // Lite3 uses single-axis revolute joints, initially at URDF zero.
      if (!type || type->Data() != sdf::JointType::REVOLUTE) {continue;}
      ecm.SetComponentData<gz::sim::components::JointPositionReset>(joint, {0.0});
      ecm.SetComponentData<gz::sim::components::JointVelocityReset>(joint, {0.0});
    }
  }

private:
  gz::sim::Model model_;
  gz::math::Pose3d initial_pose_;
  std::atomic<bool> pending_{false};
  bool clear_velocity_commands_{false}, have_initial_pose_{false};
  gz::transport::Node node_;
};
}  // namespace robot_sensors_description
GZ_ADD_PLUGIN(robot_sensors_description::RobotReset, gz::sim::System,
  gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
