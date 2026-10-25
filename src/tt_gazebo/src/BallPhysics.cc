// Gazebo (Ignition Fortress) system plugin for the table tennis ball.
//  * applies drag + Magnus force every step (ball_dynamics.hpp)
//  * handles the table bounce analytically (the table has no collision geometry)
//  * /ball/launch (gz.msgs.Twist: linear = velocity, angular = spin) resets and fires the ball
#include <memory>
#include <mutex>
#include <optional>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "tt_gazebo/ball_dynamics.hpp"

namespace tt_gazebo
{

class BallPhysics
  : public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager & ecm,
    ignition::gazebo::EventManager &) override
  {
    this->model = ignition::gazebo::Model(entity);
    this->link = ignition::gazebo::Link(this->model.LinkByName(ecm, "link"));
    this->link.EnableVelocityChecks(ecm, true);

    auto get = [&](const char * name, double & value) {
        if (sdf->HasElement(name)) {
          value = sdf->Get<double>(name);
        }
      };
    get("mass", this->params.mass);
    get("radius", this->params.radius);
    get("drag_coefficient", this->params.drag_coeff);
    get("lift_slope", this->params.lift_slope);
    get("lift_max", this->params.lift_max);
    get("restitution", this->params.restitution);
    get("friction", this->params.friction);
    get("table_top_z", this->table.top_z);
    get("table_half_x", this->table.half_x);
    get("table_half_y", this->table.half_y);
    get("launch_x", this->launchPos.X());
    get("launch_y", this->launchPos.Y());
    get("launch_z", this->launchPos.Z());

    this->node.Subscribe("/ball/launch", &BallPhysics::OnLaunch, this);
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }

    std::optional<ignition::msgs::Twist> launch;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      launch.swap(this->pendingLaunch);
    }
    if (launch) {
      this->model.SetWorldPoseCmd(ecm, ignition::math::Pose3d(this->launchPos, {}));
      this->link.SetLinearVelocity(
        ecm, {launch->linear().x(), launch->linear().y(), launch->linear().z()});
      this->link.SetAngularVelocity(
        ecm, {launch->angular().x(), launch->angular().y(), launch->angular().z()});
      return;
    }

    const auto pose = this->link.WorldPose(ecm);
    const auto lin = this->link.WorldLinearVelocity(ecm);
    const auto ang = this->link.WorldAngularVelocity(ecm);
    if (!pose || !lin || !ang) {
      return;  // velocity components appear one step after EnableVelocityChecks
    }

    Vec3 v{lin->X(), lin->Y(), lin->Z()};
    Vec3 w{ang->X(), ang->Y(), ang->Z()};

    // Table bounce (analytic): ball surface reaches the table plane while moving down.
    if (this->table.contains(pose->Pos().X(), pose->Pos().Y()) &&
      pose->Pos().Z() - this->params.radius <= this->table.top_z && v.z < 0.0 &&
      pose->Pos().Z() > this->table.top_z - 0.5 * this->params.radius)
    {
      table_bounce(this->params, v, w);
      this->link.SetLinearVelocity(ecm, {v.x, v.y, v.z});
      this->link.SetAngularVelocity(ecm, {w.x, w.y, w.z});
      return;
    }

    const Vec3 f = aerodynamic_force(this->params, v, w);
    this->link.AddWorldForce(ecm, {f.x, f.y, f.z});
  }

private:
  void OnLaunch(const ignition::msgs::Twist & msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingLaunch = msg;
  }

  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  ignition::gazebo::Link link{ignition::gazebo::kNullEntity};
  ignition::transport::Node node;
  BallParams params;
  Table table;
  ignition::math::Vector3d launchPos{-1.8, 0.0, 1.0};
  std::mutex mutex;
  std::optional<ignition::msgs::Twist> pendingLaunch;
};

}  // namespace tt_gazebo

IGNITION_ADD_PLUGIN(
  tt_gazebo::BallPhysics,
  ignition::gazebo::System,
  tt_gazebo::BallPhysics::ISystemConfigure,
  tt_gazebo::BallPhysics::ISystemPreUpdate)
IGNITION_ADD_PLUGIN_ALIAS(tt_gazebo::BallPhysics, "tt_gazebo::BallPhysics")
