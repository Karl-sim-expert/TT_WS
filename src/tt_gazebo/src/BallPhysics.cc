// Gazebo (Ignition Fortress) system plugin for the table tennis ball.
//  * applies drag + Magnus force every step (ball_dynamics.hpp)
//  * handles the table bounce analytically (the table has no collision geometry)
//  * /ball/launch (gz.msgs.Twist: linear = velocity, angular = spin) resets and fires the ball
#include <memory>
#include <mutex>
#include <optional>

#include <chrono>

#include <ignition/common/Console.hh>
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

    if (sdf->HasElement("debug")) {
      this->debug = sdf->Get<bool>("debug");
    }
    this->node.Subscribe("/ball/launch", &BallPhysics::OnLaunch, this);
    ignmsg << "[BallPhysics] loaded (link entity " << this->link.Entity()
           << ", debug=" << this->debug << ")" << std::endl;
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
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (launch && dt > 0.0) {
      // Teleport, then give the ball the requested velocity/spin with a one-step impulse
      // (velocity commands did not behave reliably, see ball bounce debugging).
      this->model.SetWorldPoseCmd(ecm, ignition::math::Pose3d(this->launchPos, {}));
      const auto lin0 = this->link.WorldLinearVelocity(ecm);
      const auto ang0 = this->link.WorldAngularVelocity(ecm);
      const ignition::math::Vector3d v1{
        launch->linear().x(), launch->linear().y(), launch->linear().z()};
      const ignition::math::Vector3d w1{
        launch->angular().x(), launch->angular().y(), launch->angular().z()};
      this->ApplyVelocityChange(
        ecm, dt, v1 - lin0.value_or(ignition::math::Vector3d::Zero),
        w1 - ang0.value_or(ignition::math::Vector3d::Zero));
      this->cooldownUntil = info.simTime + std::chrono::milliseconds(50);
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

    if (this->debug && info.simTime - this->lastLog >= std::chrono::milliseconds(250)) {
      this->lastLog = info.simTime;
      const Vec3 fd = aerodynamic_force(this->params, v, w);
      ignmsg << "[BallPhysics] t=" << std::chrono::duration<double>(info.simTime).count()
             << " pos=(" << pose->Pos().X() << "," << pose->Pos().Y() << "," << pose->Pos().Z()
             << ") v=(" << v.x << "," << v.y << "," << v.z << ") |w|=" << norm(w)
             << " F=(" << fd.x << "," << fd.y << "," << fd.z << ")" << std::endl;
    }

    // Table bounce (analytic): ball surface reaches the table plane while moving down.
    // Slow impacts are treated as resting contact (no bounce, ball held on the surface), so
    // the ball cannot sink once the bounces die out.
    const bool onTable = this->table.contains(pose->Pos().X(), pose->Pos().Y()) &&
      pose->Pos().Z() - this->params.radius <= this->table.top_z &&
      pose->Pos().Z() > this->table.top_z - 0.5 * this->params.radius;
    if (dt > 0.0 && onTable && v.z < 0.0) {
      const bool resting = -v.z < 0.5;
      if (resting || info.simTime >= this->cooldownUntil) {
        const Vec3 v0 = v;
        const Vec3 w0 = w;
        if (resting) {
          BallParams p = this->params;
          p.restitution = 0.0;
          table_bounce(p, v, w);
          v.z = 0.0;
        } else {
          table_bounce(this->params, v, w);
          this->cooldownUntil = info.simTime + std::chrono::milliseconds(20);
        }
        this->ApplyVelocityChange(
          ecm, dt, Vec3{v.x - v0.x, v.y - v0.y, v.z - v0.z},
          Vec3{w.x - w0.x, w.y - w0.y, w.z - w0.z});
        // Lift the ball back onto the surface if it has sunk in.
        if (pose->Pos().Z() < this->table.top_z + this->params.radius) {
          auto fixed = *pose;
          fixed.Pos().Z() = this->table.top_z + this->params.radius;
          this->model.SetWorldPoseCmd(ecm, fixed);
        }
        if (this->debug && !resting) {
          ignmsg << "[BallPhysics] BOUNCE at x=" << pose->Pos().X() << " y=" << pose->Pos().Y()
                 << " z=" << pose->Pos().Z() << " v " << v0.z << " -> "
                 << v.z << " (vx " << v0.x << " -> " << v.x << ")" << std::endl;
        }
        return;
      }
    }

    const Vec3 f = aerodynamic_force(this->params, v, w);
    this->link.AddWorldForce(ecm, {f.x, f.y, f.z});
  }

private:
  // Change the ball's velocity/spin by applying a one-step force/torque (impulse = m dv).
  void ApplyVelocityChange(
    ignition::gazebo::EntityComponentManager & ecm, double dt,
    const ignition::math::Vector3d & dv, const ignition::math::Vector3d & dw)
  {
    const double inertia = 2.0 / 3.0 * this->params.mass * this->params.radius *
      this->params.radius;
    this->link.AddWorldWrench(
      ecm, (this->params.mass / dt) * dv, (inertia / dt) * dw);
  }

  void ApplyVelocityChange(
    ignition::gazebo::EntityComponentManager & ecm, double dt, const Vec3 & dv, const Vec3 & dw)
  {
    this->ApplyVelocityChange(
      ecm, dt, ignition::math::Vector3d(dv.x, dv.y, dv.z),
      ignition::math::Vector3d(dw.x, dw.y, dw.z));
  }

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
  bool debug{false};
  std::chrono::steady_clock::duration cooldownUntil{};
  std::chrono::steady_clock::duration lastLog{};
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
