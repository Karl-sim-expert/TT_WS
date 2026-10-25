// Gazebo (Ignition Fortress) system plugin for the table tennis ball.
//  * applies drag + Magnus force every step (ball_dynamics.hpp)
//  * handles the table bounce analytically (the table has no collision geometry)
//  * handles the paddle impact analytically too (paddle pose/velocity from the robot's paddle link)
//  * /ball/launch (gz.msgs.Twist: linear = velocity, angular = spin) resets and fires the ball
//  * /ball/hold (gz.msgs.Vector3d) parks the ball motionless at a point (gravity off, pinned) until
//    it is launched or hit. The ball starts parked at the launch point (`park` in the SDF).
//  * /ball/paddle_hit (gz.msgs.Odometry) is published at every paddle impact: stamp = sim time,
//    pose = ball position, twist = ball velocity/spin right after the impact.
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <chrono>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/vector3d.pb.h>
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
    get("paddle_radius", this->paddle.radius);
    get("paddle_half_thickness", this->paddle.half_thickness);
    get("paddle_restitution", this->paddle.restitution);
    get("paddle_friction", this->paddle.friction);
    get("launch_x", this->launchPos.X());
    get("launch_y", this->launchPos.Y());
    get("launch_z", this->launchPos.Z());

    if (sdf->HasElement("debug")) {
      this->debug = sdf->Get<bool>("debug");
    }
    if (sdf->HasElement("robot_name")) {
      this->robotName = sdf->Get<std::string>("robot_name");
    }
    if (sdf->HasElement("park")) {
      this->held = sdf->Get<bool>("park");
      this->holdPos = this->launchPos;
    }
    this->node.Subscribe("/ball/launch", &BallPhysics::OnLaunch, this);
    this->node.Subscribe("/ball/hold", &BallPhysics::OnHold, this);
    this->hitPub = this->node.Advertise<ignition::msgs::Odometry>("/ball/paddle_hit");
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
    std::optional<ignition::msgs::Vector3d> hold;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      hold.swap(this->pendingHold);
    }
    const double dt = std::chrono::duration<double>(info.dt).count();
    this->UpdatePaddle(ecm, info, dt);
    if (hold && dt > 0.0) {
      this->holdPos = ignition::math::Vector3d(hold->x(), hold->y(), hold->z());
      this->held = true;
      this->launchPhase = 0;
      if (this->debug) {
        ignmsg << "[BallPhysics] HOLD at " << this->holdPos << std::endl;
      }
    }
    if (launch && dt > 0.0) {
      this->held = false;
      // Step 1: teleport to the machine and cancel whatever velocity/spin the ball had.
      this->model.SetWorldPoseCmd(ecm, ignition::math::Pose3d(this->launchPos, {}));
      const auto lin0 = this->link.WorldLinearVelocity(ecm);
      const auto ang0 = this->link.WorldAngularVelocity(ecm);
      this->ApplyVelocityChange(
        ecm, dt, -lin0.value_or(ignition::math::Vector3d::Zero),
        -ang0.value_or(ignition::math::Vector3d::Zero));
      this->launchVel = ignition::math::Vector3d(
        launch->linear().x(), launch->linear().y(), launch->linear().z());
      this->launchSpin = ignition::math::Vector3d(
        launch->angular().x(), launch->angular().y(), launch->angular().z());
      this->launchPhase = 1;
      return;
    }
    if (this->launchPhase == 1 && dt > 0.0) {
      // Step 2: velocity is now measured from the engine, so give exactly the requested change.
      const auto lin0 = this->link.WorldLinearVelocity(ecm);
      const auto ang0 = this->link.WorldAngularVelocity(ecm);
      if (!lin0 || !ang0) {
        return;
      }
      this->ApplyVelocityChange(ecm, dt, this->launchVel - *lin0, this->launchSpin - *ang0);
      this->launchPhase = 0;
      this->cooldownUntil = info.simTime + std::chrono::milliseconds(50);
      if (this->debug) {
        ignmsg << "[BallPhysics] LAUNCH want v=" << this->launchVel << " w=" << this->launchSpin
               << " measured before impulse v=" << *lin0 << std::endl;
      }
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

    // Paddle impact (analytic). A parked ball is at rest by definition.
    if (dt > 0.0 && this->paddleValid) {
      Vec3 pos{pose->Pos().X(), pose->Pos().Y(), pose->Pos().Z()};
      Vec3 v1 = v;
      Vec3 w1 = w;
      if (paddle_contact(this->params, this->paddle, this->paddleState, pos, v1, w1)) {
        this->ApplyVelocityChange(
          ecm, dt, Vec3{v1.x - v.x, v1.y - v.y, v1.z - v.z},
          Vec3{w1.x - w.x, w1.y - w.y, w1.z - w.z});
        auto fixed = *pose;
        fixed.Pos() = ignition::math::Vector3d(pos.x, pos.y, pos.z);
        this->model.SetWorldPoseCmd(ecm, fixed);
        this->held = false;
        this->cooldownUntil = info.simTime + std::chrono::milliseconds(20);
        this->PublishHit(info, pos, v1, w1);
        if (this->debug) {
          ignmsg << "[BallPhysics] PADDLE HIT t=" << std::chrono::duration<double>(info.simTime).count()
                 << " ball v " << norm(v) << " -> " << norm(v1) << " out=(" << v1.x << ","
                 << v1.y << "," << v1.z << ") |w|=" << norm(w1) << " paddle v=("
                 << this->paddleState.vel.x << "," << this->paddleState.vel.y << ","
                 << this->paddleState.vel.z << ")" << std::endl;
        }
        return;
      }
    }

    // Parked ball: pinned at the hold point, velocities cancelled (acts like zero gravity).
    if (this->held && dt > 0.0) {
      this->model.SetWorldPoseCmd(ecm, ignition::math::Pose3d(this->holdPos, {}));
      this->ApplyVelocityChange(ecm, dt, -*lin, -*ang);
      return;
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

  // Look up the paddle link (once the robot has been spawned) and update its pose and velocity
  // (finite difference of the world pose between steps; the arm is position controlled).
  void UpdatePaddle(
    ignition::gazebo::EntityComponentManager & ecm, const ignition::gazebo::UpdateInfo & info,
    double dt)
  {
    if (this->paddleGaveUp) {
      return;
    }
    if (this->paddleLink == ignition::gazebo::kNullEntity) {
      const auto robot = ecm.EntityByComponents(
        ignition::gazebo::components::Name(this->robotName),
        ignition::gazebo::components::Model());
      if (robot == ignition::gazebo::kNullEntity) {
        return;
      }
      const ignition::gazebo::Model robotModel(robot);
      this->paddleLink = robotModel.LinkByName(ecm, "paddle");
      if (this->paddleLink != ignition::gazebo::kNullEntity) {
        ignmsg << "[BallPhysics] paddle link found" << std::endl;
      } else {
        // The URDF converter merged the fixed paddle joint into its parent: use panda_link7 plus
        // the fixed transform link7 -> panda_link8 -> panda_hand -> paddle (see tt_robot.urdf.xacro).
        this->paddleLink = robotModel.LinkByName(ecm, "panda_link7");
        this->paddleViaLink7 = this->paddleLink != ignition::gazebo::kNullEntity;
        if (this->paddleViaLink7) {
          ignwarn << "[BallPhysics] no 'paddle' link; using panda_link7 + fixed offset" << std::endl;
        }
      }
      if (this->paddleLink == ignition::gazebo::kNullEntity) {
        ignerr << "[BallPhysics] paddle link not found: paddle impacts are disabled" << std::endl;
        this->paddleGaveUp = true;
        return;
      }
    }
    auto linkPose = ignition::gazebo::Link(this->paddleLink).WorldPose(ecm);
    if (!linkPose) {
      return;
    }
    ignition::math::Pose3d pose = *linkPose;
    if (this->paddleViaLink7) {
      const ignition::math::Pose3d offset(
        ignition::math::Vector3d(0, 0, 0.107 + 0.13),
        ignition::math::Quaterniond(0, 0, -M_PI / 4) * ignition::math::Quaterniond(0, M_PI / 2, 0));
      pose = pose * offset;
    }
    const auto z = pose.Rot().RotateVector(ignition::math::Vector3d::UnitZ);
    if (this->havePrevPaddle && dt > 0.0) {
      const auto dp = (pose.Pos() - this->prevPaddle.Pos()) / dt;
      ignition::math::Quaterniond dq = pose.Rot() * this->prevPaddle.Rot().Inverse();
      if (dq.W() < 0.0) {
        dq = ignition::math::Quaterniond(-dq.W(), -dq.X(), -dq.Y(), -dq.Z());
      }
      const ignition::math::Vector3d vec(dq.X(), dq.Y(), dq.Z());
      const double s = vec.Length();
      ignition::math::Vector3d omega;
      if (s > 1e-12) {
        omega = vec * (2.0 * std::atan2(s, dq.W()) / s / dt);
      }
      this->paddleState.vel = Vec3{dp.X(), dp.Y(), dp.Z()};
      this->paddleState.omega = Vec3{omega.X(), omega.Y(), omega.Z()};
      this->paddleValid = true;
    }
    this->paddleState.pos = Vec3{pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
    this->paddleState.axis_z = Vec3{z.X(), z.Y(), z.Z()};
    this->prevPaddle = pose;
    this->havePrevPaddle = true;
    (void)info;
  }

  void PublishHit(
    const ignition::gazebo::UpdateInfo & info, const Vec3 & pos, const Vec3 & v, const Vec3 & w)
  {
    ignition::msgs::Odometry msg;
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    msg.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000);
    msg.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000);
    msg.mutable_pose()->mutable_position()->set_x(pos.x);
    msg.mutable_pose()->mutable_position()->set_y(pos.y);
    msg.mutable_pose()->mutable_position()->set_z(pos.z);
    msg.mutable_twist()->mutable_linear()->set_x(v.x);
    msg.mutable_twist()->mutable_linear()->set_y(v.y);
    msg.mutable_twist()->mutable_linear()->set_z(v.z);
    msg.mutable_twist()->mutable_angular()->set_x(w.x);
    msg.mutable_twist()->mutable_angular()->set_y(w.y);
    msg.mutable_twist()->mutable_angular()->set_z(w.z);
    this->hitPub.Publish(msg);
  }

  void OnHold(const ignition::msgs::Vector3d & msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->pendingHold = msg;
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
  PaddleParams paddle;
  PaddleState paddleState;
  bool paddleValid{false};
  bool paddleViaLink7{false};
  bool paddleGaveUp{false};
  bool havePrevPaddle{false};
  ignition::math::Pose3d prevPaddle;
  ignition::gazebo::Entity paddleLink{ignition::gazebo::kNullEntity};
  std::string robotName{"tt_robot"};
  bool held{false};
  ignition::math::Vector3d holdPos;
  ignition::transport::Node::Publisher hitPub;
  std::optional<ignition::msgs::Vector3d> pendingHold;
  ignition::math::Vector3d launchPos{-1.8, 0.0, 1.0};
  ignition::math::Vector3d launchVel;
  ignition::math::Vector3d launchSpin;
  int launchPhase{0};
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
