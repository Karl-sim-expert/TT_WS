#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "tt_gazebo/ball_dynamics.hpp"

using namespace tt_gazebo;

static int failures = 0;
#define CHECK(cond, msg) \
  do { \
    if (!(cond)) {std::printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__); ++failures;} \
  } while (0)

int main()
{
  BallParams p;

  // Drag at 10 m/s: 0.5 * 1.2 * 0.4 * pi * 0.02^2 * 100 ~= 0.0302 N, opposing the velocity.
  Vec3 f = aerodynamic_force(p, {10, 0, 0}, {});
  CHECK(std::abs(f.x + 0.03016) < 1e-3, "drag magnitude");
  CHECK(std::abs(f.y) < 1e-12 && std::abs(f.z) < 1e-12, "drag direction");

  // Topspin on a ball moving +x (spin about +y = forward rolling): force must point down (-z).
  f = aerodynamic_force(p, {10, 0, 0}, {0, 300, 0});
  CHECK(f.z < 0, "topspin pushes the ball down");
  // Backspin: force up.
  f = aerodynamic_force(p, {10, 0, 0}, {0, -300, 0});
  CHECK(f.z > 0, "backspin lifts the ball");
  // Sidespin about +z moving +x: w x v = (0,0,w) x (v,0,0) = (0, w v, 0): force toward +y.
  f = aerodynamic_force(p, {10, 0, 0}, {0, 0, 300});
  CHECK(f.y > 0, "sidespin curves the ball sideways");

  // No-spin vertical drop: v_z' = -e v_z, horizontal speed unchanged.
  Vec3 v{2.0, 0.0, -4.0}, w{};
  BallParams frictionless = p;
  frictionless.friction = 0.0;
  table_bounce(frictionless, v, w);
  CHECK(std::abs(v.z - 3.6) < 1e-9, "restitution");
  CHECK(std::abs(v.x - 2.0) < 1e-9, "no friction keeps horizontal speed");

  // Friction slows a sliding no-spin ball and gives it topspin-like rotation (rolling direction).
  v = {5.0, 0.0, -3.0};
  w = {};
  table_bounce(p, v, w);
  CHECK(v.x < 5.0 && v.x > 0, "friction slows horizontal speed");
  CHECK(w.y > 0, "sliding forward induces rotation about +y (forward rolling)");

  // A ball already rolling without slipping keeps its horizontal speed.
  v = {5.0, 0.0, -3.0};
  w = {0.0, 5.0 / p.radius, 0.0};  // u = vt + r n x w = 5 + r*(-w_y) = 0 -> pure rolling
  table_bounce(p, v, w);
  CHECK(std::abs(v.x - 5.0) < 1e-6, "rolling ball keeps horizontal speed");

  // Backspin ball arriving with +x speed kicks back (slows more than a no-spin ball).
  Vec3 vb{5.0, 0.0, -3.0}, wb{0.0, -200.0, 0.0};
  Vec3 vn0{5.0, 0.0, -3.0}, wn0{};
  table_bounce(p, vb, wb);
  table_bounce(p, vn0, wn0);
  CHECK(vb.x <= vn0.x + 1e-9, "sliding friction is the same, so backspin never speeds the ball up");
  CHECK(wb.y > -200.0, "backspin is reduced by the bounce");

  // Topspin ball (w_y > 0 for +x travel): contact point nearly sticks, so the horizontal speed
  // drops less than for a no-spin ball.
  Vec3 vt{5.0, 0.0, -3.0}, wt{0.0, 200.0, 0.0};
  table_bounce(p, vt, wt);
  CHECK(vt.x >= vn0.x - 1e-9, "topspin keeps at least as much horizontal speed as no spin");

  // Upward-moving ball does not bounce.
  v = {1, 0, 2};
  w = {};
  table_bounce(p, v, w);
  CHECK(v.z == 2, "no bounce when leaving");

  std::printf(failures ? "%d test(s) FAILED\n" : "all tests passed\n", failures);
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
