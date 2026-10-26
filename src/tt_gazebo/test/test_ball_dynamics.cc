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

  // ---- Paddle ----
  PaddleParams pp;
  const double slab = p.radius + pp.half_thickness;
  const double pe = pp.restitution;

  // Resting ball on the face side, paddle moving along its normal at 2 m/s: the ball leaves with
  // (1 + e) * 2 m/s along the normal and no spin (no tangential slip).
  {
    PaddleState s{{0, 0, 0}, {0, 0, 1}, {0, 0, 2.0}, {}};
    Vec3 pos{0, 0, slab - 0.004}, vb{}, wb{};
    CHECK(paddle_contact(p, pp, s, pos, vb, wb), "paddle hits a resting ball");
    CHECK(std::abs(vb.z - (1.0 + pe) * 2.0) < 1e-9, "(1+e) v_paddle along the normal");
    CHECK(std::abs(vb.x) < 1e-12 && std::abs(vb.y) < 1e-12, "no sideways velocity");
    CHECK(norm(wb) < 1e-9, "no spin without slip");
    CHECK(std::abs(pos.z - slab) < 1e-12, "ball pushed out to the surface");
  }

  // Same on the other face: the normal flips.
  {
    PaddleState s{{0, 0, 0}, {0, 0, 1}, {0, 0, -2.0}, {}};
    Vec3 pos{0, 0, -(slab - 0.004)}, vb{}, wb{};
    CHECK(!paddle_contact(p, pp, s, pos, vb, wb), "back face ignored by default");
    PaddleParams both = pp;
    both.back_face_hits = true;
    CHECK(paddle_contact(p, both, s, pos, vb, wb), "back face hit");
    CHECK(std::abs(vb.z + (1.0 + pe) * 2.0) < 1e-9, "back face reflects along -z");
  }

  // Ball approaching a static paddle reflects with restitution.
  {
    PaddleState s{{0, 0, 0}, {0, 0, 1}, {}, {}};
    Vec3 pos{0, 0, slab - 0.002}, vb{0, 0, -5.0}, wb{};
    CHECK(paddle_contact(p, pp, s, pos, vb, wb), "ball hits a static paddle");
    CHECK(std::abs(vb.z - pe * 5.0) < 1e-9, "restitution on a static paddle");
  }

  // Tangential paddle motion gives friction and spin (rubber grabs the ball).
  {
    PaddleState s{{0, 0, 0}, {0, 0, 1}, {3.0, 0, 2.0}, {}};
    Vec3 pos{0, 0, slab - 0.004}, vb{}, wb{};
    CHECK(paddle_contact(p, pp, s, pos, vb, wb), "brushing hit");
    CHECK(vb.x > 0.0 && vb.x < 3.0, "ball gets some of the tangential speed");
    CHECK(wb.y < 0.0, "the contact point is dragged forward under the ball: backspin about -y");
  }

  // No contact: separating, outside the disc, outside the slab.
  {
    PaddleState s{{0, 0, 0}, {0, 0, 1}, {}, {}};
    Vec3 pos{0, 0, slab - 0.002}, vb{0, 0, 1.0}, wb{};
    CHECK(!paddle_contact(p, pp, s, pos, vb, wb), "separating ball is not hit again");
    pos = {0.2, 0, slab - 0.002};
    vb = {0, 0, -3.0};
    CHECK(!paddle_contact(p, pp, s, pos, vb, wb), "outside the disc");
    pos = {0, 0, slab + 0.01};
    CHECK(!paddle_contact(p, pp, s, pos, vb, wb), "outside the slab");
  }

  // Tilted paddle: normal (0.6, 0, 0.8), moving along it at 1 m/s -> ball leaves along the normal.
  {
    const Vec3 nrm{0.6, 0, 0.8};
    PaddleState s{{1, 2, 3}, nrm, nrm, {}};
    Vec3 pos = s.pos + (slab - 0.005) * nrm, vb{}, wb{};
    CHECK(paddle_contact(p, pp, s, pos, vb, wb), "tilted paddle hit");
    CHECK(norm(vb - (1.0 + pe) * nrm) < 1e-9, "outgoing velocity along the tilted normal");
  }

  // The swept slab: a ball arriving at 10 m/s against a paddle moving 5 m/s the other way moves
  // 15 mm per 1 ms step, far less than the slab (2 x 26 mm), so some step lands inside it.
  CHECK(2.0 * slab > 15e-3 * 2.0, "slab is wider than the per-step relative motion");

  std::printf(failures ? "%d test(s) FAILED\n" : "all tests passed\n", failures);
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
