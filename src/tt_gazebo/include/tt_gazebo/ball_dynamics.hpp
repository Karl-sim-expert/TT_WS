// Ball flight and table-bounce model (see design doc, sections 3 and 5).
// Header-only and free of Gazebo types so it can be unit-tested without a simulator.
#pragma once

#include <algorithm>
#include <cmath>

namespace tt_gazebo
{

struct Vec3
{
  double x{0}, y{0}, z{0};
};

inline Vec3 operator+(Vec3 a, Vec3 b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
inline Vec3 operator-(Vec3 a, Vec3 b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
inline Vec3 operator*(double s, Vec3 a) {return {s * a.x, s * a.y, s * a.z};}
inline double dot(Vec3 a, Vec3 b) {return a.x * b.x + a.y * b.y + a.z * b.z;}
inline double norm(Vec3 a) {return std::sqrt(dot(a, a));}
inline Vec3 cross(Vec3 a, Vec3 b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct BallParams
{
  double mass{0.0027};          // [kg]
  double radius{0.02};          // [m]
  double air_density{1.2};      // [kg/m^3]
  double drag_coeff{0.40};      // C_D
  double lift_slope{1.0};       // C_L = lift_slope * S, clipped to lift_max
  double lift_max{0.6};
  double restitution{0.9};      // e (table, normal direction)
  double friction{0.25};        // mu (table, tangential)
};

struct Table
{
  double top_z{0.76};
  double half_x{1.37};
  double half_y{0.7625};
  bool contains(double x, double y) const {return std::abs(x) <= half_x && std::abs(y) <= half_y;}
};

// Force [N] on the ball from drag and the Magnus effect (gravity is left to the physics engine).
//   F = -1/2 rho C_D A |v| v + 1/2 rho C_L A |v|^2 (w x v)/|w x v|,   S = r|w|/|v|,  C_L = C_L(S)
inline Vec3 aerodynamic_force(const BallParams & p, Vec3 v, Vec3 w)
{
  const double speed = norm(v);
  if (speed < 1e-6) {
    return {};
  }
  const double area = M_PI * p.radius * p.radius;
  Vec3 f = (-0.5 * p.air_density * p.drag_coeff * area * speed) * v;
  const Vec3 wxv = cross(w, v);
  const double wxv_norm = norm(wxv);
  if (wxv_norm > 1e-9) {
    const double spin_param = p.radius * norm(w) / speed;
    const double cl = std::min(p.lift_slope * spin_param, p.lift_max);
    f = f + (0.5 * p.air_density * cl * area * speed * speed / wxv_norm) * wxv;
  }
  return f;
}

// Analytic bounce on a surface with unit normal n (pointing toward the ball's side), for a surface
// at rest in the frame in which v and w are given.
//   v_n+ = -e v_n-,  u = v_t + r n x w,  dv_t = -mu (1+e) |v_n-| u_hat (capped at sticking),
//   dw = -(3 / 2r) n x dv_t   (hollow sphere, I = 2/3 m r^2)
inline void surface_bounce(
  const BallParams & p, double restitution, double friction, Vec3 n, Vec3 & v, Vec3 & w)
{
  const double vn = dot(v, n);
  if (vn >= 0.0) {
    return;
  }
  const Vec3 vt = v - vn * n;
  const Vec3 u = vt + p.radius * cross(n, w);
  const double u_norm = norm(u);

  double dv = friction * (1.0 + restitution) * std::abs(vn);
  dv = std::min(dv, u_norm / 2.5);  // contact-point velocity changes by 2.5 dv_t: stop at rolling

  Vec3 dvt{};
  if (u_norm > 1e-9) {
    dvt = (-dv / u_norm) * u;
  }
  v = vt + dvt + (-restitution * vn) * n;
  w = w + (-3.0 / (2.0 * p.radius)) * cross(n, dvt);
}

// Bounce on the horizontal table (n = +z).
inline void table_bounce(const BallParams & p, Vec3 & v, Vec3 & w)
{
  surface_bounce(p, p.restitution, p.friction, Vec3{0, 0, 1}, v, w);
}

// Paddle: disc of `radius` and thickness 2 * half_thickness, face normal = local +z (`axis_z`).
// The rubber has its own restitution and friction. Both faces hit (the ball is on the side of its
// centre), so a ball on the -z side is treated with the opposite normal.
struct PaddleParams
{
  double radius{0.075};
  double half_thickness{0.006};
  double rim_margin{0.01};       // extra reach beyond the disc edge (ball radius ~ rim hits)
  double restitution{0.85};
  double friction{0.45};
};

struct PaddleState
{
  Vec3 pos;       // paddle frame origin (disc centre) in world
  Vec3 axis_z;    // unit normal in world
  Vec3 vel;       // velocity of the origin
  Vec3 omega;     // angular velocity
};

// Ball-paddle contact test and impact. The contact slab is 2 (r + half_thickness) thick, wider
// than the distance a 10 m/s ball plus a fast paddle covers in one 1 ms step, so the engine's
// discrete steps cannot tunnel through. On contact (ball inside the slab and approaching the
// face) v and w are replaced by the post-impact values (the surface moves with the paddle) and
// `pos` is pushed out to the surface. Returns true if an impact happened.
inline bool paddle_contact(
  const BallParams & p, const PaddleParams & pp, const PaddleState & s, Vec3 & pos, Vec3 & v,
  Vec3 & w)
{
  const Vec3 d = pos - s.pos;
  const double h = dot(d, s.axis_z);
  const Vec3 d_plane = d - h * s.axis_z;
  const double slab = p.radius + pp.half_thickness;
  if (std::abs(h) > slab || norm(d_plane) > pp.radius + pp.rim_margin) {
    return false;
  }
  const double side = h >= 0.0 ? 1.0 : -1.0;
  const Vec3 n = side * s.axis_z;
  const Vec3 contact = d_plane + (side * pp.half_thickness) * s.axis_z;  // relative to the origin
  const Vec3 v_surface = s.vel + cross(s.omega, contact);
  Vec3 v_rel = v - v_surface;
  if (dot(v_rel, n) >= 0.0) {
    return false;  // already separating
  }
  surface_bounce(p, pp.restitution, pp.friction, n, v_rel, w);
  v = v_rel + v_surface;
  pos = pos + (slab - std::abs(h)) * n;
  return true;
}

}  // namespace tt_gazebo
