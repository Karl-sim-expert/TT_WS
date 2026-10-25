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

// Analytic bounce on a horizontal surface with normal n = +z.
//   v_n+ = -e v_n-,  u = v_t + r n x w,  dv_t = -mu (1+e) |v_n-| u_hat (capped at sticking),
//   dw = -(3 / 2r) n x dv_t   (hollow sphere, I = 2/3 m r^2)
inline void table_bounce(const BallParams & p, Vec3 & v, Vec3 & w)
{
  const double vn = v.z;
  if (vn >= 0.0) {
    return;
  }
  const Vec3 n{0, 0, 1};
  const Vec3 vt{v.x, v.y, 0.0};
  const Vec3 u = vt + p.radius * cross(n, w);
  const double u_norm = std::sqrt(u.x * u.x + u.y * u.y);

  double dv = p.friction * (1.0 + p.restitution) * std::abs(vn);
  dv = std::min(dv, u_norm / 2.5);  // contact-point velocity changes by 2.5 dv_t: stop at rolling

  Vec3 dvt{};
  if (u_norm > 1e-9) {
    dvt = {-dv * u.x / u_norm, -dv * u.y / u_norm, 0.0};
  }
  v.x += dvt.x;
  v.y += dvt.y;
  v.z = -p.restitution * vn;
  w = w + (-3.0 / (2.0 * p.radius)) * cross(n, dvt);
}

}  // namespace tt_gazebo
