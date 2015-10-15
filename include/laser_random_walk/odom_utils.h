#ifndef ODOM_UTILS_H
#define ODOM_UTILS_H

#include <vector>
#include <stdio.h>
#include <math.h>

namespace odom_utils {

/*!
 \param x
          the position to update, in meters
 \param y
          the position to update, in meters
 \param yaw
          the angle to update, in radians
 \param vx
 \param vy
 \param vyaw
 \param dt_sec
          time elapsed with these speeds in seconds
*/
inline void update_pos_rot(float & x, float & y, float & yaw,
                           const float & vx, const float & vy, const float & vyaw,
                           const float & dt_sec) {
  if (vyaw == 0) {
    x += vx * dt_sec * cos(yaw);
    y += vx * dt_sec * sin(yaw);
        return;
  } // end if vyaw == 0

  if (vy != 0) {
    printf("Formulas not available for vy != 0 (vy == %g) \n", vy);
    return;
  }

  // we were on a circle of radius vx / vyaw
  // and of center
  // xC = x + radius * cos (yaw + PI / 2) = x - radius * sin(yaw)
  // yC = y + radius * sin (yaw + PI / 2) = y + radius * cos(yaw)

  // it is parametrized by
  // x(t) = xC + radius * cos(t * vyaw + (yaw - PI / 2))
  //      = xC + radius * sin(t * vyaw + yaw)
  //      = x  + radius * (sin(t * vyaw + yaw) - sin(yaw))
  //
  // y(t) = yC + radius * sin(t * vyaw + (yaw - PI / 2))
  //      = yC - radius * cos(t * vyaw + yaw)
  //      = y  + radius * (-cos(t * vyaw + yaw) + cos(yaw))
  //
  // the yaw has been incremented by vyaw * dt

  x += vx / vyaw * ( sin(dt_sec * vyaw + yaw) - sin(yaw));
  y += vx / vyaw * (-cos(dt_sec * vyaw + yaw) + cos(yaw));
  yaw += vyaw * dt_sec;

  //  // linear update
  //  x += order.linear.x * dt_sec;
  //  y += order.linear.y * dt_sec;
  //  // angular update
  //  float dyaw = order.angular.z * dt_sec;
  //  float xnew = cos(dyaw) * x - sin(dyaw) * y;
  //  y          = sin(dyaw) * x + cos(dyaw) * y;
  //  x = xnew;
  //  yaw += dyaw;
} // end update_pos_rot();

////////////////////////////////////////////////////////////////////////////////

/*!
 \param vel_lin
    The linear speed (m/s)
 \param vel_ang
    The angular speed (rad/s)
 \param out_traj
    The trajectory that will be populated
 \param time_end
    The maximum time
 \param dt
    The time step for the simulation
 \param x0
    Initial position in x
 \param y0
    Initial position in y
 \param yaw0
    Initial position in yaw
*/
template<class _Pt2>
static void make_trajectory(const float & vel_lin, const float & vel_ang,
                            std::vector<_Pt2> & out_traj,
                            const float & time_end, const float & dt,
                            const float & x0, const float & y0, const float & yaw0) {
  float x = x0, y = y0, yaw = yaw0;
  out_traj.clear();
  out_traj.reserve(time_end / dt);
  for (float t = 0; t < time_end; t+= dt) {
    odom_utils::update_pos_rot(x, y, yaw, vel_lin, 0, vel_ang, dt);
    _Pt2 pt;
    pt.x = x; pt.y = y;
    out_traj.push_back(pt);
  } // end loop t
} // end make_trajectory();

////////////////////////////////////////////////////////////////////////////////

} // end namespace odom_utils

#endif // ODOM_UTILS_H
