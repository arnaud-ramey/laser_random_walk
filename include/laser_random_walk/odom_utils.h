#ifndef utils_H
#define utils_H

#include <vector>
#include <stdio.h>
#include <math.h>

namespace utils {

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
    utils::update_pos_rot(x, y, yaw, vel_lin, 0, vel_ang, dt);
    _Pt2 pt;
    pt.x = x; pt.y = y;
    out_traj.push_back(pt);
  } // end loop t
} // end make_trajectory();

////////////////////////////////////////////////////////////////////////////////

template<typename T>
static inline T clamp(T Value, T Min, T Max) {
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param A
 \param B
 \return the quantity AB * AB. It is faster to compute than their actual distance
 and enough for, for instance, comparing two distances
*/
template<class Point2_A, class Point2_B>
static inline double
distance_points_squared(const Point2_A & A, const Point2_B & B) {
  return (A.x - B.x) * (A.x - B.x) +  (A.y - B.y) * (A.y - B.y);
}

////////////////////////////////////////////////////////////////////////////////

/*!
 \param A
    a vector of 2D points
 \param B
    a vector of 2D points
 \param min_dist
    a threshold distance
 \return min_dist
    -1 if vectors closer than min_dist, min distance otherwise
*/
template<class _Pt2>
inline double vectors_dist_thres(const std::vector<_Pt2> & A,
                                 const std::vector<_Pt2> & B,
                                 const float dist_thres) {
  float dist_thres_sq = dist_thres * dist_thres;
  double min_dist_sq = 1E10;
  for (unsigned int A_idx = 0; A_idx < A.size(); ++A_idx) {
    for (unsigned int B_idx = 0; B_idx < B.size(); ++B_idx) {
      double dist = utils::distance_points_squared(A[A_idx], B[B_idx]);
      if (dist < dist_thres_sq)
        return -1;
      if (dist < min_dist_sq)
        min_dist_sq = dist;
    } // end loop B_idx
  } // end loop A_idx
  return sqrt(min_dist_sq);
} // end vectors_dist_thres()

////////////////////////////////////////////////////////////////////////////////

} // end namespace utils


#endif // utils_H
