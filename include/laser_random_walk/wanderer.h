#ifndef _WANDERER_H_
#define _WANDERER_H_

#include <vector>
#include <math.h>
#include "odom_utils.h"
#include "timer.h"

////////////////////////////////////////////////////////////////////////////////

template<typename T>
static inline T clamp(T Value, T Min, T Max) {
  return (Value < Min)? Min : (Value > Max)? Max : Value;
}

namespace geometry_utils {

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
 \return bool
    true if there is a a pair of points closer than min_dist
*/
template<class _Pt2>
inline bool two_vectors_closer_than_threshold(const std::vector<_Pt2> & A,
                                              const std::vector<_Pt2> & B,
                                              const float min_dist) {
  float min_dist_sq = min_dist * min_dist;
  for (unsigned int A_idx = 0; A_idx < A.size(); ++A_idx) {
    for (unsigned int B_idx = 0; B_idx < B.size(); ++B_idx) {
      if (geometry_utils::distance_points_squared(A[A_idx], B[B_idx])
          < min_dist_sq)
        return true;
    } // end loop B_idx
  } // end loop A_idx
  return false;
} // end two_vectors_closer_than_threshold()

////////////////////////////////////////////////////////////////////////////////

} // end namespace geometry_utils


/*!
* \struct SpeedOrder
* A minimalistic structure for representing an order sent to a mobile base,
* made of a linear and an angular speed.
*/
struct SpeedOrder {
  double vel_lin;
  double vel_ang;
  SpeedOrder() : vel_lin(0), vel_ang(0) {}
  SpeedOrder(const double & lin, const double & ang)
    : vel_lin(lin), vel_ang(ang) {}
  
  //////////////////////////////////////////////////////////////////////////////
  
  /*! clamp vel_ang to interval
[-max_vel_ang, max_vel_ang]
and vel_lin to interval
[-max_lin_speed, -min_lin_speed] U [min_lin_speed, max_lin_speed] */
  void clamp_speed(const double & min_vel_lin_,
                   const double & max_vel_lin_,
                   const double & max_vel_ang_) {
    vel_ang = clamp(vel_ang, -max_vel_ang_, max_vel_ang_);
    vel_lin = clamp(vel_lin, min_vel_lin_, max_vel_lin_);
  } // end clamp();
}; // end SpeedOrder

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

template<class Pt2>
class Wanderer {
public:
  struct Pose2 {
    Pt2 position;
    float yaw;
  };
  
  //////////////////////////////////////////////////////////////////////////////

  Wanderer() {
    _was_stopped = true;
    _min_vel_lin = _max_vel_lin = _max_vel_ang = -1;
    // default  params
    _current_robot_pose.position.x = _current_robot_pose.position.y = 0;
    _current_robot_pose.yaw = 0;
    set_simulation_parameters();
  }
  
  //////////////////////////////////////////////////////////////////////////////
  
  inline void set_costmap(const std::vector<Pt2> & costmap_cell_centers,
                          const double costmap_cell_width,
                          const double costmap_cell_height) {
    _costmap_cell_centers = costmap_cell_centers;
    _costmap_cell_width = costmap_cell_width;
    _costmap_cell_height = costmap_cell_height;
  }
  
  //////////////////////////////////////////////////////////////////////////////
  
  inline void set_limit_speeds(double min_vel_lin, double max_vel_lin, double max_vel_ang,
                               double min_rotate_on_place_speed = -1, 
                               double max_rotate_on_place_speed = -1) {
    _min_vel_lin = min_vel_lin;
    _max_vel_lin = max_vel_lin;
    _max_vel_ang = max_vel_ang;
    _min_rotate_on_place_speed = (min_rotate_on_place_speed != -1 ?
                                  min_rotate_on_place_speed : fabs(max_vel_ang) * 2 / 3);
    _max_rotate_on_place_speed = (max_rotate_on_place_speed != -1 ?
                                  max_rotate_on_place_speed : fabs(max_vel_ang));
  }
  
  //////////////////////////////////////////////////////////////////////////////
  
  inline virtual void set_simulation_parameters(const double time_pred = 5, 
                                                const double time_step = .2,
                                                double speed_recomputation_timeout = 1) {
    _time_pred = time_pred;
    _time_step = time_step;
    _speed_recomputation_timeout = speed_recomputation_timeout;
  }
  
  //////////////////////////////////////////////////////////////////////////////
  
  inline std::vector<Pt2> get_best_trajectory() const {
    std::vector<Pt2> out_traj;
    odom_utils::make_trajectory(// get_best_element().element.vel_lin, 
        _current_order.vel_lin,
        // get_best_element().element.vel_ang, 
        _current_order.vel_ang,
        out_traj, 
        _time_pred, _time_step,
        0, 0, 0);
    //start_pos.x, start_pos.y, start_yaw);
    return out_traj;
  }
  
  //////////////////////////////////////////////////////////////////////////////
  
  virtual bool recompute_speeds(double & best_speed_lin,
                                double & best_speed_ang) {
    bool want_recompute_speed = false;
    // recompute speeds if the last ones are too old
    if (!want_recompute_speed && _last_speed_recomputation_timer.getTimeSeconds() > _speed_recomputation_timeout) {
      printf("speed_timeout");
      want_recompute_speed = true;
    }
    // check if there will be a collision soon with the elected speeds
    if (!want_recompute_speed &&
        check_trajectory_valid(_current_order.vel_lin, _current_order.vel_ang, _costmap_cell_centers) == false) {
      printf("coming_collision !");
      want_recompute_speed = true;
    } // end
  
    if (want_recompute_speed) {
      bool new_speeds_found = false;
      _last_speed_recomputation_timer.reset();
      int nb_tries = 0;
      // choose a random speed and check it enables at least 1 second of movement
      while (!new_speeds_found) {
        // authorize on place rotations only after having tried many times
        float curr_min_vel_lin = (nb_tries < 100 ? _min_vel_lin : 0);
        if (1.f * nb_tries / MAX_TRIES > .9 )
          curr_min_vel_lin = -.1; // authorize backward moves in extreme cases
        _current_order.vel_lin = curr_min_vel_lin + drand48() * (_max_vel_lin - curr_min_vel_lin);
        _current_order.vel_ang = drand48() * 2 * _max_vel_ang - _max_vel_ang;
        new_speeds_found = check_trajectory_valid(_current_order.vel_lin, _current_order.vel_ang, _costmap_cell_centers);
        if (nb_tries++ > MAX_TRIES)
          break;
      } // end while (!new_speeds_found)
  
      if (!new_speeds_found) {
        printf("The robot is stuck! Stopping motion and exiting.");
        stop_robot();
        return false;
      } // end not new_speeds_found
  
      printf("Found a suitable couple of speeds in %g ms and %i tries: "
               "_vel_lin:%g, _vel_ang:%g",
               _last_speed_recomputation_timer.getTimeMilliseconds(), nb_tries, 
               _current_order.vel_lin, 
               _current_order.vel_ang);
      _last_speed_recomputation_timer.reset();
    } // end if (want_recompute_speed)

    // publish the computed speed
    printf("DynamicWindow: Publishing _vel_lin:%g, _vel_ang:%g",
           _current_order.vel_lin, _current_order.vel_ang);
    best_speed_lin = _current_order.vel_lin;
    best_speed_ang = _current_order.vel_ang;
    return true;
  } // end recompute_speeds()
  
  
protected:  

  ////////////////////////////////////////////////////////////////////////////////
  
  inline void set_speed(const SpeedOrder & new_speed) {
    printf("set_speed(lin:%g, ang:%g)",
           new_speed.vel_lin, new_speed.vel_ang);
    _was_stopped = (new_speed.vel_lin == 0) && (new_speed.vel_ang == 0);
    _current_order = new_speed;
  }
  
  //////////////////////////////////////////////////////////////////////////////
  
  //! stop the robot
  inline void stop_robot() {
    set_speed(SpeedOrder());
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  
  //! return true if the trajectory wont collide with the laser in the time TIME_PRED
  bool check_trajectory_valid(const float & vel_lin, const float & vel_ang,
                              const std::vector<Pt2> & laser_xy) {
    // determine the coming trajectory
    std::vector<Pt2> traj_xy;
    odom_utils::make_trajectory(vel_lin, vel_ang, traj_xy, _time_pred, _time_step, 0, 0, 0);
    // find if there might be a collision
    if (geometry_utils::two_vectors_closer_than_threshold(traj_xy, laser_xy, min_obstacle_distance))
      return false;
    else
      return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////
  
  std::vector<Pt2> _traj_buffer;

  // control
  //! timer since last computed speed
  Timer _last_speed_recomputation_timer;
  //! the current velocities, m/s or rad/s
  SpeedOrder _current_order;
  bool _was_stopped;  
  
  // obstacles
  std::vector<Pt2> _costmap_cell_centers;
  double _costmap_cell_width, _costmap_cell_height;
  
  // robot parameters
  Pose2 _current_robot_pose;
  double _min_vel_lin, _max_vel_lin, _max_vel_ang;
  double _min_rotate_on_place_speed, _max_rotate_on_place_speed;
  
  // simul parameters
  double _speed_recomputation_timeout;
  //! the forecast time in seconds
  double _time_pred;
  //! the time step simulation
  double _time_step;
  
  // wanderer params
  static const int MAX_TRIES = 100000;
  //! the distance we want to keep away from obstacles, meters
  static const double min_obstacle_distance = .4;
}; // end class Wanderer

#endif // _WANDERER_H_
