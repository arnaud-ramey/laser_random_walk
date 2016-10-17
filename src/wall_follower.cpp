/*!
  \file         wall_follower.cpp
  \author       Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date         2016/11

  ______________________________________________________________________________

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  ______________________________________________________________________________

  This node enables the following and tracking of a moving goal.
  The free space is obtained thanks to the local costmap
  delivered by the move_base componnet.

  Many parameters enable to configure the behaviour of the robot.
  Most notably minimum and maximum speed

\section Parameters
  - \b min_vel_lin
    [double, m.s-1] (default: .1)
    The minimum linear speed.

  - \b max_vel_lin
    [double, rad.s-1] (default: .3)
    The maximum linear speed.

  - \b max_vel_ang
    [double, rad.s-1] (default: .5)
    The maximum absolute angular speed. The minimum angular speed is 0.
    The search domain is then: [-max_vel_ang .. max_vel_ang]

  - \b robot_radius
    [double, meters] (default: .5)
    The robot radius, in meters.
*/
// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// ROS msg
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
// laser_random_walk
#include <laser_random_walk/wanderer.h>


typedef geometry_msgs::Point Pt2;

////////////////////////////////////////////////////////////////////////////////

//! convert from polar to xy coordinates for a laser data
template<class _Pt2>
static inline void convert_sensor_data_to_xy(const sensor_msgs::LaserScan & laser_msg,
                                             std::vector<_Pt2> & out_vector) {
  out_vector.clear();
  out_vector.reserve(laser_msg.ranges.size());
  const float* curr_range = &(laser_msg.ranges[0]);
  float curr_angle = laser_msg.angle_min;
  for (unsigned int idx = 0; idx < laser_msg.ranges.size(); ++idx) {
    //maggieDebug2("idx:%i, curr_range:%g", idx, *curr_range);
    _Pt2 pt;
    pt.x = *curr_range * cos(curr_angle);
    pt.y = *curr_range * sin(curr_angle);
    out_vector.push_back(pt);
    ++curr_range;
    curr_angle += laser_msg.angle_increment;
  } // end loop idx
} // end convert_sensor_data_to_xy()

////////////////////////////////////////////////////////////////////////////////

class ROSWallFollower : public Wanderer<Pt2> {
public:
  ROSWallFollower() : _nh_private("~") {
    _laser_sub = _nh_public.subscribe<sensor_msgs::LaserScan>
        ("scan", 1,  &ROSWallFollower::scan_cb, this);
    _vel_pub = _nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _traj_pub = _nh_public.advertise<nav_msgs::Path>("traj", 1);
    _nh_private.param("min_vel_lin", _min_v, .1);
    _nh_private.param("max_vel_lin", _max_v, .3);
    _nh_private.param("max_vel_ang", _max_w, .5);
    _nh_private.param("robot_radius", _robot_radius, .5);
    _nh_private.param("wall_distance", _wall_distance, .7);
    set_simulation_parameters(3, .2, .5);
  }

  ////////////////////////////////////////////////////////////////////////////////

  //! return < 0 if the trajectory will collide with the laser in the time TIME_PRED,
  //! distance to closest obstacle otherwise + bonus for speed
  virtual double trajectory_grade(const float & v, const float & w,
                          const std::vector<Pt2> & laser_xy) {
    //ROS_WARN("ROSWallFollower::trajectory_grade()");
    // determine the coming trajectory
    std::vector<Pt2> traj_xy;
    odom_utils::make_trajectory(v, w, traj_xy, _time_pred, _time_step, 0, 0, 0);
    // compute average distance between laser and traj, should be = wall distance
    double min_obstacle_distance_sq = _min_obstacle_distance * _min_obstacle_distance;
    unsigned int ntraj = traj_xy.size(), nlaser = laser_xy.size();
      double score = 0;
    for (unsigned int i = 0; i < ntraj; ++i) {
      double min_dist_sq = 1E10;
      for (unsigned int j = 0; j < nlaser; ++j) {
        double distsq = geometry_utils::distance_points_squared(traj_xy[i], laser_xy[j]);
        if (distsq < min_obstacle_distance_sq) // there might be a collision
          return -1;
        if (laser_xy[j].y > 0 || laser_xy[j].x > 1) //point on the right => discard
          continue;
        if (min_dist_sq > distsq)
          min_dist_sq = distsq;
      } // end loop j
      //score += fabs(sqrt(min_dist_sq) - _wall_distance);
      if (fabs(sqrt(min_dist_sq) - _wall_distance) < .1)
        ++score;
    } // end loop i
    score /= ntraj; // normalize
    // >0: to the left
    score += - .1 * w / _max_w; // better turning right
    //score += v;
    //score = w; // better turning right
    return score;
  }

  ////////////////////////////////////////////////////////////////////////////////

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan) {
    convert_sensor_data_to_xy(*scan, _scan2vec);
    geometry_msgs::Twist twist;
#if 0
    set_costmap(_scan2vec, _robot_radius);
    if (!recompute_speeds(twist.linear.x, twist.angular.z)) {
      printf("recompute_speeds() failed!\n");
      return;
    }
#else
    // find wall
    unsigned int npts = scan->ranges.size(), npts_wall = 0, npts_prev = -1;
    std::vector<bool> in_wall(npts, false), are_close(npts*npts);
    // find seed points
    Pt2 seed;
    seed.x = 0;
    seed.y = -_wall_distance; // right side of the robot: y < 0
    // fill in_wall
    for (unsigned int i = 0; i < npts; ++i) {
      in_wall[i] = (geometry_utils::distance_points_squared(_scan2vec[i], seed)
                     < _wall_distance*_wall_distance);
      // fill are_close
      for (unsigned int j = i; j < npts; ++j) {
        are_close[i * npts  + j]
            = are_close[j * npts  + i]
            = (geometry_utils::distance_points_squared(_scan2vec[i], _scan2vec[j])
               < _wall_distance*_wall_distance);
      } // end for j
    } // end for i
    // determine cluster
    while (npts_prev != npts_wall) {
      npts_prev = npts_wall;
      for (unsigned int i = 0; i < npts; ++i) {
        if (in_wall[i])
          continue;
        for (unsigned int j = 0; j < npts; ++j) {
          if (!in_wall[j] || !are_close[i * npts + j])
            continue;
          in_wall[i] = true;
          ++npts_wall;
          break;
        } // end for j
      } // end for i
    } // end while()
    ROS_INFO("Wall size:%i", npts_wall);
#endif
    _vel_pub.publish(twist);
    if (!_traj_pub.getNumSubscribers()) // do nothing if no subscriber
      return;
    std::vector<Pt2> traj_xy = get_best_trajectory();
    unsigned int npts_traj = traj_xy.size();
    _path_msg.header = scan->header;
    _path_msg.poses.resize(npts_traj);
    for (unsigned int i = 0; i < npts_traj; ++i) {
      _path_msg.poses[i].pose.position = traj_xy[i];
      _path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0);;
    }
    _traj_pub.publish(_path_msg);
  } // end scan_cb();

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  ros::NodeHandle _nh_public, _nh_private;
  double _robot_radius, _wall_distance;
  std::vector<Pt2> _scan2vec;
  ros::Subscriber _laser_sub;
  ros::Publisher _vel_pub, _traj_pub;
  nav_msgs::Path _path_msg;
}; // end class ROSWallFollower

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  ros::init(argc, argv, "wall_follower"); //Initialise and create a ROS node
  ROSWallFollower wanderer;
  ros::spin();
}

