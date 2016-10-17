/*!
  \file         laser_random_walk.cpp
  \author       Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date         2015/10/14

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
#include <laser_random_walk/wanderer.h>
// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// ROS msg
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>

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

class ROSWanderer : public Wanderer<Pt2> {
public:
  ROSWanderer() : _nh_private("~") {
    _laser_sub = _nh_public.subscribe<sensor_msgs::LaserScan>
        ("scan", 1,  &ROSWanderer::scan_cb, this);
    _vel_pub = _nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    _traj_pub = _nh_public.advertise<nav_msgs::Path>("traj", 1);
    _nh_private.param("min_vel_lin", _min_v, .1);
    _nh_private.param("max_vel_lin", _max_v, .3);
    _nh_private.param("max_vel_ang", _max_w, .5);
    _nh_private.param("robot_radius", _robot_radius, .5);
  }

  ////////////////////////////////////////////////////////////////////////////////

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan) {
    convert_sensor_data_to_xy(*scan, _scan2vec);
    set_costmap(_scan2vec, _robot_radius);
    geometry_msgs::Twist twist;
    if (!recompute_speeds(twist.linear.x, twist.angular.z)) {
      printf("recompute_speeds() failed!\n");
      return;
    }
    _vel_pub.publish(twist);
    if (!_traj_pub.getNumSubscribers()) // do nothing if no subscriber
      return;
    std::vector<Pt2> traj_xy = get_best_trajectory();
    unsigned int npts = traj_xy.size();
    _path_msg.header = scan->header;
    _path_msg.poses.resize(npts);
    for (unsigned int i = 0; i < npts; ++i) {
      _path_msg.poses[i].pose.position = traj_xy[i];
      _path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(0);;
    }
    _traj_pub.publish(_path_msg);
  } // end scan_cb();

  ////////////////////////////////////////////////////////////////////////////////

  //! return < 0 if the trajectory will collide with the laser in the time TIME_PRED,
  //! distance to closest obstacle otherwise + bonus for speed
  virtual double trajectory_grade(const float & v, const float & w,
                                  const std::vector<Pt2> & laser_xy) {
    //printf("Wanderer::trajectory_grade()\n");

    // determine the coming trajectory
    std::vector<Pt2> traj_xy;
    utils::make_trajectory(v, w, traj_xy, _time_pred, _time_step, 0, 0, 0);
    // find if there might be a collision
    double dist = utils::vectors_dist_thres(traj_xy, laser_xy, _min_obstacle_distance);
    if (dist < 0)
      return -1;
    //return dist + fabs(v); // bonus for speed
    return .1 * std::max(fabs(v)/_max_v, fabs(w)/_max_w) + drand48();
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  ros::Subscriber _laser_sub;
  ros::Publisher _vel_pub, _traj_pub;
  nav_msgs::Path _path_msg;
  ros::NodeHandle _nh_public, _nh_private;
  std::vector<Pt2> _scan2vec;
  double _robot_radius;
}; // end class ROSWanderer

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  ros::init(argc, argv, "laser_random_walk"); //Initialise and create a ROS node
  ROSWanderer wanderer;
  ros::spin();
}
