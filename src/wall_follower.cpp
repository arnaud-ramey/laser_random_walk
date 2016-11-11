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
#include <laser_random_walk/wanderer.h>
// ROS
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
// ROS msg
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

typedef geometry_msgs::Point32 Pt2;

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
    //ROS_INFO("idx:%i, curr_range:%g", idx, *curr_range);
    _Pt2 pt;
    pt.x = *curr_range * cos(curr_angle);
    pt.y = *curr_range * sin(curr_angle);
    out_vector.push_back(pt);
    ++curr_range;
    curr_angle += laser_msg.angle_increment;
  } // end loop idx
} // end convert_sensor_data_to_xy()

////////////////////////////////////////////////////////////////////////////////

class ROSWallFollower {
public:
  ROSWallFollower() : _nh_private("~") {
    _laser_sub = _nh_public.subscribe<sensor_msgs::LaserScan>
        ("scan", 1,  &ROSWallFollower::scan_cb, this);
    _wall_pub = _nh_public.advertise<sensor_msgs::PointCloud>("wall", 1);
    _goal_pub = _nh_public.advertise<geometry_msgs::PoseStamped>("goal", 1);
    _nh_private.param("wall_distance", _wall_distance, .7);
  }

  ////////////////////////////////////////////////////////////////////////////////

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan) {
    vision_utils::Timer timer;
    convert_sensor_data_to_xy(*scan, _scan2vec);
    // find right wall
    unsigned int npts = scan->ranges.size(), npts_prev = -1;
    in_wall.resize(npts, false);
    are_close.resize(npts*npts);
    // find seed points
    Pt2 seed;
    seed.x = 0;
    seed.y = -_wall_distance; // right side of the robot: y < 0
    // fill in_wall
    for (unsigned int i = 0; i < npts; ++i) {
      in_wall[i] = (utils::distance_points_squared(_scan2vec[i], seed)
                    < _wall_distance*_wall_distance);
      // fill are_close
      for (unsigned int j = i; j < npts; ++j) {
        are_close[i * npts  + j]
            = are_close[j * npts  + i]
            = (utils::distance_points_squared(_scan2vec[i], _scan2vec[j])
               < _wall_distance*_wall_distance);
      } // end for j
    } // end for i

    // determine wall cluster
    std::vector<Pt2> wall;
    while (npts_prev != wall.size()) {
      npts_prev = wall.size();
      for (unsigned int i = 0; i < npts; ++i) {
        if (in_wall[i])
          continue;
        for (unsigned int j = 0; j < npts; ++j) {
          if (!in_wall[j] || !are_close[i * npts + j])
            continue;
          wall.push_back(_scan2vec[i]);
          in_wall[i] = true;
          break;
        } // end for j
      } // end for i
    } // end while()
    unsigned int wall_size = wall.size();
    ROS_INFO("Wall size:%i, time %g ms", wall_size, timer.time());

    // find point at the foreleft
    if (wall_size == 0) {
      ROS_WARN("Could not find wall!");
      return;
    }
    Pt2 wall_end = wall.front();
    double max_theta = atan2(wall[0].y, wall[0].x);
    for (unsigned int i = 0; i < wall_size; ++i) {
      double theta = atan2(wall[i].y, wall[i].x);
      if (max_theta > theta)
        continue;
      max_theta = theta;
      wall_end = wall[i];
    } // end for i

    // set goal
    Pt2 goal;
    goal.x = wall_end.x - _wall_distance * sin(max_theta);
    goal.y = wall_end.y + _wall_distance * cos(max_theta);

    // publish messages
    if (_wall_pub.getNumSubscribers()) {
      _wall_msg.header = scan->header;
      _wall_msg.points = wall;
      _wall_pub.publish(_wall_msg);
    }
    if (_goal_pub.getNumSubscribers()) {
      geometry_msgs::PoseStamped goal_msg;
      goal_msg.header = scan->header;
      goal_msg.pose.position.x = goal.x;
      goal_msg.pose.position.y = goal.y;
      goal_msg.pose.orientation = tf::createQuaternionMsgFromYaw(max_theta);
      _goal_pub.publish(goal_msg);
    }
  } // end scan_cb();

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  ros::NodeHandle _nh_public, _nh_private;
  double _wall_distance;
  std::vector<bool> in_wall, are_close;
  std::vector<Pt2> _scan2vec;
  ros::Subscriber _laser_sub;
  ros::Publisher _vel_pub, _wall_pub, _goal_pub;
  sensor_msgs::PointCloud _wall_msg;
}; // end class ROSWallFollower

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  ros::init(argc, argv, "wall_follower"); //Initialise and create a ROS node
  ROSWallFollower wanderer;
  ros::spin();
}

