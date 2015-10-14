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
    [double] m.s-1 (default: .1)
    The minimum linear speed.

  - \b max_vel_lin
    [double] m.s-1 (default: .3)
    The maximum linear speed.

  - \b max_vel_ang
    [double] rad.s-1 (default: .5)
    The maximum absolute angular speed. The minimum angular speed is 0.
    The search domain is then: [-max_vel_ang .. max_vel_ang]
*/
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <laser_random_walk/wanderer.h>
#include <sensor_msgs/LaserScan.h>

typedef geometry_msgs::Point Pt2;

////////////////////////////////////////////////////////////////////////////////

//! convert from polar to xy coordinates for a laser data
template<class Pt2>
static inline void convert_sensor_data_to_xy(const sensor_msgs::LaserScan & laser_msg,
                                             std::vector<Pt2> & out_vector) {
  out_vector.clear();
  out_vector.reserve(laser_msg.ranges.size());
  const float* curr_range = &(laser_msg.ranges[0]);
  float curr_angle = laser_msg.angle_min;
  for (unsigned int idx = 0; idx < laser_msg.ranges.size(); ++idx) {
    //maggieDebug2("idx:%i, curr_range:%g", idx, *curr_range);
    Pt2 pt;
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
    laser_sub = _nh_public.subscribe<sensor_msgs::LaserScan>
        ("scan", 1,  &ROSWanderer::scan_cb, this);
    double _min_vel_lin,_max_vel_lin, _max_vel_ang;
    _nh_private.param("min_vel_lin", _min_vel_lin, .1);
    _nh_private.param("max_vel_lin", _max_vel_lin, .3);
    _nh_private.param("max_vel_ang", _max_vel_ang, .5);
    set_limit_speeds(_min_vel_lin,_max_vel_lin, _max_vel_ang);
  }

  void scan_cb(const sensor_msgs::LaserScanConstPtr & scan) {
    convert_sensor_data_to_xy(*scan, _scan2vec);
    set_costmap(_scan2vec, .1, .1);
  }

  ros::Subscriber laser_sub;
  ros::NodeHandle _nh_public, _nh_private;
  std::vector<Pt2> _scan2vec;
}; // end class ROSWanderer

////////////////////////////////////////////////////////////////////////////////

int main (int argc, char** argv) {
  ROSWanderer wanderer;
  ros::init(argc, argv, "laser_random_walk"); //Initialise and create a ROS node
  ros::spin();
}
