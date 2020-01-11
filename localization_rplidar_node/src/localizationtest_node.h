/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H
#define ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/GetMap.h>
#include "amcl.h"


namespace roborts_localizationtest {

class LocalizationNodetest {
 public:

  bool getstaticmap(ros::NodeHandle &nh);
 // bool GetLaserPose();

 private:
  
//ROS Node handle
//ros::NodeHandle nh;
//ROS Service
ros::ServiceClient static_map_srv;
////Parameters
Vec3d init_pose_;
Vec3d init_cov_;
//map
double initial_pose_x=1;
double initial_pose_y=1;
double initial_pose_a=0;
double initial_cov_xx=0.1;
double initial_cov_yy=0.1;
double initial_cov_aa=0.1;

std::unique_ptr<Amcl> amcl_ptr_;
//map
 // bool map_init ;
//  bool static_map_received = false;
//   bool latest_tf_valid_ = false;
//   bool sent_first_transform_ = false;
//   bool publish_first_distance_map_ = false;
};

} //roborts_localizationtest
#endif // ROBORTS_LOCALIZATION_LOCALIZATION_NODE_H
