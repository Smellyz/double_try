/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */

#include "localizationtest_node.h"
#define RAD2DEG(x) ((x)*180./M_PI)

namespace roborts_localizationtest{

bool LocalizationNodetest::getstaticmap(ros::NodeHandle &nh)
{ 
  nav_msgs::GetMap::Request req;
  nav_msgs::GetMap::Response res;
  init_pose_ ={ initial_pose_x,initial_pose_y,initial_pose_a }
  init_cov_ = {initial_cov_xx,initial_cov_yy,initial_cov_aa}
  static_map_srv = nh.serviceClient<nav_msgs::GetMap>("static_map");
  ros::service::waitForService("static_map", -1);
  
if(static_map_srv.call(req,res)) 
 {
    std::cout << "Received Static Map"<< std::endl;
    std::cout << res <<std::endl;
    //ROS_INFO(": %d", res.map);
    amcl_ptr_= std::make_unique<Amcl>();
    amcl_ptr_->HandleMapMessage(res.map, init_pose_, init_cov_);
   //first_map_received_ = true;
    return true;
  } 
else{
    std::cout << "Get static map failed"<< std::endl;
    return false;
  }

}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
       //std::cout << "localizationflag" <<i<< std::endl;
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
}
} // roborts_localization

int main(int argc, char **argv)
{  
  
    ros::init(argc, argv, "localizationtest_node");
    ros::NodeHandle nh;
    LocalizationNodetest testmap;
    bool map_init= false ;
    std::cout << "localizationflag" << std::endl;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);
    map_init= testmap.getstaticmap(nh);

    ros::spin();

    return 0;
}
