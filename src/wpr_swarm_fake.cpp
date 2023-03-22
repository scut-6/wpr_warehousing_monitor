/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "WPR_Swarm.h"

int main(int argc, char **argv)
{
  setlocale(LC_CTYPE,"zh_CN.utf8");
  ros::init(argc,argv, "wpr_swarm_fake"); 
  ROS_INFO("[wpr_swarm_fake] Start");

  ros::NodeHandle n;

  tf::TransformBroadcaster odom_broadcaster;
  float robot_x,robot_y,robot_yaw;
  robot_x = robot_y = robot_yaw = 0;
  tf::Transform robot_tf;

  CWPR_Swarm wpr_swarm;

  wpr_swarm.Initial();
  ROS_WARN("wpr_swarm.InitUDPServer(20201)");
  wpr_swarm.InitUDPServer(20201);

  ros::Rate r(30);
  while(ros::ok())
  {
    wpr_swarm.UpdateAll();
    // robot_x += 0.001;
    // robot_yaw +=0.001;
    // robot_tf.setOrigin( tf::Vector3(robot_x, robot_y, 0) );
    // tf::Quaternion q;
    // q.setEuler(0,0,robot_yaw);
    // //printf("[wpr_server_node] ( %.2f , %.2f ) Yaw= %f \n",robot_x,robot_y,robot_yaw*180/3.1415);
    // robot_tf.setRotation(q);
    // odom_broadcaster.sendTransform(tf::StampedTransform(robot_tf, ros::Time::now(), "map", "wpr1_1/base_footprint"));

    r.sleep();
    ros::spinOnce();
  }
 
  return 0;
}