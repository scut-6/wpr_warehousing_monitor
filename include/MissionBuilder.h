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
#ifndef _MISSION_BUILDER_H
#define _MISSION_BUILDER_H
#include "MissionManager.h"

class CMissionBuilder : public CMissionManager
{
public:
    CMissionBuilder();
    virtual ~CMissionBuilder();
    void RobotFollowPathTask(int inRobot,int inPath);
    void RobotGotoTask(int inRobot,string inWaypoint);
    void RobotPoseTask(int inRobot,string inWaypoint);
    void RobotPathToTask(int inRobot,string inWaypoint);
    void GrabStorageRack(int inGrabRobot);
    void GrabMobileRack(int inGrabRobot,int inRackRobot);
    void GrabPalletRack(int inGrabRobot);
    void PlaceStorageRack(int inPlaceRobot);
    void PlaceMobileRack(int inPlaceRobot,int inRackRobot);
    void PlacePalletRack(int inPlaceRobot);
    void RobotsSyncGoto(int inRobot_1,string inWaypoint_1,int inRobot_2,string inWaypoint_2);
    void RobotsSyncPathTo(int inRobot_1,string inWaypoint_1,int inRobot_2,string inWaypoint_2);
    void RobotLeaveDock(int inRobot,int inStopRobot1,int inStopRobot2, int inStopRobot3);
    void RobotGoCharging(int inRobotID);
    void AllGrabPallet();
    void AllPlacePallet();
    void AllPathTo(string inWaypoint_1, string inWaypoint_2, string inWaypoint_3, string inWaypoint_4, string inWaypoint_5);
};
#endif  //_MISSION_BUILDER_H