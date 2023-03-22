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
#include "MissionBuilder.h"

static bool res = false;
static int RobotCarryBox = 0;

CMissionBuilder::CMissionBuilder()
{
}

CMissionBuilder::~CMissionBuilder()
{
}

void CMissionBuilder::RobotGotoTask(int inRobot,string inWaypoint)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot;
    oss << "号导航去往航点";
    oss << inWaypoint;
    new_mis.mission = oss.str();
    GetTaskGoto(inWaypoint,&nt);
    nt.command.id = inRobot;
    nt.command.command = CMD_ROBOT_GOTO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::RobotPoseTask(int inRobot,string inWaypoint)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot;
    oss << "号定位到航点";
    oss << inWaypoint;
    new_mis.mission = oss.str();
    GetTaskGoto(inWaypoint,&nt);
    nt.command.id = inRobot;
    nt.command.command = CMD_ROBOT_POSE;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}
    
void CMissionBuilder::RobotPathToTask(int inRobot,string inWaypoint)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot;
    oss << "号循线去往航点";
    oss << inWaypoint;
    new_mis.mission = oss.str();
    GetTaskPathTo(inWaypoint,&nt);
    nt.command.id = inRobot;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::GrabMobileRack(int inGrabRobot,int inRackRobot)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inGrabRobot;
    oss << "号从";
    oss << inRackRobot;
    oss << "号抓取物品";
    new_mis.mission = oss.str();
    nt.command.id = inGrabRobot;
    nt.command.command = CMD_GRAB_MOBILE;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    nt.command.id = inRackRobot;
    nt.command.command = CMD_STOP;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}
    
void CMissionBuilder::GrabStorageRack(int inGrabRobot)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inGrabRobot;
    oss << "号抓取Storage货架物品";
    new_mis.mission = oss.str();
    nt.command.id = inGrabRobot;
    nt.command.command = CMD_GRAB_STORAGE;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::GrabPalletRack(int inGrabRobot)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inGrabRobot;
    oss << "号抓取Pallet货架物品";
    new_mis.mission = oss.str();
    nt.command.id = inGrabRobot;
    nt.command.command = CMD_GRAB_PALLET;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::PlaceStorageRack(int inPlaceRobot)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inPlaceRobot;
    oss << "号放置Storage货架";
    new_mis.mission = oss.str();
    nt.command.id = inPlaceRobot;
    nt.command.command = CMD_PLACE_STORAGE;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::PlaceMobileRack(int inPlaceRobot,int inRackRobot)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inPlaceRobot;
    oss << "号放置物品到";
    oss << inRackRobot;
    oss << "号";
    new_mis.mission = oss.str();
    nt.command.id = inPlaceRobot;
    nt.command.command = CMD_PLACE_MOBILE;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    nt.command.id = inRackRobot;
    nt.command.command = CMD_STOP;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::PlacePalletRack(int inPlaceRobot)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inPlaceRobot;
    oss << "号放置Pallet货架";
    new_mis.mission = oss.str();
    nt.command.id = inPlaceRobot;
    nt.command.command = CMD_PLACE_PALLET;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::RobotsSyncGoto(int inRobot_1,string inWaypoint_1,int inRobot_2,string inWaypoint_2)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot_1;
    oss << "号与";
    oss << inRobot_2;
    oss << "号同步导航";
    new_mis.mission = oss.str();
    GetTaskGoto(inWaypoint_1,&nt);
    nt.command.id = inRobot_1;
    nt.command.command = CMD_ROBOT_GOTO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    GetTaskGoto(inWaypoint_2,&nt);
    nt.command.id = inRobot_2;
    nt.command.command = CMD_ROBOT_GOTO;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::RobotsSyncPathTo(int inRobot_1,string inWaypoint_1,int inRobot_2,string inWaypoint_2)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot_1;
    oss << "号与";
    oss << inRobot_2;
    oss << "号同步循线";
    new_mis.mission = oss.str();
    GetTaskGoto(inWaypoint_1,&nt);
    nt.command.id = inRobot_1;
    nt.command.command = CMD_PATH_TO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    GetTaskGoto(inWaypoint_2,&nt);
    nt.command.id = inRobot_2;
    nt.command.command = CMD_PATH_TO;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::RobotLeaveDock(int inRobot,int inStopRobot1,int inStopRobot2, int inStopRobot3)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot;
    oss << "号离开充电坞";
    new_mis.mission = oss.str();
    new_mis.state = M_ST_WAIT;
    memset(&nt,0,sizeof(nt));
    nt.command.id = inRobot;
    nt.command.command = CMD_LEAVE_DOCK;
    nt.state = T_ST_WAIT; 
    new_mis.arTask.push_back(nt);
    if(inStopRobot1 > 0)
    {
        nt.command.id = inStopRobot1;
        nt.command.command = CMD_STOP;
        new_mis.arTask.push_back(nt);
    }
    if(inStopRobot2 > 0)
    {
        nt.command.id = inStopRobot2;
        nt.command.command = CMD_STOP;
        new_mis.arTask.push_back(nt);
    }
    if(inStopRobot3 > 0)
    {
        nt.command.id = inStopRobot3;
        nt.command.command = CMD_STOP;
        new_mis.arTask.push_back(nt);
    }
    AddNewMission(&new_mis);
}

void CMissionBuilder::RobotGoCharging(int inRobotID)
{ 
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << "c" << inRobotID;
    std::string charger_name = oss.str();
    GetTaskPathToCharger(charger_name, 1.0, &nt);
    oss.str("");
    oss << inRobotID <<"号去充电";
    new_mis.mission = oss.str();
    nt.command.id = inRobotID;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
    oss.str("");
    oss << inRobotID <<"号进入充电坞";
    new_mis.mission = oss.str();
    new_mis.arTask.clear();
    nt.command.id = inRobotID;
    nt.command.command = CMD_DOCKING;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionBuilder::AllGrabPallet()
{
    stMission new_mis;
    stTask nt;
    new_mis.mission = "全员抓取Pallet";
    for(int i= 1; i<6;i++)
    {
        if(arCmdSend[i-1].bInited == true)
        {
            nt.command.id = i;
            nt.command.command = CMD_GRAB_PALLET;
            nt.state = T_ST_WAIT;
            new_mis.arTask.push_back(nt);
        }
    }
    AddNewMission(&new_mis);
}

void CMissionBuilder::AllPlacePallet()
{
    stMission new_mis;
    stTask nt;
    new_mis.mission = "全员放置Pallet";
    for(int i= 1; i<6;i++)
    {
        if(arCmdSend[i-1].bInited == true)
        {
            nt.command.id = i;
            nt.command.command = CMD_PLACE_PALLET;
            nt.state = T_ST_WAIT;
            new_mis.arTask.push_back(nt);
        }
    }
    AddNewMission(&new_mis);
}

void CMissionBuilder::AllPathTo(string inWaypoint_1, string inWaypoint_2, string inWaypoint_3, string inWaypoint_4, string inWaypoint_5)
{
    stMission new_mis;
    stTask nt;
    new_mis.mission = "全员PathTo";
    GetTaskGoto(inWaypoint_1,&nt);
    nt.command.id = 1;
    nt.command.command = CMD_PATH_TO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);

    GetTaskGoto(inWaypoint_2,&nt);
    nt.command.id = 2;
    nt.command.command = CMD_PATH_TO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);

    GetTaskGoto(inWaypoint_3,&nt);
    nt.command.id = 3;
    nt.command.command = CMD_PATH_TO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);

    GetTaskGoto(inWaypoint_4,&nt);
    nt.command.id = 4;
    nt.command.command = CMD_PATH_TO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);

    GetTaskGoto(inWaypoint_5,&nt);
    nt.command.id = 5;
    nt.command.command = CMD_PATH_TO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);

    AddNewMission(&new_mis);
}