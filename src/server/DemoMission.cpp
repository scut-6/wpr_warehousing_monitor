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
#include "DemoMission.h"
#include "MissionManager.h"

static int nFind = -1;

CDemoMission::CDemoMission()
{
}

CDemoMission::~CDemoMission()
{
}

/**************************************************
 * 任务系统初始化
 * ************************************************/
void CDemoMission::Start()
{
    ROS_WARN("[CDemoMission] 任务开始!! ");

    Signal_2();
    
    // 所有机器人去往抓取的航点
    //AllPathTo("1" , "2" , "3" , "4" , "5");
    // 所有机器人进行抓取
    //AllGrabPallet();
    // 所有机器人去往放置的的航点
    //AllPathTo("2" , "3" , "4" , "5" , "1");
    // 所有机器人放置物品到货架上
    //AllPlacePallet();

    bPause = false;
}

/**************************************************
 * 任务切换
 * ************************************************/
void CDemoMission::MissionComplete(stMission* inMission)
{
    ROS_WARN("[CDemoMission] Mission任务 \"%s\" 执行完毕!! ",inMission->mission.c_str());
    
    nFind = inMission->mission.find("全员放置Pallet");
    if(nFind >= 0)
    {
        Signal_2();
        
        // 所有机器人回初始位置
        //AllPathTo("A" , "B" , "C" , "D" , "E");
        // 所有机器人进行抓取
        //AllGrabPallet();
        // 所有机器人去往放置的的航点
        //AllPathTo("2" , "3" , "4" , "5" , "1");
        // 所有机器人放置物品到货架上
        //AllPlacePallet();
    }

}

// 信号1
void CDemoMission::Signal_1()
{
    RobotPathToTask(4,"4");
    GrabPalletRack(4);
    RobotPathToTask(4,"5");
    PlacePalletRack(4);
    bPause = false;
}

// 信号2
void CDemoMission::Signal_2()
{
    RobotPathToTask(1,"1");
    RobotPathToTask(2,"2");
    RobotPathToTask(3,"3");
    AllGrabPallet();
    RobotPoseTask(1,"1");
    RobotPoseTask(2,"2");
    RobotPoseTask(3,"3");
    RobotPathToTask(1,"2");
    RobotPathToTask(2,"3");
    RobotPathToTask(3,"4");
    AllPlacePallet();
    RobotPoseTask(1,"2");
    RobotPoseTask(2,"3");
    RobotPoseTask(3,"4");
}

// 信号3
void CDemoMission::Signal_3()
{
    // RobotsSyncPathTo(3,"4",  4, "3");
    // RobotsSyncPathTo(3,"3",  4, "4");
    // RobotPathToTask(1,"1");
    // RobotPathToTask(2,"2");
    // RobotPathToTask(3,"3");
    // RobotPathToTask(4,"4");
    // RobotPathToTask(5,"5");
    AllPathTo("1" , "2" , "3" , "4" , "5");
    AllGrabPallet();
    // for(int i=1;i<6;i++)
    //     GrabPalletRack(i);
    AllPathTo("2" , "3" , "4" , "5" , "1");
    AllPlacePallet();
}

void CDemoMission::Tac_1()
{

}