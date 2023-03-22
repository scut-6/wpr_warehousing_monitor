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
#include "WPR_Swarm.h"

static const double Pi = 3.14159265358979323846264338327950288419717;

CWPR_Swarm::CWPR_Swarm()
{
    robot_fake = new stRobotFake[ROBOT_FAKE_NUM];
    memset(robot_fake,0,ROBOT_FAKE_NUM*sizeof(stRobotFake));
    arInfoSend = new CRobotInfoSend[ROBOT_FAKE_NUM];
    for(int i=0;i<ROBOT_FAKE_NUM;i++)
    {
        memset(&(arInfoSend[i].info_msg),0,sizeof(stRobotInfoMsg));
        robot_fake[i].nLastPathIndex = -1;

        for(int j=0;j<12;j++)
            robot_fake[i].joint_pos[j] = 0.0f;
        robot_fake[i].joint_pos[7] = -1.4;
        robot_fake[i].joint_pos[8] = 1.4;
    }
    //////////////
    robot_fake[0].nDevType = DEV_WPB_MANI;
    robot_fake[0].map_x = 0;
    robot_fake[0].map_y = 0;
    robot_fake[0].map_yaw = 0;
    robot_fake[0].path_finder.Initialize();

    robot_fake[1].nDevType = DEV_WPB_MANI;
    robot_fake[1].map_x = -1.8;
    robot_fake[1].map_y = 2.9;
    robot_fake[1].map_yaw = 1.57;
    robot_fake[1].path_finder.Initialize();

    robot_fake[2].nDevType = DEV_WPB_MANI;
    robot_fake[2].map_x = -0.9;
    robot_fake[2].map_y = 2.9;
    robot_fake[2].map_yaw = 1.57;
    robot_fake[2].path_finder.Initialize();

    // robot_fake[3].nDevType = DEV_WPB_MANI;
    // robot_fake[3].map_x = -0.2;
    // robot_fake[3].map_y = 2.9;
    // robot_fake[3].map_yaw = 1.57;
    // robot_fake[3].path_finder.Initialize();

    // robot_fake[4].nDevType = DEV_WPB_MANI;
    // robot_fake[4].map_x = 1;
    // robot_fake[4].map_y =  2.9;
    // robot_fake[4].map_yaw = 0;
    // robot_fake[4].path_finder.Initialize();

    //////////////
    fTimeScale = 2;
}

CWPR_Swarm::~CWPR_Swarm()
{
    delete []robot_fake;
    delete []arInfoSend;
}

void CWPR_Swarm::Initial()
{
    arInfoSend[0].SetDevType(DEV_WPB_MANI);
    arInfoSend[1].SetDevType(DEV_WPB_MANI);
    arInfoSend[2].SetDevType(DEV_WPB_MANI);
    arInfoSend[3].SetDevType(DEV_WPB_MANI);
    arInfoSend[4].SetDevType(DEV_WPB_MANI);
    arInfoSend[5].SetDevType(DEV_WPB_MANI);
    arInfoSend[6].SetDevType(DEV_WPB_MANI);    
    for(int i=0;i<ROBOT_FAKE_NUM;i++)
    {
        arInfoSend[i].SetID(i+1);
        string ServerIP = "127.0.0.1";
        arInfoSend[i].InitUDPClient(ServerIP.c_str(),20202);
    }
}

void CWPR_Swarm::RecvNewPackage(stCommandMsg* inCmd)
{
    //ROS_WARN("CWPR_Swarm ID = %d  CMD= %s",inCmd->id, str_helper.GetCommandStr(inCmd->command).c_str());
    if(inCmd->id < 1 || inCmd->id > ROBOT_FAKE_NUM)
    {
        ROS_WARN("CWPR_Swarm::RecvNewPackage Invalid ID = %d",inCmd->id);
        return;
    }

    int nRobotID = inCmd->id;
    if(robot_fake[nRobotID-1].nLastRecvCmd == inCmd->command)
    {
        if(inCmd->command == CMD_ROBOT_GOTO && (robot_fake[nRobotID-1].target_x !=  inCmd->map_x || robot_fake[nRobotID-1].target_y !=  inCmd->map_y || robot_fake[nRobotID-1].target_yaw !=  inCmd->map_yaw))
        {}
        else if(inCmd->command == CMD_PATH_TO && (robot_fake[nRobotID-1].target_x !=  inCmd->map_x || robot_fake[nRobotID-1].target_y !=  inCmd->map_y || robot_fake[nRobotID-1].target_yaw !=  inCmd->map_yaw))
        {}
        else if(inCmd->command == CMD_ROBOT_POSE && (robot_fake[nRobotID-1].target_x !=  inCmd->map_x || robot_fake[nRobotID-1].target_y !=  inCmd->map_y || robot_fake[nRobotID-1].target_yaw !=  inCmd->map_yaw))
        {}
        else if(inCmd->command == CMD_ROBOT_TELEOP)
        {}
        else if(inCmd->command == CMD_FOLLOW_PATH && inCmd->path_index != robot_fake[nRobotID-1].nLastPathIndex)
        {
           robot_fake[nRobotID-1].nLastPathIndex = inCmd->path_index;
        }
        else
        {
            return;
        }
    }

    switch (inCmd->command)
    {
    case CMD_STOP:
        robot_fake[nRobotID-1].nState = RBT_ST_STOP;
        break;
    case CMD_ROBOT_GOTO:
        robot_fake[nRobotID-1].nState = RBT_ST_GOTO;
        robot_fake[nRobotID-1].target_x = inCmd->map_x;
        robot_fake[nRobotID-1].target_y = inCmd->map_y;
        robot_fake[nRobotID-1].target_yaw = inCmd->map_yaw;
        break;
    case CMD_PATH_TO:
        robot_fake[nRobotID-1].nState = RBT_ST_FOLLOW_PATH;
        robot_fake[nRobotID-1].target_x = inCmd->map_x;
        robot_fake[nRobotID-1].target_y = inCmd->map_y;
        robot_fake[nRobotID-1].target_yaw = inCmd->map_yaw;
        robot_fake[nRobotID-1].nPathIndex = robot_fake[nRobotID-1].path_finder.GetPathIndex(
            robot_fake[nRobotID-1].map_x,
            robot_fake[nRobotID-1].map_y,
            robot_fake[nRobotID-1].target_x,
            robot_fake[nRobotID-1].target_y);
        robot_fake[nRobotID-1].nPointIndex = 0;
        robot_fake[nRobotID-1].bPathTo = true;
        break;
    case CMD_GRAB_BOX:
        robot_fake[nRobotID-1].nState = RBT_ST_BOX_DETECT;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_GRAB_COLOR:
        robot_fake[nRobotID-1].nState = RBT_ST_BOX_GRAB;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_PLACE_COLOR:
        robot_fake[nRobotID-1].nState = RBT_ST_PLACE_COLOR;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_PLACE_MOBILE:
        robot_fake[nRobotID-1].nState = RBT_ST_MOBILE_DETECT;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_GRAB_MOBILE:
        robot_fake[nRobotID-1].nState = RBT_ST_GM_DETECT;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_PLACE_PALLET:
        robot_fake[nRobotID-1].nState = RBT_ST_PALLET_DETECT;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_GRAB_PALLET:
        robot_fake[nRobotID-1].nState = RBT_ST_GP_DETECT;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_PLACE_STORAGE:
        robot_fake[nRobotID-1].nState = RBT_ST_PS_DETECT;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_GRAB_STORAGE:
        robot_fake[nRobotID-1].nState = RBT_ST_GS_DETECT;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_DOCKING:
        robot_fake[nRobotID-1].nState = RBT_ST_DOCK_FACETO;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_CHARGING:
        robot_fake[nRobotID-1].nState = RBT_ST_DOCK_FACETO;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_LEAVE_DOCK:
        robot_fake[nRobotID-1].nState = RBT_ST_DOCK_LEAVE;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_CONVEY_BOX:
        robot_fake[nRobotID-1].nState = RBT_ST_CONVEY_BOX;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
        break;
    case CMD_LT_POSE_INIT:
        robot_fake[nRobotID-1].nState = RBT_ST_LT_INIT;
        robot_fake[nRobotID-1].nCount = 30*fTimeScale;
    case CMD_LT_POSE_SET:
        robot_fake[nRobotID-1].nState = RBT_ST_STOP;
        break;
    case CMD_ROBOT_POSE:
        robot_fake[nRobotID-1].map_x =  inCmd->map_x;
        robot_fake[nRobotID-1].map_y =  inCmd->map_y;
        robot_fake[nRobotID-1].map_yaw =  inCmd->map_yaw;
        break;
    case CMD_ROBOT_TELEOP:
        robot_fake[nRobotID-1].nState= RBT_ST_TELEOP;
        robot_fake[nRobotID-1].vel_x =  inCmd->vel_x;
        robot_fake[nRobotID-1].vel_y =  inCmd->vel_y;
        robot_fake[nRobotID-1].vel_angular =  inCmd->vel_angular;
        break; 
    case CMD_FOLLOW_PATH:
        robot_fake[nRobotID-1].nState = RBT_ST_FOLLOW_PATH;
        robot_fake[nRobotID-1].nPathIndex = inCmd->path_index;
        robot_fake[nRobotID-1].nPointIndex = 0;
        robot_fake[nRobotID-1].bPathTo = false;
        break;
    default:
        ROS_WARN("CWPR_Swarm::RecvNewPackage Unkown CMD = %d",inCmd->command);
        break;
    }
    robot_fake[nRobotID-1].nLastRecvCmd = inCmd->command;
}

void CWPR_Swarm::UpdateAll()
{
    //for(int i=2;i<ROBOT_FAKE_NUM;i++)  //只模拟固定机械臂
    for(int i=0;i<ROBOT_FAKE_NUM;i++)//模拟所有机器人
    {
        if(robot_fake[i].nState == RBT_ST_GOTO)
        {
        //ROS_WARN("RBT_ST_GOTO robot_fake i = %d map (%.2f , %.2f)",i, robot_fake[i].map_x, robot_fake[i].map_y);
            float dx = robot_fake[i].target_x - robot_fake[i].map_x;
            float dy = robot_fake[i].target_y - robot_fake[i].map_y;
            float dyaw = robot_fake[i].target_yaw - robot_fake[i].map_yaw;
            if(fabs(dx) > 0.02 || fabs(dy) > 0.02 || fabs(dyaw) > 0.03)
            {
                if(dx > 0) dx=0.01; else dx = -0.01;
                robot_fake[i].map_x += dx;
                if(dy > 0) dy=0.01; else dy = -0.01;
                robot_fake[i].map_y += dy;
                if(dyaw > 0) dyaw=0.03; else dyaw = -0.03;
                robot_fake[i].map_yaw += dyaw;
                //printf("[Goto] id= %d ( %.2f , %.2f )  %.2f \n",i+1,robot_fake[i].map_x,robot_fake[i].map_y,robot_fake[i].map_yaw);
            }
            else
            {
                robot_fake[i].nState = RBT_ST_ARRIVED;
            }
        }
        if(robot_fake[i].nState >= RBT_ST_BOX_DETECT && robot_fake[i].nState < RBT_ST_BOX_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                if(robot_fake[i].nDevType == DEV_WPR_1)
                    if(robot_fake[i].nState == RBT_ST_BOX_GRAB)
                    {
                        robot_fake[i].joint_pos[0] = 0.1;
                        robot_fake[i].joint_pos[1] = 0.0;
                        robot_fake[i].joint_pos[2] = 0.0;
                    }
                    else
                    {
                        robot_fake[i].joint_pos[0] = 0.0;
                        robot_fake[i].joint_pos[1] = 1.57;
                        robot_fake[i].joint_pos[2] = -0.7;
                    }
                robot_fake[i].nCount = 30*fTimeScale;
            }
        }
        if(robot_fake[i].nState >= RBT_ST_GM_DETECT && robot_fake[i].nState < RBT_ST_GM_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                if(robot_fake[i].nDevType == DEV_WPR_1)
                    if(robot_fake[i].nState == RBT_ST_GM_GRAB)
                    {
                        robot_fake[i].joint_pos[0] = 0.1;
                        robot_fake[i].joint_pos[1] = 0.0;
                        robot_fake[i].joint_pos[2] = 0.0;
                    }
                    else
                    {
                        robot_fake[i].joint_pos[0] = 0.0;
                        robot_fake[i].joint_pos[1] = 1.57;
                        robot_fake[i].joint_pos[2] = -0.7;
                    }
                robot_fake[i].nCount = 30*fTimeScale;
            }
        }
        if(robot_fake[i].nState >= RBT_ST_MOBILE_DETECT && robot_fake[i].nState < RBT_ST_MOBILE_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                if(robot_fake[i].nDevType == DEV_WPR_1)
                    if(robot_fake[i].nState == RBT_ST_MOBILE_PLACE)
                    {
                        robot_fake[i].joint_pos[0] = 0.2;
                        robot_fake[i].joint_pos[1] = 0.0;
                        robot_fake[i].joint_pos[2] = 0.0;
                    }
                    else
                    {
                        robot_fake[i].joint_pos[0] = 0.0;
                        robot_fake[i].joint_pos[1] = 1.57;
                        robot_fake[i].joint_pos[2] = -0.7;
                    }
                robot_fake[i].nCount = 30*fTimeScale;
            } 
        }
        if(robot_fake[i].nState >= RBT_ST_GS_DETECT && robot_fake[i].nState < RBT_ST_GS_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                if(robot_fake[i].nDevType == DEV_WPR_1)
                    if(robot_fake[i].nState == RBT_ST_GS_GRAB)
                    {
                        robot_fake[i].joint_pos[0] = 0.1;
                        robot_fake[i].joint_pos[1] = 0.0;
                        robot_fake[i].joint_pos[2] = 0.0;
                    }
                    else
                    {
                        robot_fake[i].joint_pos[0] = 0.0;
                        robot_fake[i].joint_pos[1] = 1.57;
                        robot_fake[i].joint_pos[2] = -0.7;
                    }
                robot_fake[i].nCount = 30*fTimeScale;
            }
        }
        if(robot_fake[i].nState >= RBT_ST_PS_DETECT && robot_fake[i].nState < RBT_ST_PS_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                if(robot_fake[i].nDevType == DEV_WPR_1)
                    if(robot_fake[i].nState == RBT_ST_PS_PLACE)
                    {
                        robot_fake[i].joint_pos[0] = 0.3;
                        robot_fake[i].joint_pos[1] = 0.0;
                        robot_fake[i].joint_pos[2] = 0.0;
                    }
                    else
                    {
                        robot_fake[i].joint_pos[0] = 0.0;
                        robot_fake[i].joint_pos[1] = 1.57;
                        robot_fake[i].joint_pos[2] = -0.7;
                    }
                robot_fake[i].nCount = 30*fTimeScale;
            } 
        }
        // 抓取小货架
        if(robot_fake[i].nState >= RBT_ST_GP_DETECT && robot_fake[i].nState < RBT_ST_GP_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                if(robot_fake[i].nDevType == DEV_WPB_MANI)
                    if(robot_fake[i].nState == RBT_ST_GP_GRAB)
                    {
                        robot_fake[i].joint_pos[7] = 0.0;
                        robot_fake[i].joint_pos[8] = 0.0;
                    }
                    else
                    {
                        robot_fake[i].joint_pos[7] = -1.4;
                        robot_fake[i].joint_pos[8] = 1.4;
                    }
                robot_fake[i].nCount = 30*fTimeScale;
            }
        }
        // 放置小货架
        if(robot_fake[i].nState >= RBT_ST_PALLET_DETECT && robot_fake[i].nState < RBT_ST_PALLET_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                if(robot_fake[i].nDevType == DEV_WPB_MANI)
                    if(robot_fake[i].nState == RBT_ST_PALLET_PLACE)
                    {
                        robot_fake[i].joint_pos[7] = 0.0;
                        robot_fake[i].joint_pos[8] = 0.0;
                    }
                    else
                    {
                        robot_fake[i].joint_pos[7] = -1.4;
                        robot_fake[i].joint_pos[8] = 1.4;
                    }
                robot_fake[i].nCount = 30*fTimeScale;
            } 
        }
         if(robot_fake[i].nState >= RBT_ST_PLACE_COLOR && robot_fake[i].nState < RBT_ST_PLACE_COLOR_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                if(robot_fake[i].nDevType == DEV_WPR_1)
                    if(robot_fake[i].nState == RBT_ST_PLACE_COLOR)
                    {
                        robot_fake[i].joint_pos[0] = 0.2;
                        robot_fake[i].joint_pos[1] = 0.0;
                        robot_fake[i].joint_pos[2] = 0.0;
                    }
                    else
                    {
                        robot_fake[i].joint_pos[0] = 0.0;
                        robot_fake[i].joint_pos[1] = 1.57;
                        robot_fake[i].joint_pos[2] = -0.7;
                    }
                robot_fake[i].nCount = 30*fTimeScale;
            } 
        }
        if(robot_fake[i].nState >= RBT_ST_DOCK_FACETO && robot_fake[i].nState < RBT_ST_DOCK_DONE)
        {
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                robot_fake[i].nCount = 30*fTimeScale;
            } 
        }

        if(robot_fake[i].nState >= RBT_ST_DOCK_LEAVE && robot_fake[i].nState < RBT_ST_LEAVE_DONE)
        {
            if(robot_fake[i].nState == RBT_ST_DOCK_LEAVE)
            {
                robot_fake[i].map_x += 0.01;
                robot_fake[i].map_y += 0.01;
                robot_fake[i].map_yaw += 0.026;
            }
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                robot_fake[i].nCount = 30*fTimeScale;
            }
        }

        if(robot_fake[i].nState >= RBT_ST_CONVEY_BOX && robot_fake[i].nState < RBT_ST_EXIT_BOX)
        {
            //ROS_WARN("[RBT_ST_CONVEY_BOX] nCount = %d",robot_fake[i].nCount);
            if(robot_fake[i].nCount > 0)
                robot_fake[i].nCount--;
            else
            {
                robot_fake[i].nState ++;
                robot_fake[i].nCount = 30*fTimeScale;
            } 
        }

        if(robot_fake[i].nState == RBT_ST_TELEOP)
        {
                robot_fake[i].map_x += robot_fake[i].vel_x/30;
                robot_fake[i].map_y += robot_fake[i].vel_y/30;
                robot_fake[i].map_yaw += robot_fake[i].vel_angular/30;
        }

        if(robot_fake[i].nState == RBT_ST_FOLLOW_PATH)
        {
            int path_index = robot_fake[i].nPathIndex;
            if(path_index >= robot_fake[i].path_finder.arPathHolder.size())
            {
                robot_fake[i].nState = RBT_ST_FOLLOW_PATH_END;
            }
            else if(robot_fake[i].nPointIndex >= robot_fake[i].path_finder.arPathHolder[path_index].arPoint.size())
            {
                if(robot_fake[i].bPathTo == true)
                {
                    float dx = robot_fake[i].target_x - robot_fake[i].map_x;
                    float dy = robot_fake[i].target_y - robot_fake[i].map_y;
                    if(fabs(dx) > 0.04 || fabs(dy) > 0.04)
                    {
                        if(dx > 0) dx=0.01; else dx = -0.01;
                        robot_fake[i].map_x += dx;
                        if(dy > 0) dy=0.01; else dy = -0.01;
                        robot_fake[i].map_y += dy;
                        robot_fake[i].map_yaw = CalPathAngle(
                            -robot_fake[i].map_x,
                            robot_fake[i].map_y,
                            -robot_fake[i].target_x ,
                            robot_fake[i].target_y
                        );
                    }
                    else
                    {
                        robot_fake[i].map_x = robot_fake[i].target_x;
                        robot_fake[i].map_y = robot_fake[i].target_y;
                        robot_fake[i].map_yaw = robot_fake[i].target_yaw;
                        robot_fake[i].nState = RBT_ST_FOLLOW_PATH_END;
                    }
                }
            }
            else
            {
                int pnt_index =  robot_fake[i].nPointIndex;
                float dx = robot_fake[i].path_finder.arPathHolder[path_index].arPoint[pnt_index].x - robot_fake[i].map_x;
                float dy = robot_fake[i].path_finder.arPathHolder[path_index].arPoint[pnt_index].y - robot_fake[i].map_y;
                 if(fabs(dx) > 0.04 || fabs(dy) > 0.04)
                {
                    if(dx > 0) dx=0.01; else dx = -0.01;
                    robot_fake[i].map_x += dx;
                    if(dy > 0) dy=0.01; else dy = -0.01;
                    robot_fake[i].map_y += dy;
                    robot_fake[i].map_yaw = CalPathAngle(
                        -robot_fake[i].map_x,
                        robot_fake[i].map_y,
                        -robot_fake[i].path_finder.arPathHolder[path_index].arPoint[pnt_index].x ,
                        robot_fake[i].path_finder.arPathHolder[path_index].arPoint[pnt_index].y
                    );
                }
                else
                {
                    pnt_index ++;
                    robot_fake[i].nPointIndex = pnt_index;
                 }
                 //ROS_WARN("CWPR_Swarm pntIndex = %d  size=%d",robot_fake[i].nPointIndex ,(int)robot_fake[i].path_finder.arPathHolder[path_index].arPoint.size());
            }
        }

        arInfoSend[i].info_msg.state = robot_fake[i].nState;
        arInfoSend[i].info_msg.map_x = robot_fake[i].map_x;
        arInfoSend[i].info_msg.map_y = robot_fake[i].map_y;
        arInfoSend[i].info_msg.map_yaw = robot_fake[i].map_yaw;
        arInfoSend[i].info_msg.cmd_recv = robot_fake[i].nLastRecvCmd;
        for(int j=0;j<10;j++)
        {
            arInfoSend[i].info_msg.joint_pos[j] = robot_fake[i].joint_pos[j];
        }
        arInfoSend[i].SendInfo();
    }
}

float CWPR_Swarm::CalPathAngle(float inSrcX, float inSrcY, float inDestX, float inDestY)
{
    float res = 0;
	float dx = inSrcX - inDestX;
	float dy = inSrcY - inDestY;
	if (dx == 0)
	{
		if (dy > 0)
		{
			res = 180 - 90;
		} 
		else
		{
			res = 0 - 90;
		}
	} 
	else
	{
		double fTan = dy / dx;
		res = atan(fTan)*180/Pi;
		if (dx < 0)
		{
			res = res - 180;
		}
	}
	res -= 90;
	if (res < 0)
	{
		res += 360;
	}
	if (res > 360)
	{
		res -= 360;
	}

    res *= Pi/180;
    res = 1.5*Pi -res;
    if(res < 0)
    {
        res += 2*Pi;
    }
	return res;
}