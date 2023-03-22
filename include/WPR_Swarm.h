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
#ifndef _WPR_SWARM_H
#define _WPR_SWARM_H

#include "ServerCmdRecv.h"
#include "StringHelper.h"
#include "RobotInfoSend.h"
#include "PathHolder.h"
#include "PathFinder.h"
#include "WaypointHolder.h"

#define ROBOT_FAKE_NUM 3

typedef struct stRobotFake
{
    int nDevType;
    int nState;
    int nLastRecvCmd;
    float map_x;
    float map_y;
    float map_yaw;
    float target_x;
    float target_y;
    float target_yaw;
    bool bPathTo;
    float vel_x;
    float vel_y;
    float vel_angular;
    float joint_pos[12];
    CPathFinder path_finder;
    int nPathIndex;
    int nLastPathIndex;
    int nPointIndex;
    int nCount;
}stRobotFake;

class CWPR_Swarm : public CServerCmdRecv
{
public:
    CWPR_Swarm();
    virtual ~CWPR_Swarm();
    void Initial();
    void RecvNewPackage(stCommandMsg* inCmd);
    void UpdateAll();
    float CalPathAngle(float inSrcX, float inSrcY, float inDestX, float inDestY);
    CStringHelper str_helper;
    stRobotFake* robot_fake;
    CRobotInfoSend* arInfoSend;
    float fTimeScale;
};
#endif
