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
#include "ArmAction.h"

CArmAction::CArmAction()
{
   
}

CArmAction::~CArmAction()
{
}

 // 第一套动作
void CArmAction::Init_1()
{
    AddKeyframe(0 ,0 ,0 ,0 ,0 ,0 ,0);
    AddKeyframe(-1.57 ,0.9 ,0.9 ,0 ,0 ,1.57 ,1.0);
    AddKeyframe(-1.57 ,1.8 ,1.8 ,0 ,0 ,1.57 ,0);
    AddKeyframe(-1.57 ,0.9 ,0.9 ,0 ,0 ,1.57 ,0);
    AddKeyframe(0 ,0 ,0 ,0 ,0 ,0 ,0);
    AddKeyframe(1.57 ,0.9 ,0.9 ,0 ,0 ,1.57 ,.0);
    AddKeyframe(1.57 ,1.8 ,1.8 ,0 ,0 ,1.57 ,0);
    AddKeyframe(1.57 ,0.9 ,0.9 ,0 ,0 ,1.57 ,1.0);
    AddKeyframe(0 ,0 ,0 ,0 ,0 ,0 ,0);
}

 // 第二套动作
void CArmAction::Init_2()
{
    AddKeyframe(0 ,0 ,0 ,0 ,0 ,0 ,0);
    AddKeyframe(1.57 ,0.9 ,0.9 ,0 ,0 ,1.57 ,1.0);
    AddKeyframe(1.57 ,1.8 ,1.8 ,0 ,0 ,1.57 ,0);
    AddKeyframe(1.57 ,0.9 ,0.9 ,0 ,0 ,1.57 ,0);
    AddKeyframe(0 ,0 ,0 ,0 ,0 ,0 ,0);
}

void CArmAction::AddKeyframe(float inJoint1 , float inJoint2 , float inJoint3 , float inJoint4 , float inJoint5 , float inJoint6 , float inJoint7 )
{
    if(action.size() == 0)
    {
        // 第一帧动作就直接添加
        stArmJoints arm;
        arm.joint_pos[0] = inJoint1;
        arm.joint_pos[1] = inJoint2;
        arm.joint_pos[2] = inJoint3;
        arm.joint_pos[3] = inJoint4;
        arm.joint_pos[4] = inJoint5;
        arm.joint_pos[5] = inJoint6;
        arm.joint_pos[6] = inJoint7;
        action.push_back(arm);
    }
    else
    {
        // 在上一帧动作到当前新动作之间，插入过渡帧
        int nActionSize = action.size();
        stArmJoints actLast = action[nActionSize-1];
        stArmJoints actCur;
        actCur.joint_pos[0] = inJoint1;
        actCur.joint_pos[1] = inJoint2;
        actCur.joint_pos[2] = inJoint3;
        actCur.joint_pos[3] = inJoint4;
        actCur.joint_pos[4] = inJoint5;
        actCur.joint_pos[5] = inJoint6;
        actCur.joint_pos[6] = inJoint7;

        int step = 100;
        stArmJoints act_step ;
        for(int i=0;i<7;i++)
        {
            act_step.joint_pos[i] = (actCur.joint_pos[i] - actLast.joint_pos[i]) / step;
        }
        for(int j=0;j<step;j++)
        {
             for(int i=0;i<7;i++)
             {
                    actLast.joint_pos[i] += act_step.joint_pos[i];
             }
            action.push_back(actLast);
        }
    }
}
