#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander
from copy import deepcopy

class ProbotDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('probot_demo')
        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator_i5')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'pedestal_Link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.05)
       
        # 设置允许的最大速度和加速度
        # arm.set_max_acceleration_scaling_factor(0.1)
        # arm.set_max_velocity_scaling_factor(0.1)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        
        #set end_speed limit
        arm.limit_max_cartesian_link_speed(0.01,end_effector_link)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
               
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        position1_up = [0.014395985471471851, 1.0428055054185135, -0.5741740403015881, -0.07601717937126147, -1.521409073007085, 0.014507437352708059]
        position1_down = [2.8185769893737342, 0.9532350341838829, -0.6602487723719701, 8.378487609608126e-05, -1.5402176019244957, 1.3216234069319301]
        position2_up = [2.8269182843302807, 1.1214448021826973, -0.9352738543259738, -0.44327957782101063, -1.5399460030753567, 1.3299937140136813]
        position2_down = [2.8185769893737342, 0.9532350341838829, -0.6602487723719701, 8.378487609608126e-05, -1.5402176019244957, 1.3216234069319301]
        position3_down = [0.014395985471471851, 1.0428055054185135, -0.5741740403015881, -0.07601717937126147, -1.521409073007085, 0.014507437352708059]

        # 设置机器臂当前的状态作为运动初始状态        
        arm.set_joint_value_target(position1_up)           
        arm.go()
        arm.set_joint_value_target(position1_down)           
        arm.go()
        
        arm.set_joint_value_target(position1_up)           
        arm.go()
        arm.set_joint_value_target(position2_up)           
        arm.go()
        arm.set_joint_value_target(position2_down)           
        arm.go()

        arm.set_joint_value_target(position2_up)           
        arm.go()
        arm.set_joint_value_target(position3_down)           
        arm.go()



        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    ProbotDemo()
    
