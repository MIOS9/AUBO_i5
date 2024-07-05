import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped

def initialize_moveit():
    # 初始化ROS节点
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    
    # 初始化MoveIt!的接口
    moveit_commander.roscpp_initialize(sys.argv)
    
    # 实例化RobotCommander对象
    robot = moveit_commander.RobotCommander()
    
    # 实例化PlanningSceneInterface对象
    scene = moveit_commander.PlanningSceneInterface()
    
    # 实例化MoveGroupCommander对象，控制机械臂的运动
    group_name = "manipulator_i5"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    return robot, scene, move_group

def configure_move_group(move_group, velocity_scaling=0.1, acceleration_scaling=0.1, planning_time=5.0):
    # 设置允许的最大速度和加速度，范围0~1
    move_group.set_max_acceleration_scaling_factor(acceleration_scaling)
    move_group.set_max_velocity_scaling_factor(velocity_scaling)
    
    # 增加规划时间以允许找到更好的路径
    move_group.set_planning_time(planning_time)
    
    # 设置目标位置和姿态的公差
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.01)
    move_group.set_planner_id("RRTConnectkConfigDefault")

def add_box3(scene):
    # 添加box3障碍物并附着到wrist3_link
    box3_pose = PoseStamped()
    box3_pose.header.frame_id = "wrist3_link"
    box3_pose.pose.position.x = -0.48
    box3_pose.pose.position.y = -0.12
    box3_pose.pose.position.z = 0.89
    box3_pose.pose.orientation.w = 1.0
    box3_name = "box3"
    box3_size = (0.02, 0.02, 0.10)
    scene.add_box(box3_name, box3_pose, box3_size)
    scene.attach_box("wrist3_link", box3_name)
    
    rospy.sleep(1)
    print("Box3 added and attached to wrist3_link.")
    rospy.sleep(2)

def add_cylinder(scene):
    # 添加圆柱体障碍物
    cylinder_id = "cylinder"
    cylinder_height = 1.5 
    cylinder_radius = 0.20
    cylinder_pose = geometry_msgs.msg.PoseStamped()
    cylinder_pose.header.frame_id = "world"
    cylinder_pose.pose.position.x = -0.52
    cylinder_pose.pose.position.y = -0.42
    cylinder_pose.pose.position.z = 0.77
    cylinder_pose.pose.orientation.w = 1.0
    scene.add_cylinder(cylinder_id, cylinder_pose, height=cylinder_height, radius=cylinder_radius)

def evaluate_plan(plan):
    return plan.joint_trajectory.points[-1].time_from_start.to_sec()

def execute_joint_goals(move_group, joint_goals, attempts=5):
    for joint_goal in joint_goals:
        best_plan = None
        best_score = float('inf')
        
        for _ in range(attempts):
            move_group.set_joint_value_target(joint_goal)
            plan_success, plan, _, _ = move_group.plan()
            
            if plan_success:
                score = evaluate_plan(plan)
                if score < best_score:
                    best_score = score
                    best_plan = plan
        
        if best_plan is not None:
            execution_success = move_group.execute(best_plan, wait=True)
            
            if not execution_success:
                rospy.logerr(f"Execution failed for joint goal: {joint_goal}")
        else:
            rospy.logerr(f"Planning failed for joint goal: {joint_goal}")
        
        current_joints = move_group.get_current_joint_values()
        rospy.loginfo("当前的关节角：%s", current_joints)
        current_pose = move_group.get_current_pose().pose
        rospy.loginfo("当前的位置: x=%f, y=%f, z=%f", current_pose.position.x, current_pose.position.y, current_pose.position.z)


def clear_environment(scene):
    # 获取场景中所有物体的名称
    object_ids = scene.get_known_object_names()
    
    # 移除所有物体
    for obj_id in object_ids:
        scene.remove_world_object(obj_id)
        
    # 移除所有附着物体
    attached_objects = scene.get_attached_objects().keys()
    for attached_obj in attached_objects:
        scene.remove_attached_object(attached_obj)




def main():
    robot, scene, move_group = initialize_moveit()
    configure_move_group(move_group)
    add_box3(scene)
    
    joint_goal_list1 = [
        [0.014395985471471851, 1.0428055054185135, -0.5741740403015881, -0.07601717937126147, -1.521409073007085, 0.014507437352708059],
        [2.8185769893737342, 0.9532350341838829, -0.6602487723719701, 8.378487609608126e-05, -1.5402176019244957, 1.3216234069319301],
        [2.8269182843302807, 1.1214448021826973, -0.9352738543259738, -0.44327957782101063, -1.5399460030753567, 1.3299937140136813],
        [2.8185769893737342, 0.9532350341838829, -0.6602487723719701, 8.378487609608126e-05, -1.5402176019244957, 1.3216234069319301],
        [1.2857596278954482, 0.9532954646151091, -0.660390030210223, 0.00017980303664973031, -1.5400688473168425, 1.3215260635091317],
        [0.014395985471471851, 1.0428055054185135, -0.5741740403015881, -0.07601717937126147, -1.521409073007085, 0.014507437352708059]
    ]
    
    joint_goal_list2 = [
        [0.014395985471471851, 1.0428055054185135, -0.5741740403015881, -0.07601717937126147, -1.521409073007085, 0.014507437352708059],
        [2.8185769893737342, 0.9532350341838829, -0.6602487723719701, 8.378487609608126e-05, -1.5402176019244957, 1.3216234069319301],
        [2.8269182843302807, 1.1214448021826973, -0.9352738543259738, -0.44327957782101063, -1.5399460030753567, 1.3299937140136813],
        [2.8185769893737342, 0.9532350341838829, -0.6602487723719701, 8.378487609608126e-05, -1.5402176019244957, 1.3216234069319301],
        [0.014395985471471851, 1.0428055054185135, -0.5741740403015881, -0.07601717937126147, -1.521409073007085, 0.014507437352708059]
    ]

    # 清空场景
    clear_environment(scene)

    # 执行前两次运动，使用joint_goal_list2，速度和加速度为1
    configure_move_group(move_group, velocity_scaling=0.1, acceleration_scaling=0.1)
    execute_joint_goals(move_group, joint_goal_list2)
    # execute_joint_goals(move_group, joint_goal_list2)

   
    # # 第三次运动，使用joint_goal_list2，速度和加速度为0.3
    # configure_move_group(move_group, velocity_scaling=0.3, acceleration_scaling=0.3)
    # execute_joint_goals(move_group, joint_goal_list2)
    
    # # 第四次运动，使用joint_goal_list1，速度为0.3，并添加圆柱体障碍物
    # add_cylinder(scene)
    # execute_joint_goals(move_group, joint_goal_list1)

    # 清空场景
    clear_environment(scene)
    
    # 关闭并退出MoveIt!
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()