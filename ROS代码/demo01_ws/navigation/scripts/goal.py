#! /usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped


def send_goal(x, y, z, frame_id="map"):
    # 初始化节点
    rospy.init_node('send_goal_node', anonymous=True)

    # 创建一个动作客户端，与move_base节点连接
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    # 等待move_base服务器启动
    rospy.loginfo("Waiting for move_base action server to start...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # 创建目标点
    goal = MoveBaseGoal()
    
    # 设置目标坐标
    goal.target_pose.header.frame_id = frame_id  # 参考坐标系
    goal.target_pose.header.stamp = rospy.Time.now()  # 当前时间
    
    # 设置目标位置
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    
    # 设置目标方向 (这里使用四元数表示方向)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    # 发送目标到move_base
    rospy.loginfo("Sending goal: [x: {}, y: {}, z: {}]".format(x, y, z))
    client.send_goal(goal)

    # 等待结果
    client.wait_for_result()

    # 打印结果
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.loginfo("Failed to reach the goal.")

if __name__ == "__main__":
    try:
        send_goal(-2.9,0.09,0)
        send_goal(-2.95,-0.99,0)
        send_goal(-1.32,-1.08,0)
        send_goal(-1.05,-3.43,0)
        send_goal(0.14,-0.7,0)
        send_goal(0.06,0.07,0)
    except rospy.ROSInterruptException:
        pass