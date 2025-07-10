#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# 用于获取小车当前位置
current_position = None

# 里程计回调函数，更新当前位置
def odom_callback(msg):
    global current_position
    # 从消息中获取当前的小车位置
    current_position = msg.pose.pose.position

# 计算两点之间的欧几里得距离
def calculate_distance(start, end):
    return math.sqrt((end.x - start.x) ** 2 + (end.y - start.y) ** 2)

def move_translation(target_distance):
    # 初始化 ROS 节点
    rospy.init_node('move_forward_node')

    # 发布器发布控制命令到 cmd_vel
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 订阅里程计话题
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # 确保初始位置已知
    while current_position is None:
        rospy.sleep(0.1)

    # 获取当前的起始位置
    start_position = current_position

    # 创建控制消息
    move_cmd = Twist()
    move_cmd.linear.y = 0.2  # 设置前进的线速度，单位 m/s

    # 启动一个计时器，持续前进直到达到目标距离
    rate = rospy.Rate(10)  # 控制循环频率为 10Hz

    while not rospy.is_shutdown():
        # 发布控制命令让小车继续前进
        cmd_vel_pub.publish(move_cmd)

        # 获取当前的位置
        if current_position is not None:
            distance_travelled = calculate_distance(start_position, current_position)
            rospy.loginfo(f"Distance travelled: {distance_travelled:.2f} meters")

            # 检查是否已经达到目标距离
            if distance_travelled >= target_distance:
                rospy.loginfo(f"Target distance {target_distance} meters reached!")
                break

        rate.sleep()

    # 停止小车
    stop_cmd = Twist()
    cmd_vel_pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        target_distance = 0.1 # 设定目标距离为 5 米
        move_translation(target_distance)
    except rospy.ROSInterruptException:
        pass
