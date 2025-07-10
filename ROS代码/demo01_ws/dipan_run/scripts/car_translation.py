#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from time import time

def move(forward,translation,rotate):
    # 初始化 ROS 节点
    rospy.init_node('move_robot', anonymous=True)

    # 创建发布者，向 /cmd_vel 发布 Twist 类型的消息
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 设置发布频率
    rate = rospy.Rate(100)  # 10Hz

    # 创建一个 Twist 消息对象
    car = Twist()

    # 设置线速度（单位：米/秒）
    car.linear.x = forward  # 向前的线速度
    car.linear.y = translation  # 左右方向线速度
    car.linear.z = 0.0  # 上下方向线速度

    # 设置角速度（单位：弧度/秒）
    car.angular.x = 0.0  # 绕 X 轴旋转的角速度
    car.angular.y = 0.0  # 绕 Y 轴旋转的角速度
    car.angular.z = rotate  # 绕 Z 轴旋转的角速度

    start_time = time()

    # 在 2 秒内持续发布消息
    while time() - start_time < 2:
        pub.publish(car)  # 发布消息
        rate.sleep() 


if __name__ == '__main__':
    try:
        move(0,0.2,0)
        move(0,0,0)
    except rospy.ROSInterruptException:
        pass
