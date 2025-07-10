#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
    # 提取位姿信息
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y 
    
    s_x = msg.pose.pose.orientation.x
    s_y = msg.pose.pose.orientation.y
    s_z = msg.pose.pose.orientation.z
    s_w = msg.pose.pose.orientation.w  # 四元数的w部分，表示朝向

    # 打印 (x, y, w) 数据
    rospy.loginfo("(%f,%f,%f,%f)", x, y, s_x, s_y, s_z, s_w)


def listener():
    # 初始化 ROS 节点
    rospy.init_node('amcl_pose_listener', anonymous=True)
    
    # 订阅 /amcl_pose 主题
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, callback)
    
    # 设置频率为 2Hz (每2秒执行一次)
    rate = rospy.Rate(0.5)  # 0.5 Hz 即每2秒一次
    
    while not rospy.is_shutdown():
        # 这里可以添加一些处理代码，或者仅仅是空循环
        rate.sleep()  # 控制循环频率为2秒

if __name__ == '__main__':
    listener()
