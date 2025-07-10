#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math

current_yaw = 0.0

def quaternion_to_euler(w, x, y, z):

    # 计算 yaw (Z 轴旋转)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # 返回 roll, pitch, yaw（单位：弧度）
    return yaw


# 回调函数，处理IMU数据并转换为欧拉角
def imu_callback(data):
    global current_yaw 
    orientation = data.orientation
    yaw = quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
    current_yaw = yaw

def rotate_to_target(target_yaw, tolerance=0.1, angular_speed=0.2):
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # 确定旋转方向
    angular_direction = 1 if target_yaw > current_yaw else -1

    # 持续旋转直到达到目标角度
    while abs(current_yaw - target_yaw) > tolerance:
        cmd_vel = Twist()
        cmd_vel.angular.z = angular_direction * angular_speed
        pub.publish(cmd_vel)
        rospy.sleep(0.1)

    # 停止旋转
    cmd_vel.angular.z = 0.0
    pub.publish(cmd_vel)
    rospy.loginfo("目标角度已到达，停止旋转")

def main():
    rospy.init_node('rotate_robot')

    # 订阅IMU话题
    rospy.Subscriber('/wit/imu', Imu, imu_callback)
    
    # 等待IMU数据
    rospy.sleep(1)

    # 设置目标角度并执行旋转
    target_yaw = math.pi / 2  # 90度
    rotate_to_target(target_yaw)

if __name__ == '__main__':
    main()
