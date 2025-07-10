#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from time import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
# import ctypes
# libgcc_s = ctypes.CDLL('libgcc_s.so.1')

pos1 = (1.308630,-0.010015,0.999913)
pos2 = (3.502738,-0.046094,0.999990)
pos3 = (3.491757,-0.021587,0.746807)
pos3 = (3.583249,0.547428,0.757746)
pos4 = (3.659897,1.048093,0.762070)
pos5 = (3.670109,1.054352,0.094982)
pos6 = (2.138723,1.403021,0.125596)
pos7 = (2.102930,1.434407,0.793783)
pos8 = (2.456965,3.538489,0.756018)
pos9 = (2.474586,3.535513,0.189593)
pos10 = (0,0,0)
pos11 = (0,0,0)
pos12 = (0,0,0)
pos13 = (0,0,0)
pos14 = (0,0,0)
pos15 = (0,0,0)
pos16 = (0,0,0)
pos17 = (0,0,0)
pos18 = (0,0,0)
pos19 = (0,0,0)
pos20 = (0,0,0)
pos21 = (0,0,0)
pos22 = (0,0,0)
pos23 = (0,0,0)
pos24 = (0,0,0)
pos25 = (0,0,0)
pos26 = (0,0,0)
pos27 = (0,0,0)
pos28 = (0,0,0)


pos29 = (3.659897,1.048093,0.762070)
# 创建CvBridge实例
bridge = CvBridge()

# 发布器，用于发布新图像消息
image_pub = None

# 用于获取小车当前位置
current_position = None

def __init__():
     # 初始化 ROS 节点
    rospy.init_node('nav_publisher', anonymous=True)

def nav_publisher(pos, frame_id="map"):


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
    goal.target_pose.pose.position.x = pos[0]
    goal.target_pose.pose.position.y = pos[1]
    goal.target_pose.pose.position.z = 0
    
    # 设置目标方向 (这里使用四元数表示方向)
    goal.target_pose.pose.orientation.x = pos[2]
    goal.target_pose.pose.orientation.y = pos[3]
    goal.target_pose.pose.orientation.z = pos[4]
    goal.target_pose.pose.orientation.w = pos[5]

    # 发送目标到move_base
    rospy.loginfo("Sending goal: [x: {}, y: {}, w: {}]".format(pos[0], pos[1], pos[2]))
    client.send_goal(goal)

    # 等待结果
    client.wait_for_result()

    # 打印结果
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.loginfo("Failed to reach the goal.")
 

def chatter_callback1(msg):
    if(msg != "2.0"):   #识别到了绿灯
        rospy.loginfo("发现绿灯")
        #nav_publisher(pos2)


def chatter_callback2(msg):
    while(msg == "2.0"):#识别到了绿灯
            nav_publisher(pos13)

def wait_for_green1():
    # 创建订阅方，订阅名为 "chatter" 的话题，回调函数为 chatter_callback
    rospy.Subscriber("/yolov5/results", String, chatter_callback1)


def wait_for_green2():
    # 创建订阅方，订阅名为 "chatter" 的话题，回调函数为 chatter_callback
    rospy.Subscriber("/yolov5/results", String, chatter_callback2)

def tts_chat(chat):
    pub = rospy.Publisher('tts_text', String, queue_size=10)
    rospy.sleep(2)
    msg = String()
    msg.data = chat
    pub.publish(msg)    

def stop_forward():
    img_path = '/home/mowen/demo01_ws/photo/lupai.jpg'
    # 使用 OpenCV 读取图片
    #img = cv2.imread(img_path)

    # 显示图片
    #cv2.imshow('Image', img)
    tts_chat("前方是禁止直行指示牌")

def get_photo():
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    
    # 创建发布器，发布到 "/new_image_raw" 话题
    global image_pub    
    image_pub = rospy.Publisher("/new_image_raw", Image, queue_size=1)

def photo_pub():
    print("hello")

def image_callback(msg):
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 这里可以进行图像处理操作（如果需要）

        # 将处理后的OpenCV图像转换回ROS图像消息
        new_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")

        rospy.sleep(1)
        # 发布新图像消息到 "/new_image_raw" 话题
        image_pub.publish(new_msg)
    
    except Exception as e:
        rospy.logerr("Error in image callback: %s", e)


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

def img_stitch():
    print("hello")

def person_count():
    print("hello")

def e_bike_count():
    print("hello")

def move(forward,translation,rotate,time_run):
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
    while time() - start_time < time_run:
        pub.publish(car)  # 发布消息
        rate.sleep() 

if __name__ == '__main__': 
    try:  
        __init__()
        tts_chat("前方禁止通行")
        #stop_forward() #禁止通行
        #nav_publisher(pos1)
        # wait_for_green1()
        # rospy.spin()

        
        
        # nav_publisher(pos1)
        # nav_publisher(pos2)
        # nav_publisher(pos3)
        # nav_publisher(pos4)
        # nav_publisher(pos5)
        # nav_publisher(pos6)
        # nav_publisher(pos7)
        # nav_publisher(pos8)
        # nav_publisher(pos9)

    #     nav_publisher(pos1)
    #     wait_for_green1()   #等待第一个绿灯 pos2
    #     nav_publisher(pos3)
    #     nav_publisher(pos4)
    #     stop_forward() #禁止通行

    #     nav_publisher(pos5)
    #     get_photo()
    #     target_distance = 0.1 # 设定目标距离为 5 米
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     img_stitch()           #A区

    #     nav_publisher(pos6)
    #     get_photo()
    #     target_distance = 0.1 # 设定目标距离为 5 米
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     img_stitch()          #B区
    #     person_count()

    #     nav_publisher(pos7)
    #     nav_publisher(pos8)
    #     nav_publisher(pos9)
    #     nav_publisher(pos10)
    #     nav_publisher(pos11) #到达第一栋

    #     get_photo()
    #     photo_pub()          #识别第一栋

    #     nav_publisher(pos12)
    #     wait_for_green2()    #等待第二个绿灯 (pos13)

    #     nav_publisher(pos14)

    #     get_photo()
    #     photo_pub()          #识别第三栋

    #     nav_publisher(pos15)

    #     get_photo()
    #     photo_pub()          #识别垃圾桶

    #     nav_publisher(pos16)
    #     nav_publisher(pos17)
    #     nav_publisher(pos18)
    #     nav_publisher(pos19) #到达车牌

    #     get_photo()
    #     photo_pub()          #识别车牌

    #     target_distance = 0.1 # 设定目标距离为 5 米
    #     move_translation(target_distance)
    #     get_photo()
    #     photo_pub()          #识别车牌

    #     move_translation(target_distance)
    #     get_photo()
    #     photo_pub()          #识别车牌

    #     nav_publisher(pos20)
    #     nav_publisher(pos21)
    #     nav_publisher(pos22) #到达第二栋

    #     get_photo()
    #     photo_pub()          #识别第二栋

    #     nav_publisher(pos23)
    #     nav_publisher(pos24)
    #     nav_publisher(pos25) #电动车位置

    #     target_distance = 0.1 # 设定目标距离为 5 米
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     get_photo()
    #     move_translation(target_distance)
    #     img_stitch()          #电动车

    #     e_bike_count()

    #     nav_publisher(pos26)
    #     nav_publisher(pos27)
    #     nav_publisher(pos28)

    #     move(-0.2,0,0,2)
    except rospy.ROSInterruptException:
        pass   
