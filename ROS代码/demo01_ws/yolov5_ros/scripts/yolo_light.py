#!/usr/bin/env python3
import rospy
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

class YOLOv5Node:
    def __init__(self):
        rospy.init_node('yolov5_node', anonymous=True)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/mowen/demo01_ws/model/light.pt')
        # 加载YOLOv5模型
        #self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # 加0载预训练的yolov5s模型
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        # 发布检测结果
        self.result_pub = rospy.Publisher('/yolov5/results', String, queue_size=10)
        
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            # 使用YOLOv5模型进行目标检测
            results = self.model(cv_image)
            
            # 可视化检测结果
            #results.show()  # 显示带有框选目标的图片
            
            # 获取检测框的结果
            detections = results.xywh[0]
            for detection in detections:
                x_center, y_center, width, height, confidence, class_id = detection.tolist()
                print(f"Class ID: {class_id}, Confidence: {confidence}, Box: ({x_center}, {y_center}, {width}, {height})")
                rospy.loginfo(class_id)
                self.result_pub.publish(str(class_id))
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == '__main__':
    try:
        yolov5_node = YOLOv5Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass