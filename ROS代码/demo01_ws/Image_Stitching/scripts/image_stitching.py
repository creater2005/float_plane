#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np






class ImageStitching:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('image_stitching_node', anonymous=True)

        # 创建CvBridge实例，用于ROS图像消息和OpenCV图像之间的转换
        self.bridge = CvBridge()

        # 订阅原始图像话题
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)

        # 发布拼接后的图像
        self.image_pub = rospy.Publisher("/stiched_image", Image, queue_size=10)

        # 用于存储图像
        self.images = []
    
    def preprocess_image(self, image):
        # 去噪处理，使用高斯模糊
        image = cv2.GaussianBlur(image, (5, 5), 0)
        # 增强对比度
        image = cv2.convertScaleAbs(image, alpha=1.5, beta=0)
        return image

    def image_callback(self, msg):
        try:
            

            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = self.preprocess_image(cv_image)
            cv_image = cv2.convertScaleAbs(cv_image)
            # 存储接收到的图像
            self.images.append(cv_image)

            # 如果已经有多张图像，可以进行拼接
            if len(self.images) >= 3:
                self.stitch_images()

        except Exception as e:
            rospy.logerr("Error in converting image: %s", str(e))

    def stitch_images(self):
        # 检查是否有足够的图像进行拼接
        if len(self.images) < 3:
            rospy.logwarn("Not enough images to stitch.")
            return

        # 使用OpenCV的Stitcher进行拼接
        stitcher = cv2.Stitcher_create()
        status, stitched_image = stitcher.stitch(self.images)
        if status == cv2.Stitcher_OK:
            rospy.loginfo("Image stitching successful.")
            # 将拼接后的图像发布到话题
            output_path = '/home/mowen/demo01_ws/photo/2.png'
            cv2.imwrite(output_path,stitched_image,[cv2.IMWRITE_PNG_COMPRESSION, 9])
            self.publish_stitched_image(stitched_image)
        else:
            rospy.logwarn("Image stitching failed with error code %d", status)

    def publish_stitched_image(self, cv_image):
        try:
            # 将拼接后的OpenCV图像转换为ROS消息
            msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            # 发布拼接后的图像
            self.image_pub.publish(msg)
        except Exception as e:
            rospy.logerr("Error in publishing stitched image: %s", str(e))

if __name__ == '__main__':
    try:
        # 创建图像拼接对象并运行ROS节点
        image_stitching = ImageStitching()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




# import rospy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# class ImageStitching:
#     def __init__(self):
#         rospy.init_node('image_stitching_node', anonymous=True)
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
#         self.image_pub = rospy.Publisher("/stiched_image", Image, queue_size=10)
#         self.images = []

#     def preprocess_image(self, image):
#         # 图像预处理：去噪和增强对比度
#         image = cv2.GaussianBlur(image, (5, 5), 0)  # 去噪
#         image = cv2.convertScaleAbs(image, alpha=1.5, beta=0)  # 增强对比度
#         return image

#     def image_callback(self, msg):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             cv_image = self.preprocess_image(cv_image)  # 预处理图像
#             self.images.append(cv_image)

#             if len(self.images) >= 2:
#                 self.stitch_images()

#         except Exception as e:
#             rospy.logerr("Error in converting image: %s", str(e))

#     def stitch_images(self):
#         if len(self.images) < 2:
#             rospy.logwarn("Not enough images to stitch.")
#             return

#         stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)  # 使用全景拼接模式
#         stitcher.setPanoConfidenceThresh(0.8)  # 提高置信度阈值
#         status, stitched_image = stitcher.stitch(self.images)

#         if status == cv2.Stitcher_OK:
#             rospy.loginfo("Image stitching successful.")
#             output_path = '/home/mowen/demo01_ws/photo/2.jpg'
#             cv2.imwrite(output_path, stitched_image, [int(cv2.IMWRITE_JPEG_QUALITY), 95])  # 保持高质量
#             self.publish_stitched_image(stitched_image)
#         else:
#             rospy.logwarn("Image stitching failed with error code %d", status)

#     def publish_stitched_image(self, cv_image):
#         try:
#             msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
#             self.image_pub.publish(msg)
#         except Exception as e:
#             rospy.logerr("Error in publishing stitched image: %s", str(e))

# if __name__ == '__main__':
#     try:
#         image_stitching = ImageStitching()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
