#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

from darkflow.net.build import TFNet
import imp
import os

DF_PATH = os.path.dirname(imp.find_module('darkflow')[1])

class Processor():
    def __init__(self):
        self.sub = rospy.Subscriber("/front_camera/camera/image_color", Image, self.callback)
        self.bridge = CvBridge()
        # options = {"model": os.path.join(DF_PATH, "cfg/yolo-buoy.cfg"), "load": 16000, "threshold": 0.3}
        options = {"model": "cfg/yolo-buoy.cfg", "load": 11000, "threshold": 0.3}
        self.tfnet = TFNet(options)

        self.tfpub = rospy.Publisher("/front_camera/processed/yolo", Image, queue_size=1)
        # self.threshpub = rospy.Publisher("/front_camera/processed/threshold", Image, queue_size=1)

    def callback(self, data):
        imgcv = self.image_ros_to_cv2(data)
        self.tfpub.publish(tfnet.return_predict(imgcv))

        # l, a, b = cv2.split(cv2.cvtColor(imgcv,cv2.COLOR_BGR2LAB))
        # mask = detect.get_salient(a)
        # mask = cv2.threshold(mask, thresh_limit, 255, cv2.THRESH_BINARY)[1]
        # mask = cv2.dilate(mask,None,iterations = 4)
        # mask = cv2.erode(mask,None,iterations = 4)
        # _, ctr,hierachy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        # self.

    def image_ros_to_cv2(self, img):
        try:
            frame = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return frame        

if __name__ == '__main__':

    cur_dir = os.getcwd()
    os.chdir(DF_PATH)
    print(os.getcwd())
    rospy.init_node('ml_vision')
    processor = Processor()
    rospy.spin()