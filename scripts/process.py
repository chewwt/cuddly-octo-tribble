#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

from net import Net
# from darkflow.net.build import TFNet
# import imp
# import os

# DF_PATH = os.path.dirname(imp.find_module('darkflow')[1])

class Processor():
    def __init__(self):
        self.bridge = CvBridge()
        # options = {"model": os.path.join(DF_PATH, "cfg/yolo-buoy.cfg"), "load": 16000, "threshold": 0.3}
        # options = {"model": "cfg/yolo-buoy.cfg", "load": 11000, "threshold": 0.3}
        # self.tfnet = TFNet(options)
        self.net = Net()

        # while True:
        #     try:
        #         print(self.net)
        #         break
        #     except AttributeError:
        #         pass

        self.tfpub = rospy.Publisher("/front_camera/processed/yolo/compressed", CompressedImage, queue_size=1)
        self.sub = rospy.Subscriber("/front_camera/camera/image_color", Image, self.callback)
        
        # self.threshpub = rospy.Publisher("/front_camera/processed/threshold", Image, queue_size=1)

    def callback(self, data):
        # print(data)
        imgcv = self.image_ros_to_cv2(data)
        labels = self.net.process(imgcv)

        # print(len(labels))
        # print(labels[0])
        # print(type(labels[0]))

        for obj in labels:
            if obj['label'] == 'yellowbuoy':
                color = (0, 255, 255)
            elif obj['label'] == 'greenbuoy':
                color = (0, 255, 0)
            else:
                color = (0, 0, 255)

            cv2.rectangle(imgcv, (obj['topleft']['x'], obj['topleft']['y']), (obj['bottomright']['x'], obj['bottomright']['y']), color, 2)
            cv2.putText(imgcv, obj['label'][0:-4], (obj['topleft']['x'], obj['topleft']['y'] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4 , color, 1, cv2.LINE_AA)            
            cv2.putText(imgcv, str(obj['confidence']), (obj['topleft']['x'], obj['topleft']['y'] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)            

        # cv2.imshow('test',imgcv)
        # cv2.waitKey(0)
        # print(imgcv)
        # print(type(imgcv))
        # print(type(imgcv[0][0][0]))
        # self.tfpub.publish(self.cv2_to_image_ros(np.uint8(imgcv)))
        self.tfpub.publish(writeCompressed(imgcv))
        # self.tfpub.publish(tfnet.return_predict(imgcv))

        # l, a, b = cv2.split(cv2.cvtColor(imgcv,cv2.COLOR_BGR2LAB))
        # mask = detect.get_salient(a)
        # mask = cv2.threshold(mask, thresh_limit, 255, cv2.THRESH_BINARY)[1]
        # mask = cv2.dilate(mask,None,iterations = 4)
        # mask = cv2.erode(mask,None,iterations = 4)
        # _, ctr,hierachy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    def image_ros_to_cv2(self, img):
        try:
            frame = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return frame 

    def cv2_to_image_ros(self, img):
        try:
            frame = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return frame     

def writeCompressed(img):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpeg', img)[1]).tostring()
    return msg

if __name__ == '__main__':

    # cur_dir = os.getcwd()
    # os.chdir(DF_PATH)
    # print(os.getcwd())
    rospy.init_node('ml_vision')
    processor = Processor()
    rospy.spin()