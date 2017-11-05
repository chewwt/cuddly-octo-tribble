#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image

from net import Net

class Processor():
    def __init__(self):
        self.bridge = CvBridge()
        self.net = Net()
        self.tfpub = rospy.Publisher("/front_camera/processed/yolo/compressed", CompressedImage, queue_size=1)
        self.sub = rospy.Subscriber("/front_camera/camera/image_color", Image, self.callback)       

    def callback(self, data):
        imgcv = self.image_ros_to_cv2(data)
        labels = self.net.process(imgcv)

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

       self.tfpub.publish(writeCompressed(imgcv))

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

    rospy.init_node('ml_vision')
    processor = Processor()
    rospy.spin()