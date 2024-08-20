#!/usr/bin/python

import rospy
import numpy as np
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class getBoundBoxObjectDist():
    def __init__(self):
        self.x_mid = 0
        self.y_mid = 0
        rospy.Subscriber('/yolov5_ros/detect_out', String, callback=self.callback, queue_size=1)
        rospy.Subscriber('/camera/depth/image_raw', Image, callback=self.depthCallback, queue_size=1)

    def callback(self, data):
        raw_data = data.data  # data type -> a string of items separated by whitespace
        # print('raw data: {}'.format(raw_data))

        # raw_data = str(raw_data[0])  # convert to string vector
        # prob, xmin, ymin, xmax, ymax, id, cls = str(obj).split('\n')  # split each item into individual variable
        # x, y, w, h = raw_data.split(' ')
        ### Data cleaning
        # prob = float(prob.split(' ')[1])
        # xmin = float(xmin.split(' ')[1])
        # ymin = float(ymin.split(' ')[1])
        # xmax = float(xmax.split(' ')[1])
        # ymax = float(ymax.split(' ')[1])
        # cls = cls.split(' ')[1].replace('"', '')

        xmin, ymin, xmax, ymax = raw_data.split(' ')
        self.x_min = int(xmin)
        self.y_min = int(ymin)
        self.x_max = int(xmax)
        self.y_max = int(ymax)
        self.x_mid = ((self.x_max - self.x_min) // 2) + self.x_min
        self.y_mid = ((self.y_max - self.y_min) // 2) + self.y_min




    def depthCallback(self, msg_depth):
        bridge = CvBridge()
        depthImg = bridge.imgmsg_to_cv2(img_msg=msg_depth, desired_encoding='passthrough')
        depthImg = np.array(depthImg)
        # Searh 10px left & right from centroid
        x_left = self.x_mid - 5
        y_left = self.y_mid - 5
        z = np.mean(depthImg[y_left:y_left + 10, x_left:x_left + 10])
        if (~np.isnan(z) and z > 0):
            print('Distance to object: {}cm, midpoint: ({}, {})'.format(int(z/10), self.x_mid, self.y_mid))

    





if __name__ == '__main__':
    rospy.init_node('find_mates', anonymous=True)
    getBoundBoxObjectDist()
    rospy.spin()