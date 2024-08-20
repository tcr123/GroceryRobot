#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import json
import math

class getBoundingBoxDistance():
    def __init__(self):
        self.results = None
        self.depth_image = None
        self.bridge = CvBridge()
        self.is_centered = False
        self.reached_person = False
        rospy.init_node('get_distance')
        self.cmd_vel_Pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1, buff_size=10000000)
        rospy.sleep(1)
        self.yolo_sub = rospy.Subscriber('/detection_results', String, self.bounding_box)
        rospy.sleep(1)

    def depth_callback(self, msg):
        self.depth_image = CvBridge().imgmsg_to_cv2(msg, "passthrough")

    def bounding_box(self, detected_msg):

        detected_msg = json.loads(detected_msg.data)
        print(detected_msg)
        self.results = detected_msg
        self.cmd_vel = Twist()

        xmin = self.results['xmin']
        xmax = self.results['xmax']
        ymin = self.results['ymin']
        ymax = self.results['ymax']

        person_detected_depth = self.depth_image[xmin:xmax, ymin:ymax]
        median_distance = np.median(person_detected_depth) / 1000

        center_x = 640 // 2
        center_y = 480 // 2
        self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z = 0.0, 0.0, 0.0
        self.cmd_vel.angular.x, self.cmd_vel.angular.y, self.cmd_vel.angular.z = 0.0, 0.0, 0.0

        if np.isnan(median_distance) or median_distance == 0.0:
            print("hi")

        
        if ( center_x - 10 < (xmax + xmin) // 2 < center_x + 10 ) and  ( center_y - 10 < (ymax + ymin) // 2 < center_y + 10 ):
            rospy.loginfo('Person at center!')
            self.is_centered = True

        elif not self.is_centered:
            diff_from_center_x = center_x - (xmax + xmin) // 2
            rospy.loginfo(diff_from_center_x)
            if diff_from_center_x > 0:
                if diff_from_center_x < 20:
                    print("center")
                    self.is_centered = True
                elif diff_from_center_x > 50:
                    self.cmd_vel.angular.z=0.2
                elif diff_from_center_x > 160:
                    self.cmd_vel.angular.z=0.3
                elif diff_from_center_x > 250:
                    self.cmd_vel.angular.z= 0.5
                else:
                    self.cmd_vel.angular.z=-((abs(diff_from_center_x)-20)/320)
            else:
                if abs(diff_from_center_x) < 20:
                    print("center")
                    self.is_centered = True
                elif abs(diff_from_center_x) > 50:
                    self.cmd_vel.angular.z = -0.2
                elif abs(diff_from_center_x) > 160:
                    self.cmd_vel.angular.z= -0.3
                elif abs(diff_from_center_x) > 250:
                    self.cmd_vel.angular.z=-0.5 
                else:
                    self.cmd_vel.angular.z=(abs(diff_from_center_x)-20)/320


            self.cmd_vel_Pub.publish(self.cmd_vel)

                                    
        rospy.loginfo('Item is %.2f m away.' % (median_distance))
        if self.is_centered and not self.reached_person:
            if median_distance > 1.0:
                self.cmd_vel.linear.x = 0.2
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_Pub.publish(self.cmd_vel)
            
            elif median_distance > 0.7:
                self.cmd_vel.linear.x = 0.1
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_Pub.publish(self.cmd_vel)

            elif median_distance > 0.6:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_Pub.publish(self.cmd_vel)
                self.reached_person = True

    

if __name__ == '__main__':
    try:
        getBoundingBoxDistance = getBoundingBoxDistance()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Error with demo.py")