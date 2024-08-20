#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from robot_vision_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge
import numpy as np

class getBoundingBoxDistance():
    def __init__(self):
        self.results = None
        self.depth_image = None
        self.bridge = CvBridge()
        self.is_centered = False
        self.reached_person = False
        self.cmd_vel_Pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.depth_sub = rospy.Subscriber('camera/depth/image_raw', Image, callback=self.get_depth_image, queue_size=1)
        rospy.sleep(1)
        self.yolo_sub = rospy.Subscriber('yolo_ros/bounding_boxes', BoundingBoxes, callback=self.bounding_box, queue_size=1)
        rospy.sleep(1)

    def bounding_box(self, detected_msg):
        self.results = detected_msg.bounding_boxes
        self.cmd_vel = Twist()

        for result in self.results:
            if result.Class != 'person':
                continue

            xmin = result.xmin
            xmax = result.xmax
            ymin = result.ymin
            ymax = result.ymax

            person_detected_depth = self.depth_image[xmin:xmax, ymin:ymax]
            median_distance = np.median(person_detected_depth) / 1000

            center_x = 640 // 2
            center_y = 480 // 2

            self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z = 0.0, 0.0, 0.0
            self.cmd_vel.angular.x, self.cmd_vel.angular.y, self.cmd_vel.angular.z = 0.0, 0.0, 0.0

            if np.isnan(median_distance) or median_distance == 0.0:
                continue 

            
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


                # self.cmd_vel_Pub.publish(self.cmd_vel)

                                      
            rospy.loginfo('Person is %.2f m away.' % (median_distance))
            if self.is_centered and not self.reached_person:
                if median_distance > 1.0:
                    self.cmd_vel.linear.x = 0.2
                    self.cmd_vel.angular.z = 0.0
                    # self.cmd_vel_Pub.publish(self.cmd_vel)
                
                elif median_distance > 0.7:
                    self.cmd_vel.linear.x = 0.1
                    self.cmd_vel.angular.z = 0.0
                    # self.cmd_vel_Pub.publish(self.cmd_vel)

                elif median_distance > 0.6:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    # self.cmd_vel_Pub.publish(self.cmd_vel)
                    self.reached_person = True
                    

    
        # rospy.loginfo(depth_img_msg.data)
        # self.depth_image = depth_img_msg.data


if __name__ == '__main__':
    rospy.init_node('get_distance')
    getBoundingBoxDistance()
    rospy.spin()