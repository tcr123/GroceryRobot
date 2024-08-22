#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import json
import math
import os
from gtts import gTTS
from cabinet_edge_detector import CabinetEdgeDetector
import cv2 

class getBoundingBoxDistance():
    def __init__(self):
        self.results = None
        self.depth_image = None
        self.bridge = CvBridge()
        self.size = 30
        self.is_centered = [None] * self.size
        self.is_reached_person = [None] * self.size
        self.index = 0
        self.item = None
        self.first_signal = False
        self.take_picture_signal = False
        self.take_picture_done_signal = False
        self.image_path = '/home/mustar/catkin_ws/src/grocery/images/edge_detection.jpeg'
        self.threshold_circular_distance = 20
        self.threshold_distance = 1.4

        rospy.init_node('get_shelf_distance')
        self.cmd_vel_Pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.second_move = rospy.Publisher('task_status', String, queue_size=10)
        self.yolo_sub = rospy.Subscriber('/shelf_detection_results', String, self.bounding_box)
        self.prev_yolo_sub = rospy.Subscriber('/detection_results', String, self.get_item_name)
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1, buff_size=10000000)
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=10000000)

    def get_item_name(self, msg):
        item = json.loads(msg.data)
        self.item = item['Class']

    def text2audio(self, text):
        tts = gTTS(text)
        tts.save("main_audio.mp3")
        os.system("mpg321 main_audio.mp3")
        os.remove("main_audio.mp3")

    def append(self, list, value):
        print(self.index)
        list[self.index] = value
        self.index = (self.index + 1) % self.size

    def is_half_true(self):
        true_count = sum(1 for item in self.is_centered if item is True)
        return true_count >= self.size // 2
    
    def is_half_distance_true(self):
        true_count = sum(1 for item in self.is_reached_person if item is True)
        return true_count >= self.size // 2

    def depth_callback(self, msg):
        self.depth_image = CvBridge().imgmsg_to_cv2(msg, "passthrough")

    def image_callback(self, msg):
        if self.take_picture_signal == True:
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(self.image_path, cv_image)
            self.take_picture_done_signal = True

    def bounding_box(self, detected_msg):
        if self.first_signal == False:
            detected_msg = json.loads(detected_msg.data)
            print(detected_msg)
        
            self.results = detected_msg
            self.cmd_vel = Twist()

            xmin = self.results['xmin']
            xmax = self.results['xmax']
            ymin = self.results['ymin']
            ymax = self.results['ymax']

            person_detected_depth = self.depth_image[xmin:xmax, ymin:ymax]
            valid_depths = person_detected_depth[~np.isnan(person_detected_depth)]
            median_distance = np.nanmedian(valid_depths) / 1000
            print(f"Direct hypothenus: {median_distance}")
            new_median_distance = math.sqrt((median_distance**2) - (0.45**2))

            center_x = 640 // 2
            center_y = 480 // 2
            self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z = 0.0, 0.0, 0.0
            self.cmd_vel.angular.x, self.cmd_vel.angular.y, self.cmd_vel.angular.z = 0.0, 0.0, 0.0

            if np.isnan(new_median_distance) or new_median_distance == 0.0:
                print("hi")

            
            if ( center_x - 10 < (xmax + xmin) // 2 < center_x + 10 ) and  ( center_y - 10 < (ymax + ymin) // 2 < center_y + 10 ):
                rospy.loginfo('Person at center!')
                self.append(self.is_centered, True)

            elif self.is_half_true() == False:
                diff_from_center_x = center_x - (xmax + xmin) // 2
                rospy.loginfo(f"DIFFERENT FROM CENTER X {diff_from_center_x}")

                if diff_from_center_x > 0:
                    if diff_from_center_x < self.threshold_circular_distance:
                        print("center")
                        self.append(self.is_centered, True)
                    elif diff_from_center_x > 50:
                        self.cmd_vel.angular.z=0.2
                    elif diff_from_center_x > 160:
                        self.cmd_vel.angular.z=0.3
                    elif diff_from_center_x > 250:
                        self.cmd_vel.angular.z= 0.5
                    else:
                        self.cmd_vel.angular.z=-((abs(diff_from_center_x)-self.threshold_circular_distance)/320)
                else:
                    if abs(diff_from_center_x) < self.threshold_circular_distance:
                        print("center")
                        self.append(self.is_centered, True)
                    elif abs(diff_from_center_x) > 50:
                        self.cmd_vel.angular.z = -0.2
                    elif abs(diff_from_center_x) > 160:
                        self.cmd_vel.angular.z= -0.3
                    elif abs(diff_from_center_x) > 250:
                        self.cmd_vel.angular.z=-0.5 
                    else:
                        self.cmd_vel.angular.z=(abs(diff_from_center_x)-self.threshold_circular_distance)/320


                self.cmd_vel_Pub.publish(self.cmd_vel)

                                        
            rospy.loginfo('Item is %.2f m away.' % (new_median_distance))
            print(f"Half True: {self.is_half_true()}")
            print(f"Distance half true:: {self.is_half_distance_true()}")
            if self.is_half_true() and not self.is_half_distance_true():
                if median_distance > 1.0:
                    self.cmd_vel.linear.x = 0.2
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)
                
                elif median_distance > 0.7:
                    self.cmd_vel.linear.x = 0.1
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)

                elif median_distance < self.threshold_distance:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)
                    self.append(self.is_reached_person, True)

            # Before putting objects, detect edge and know which layer to put
            if self.is_half_true() and self.is_half_distance_true():
                self.take_picture_signal = True
                if self.take_picture_done_signal:
                    processor = CabinetEdgeDetector(self.image_path)
                    processed_image = processor.preprocess_image()
                    processor.debug_contours(processed_image)
                    cabinet_list = processor.run(processed_image)
                    print(cabinet_list)
                    # Check which layer is it going to put into

            # Check angular distance, horizontal distance and what item is selected
            if self.is_half_true() and self.is_half_distance_true() and self.item is not None :
                print(f'I am putting {self.item} into the shelf')
                self.text2audio(f'I am putting {self.item} into the shelf')
                rospy.sleep(5)
                self.second_move.publish("True")
                self.first_signal = True
                print("STOP")

            self.index += 1

if __name__ == '__main__':
    try:
        getBoundingBoxDistance = getBoundingBoxDistance()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Error with demo.py")