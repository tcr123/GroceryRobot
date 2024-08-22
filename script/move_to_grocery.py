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

class getBoundingBoxDistance():
    def __init__(self):
        self.results = None
        self.depth_image = None
        self.bridge = CvBridge()
        self.size = 30
        self.is_centered = [None] * self.size
        self.is_reached_person = [None] * self.size
        rospy.init_node('get_distance')
        self.cmd_vel_Pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1, buff_size=10000000)
        rospy.sleep(1)
        self.yolo_sub = rospy.Subscriber('/detection_results', String, self.bounding_box)
        self.yolo_sub2 = rospy.Subscriber('/detection_results2', String, self.move_robot)
        rospy.sleep(1)
        self.first_signal = False
        self.second_signal = False
        # self.final_move = rospy.Publisher('task_status', String, queue_size=10)
        self.arm_move = rospy.Publisher('arm_status', String, queue_size=10)
        self.index = 0
        self.threshold_circular_distance = 20 # in unit mm
        self.threshold_distance = 0.7 # in unit m
        self.distance_pub = rospy.Publisher('distance_check', String, queue_size=10)

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
            
            # distance with the detected object
            # median_distance = np.median(person_detected_depth) / 1000
            print(f"Direct hypothenus: {median_distance}")
            new_median_distance = math.sqrt((median_distance**2) - (0.45**2))

            center_x = 640 // 2
            center_y = 480 // 2
            self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z = 0.0, 0.0, 0.0
            self.cmd_vel.angular.x, self.cmd_vel.angular.y, self.cmd_vel.angular.z = 0.0, 0.0, 0.0

            if np.isnan(new_median_distance) or new_median_distance == 0.0:
                print("Unknown Error Occurred, median distance is 0")

            # Check if the item is at centre
            if ( center_x - 10 < (xmax + xmin) // 2 < center_x + 10 ) and  ( center_y - 10 < (ymax + ymin) // 2 < center_y + 10 ):
                rospy.loginfo('Person at center!')
                self.append(self.is_centered, True)

            # If the item is not facing centre
            elif self.is_half_true() == False:
                diff_from_center_x = center_x - (xmax + xmin) // 2
                rospy.loginfo(f"DIFFERENT FROM CENTER X {diff_from_center_x}")

                # Turn the robot to face it (clockwise)
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
                        self.cmd_vel.angular.z=-((abs(diff_from_center_x)- self.threshold_circular_distance)/320)

                # Turn the robot to face it (anticlockwise)
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
                        self.cmd_vel.angular.z=(abs(diff_from_center_x)- self.threshold_circular_distance)/320


                self.cmd_vel_Pub.publish(self.cmd_vel)

            # If the robot is not at centre and too far away                       
            rospy.loginfo('Item is %.2f m away.' % (new_median_distance))
            print(f"Half True: {self.is_half_true()}")
            print(f"Distance half true:: {self.is_half_distance_true()}")
            if self.is_half_true() and not self.is_half_distance_true():

                # If the distance is more than 1 meter
                if new_median_distance > 1.0:
                    self.cmd_vel.linear.x = 0.2
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)
                
                # If the distance is more than 0.7 meter
                elif new_median_distance > 0.7:
                    self.cmd_vel.linear.x = 0.1
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)

                # If the distance is more than 0.6 meter
                elif new_median_distance < self.threshold_distance:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)
                    self.append(self.is_reached_person, True)

            if self.is_half_true() and self.is_half_distance_true():
                print(f'I am taking up {detected_msg["Class"]}')
                # self.text2audio(f'I am taking up {detected_msg["Class"]}')
                self.arm_move.publish('True')
                self.first_signal = True
                print("STOP!")
                # After publishing, put down robot arm

            self.index += 1

    # After putting down robot arm and camera, check if need to move forward or not
    def move_robot(self, detected_msg):
        if self.second_signal == False:
            detected_msg = json.loads(detected_msg.data)
            print(detected_msg)
            self.results = detected_msg
            self.cmd_vel = Twist()

            xmin = self.results['xmin']
            xmax = self.results['xmax']
            ymin = self.results['ymin']
            ymax = self.results['ymax']

            person_detected_depth = self.depth_image[xmin:xmax, ymin:ymax]
            
            # distance with the detected object
            median_distance = np.median(person_detected_depth) / 1000

            center_x = 640 // 2
            center_y = 480 // 2
            self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.linear.z = 0.0, 0.0, 0.0
            self.cmd_vel.angular.x, self.cmd_vel.angular.y, self.cmd_vel.angular.z = 0.0, 0.0, 0.0

            if np.isnan(median_distance) or median_distance == 0.0:
                print("Unknown Error Occurred, median distance is 0")

            
            # Check if the item is at centre
            if ( center_x - 15 < (xmax + xmin) // 2 < center_x + 15 ) and  ( center_y - 10 < (ymax + ymin) // 2 < center_y + 10 ):
                rospy.loginfo('Person at center!')
                self.append(self.is_centered, True)

            # If the item is not facing centre
            elif not self.is_half_true():
                diff_from_center_x = center_x - (xmax + xmin) // 2
                rospy.loginfo(diff_from_center_x)

                # Turn the robot to face it (clockwise)
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
                        self.cmd_vel.angular.z=-((abs(diff_from_center_x)- self.threshold_circular_distance)/320)

                # Turn the robot to face it (anticlockwise)
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
                        self.cmd_vel.angular.z=(abs(diff_from_center_x)- self.threshold_circular_distance)/320


                self.cmd_vel_Pub.publish(self.cmd_vel)

            # If the robot is not at centre and too far away                       
            rospy.loginfo('Item is %.2f m away.' % (median_distance))
            if self.is_half_true() and not self.is_half_distance_true():

                # If the distance is more than 1 meter
                if median_distance > 1.0:
                    self.cmd_vel.linear.x = 0.2
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)
                
                # If the distance is more than 0.7 meter
                elif median_distance > 0.7:
                    self.cmd_vel.linear.x = 0.1
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)

                # If the distance is more than 0.6 meter
                elif median_distance < self.threshold_distance - 0.20:
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 0.0
                    self.cmd_vel_Pub.publish(self.cmd_vel)
                    self.append(self.is_reached_person, True)

            if self.is_half_true() and self.is_half_distance_true():
                print(f'I am taking up {detected_msg["Class"]}')
                self.text2audio(f'I am taking up {detected_msg["Class"]}')
                self.distance_pub.publish('True')
                self.second_signal = True
                
                # Step backward
                self.cmd_vel.linear.x = 0.1
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_Pub.publish(self.cmd_vel)
                rospy.sleep(2)
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel_Pub.publish(self.cmd_vel)
                # After publishing, put down robot arm

            self.index += 1
    

if __name__ == '__main__':
    try:
        getBoundingBoxDistance = getBoundingBoxDistance()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Error with demo.py")