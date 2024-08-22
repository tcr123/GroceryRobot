#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import ultralytics
import uuid
import json
import math

class GroceryDetection:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('yolov8_object_detection')
        # Define a publisher to publish detection results
        self.detection_publisher = rospy.Publisher('/detection_results', String, queue_size=10)
        self.detection_publisher2 = rospy.Publisher('/detection_results2', String, queue_size=10)
        # Load the trained YOLO model
        self.model = ultralytics.YOLO('/home/mustar/catkin_ws/src/grocery/src/twiggies.pt')
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=10000000)
        self.sub2 = rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1, buff_size=10000000)
        self.location_sub = rospy.Subscriber('location_table', String, self.self_arrange)
        self.depth_image = None  
        self.CONFIDENCE_THRESHOLD = 0.5
        self.status = False # Initially should be false if involve navigation
        self.camera_down = rospy.Subscriber('camera_down', String, self.detect_again)
        self.detect_again_status = False
        self.distance_sub = rospy.Subscriber('distance_check', String, self.close_camera)

    def close_camera(self):
        cv2.destroyAllWindows()

    # After reaching the table
    def self_arrange(self, msg):
        status = msg.data
        print(f"Status: {status}")
        if status == 'True':
            self.status = True

    # Reached the table, lowered the camera
    def detect_again(self, msg):
        status = msg.data
        print(f"Status: {status}")
        if status == 'True':
            self.detect_again_status = True

    def depth_callback(self, msg):
        # Start calling this function after the robot reach the location
        if self.status:
            self.depth_image = CvBridge().imgmsg_to_cv2(msg, "passthrough")

        if self.detect_again_status:
            self.depth_image = CvBridge().imgmsg_to_cv2(msg, "passthrough")

    def image_callback(self, msg):
        # Start calling this function after the robot reach the location
        if self.status:
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            # Inference
            results = self.model.track(cv_image, persist=True)
            rospy.loginfo("Detection completed, processing results")

            # Assuming results is a list of Results objects
            if results:
                result = results[0]  # Get the first result set
                boxes = result.boxes
                
                # Process detection boxes
                for box in boxes:
                    conf = box.conf.item()
                    if conf >= self.CONFIDENCE_THRESHOLD:
                        xyxy = box.xyxy.numpy() if isinstance(box.xyxy, torch.Tensor) else box.xyxy
                        # Assuming conf is a tensor with a single value
                        cls = box.cls.item()  # Assuming cls is a tensor with a single value
                        class_name = result.names[int(cls)]
                        

                        # Ensure xyxy is flattened to a 1D array if it's not already
                        if xyxy.ndim > 1:
                            xyxy = xyxy.flatten()

                        x1, y1, x2, y2 = map(int, xyxy)

                        # grocery_detected_depth = self.depth_image[y1:y2, x1:x2]
                        # median_distance = np.median(grocery_detected_depth) / 1000

                        # Assuming depth_image is the correctly reshaped depth data array
                        if self.depth_image is not None:
                            # Extract the depth values within the bounding box
                            depth_region = self.depth_image[x1:x2, y1:y2]
                            valid_depths = depth_region[~np.isnan(depth_region)]  # Filter out NaN values

                            if valid_depths.size > 0:
                                median_distance = np.nanmedian(valid_depths) / 1000
                                print(f"Direct hypothenus: {median_distance}")
                                new_median_distance = math.sqrt((median_distance**2) - (0.45**2))
                                rospy.loginfo("Median distance: %.2f meters" % new_median_distance)
                            else:
                                rospy.logwarn("No valid depth data available within the detection region.")
                        else:
                            rospy.logwarn("Depth image is not available.")


                        # Extract coordinatesx1:x2, y1:y2
                        
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = ("%s %.2f" % (class_name,conf))
                        cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        # Prepare the message
                        if self.depth_image is not None:
                            detection_message = ("Detected %s with confidence %.2f at [%d , %d , %d , %d], distance %.2f meter" % (class_name, conf,x1,x2,y1,y2,new_median_distance))
                        else:
                            detection_message = ("Detected %s with confidence %.2f at [%d , %d , %d , %d], distance undetected meter" % (class_name, conf,x1,x2,y1,y2))
                        rospy.loginfo(detection_message)

                        bounding_box = {'conf': conf, 'xmin': x1, 'ymin': y1, 'xmax': x2, 'ymax': y2, 'id': str(uuid.uuid4()), 'Class': class_name}
                        bounding_box = json.dumps(bounding_box)
                        
                        # Publish the message
                        self.detection_publisher.publish(bounding_box)

            else:
                rospy.loginfo("No detections")
                self.detection_publisher.publish("No detections")

            # Optional: Show image
            cv2.imshow("YOLOv8 Detection", cv_image)
            cv2.waitKey(1)
        
        if self.detect_again_status:
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

            # Inference
            results = self.model.track(cv_image, persist=True)
            rospy.loginfo("Detection completed, processing results")

            # Assuming results is a list of Results objects
            if results:
                result = results[0]  # Get the first result set
                boxes = result.boxes
                
                # Process detection boxes
                for box in boxes:
                    conf = box.conf.item()
                    if conf >= self.CONFIDENCE_THRESHOLD:
                        xyxy = box.xyxy.numpy() if isinstance(box.xyxy, torch.Tensor) else box.xyxy
                        # Assuming conf is a tensor with a single value
                        cls = box.cls.item()  # Assuming cls is a tensor with a single value
                        class_name = result.names[int(cls)]
                        

                        # Ensure xyxy is flattened to a 1D array if it's not already
                        if xyxy.ndim > 1:
                            xyxy = xyxy.flatten()

                        x1, y1, x2, y2 = map(int, xyxy)

                        # grocery_detected_depth = self.depth_image[y1:y2, x1:x2]
                        # median_distance = np.median(grocery_detected_depth) / 1000

                        # Assuming depth_image is the correctly reshaped depth data array
                        if self.depth_image is not None:
                            # Extract the depth values within the bounding box
                            depth_region = self.depth_image[x1:x2, y1:y2]
                            valid_depths = depth_region[~np.isnan(depth_region)]  # Filter out NaN values

                            if valid_depths.size > 0:
                                median_distance = np.nanmedian(valid_depths) / 1000
                                rospy.loginfo("Median distance: %.2f meters" % median_distance)
                            else:
                                rospy.logwarn("No valid depth data available within the detection region.")
                        else:
                            rospy.logwarn("Depth image is not available.")


                        # Extract coordinatesx1:x2, y1:y2
                        
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        label = ("%s %.2f %.2f" % (class_name,conf,median_distance))
                        cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        # Prepare the message
                        detection_message = ("Detected %s with confidence %.2f at [%d , %d , %d , %d], distance %.2f meter" % (class_name, conf,x1,x2,y1,y2,median_distance))
                        rospy.loginfo(detection_message)

                        bounding_box = {'conf': conf, 'xmin': x1, 'ymin': y1, 'xmax': x2, 'ymax': y2, 'id': str(uuid.uuid4()), 'Class': class_name}
                        bounding_box = json.dumps(bounding_box)
                        
                        # Publish the message
                        self.detection_publisher2.publish(bounding_box)

            else:
                rospy.loginfo("No detections")
                self.detection_publisher2.publish("No detections")

            # Optional: Show image
            cv2.imshow("YOLOv8 Detection", cv_image)
            cv2.waitKey(1)
        

if __name__ == "__main__":
    try:
        grocery = GroceryDetection()
        rospy.spin()
        # Make sure to properly handle shutdown
        cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error with yolondistance.py")
