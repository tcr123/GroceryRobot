#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
import ultralytics

# Initialize the ROS node
rospy.init_node('yolov8_object_detection')

# Define a publisher to publish detection results
detection_publisher = rospy.Publisher('/detection_results', String, queue_size=10)

# Load the trained YOLO model
model = ultralytics.YOLO('../src/best.pt')

depth_image = None  

# def depth_callback(msg):
#     global depth_image
#     try:
#         # Convert ROS image to OpenCV format
#         # rospy.loginfo(msg)
#         # np_arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
#         # np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
#         # depth_image = np_arr
#         # rospy.loginfo(depth_image)
#         # cv_image = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)


#         # Determine the correct dtype based on the encoding
#         if msg.encoding == '16UC1':
#             dtype = np.uint16  # 16-bit unsigned int, common for mm measurements
#         elif msg.encoding == '32FC1':
#             dtype = np.float32  # 32-bit floating point, common for meters
#         else:
#             rospy.logerr("Unsupported depth image encoding: {}".format(msg.encoding))
#             return
        
#         # Convert the byte data to a numpy array of the correct dtype
#         depth_array = np.frombuffer(msg.data, dtype=dtype)

#         # Reshape the flat array into a 2D array with the correct dimensions
#         depth_image = depth_array.reshape(msg.height, msg.width)

#     except Exception as e:
#         rospy.logerr("Failed to convert image: %s" % str(e))
#         # return
    
#     # rospy.loginfo(msg.data)

def image_callback(msg):
    try:
        # Convert ROS image to OpenCV format
        np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        cv_image = cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)
    except Exception as e:
        rospy.logerr("Failed to convert image: %s" % str(e))
        return

    # Inference
    results = model(cv_image)
    rospy.loginfo("Detection completed, processing results")

    # Assuming results is a list of Results objects
    if results:
        result = results[0]  # Get the first result set
        boxes = result.boxes
        
        # Process detection boxes
        for box in boxes:
            xyxy = box.xyxy.numpy() if isinstance(box.xyxy, torch.Tensor) else box.xyxy
            conf = box.conf.item()  # Assuming conf is a tensor with a single value
            cls = box.cls.item()  # Assuming cls is a tensor with a single value
            class_name = result.names[int(cls)]
            

            # Ensure xyxy is flattened to a 1D array if it's not already
            if xyxy.ndim > 1:
                xyxy = xyxy.flatten()

            x1, y1, x2, y2 = map(int, xyxy)

            # grocery_detected_depth = depth_image[y1:y2, x1:x2]
            # median_distance = np.median(grocery_detected_depth) / 1000

            # Assuming depth_image is the correctly reshaped depth data array
            # if depth_image is not None:
            #     # Extract the depth values within the bounding box
            #     depth_region = depth_image[x1:x2, y1:y2]
            #     valid_depths = depth_region[~np.isnan(depth_region)]  # Filter out NaN values

            #     if valid_depths.size > 0:
            #         median_distance = np.nanmedian(valid_depths)
            #         rospy.loginfo(f"Median distance: {median_distance} meters")
            #     else:
            #         rospy.logwarn("No valid depth data available within the detection region.")
            # else:
            #     rospy.logwarn("Depth image is not available.")


            # Extract coordinatesx1:x2, y1:y2
            
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # label = f"{class_name} {conf:.2f}, {median_distance} meter"
            label = f"{class_name} {conf:.2f}"
            cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

             # Prepare the message
            # detection_message = f"Detected {class_name} with confidence {conf:.2f} at [{x1}, {y1}, {x2}, {y2}], distance {median_distance} meter"
            detection_message = f"Detected {class_name} with confidence {conf:.2f} at [{x1}, {y1}, {x2}, {y2}]"
            rospy.loginfo(detection_message)
            
            # Publish the message
            # detection_publisher.publish(detection_message)

    else:
        rospy.loginfo("No detections")
        # detection_publisher.publish("No detections")

    # Optional: Show image
    cv2.imshow("YOLOv8 Detection", cv_image)
    cv2.waitKey(1)

# Subscribe to the Astra camera image topic
image_subscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

# Define a subscriber to subscribe to depth image
# depthImage_subscriber = rospy.Subscriber('/camera/depth/image', Image, depth_callback)

# Keep the node running
rospy.spin()

# Make sure to properly handle shutdown
cv2.destroyAllWindows()