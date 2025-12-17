#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped

class TargetBoxDetector:
    def __init__(self):
        rospy.init_node('target_box_detector', anonymous=True)
        
        self.bridge = CvBridge()
        self.target_detected = False
        self.detection_count = 0
        self.required_detections = 5
        
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        self.min_area = 500
        
        # Use relative topic names to work with namespaces (remapped in launch file)
        camera_topic = rospy.get_param('~camera_topic', '/limo/color/image_raw')
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
        
        self.detection_pub = rospy.Publisher('target_detected', Bool, queue_size=10)
        self.status_pub = rospy.Publisher('detection_status', String, queue_size=10)
        self.debug_image_pub = rospy.Publisher('debug_image', Image, queue_size=10)
        
        rospy.loginfo("Target Box Detector initialized. Waiting for camera images...")
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
        
        detected, debug_image = self.detect_red_box(cv_image)
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Debug image publish error: {e}")
        
        if detected:
            self.detection_count += 1
            if self.detection_count >= self.required_detections and not self.target_detected:
                self.target_detected = True
                rospy.logwarn("=" * 60)
                rospy.logwarn("TARGET BOX DETECTED!")
                rospy.logwarn("=" * 60)
                self.detection_pub.publish(Bool(data=True))
                self.status_pub.publish(String(data="TARGET_FOUND"))
        else:
            self.detection_count = max(0, self.detection_count - 1)
            if self.detection_count == 0 and self.target_detected:
                rospy.loginfo("Target box lost from view")
                self.target_detected = False
                self.detection_pub.publish(Bool(data=False))
                self.status_pub.publish(String(data="TARGET_LOST"))
    
    def detect_red_box(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        debug_image = image.copy()
        detected = False
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area > self.min_area:
                detected = True
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.drawContours(debug_image, [contour], -1, (0, 255, 255), 2)
                cv2.putText(debug_image, f"TARGET (Area: {int(area)})", 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 255, 0), 2)
        
        status_text = "DETECTED!" if detected else "Searching..."
        status_color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(debug_image, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
        cv2.putText(debug_image, f"Confidence: {self.detection_count}/{self.required_detections}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return detected, debug_image
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = TargetBoxDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass