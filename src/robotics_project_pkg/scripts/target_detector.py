#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry

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
        
        # Store robot location
        self.robot_position = None
        self.robot_orientation = None
        
        # Store last detected location
        self.last_target_location = None
        
        # Get robot namespace
        self.robot_namespace = rospy.get_namespace().strip('/')
        
        # Use relative topic names to work with namespaces
        # If running with namespace (e.g., /robot1), it will subscribe to robot1/odom
        camera_topic = rospy.get_param('~camera_topic', '/limo/color/image_raw')
        odom_topic = rospy.get_param('~odom_topic', 'odom')  # Relative topic, will use namespace
        
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        
        self.detection_pub = rospy.Publisher('target_detected', Bool, queue_size=10)
        self.status_pub = rospy.Publisher('detection_status', String, queue_size=10)
        self.debug_image_pub = rospy.Publisher('debug_image', Image, queue_size=10)
        self.location_pub = rospy.Publisher('target_location', Point, queue_size=10)
        self.robot_pose_pub = rospy.Publisher('robot_pose_at_detection', PoseStamped, queue_size=10)
        
        rospy.loginfo(f"Target Box Detector initialized for {self.robot_namespace if self.robot_namespace else 'root namespace'}")
        rospy.loginfo(f"Subscribing to: {odom_topic}")
        rospy.loginfo("Waiting for camera images and odometry...")
    
    def odom_callback(self, msg):
        """Callback to receive robot odometry/position"""
        self.robot_position = msg.pose.pose.position
        self.robot_orientation = msg.pose.pose.orientation
    
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
        
        detected, debug_image, target_location = self.detect_red_box(cv_image)
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Debug image publish error: {e}")
        
        if detected:
            self.detection_count += 1
            self.last_target_location = target_location
            
            if self.detection_count >= self.required_detections and not self.target_detected:
                self.target_detected = True
                rospy.logwarn("=" * 60)
                rospy.logwarn("TARGET BOX DETECTED!")
                
                if self.robot_position is not None:
                    rospy.logwarn("-" * 60)
                    rospy.logwarn(f"ROBOT LOCATION AT DETECTION ({self.robot_namespace if self.robot_namespace else 'root'}):")
                    rospy.logwarn(f"  Position (x, y, z): ({self.robot_position.x:.3f}, {self.robot_position.y:.3f}, {self.robot_position.z:.3f})")
                    
                    # Convert quaternion to yaw angle for easier understanding
                    yaw = self.quaternion_to_yaw(self.robot_orientation)
                    rospy.logwarn(f"  Orientation (yaw): {yaw:.3f} radians ({np.degrees(yaw):.1f} degrees)")
                    # Publish robot pose
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = rospy.Time.now()
                    pose_msg.header.frame_id = "odom"
                    pose_msg.pose.position = self.robot_position
                    pose_msg.pose.orientation = self.robot_orientation
                    self.robot_pose_pub.publish(pose_msg)
                else:
                    rospy.logwarn("Robot location not available yet (waiting for odometry data)")
                
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
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle (rotation around z-axis)"""
        # Formula: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
    
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
        target_location = None
        largest_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if area > self.min_area:
                detected = True
                x, y, w, h = cv2.boundingRect(contour)
                
                # Keep track of largest detection (assuming it's the main target)
                if area > largest_area:
                    largest_area = area
                    center_x = x + w / 2
                    center_y = y + h / 2
                    target_location = {
                        'x': x,
                        'y': y,
                        'width': w,
                        'height': h,
                        'center_x': center_x,
                        'center_y': center_y,
                        'area': area
                    }
                
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
                cv2.drawContours(debug_image, [contour], -1, (0, 255, 255), 2)
                
                # Draw center point
                center_x_int = int(x + w / 2)
                center_y_int = int(y + h / 2)
                cv2.circle(debug_image, (center_x_int, center_y_int), 5, (255, 0, 0), -1)
                
                cv2.putText(debug_image, f"TARGET (Area: {int(area)})", 
                           (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, (0, 255, 0), 2)
                cv2.putText(debug_image, f"Center: ({center_x_int}, {center_y_int})", 
                           (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (255, 255, 255), 2)
        
        status_text = "DETECTED!" if detected else "Searching..."
        status_color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(debug_image, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
        cv2.putText(debug_image, f"Confidence: {self.detection_count}/{self.required_detections}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Display robot position if available
        if self.robot_position is not None:
            robot_text = f"Robot: ({self.robot_position.x:.2f}, {self.robot_position.y:.2f})"
            cv2.putText(debug_image, robot_text, 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)
        
        return detected, debug_image, target_location
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = TargetBoxDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass