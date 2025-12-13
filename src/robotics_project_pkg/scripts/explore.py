#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time
import random

class ReactiveExplorer:
    def __init__(self):
        rospy.init_node('reactive_explorer')

        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # Subscribe to the scan topic
        rospy.Subscriber('limo/scan', LaserScan, self.scan_callback)

        # Initialize state variables
        self.front_distance = 1.0
        self.scan_received = False  # <--- NEW: Flag to track if a scan has been received
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Initialize the random number generator for rotation
        random.seed(time.time())

    def scan_callback(self, msg):
        """Update front distance using a small center slice of laser scan"""
        # --- Existing logic to find minimum distance ---
        center = len(msg.ranges) // 2
        # Check a 10-degree slice around the front (assuming a typical 360-point scan)
        center_angles = range(center - 10, center + 10)
        # Filter out invalid (0.0 or inf) readings
        distances = [msg.ranges[i] for i in center_angles if msg.ranges[i] > msg.range_min and msg.ranges[i] < msg.range_max]
        
        # Update front distance
        # Use range_max if no valid distance is found (e.g., if filtered list is empty)
        self.front_distance = min(distances) if distances else msg.range_max 
        
        # <--- NEW: Set flag after the first successful callback ---
        if not self.scan_received:
            self.scan_received = True
            rospy.loginfo("Initial laser scan data received. Starting exploration...")
        # --------------------------------------------------------

    def rotate_fixed_angle(self, angle_deg):
        """Rotate robot by a fixed angle in degrees (positive=left, negative=right)"""
        twist = Twist()
        angular_speed = 0.5  # rad/s
        angle_rad = math.radians(abs(angle_deg))
        duration = angle_rad / angular_speed  # time to rotate desired angle
        
        twist.linear.x = 0.0
        twist.angular.z = angular_speed if angle_deg > 0 else -angular_speed

        start_time = time.time()
        while time.time() - start_time < duration and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        # Stop rotation
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def explore(self):
        # <--- NEW: Wait for the first scan data before starting the main loop ---
        rospy.loginfo("Waiting for initial laser scan data...")
        while not self.scan_received and not rospy.is_shutdown():
            self.rate.sleep()
        
        # Main exploration loop
        while not rospy.is_shutdown():
            if self.front_distance > 0.8:
                # Path clear → move forward
                twist = Twist()
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
            else:
                # Obstacle detected → rotate
                # <--- FIX: Simplified random direction choice (no need for seed management) ---
                direction = random.choice([-1, 1]) 
                rospy.loginfo(f"Obstacle detected! Rotating {'left' if direction>0 else 'right'} 90°...")
                
                # Keep rotating by 90 degrees until the path clears
                # Note: `rospy.sleep(0.1)` in the original code is no longer necessary
                # because the next rotation is only called after the current one completes 
                # and the main loop cycles, allowing time for the scan to update.
                while self.front_distance <= 0.8 and not rospy.is_shutdown():
                    self.rotate_fixed_angle(direction * 90)
                    # After rotation, immediately check the distance before the main loop cycle
                    if self.front_distance > 0.8:
                        rospy.loginfo("Path clear after rotation. Moving forward.")
                        break # Exit the inner rotation loop

            self.rate.sleep()

if __name__ == '__main__':
    try:
        explorer = ReactiveExplorer()
        # Initial rospy.sleep(1) is not strictly needed anymore, 
        # as the 'explore' method now handles the wait for the first scan.
        explorer.explore()
    except rospy.ROSInterruptException:
        pass