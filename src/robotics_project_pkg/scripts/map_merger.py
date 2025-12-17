#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

class MapMerger:
    def __init__(self):
        rospy.init_node('manual_map_merger', anonymous=False)
        
        # Robot spawn positions from launch file
        # robot1: (0.5, 6.5)   - bottom left
        # robot2: (14.5, 0.5)  - bottom right
        # robot3: (0.5, 11.5)  - top left
        
        # Calculate offsets relative to robot1 (reference)
        # These are the differences in spawn positions
        self.robot_offsets = {
            'robot1': {'x': 0.0, 'y': 0.0},           # Reference robot
            'robot2': {'x': 0.0, 'y': 0.0},         # 14.5-0.5=14, 0.5-6.5=-6
            'robot3': {'x': 0.0, 'y': 0.0}            # 0.5-0.5=0, 11.5-6.5=5
        }
        
        # Store received maps
        self.maps = {}
        self.map_received = {
            'robot1': False,
            'robot2': False,
            'robot3': False
        }
        
        # Subscribe to each robot's map
        self.map_subs = []
        for robot_name in self.robot_offsets.keys():
            sub = rospy.Subscriber(
                f'/{robot_name}/map',
                OccupancyGrid,
                self.map_callback,
                callback_args=robot_name
            )
            self.map_subs.append(sub)
        
        # Publisher for merged map
        self.merged_map_pub = rospy.Publisher(
            '/map_merged',
            OccupancyGrid,
            queue_size=1,
            latch=True
        )
        
        # Merge rate
        self.merge_rate = rospy.Rate(1.0)
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("Manual Map Merger initialized")
        rospy.loginfo(f"Listening to maps from: {list(self.robot_offsets.keys())}")
        rospy.loginfo("\nMap offsets (translation only, no rotation):")
        for robot, offset in self.robot_offsets.items():
            rospy.loginfo(f"  {robot}: x={offset['x']:+.1f}m, y={offset['y']:+.1f}m")
        rospy.loginfo("=" * 70)
    
    def map_callback(self, msg, robot_name):
        """Callback to receive maps from robots"""
        self.maps[robot_name] = msg
        if not self.map_received[robot_name]:
            self.map_received[robot_name] = True
            rospy.loginfo(f"✓ Received first map from {robot_name}")
            rospy.loginfo(f"  Size: {msg.info.width}x{msg.info.height} cells")
            rospy.loginfo(f"  Resolution: {msg.info.resolution}m/cell")
            rospy.loginfo(f"  Origin: ({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})")
    
    def merge_maps(self):
        """Merge all robot maps into a single map"""
        
        if not any(self.map_received.values()):
            return None
        
        # Get resolution from first available map
        resolution = None
        for robot_name, received in self.map_received.items():
            if received:
                resolution = self.maps[robot_name].info.resolution
                break
        
        if resolution is None:
            return None
        
        # First pass: determine bounds in world frame
        min_x, max_x = float('inf'), float('-inf')
        min_y, max_y = float('inf'), float('-inf')
        
        for robot_name, received in self.map_received.items():
            if not received:
                continue
            
            map_msg = self.maps[robot_name]
            offset = self.robot_offsets[robot_name]
            
            # Map origin in local frame
            origin_x = map_msg.info.origin.position.x
            origin_y = map_msg.info.origin.position.y
            
            # Map dimensions
            map_width = map_msg.info.width * resolution
            map_height = map_msg.info.height * resolution
            
            # Calculate bounds in world frame (translation only, no rotation)
            world_origin_x = origin_x + offset['x']
            world_origin_y = origin_y + offset['y']
            world_max_x = world_origin_x + map_width
            world_max_y = world_origin_y + map_height
            
            min_x = min(min_x, world_origin_x)
            max_x = max(max_x, world_max_x)
            min_y = min(min_y, world_origin_y)
            max_y = max(max_y, world_max_y)
            
            rospy.loginfo_once(f"{robot_name} world bounds: x=[{world_origin_x:.2f}, {world_max_x:.2f}], y=[{world_origin_y:.2f}, {world_max_y:.2f}]")
        
        # Add small margin
        margin = 0.5
        min_x -= margin
        min_y -= margin
        max_x += margin
        max_y += margin
        
        # Create merged map
        merged_width = int(np.ceil((max_x - min_x) / resolution))
        merged_height = int(np.ceil((max_y - min_y) / resolution))
        
        rospy.loginfo_once("=" * 70)
        rospy.loginfo_once(f"MERGED MAP PROPERTIES:")
        rospy.loginfo_once(f"  Size: {merged_width}x{merged_height} cells")
        rospy.loginfo_once(f"  Dimensions: {(max_x-min_x):.2f}m x {(max_y-min_y):.2f}m")
        rospy.loginfo_once(f"  Bounds: x=[{min_x:.2f}, {max_x:.2f}], y=[{min_y:.2f}, {max_y:.2f}]")
        rospy.loginfo_once(f"  Origin: ({min_x:.2f}, {min_y:.2f})")
        rospy.loginfo_once(f"  Resolution: {resolution}m/cell")
        rospy.loginfo_once("=" * 70)
        
        # Initialize with unknown (-1)
        merged_data = np.full(merged_width * merged_height, -1, dtype=np.int8)
        
        # Second pass: copy map data
        for robot_name, received in self.map_received.items():
            if not received:
                continue
            
            map_msg = self.maps[robot_name]
            offset = self.robot_offsets[robot_name]
            
            # Map origin in local frame
            origin_x = map_msg.info.origin.position.x
            origin_y = map_msg.info.origin.position.y
            
            # Convert to numpy array
            robot_map = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
            
            cells_merged = 0
            cells_skipped = 0
            
            # Copy each cell
            for local_y in range(map_msg.info.height):
                for local_x in range(map_msg.info.width):
                    
                    cell_value = robot_map[local_y, local_x]
                    
                    # Skip unknown cells
                    if cell_value == -1:
                        cells_skipped += 1
                        continue
                    
                    # Convert local grid position to local coordinates (cell center)
                    local_world_x = origin_x + (local_x + 0.5) * resolution
                    local_world_y = origin_y + (local_y + 0.5) * resolution
                    
                    # Transform to world frame (translation only)
                    world_x = local_world_x + offset['x']
                    world_y = local_world_y + offset['y']
                    
                    # Convert to merged map indices
                    merged_x = int((world_x - min_x) / resolution)
                    merged_y = int((world_y - min_y) / resolution)
                    
                    # Check bounds
                    if 0 <= merged_x < merged_width and 0 <= merged_y < merged_height:
                        merged_idx = merged_y * merged_width + merged_x
                        
                        # Update cell
                        if merged_data[merged_idx] == -1:
                            # First time seeing this cell
                            merged_data[merged_idx] = cell_value
                        else:
                            # Handle overlapping cells
                            if cell_value > 50 or merged_data[merged_idx] > 50:
                                # If either is an obstacle, take max (obstacles have priority)
                                merged_data[merged_idx] = max(merged_data[merged_idx], cell_value)
                            else:
                                # Both are free space, average them
                                merged_data[merged_idx] = int((merged_data[merged_idx] + cell_value) / 2)
                        
                        cells_merged += 1
            
            rospy.loginfo_throttle(10.0, f"{robot_name}: merged {cells_merged} cells (skipped {cells_skipped} unknown)")
        
        # Create merged map message
        merged_map = OccupancyGrid()
        merged_map.header.stamp = rospy.Time.now()
        merged_map.header.frame_id = "world"
        
        merged_map.info.resolution = resolution
        merged_map.info.width = merged_width
        merged_map.info.height = merged_height
        
        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.origin.position.z = 0.0
        merged_map.info.origin.orientation.w = 1.0
        
        merged_map.data = merged_data.tolist()
        
        return merged_map
    
    def run(self):
        """Main loop"""
        rospy.loginfo("Starting map merger loop...")
        
        # Wait for all maps
        while not rospy.is_shutdown() and not all(self.map_received.values()):
            received_count = sum(self.map_received.values())
            rospy.loginfo_throttle(2.0, f"Waiting for all maps... ({received_count}/3)")
            self.merge_rate.sleep()
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("✓ All maps received! Starting continuous merge...")
        rospy.loginfo("=" * 70)
        
        while not rospy.is_shutdown():
            merged_map = self.merge_maps()
            
            if merged_map is not None:
                self.merged_map_pub.publish(merged_map)
                rospy.loginfo_throttle(10.0, "✓ Published merged map to /map_merged")
            
            self.merge_rate.sleep()

if __name__ == '__main__':
    try:
        merger = MapMerger()
        merger.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Map merger shutting down...")
        pass