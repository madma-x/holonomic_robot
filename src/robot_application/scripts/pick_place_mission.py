#!/usr/bin/env python3
"""Pick and place mission - object manipulation with navigation."""

import rclpy
from robot_application.mission_base import MissionBase, MissionState
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class PickPlaceMission(MissionBase):
    """Pick and place mission: navigate and manipulate objects."""
    
    def __init__(self):
        super().__init__('pick_place')
        self.tasks = []
        self.current_task_idx = 0
    
    def load_mission_config(self):
        """Load pick-place tasks from config file."""
        try:
            config_dir = get_package_share_directory('robot_application')
            config_file = os.path.join(config_dir, 'config', 'pick_place_tasks.yaml')
            
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            self.tasks = config['pick_place_tasks']
            self.get_logger().info(f'Loaded {len(self.tasks)} pick-place tasks')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {str(e)}')
            # Use default task
            self.tasks = [{
                'name': 'default_task',
                'pickup_location': {'x': 1.0, 'y': 0.5, 'theta': 0.0},
                'place_location': {'x': 2.0, 'y': 0.5, 'theta': 0.0},
                'pickup_method': 'vacuum',  # 'vacuum' or 'gripper'
                'approach_distance': 0.3,
                'pickup_duration': 2.0
            }]
            self.get_logger().info('Using default task')
    
    def execute_mission(self):
        """Execute pick-place mission."""
        self.get_logger().info('Starting pick-place mission')
        
        try:
            for idx, task in enumerate(self.tasks):
                if self.stop_requested:
                    break
                
                self.current_task_idx = idx
                self.progress = (idx / len(self.tasks)) * 100.0
                
                self.get_logger().info(f'Task {idx + 1}/{len(self.tasks)}: {task["name"]}')
                
                # Execute pick-place sequence
                if not self.execute_pick_place_task(task):
                    self.get_logger().error(f'Task {task["name"]} failed')
                    self.state = MissionState.FAILED
                    return
                
                self.get_logger().info(f'Task {task["name"]} completed')
            
            self.get_logger().info('All tasks completed successfully')
            self.state = MissionState.COMPLETED
            self.progress = 100.0
            
        except Exception as e:
            self.get_logger().error(f'Mission error: {str(e)}')
            self.state = MissionState.FAILED
    
    def execute_pick_place_task(self, task: dict) -> bool:
        """Execute a single pick-place task."""
        pickup_loc = task['pickup_location']
        place_loc = task['place_location']
        method = task.get('pickup_method', 'vacuum')
        approach_dist = task.get('approach_distance', 0.3)
        pickup_duration = task.get('pickup_duration', 2.0)
        
        # Step 1: Navigate to pickup location
        self.get_logger().info('Step 1: Navigate to pickup location')
        if not self.navigate_to_pose(pickup_loc['x'], pickup_loc['y'], pickup_loc['theta']):
            return False
        
        # Step 2: Approach object (if approach distance specified)
        if approach_dist > 0:
            self.get_logger().info(f'Step 2: Approach object ({approach_dist}m)')
            import math
            approach_x = pickup_loc['x'] + approach_dist * math.cos(pickup_loc['theta'])
            approach_y = pickup_loc['y'] + approach_dist * math.sin(pickup_loc['theta'])
            if not self.navigate_to_pose(approach_x, approach_y, pickup_loc['theta']):
                return False
        
        # Step 3: Pick up object
        self.get_logger().info(f'Step 3: Pick up object using {method}')
        if method == 'vacuum':
            # Activate vacuum pump
            if not self.control_pump(0, enable=True, duty_cycle=0.8, duration=pickup_duration):
                return False
        elif method == 'gripper':
            # Lower gripper
            if not self.move_servo(1, angle=90.0, speed=30.0):
                return False
            time.sleep(0.5)
            # Close gripper
            if not self.move_servo(0, angle=45.0, speed=20.0):
                return False
            time.sleep(0.5)
            # Raise gripper
            if not self.move_servo(1, angle=0.0, speed=30.0):
                return False
        
        # Step 4: Navigate to place location
        self.get_logger().info('Step 4: Navigate to place location')
        if not self.navigate_to_pose(place_loc['x'], place_loc['y'], place_loc['theta']):
            # Still holding object, try to release before failing
            if method == 'vacuum':
                self.control_pump(0, enable=False)
            return False
        
        # Step 5: Release object
        self.get_logger().info('Step 5: Release object')
        if method == 'vacuum':
            # Turn off vacuum
            if not self.control_pump(0, enable=False):
                return False
        elif method == 'gripper':
            # Lower gripper
            if not self.move_servo(1, angle=90.0, speed=30.0):
                return False
            time.sleep(0.5)
            # Open gripper
            if not self.move_servo(0, angle=0.0, speed=20.0):
                return False
            time.sleep(0.5)
            # Raise gripper
            if not self.move_servo(1, angle=0.0, speed=30.0):
                return False
        
        # Step 6: Return to safe position
        self.get_logger().info('Step 6: Return to safe position')
        if approach_dist > 0:
            import math
            retreat_x = place_loc['x'] - approach_dist * math.cos(place_loc['theta'])
            retreat_y = place_loc['y'] - approach_dist * math.sin(place_loc['theta'])
            if not self.navigate_to_pose(retreat_x, retreat_y, place_loc['theta']):
                return False
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    mission = PickPlaceMission()
    mission.load_mission_config()
    
    try:
        rclpy.spin(mission)
    except KeyboardInterrupt:
        pass
    finally:
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
