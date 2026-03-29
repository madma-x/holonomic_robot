#!/usr/bin/env python3
"""Patrol mission - autonomous waypoint navigation."""

import rclpy
from robot_application.mission_base import MissionBase, MissionState
import time
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class PatrolMission(MissionBase):
    """Patrol mission: navigate through predefined waypoints."""
    
    def __init__(self):
        super().__init__('patrol')
        self.waypoints = []
        self.pause_duration = 2.0
        self.num_cycles = -1  # -1 = infinite
        self.current_cycle = 0
        self.current_waypoint_idx = 0
    
    def load_mission_config(self):
        """Load patrol waypoints from config file."""
        try:
            config_dir = get_package_share_directory('robot_application')
            config_file = os.path.join(config_dir, 'config', 'patrol_waypoints.yaml')
            
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            
            self.waypoints = config['patrol_waypoints']
            self.pause_duration = config.get('pause_duration_sec', 2.0)
            self.num_cycles = config.get('num_cycles', -1)
            
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            self.get_logger().info(f'Pause duration: {self.pause_duration}s')
            self.get_logger().info(f'Cycles: {"infinite" if self.num_cycles < 0 else self.num_cycles}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {str(e)}')
            # Use default waypoints
            self.waypoints = [
                {'x': 1.0, 'y': 0.0, 'theta': 0.0},
                {'x': 1.0, 'y': 1.0, 'theta': 1.57},
                {'x': 0.0, 'y': 1.0, 'theta': 3.14},
                {'x': 0.0, 'y': 0.0, 'theta': 0.0}
            ]
            self.get_logger().info('Using default waypoints')
    
    def execute_mission(self):
        """Execute patrol mission."""
        self.get_logger().info('Starting patrol mission')
        self.current_cycle = 0
        
        try:
            while not self.stop_requested:
                # Check if we've completed all cycles
                if self.num_cycles > 0 and self.current_cycle >= self.num_cycles:
                    self.get_logger().info(f'Completed {self.current_cycle} patrol cycles')
                    self.state = MissionState.COMPLETED
                    self.progress = 100.0
                    break
                
                # Navigate through waypoints
                for idx, waypoint in enumerate(self.waypoints):
                    if self.stop_requested:
                        break
                    
                    # Handle pause
                    while self.pause_requested and not self.stop_requested:
                        time.sleep(0.1)
                    
                    self.current_waypoint_idx = idx
                    
                    # Calculate progress
                    total_steps = len(self.waypoints) * (self.num_cycles if self.num_cycles > 0 else 1)
                    completed_steps = self.current_cycle * len(self.waypoints) + idx
                    self.progress = (completed_steps / total_steps) * 100.0
                    
                    self.get_logger().info(
                        f'Cycle {self.current_cycle + 1}, Waypoint {idx + 1}/{len(self.waypoints)}'
                    )
                    
                    # Navigate to waypoint
                    success = self.navigate_to_pose(
                        waypoint['x'], 
                        waypoint['y'], 
                        waypoint['theta']
                    )
                    
                    if not success:
                        if self.enable_recovery:
                            self.get_logger().warn('Navigation failed, attempting recovery')
                            if self.recover_navigation(waypoint):
                                success = True
                        
                        if not success:
                            self.get_logger().error('Navigation failed, aborting mission')
                            self.state = MissionState.FAILED
                            return
                    
                    # Pause at waypoint
                    self.get_logger().info(f'Pausing for {self.pause_duration}s')
                    time.sleep(self.pause_duration)
                
                self.current_cycle += 1
                
                # For infinite cycles, reset progress tracking
                if self.num_cycles < 0:
                    self.progress = 0.0
            
            if self.stop_requested:
                self.get_logger().info('Patrol mission stopped by user')
                self.state = MissionState.IDLE
            
        except Exception as e:
            self.get_logger().error(f'Mission error: {str(e)}')
            self.state = MissionState.FAILED
    
    def recover_navigation(self, waypoint: dict) -> bool:
        """Attempt to recover from navigation failure."""
        self.state = MissionState.RECOVERING
        
        for attempt in range(self.max_recovery_attempts):
            self.get_logger().info(f'Recovery attempt {attempt + 1}/{self.max_recovery_attempts}')
            
            # Strategy 1: Rotate in place to improve localization
            self.get_logger().info('Rotating to improve localization')
            current_theta = waypoint['theta']
            for angle_offset in [1.57, 3.14, -1.57]:  # 90°, 180°, -90°
                recovery_theta = current_theta + angle_offset
                if self.navigate_to_pose(waypoint['x'], waypoint['y'], recovery_theta):
                    time.sleep(1.0)
                    break
            
            # Strategy 2: Retry navigation
            if self.navigate_to_pose(waypoint['x'], waypoint['y'], waypoint['theta']):
                self.get_logger().info('Recovery successful')
                self.state = MissionState.RUNNING
                return True
            
            time.sleep(2.0)
        
        self.get_logger().error('Recovery failed')
        self.state = MissionState.RUNNING
        return False


def main(args=None):
    rclpy.init(args=args)
    
    mission = PatrolMission()
    
    # Load mission configuration on startup
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
