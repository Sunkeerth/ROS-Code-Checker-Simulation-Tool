#!/usr/bin/env python3
"""
Simple correct ROS-like package
No ROS imports needed
"""

import math
import time

class SimpleRobotController:
    def __init__(self):
        print("Robot controller initialized")
        self.joint_limits = [-3.14, 3.14]  # Safe limits
    
    def safe_move(self, joint_angles):
        """Safe movement with limits"""
        for angle in joint_angles:
            if angle < self.joint_limits[0] or angle > self.joint_limits[1]:
                print(f"Warning: Joint angle {angle} exceeds limits")
                return False
        
        print(f"Moving to: {joint_angles}")
        time.sleep(0.5)  # Simulate movement
        return True
    
    def pick_and_place(self):
        """Simple pick and place sequence"""
        positions = [
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
            [0.5, -1.2, 1.8, -1.0, -1.0, 0.5],
            [0.5, -1.0, 2.0, -0.8, -1.2, 0.5],
            [1.2, -0.8, 1.3, -0.6, -1.0, 1.2],
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        ]
        
        for pos in positions:
            self.safe_move(pos)
        
        print("Pick and place completed successfully")
        return True

if __name__ == "__main__":
    controller = SimpleRobotController()
    controller.pick_and_place()
    