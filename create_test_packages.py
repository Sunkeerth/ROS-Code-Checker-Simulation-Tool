import os
import zipfile

def create_test_packages():
    """Create test package ZIP files"""
    
    # Create correct package
    correct_files = {
        'package.xml': '''<?xml version="1.0"?>
<package format="2">
  <name>correct_package</name>
  <version>0.0.0</version>
  <description>Correct test package</description>
  <maintainer email="test@test.com">Test</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
</package>''',
        
        'CMakeLists.txt': '''cmake_minimum_required(VERSION 3.0.2)
project(correct_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

install(PROGRAMS src/simple_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)''',
        
        'src/simple_node.py': '''#!/usr/bin/env python3
"""
Simple correct package without ROS imports
"""

import math

def safe_function():
    # Proper code structure
    joint_angles = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    
    # Safe values within limits
    for angle in joint_angles:
        if abs(angle) > 3.14:
            return False
    
    # Proper loop with exit condition
    for i in range(10):
        print(f"Iteration {i}")
    
    return True

if __name__ == "__main__":
    safe_function()
    print("✅ Program completed successfully")
'''
    }
    
    # Create faulty package
    faulty_files = {
        'src/bad_node.py': '''#!/usr/bin/env python3
"""
Faulty package with errors
"""

import time  # Wrong import for ROS

# Missing ROS init
# rospy.init_node('bad_node')

joint = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]  # Unsafe value

while True:  # Infinite loop
    time.sleep(0.1)  # Blocking sleep
    # No break condition
'''
    }
    
    # Create ZIP files
    with zipfile.ZipFile('tests/correct_package.zip', 'w') as zipf:
        for file_path, content in correct_files.items():
            zipf.writestr(file_path, content)
    
    with zipfile.ZipFile('tests/faulty_package.zip', 'w') as zipf:
        for file_path, content in faulty_files.items():
            zipf.writestr(file_path, content)
    
    print("✅ Test packages created:")
    print("   - tests/correct_package.zip")
    print("   - tests/faulty_package.zip")

if __name__ == "__main__":
    create_test_packages()