#ROS Code Checker & Simulation Preview Tool :

A lightweight tool that lets users upload ROS packages, check code quality, and preview a simple robotic arm simulation.
Built for students, interns, and developers who want a fast way to test ROS nodes without installing full robot simulators.

#Features :

✅ No ROS Installation Required - Works on any system with Python 3.7+.
✅ Static Code Analysis - Detects common ROS coding errors and safety issues
✅ Style Validation - Checks against ROS C++ and Python style guides
✅ Safety Checks - Identifies unsafe practices like unvalidated joint limits
✅ Web Interface - User-friendly GUI for easy code analysis
✅ Batch Processing - Analyze entire ROS packages or single files

Installation
bash
# 1. Clone the repository
cd ros_code_checker_tool

# 2. Create virtual environment (recommended)
python -m venv venv

# 3. Activate virtual environment
 On Windows:
venv\Scripts\activate
 On Linux/Mac:
source venv/bin/activate

# 4 . Run the Tool 
python run.py --web 

# Architecture Diagram :

                +---------------------------+
                |        Web Interface      |
                |       (Flask / HTML)      |
                +--------------+------------+
                               |
                               | Upload ZIP
                               v
                +---------------------------+
                |         Backend API       |
                |         (run.py)          |
                +--------------+------------+
                               |
                               v
        +------------------------------------------------+
        |                  Code Checker                  |
        |------------------------------------------------|
        | Syntax Check | Structure Check | Safety Rules  |
        | flake8/g++   | pkg.xml, CMake  | joint limits  |
        +------+-------+---------+-------+----------------+
               |                 |
               | Valid           | Errors/Warnings
               v                 v
     +------------------+      +----------------------+
     | Simulation Runner|      |    Report Generator  |
     |   (Mock UR5)     |      |   TXT + JSON Output  |
     +--------+---------+      +----------+-----------+
              |                           |
              | Frames + Logs             | Reports
              v                           v
     +-----------------------+    +--------------------------+
     |  Simulation Preview   |    |  report.txt / report.json|
     +-----------------------+    +--------------------------+

Test Packages Included  :

✔ correct_package

This package:
Has proper init_node
Contains safe joint values
Publishes valid motion commands
Successfully completes the mock simulation

❌ faulty_package

This package demonstrates:
Missing init_node
Infinite loop without sleep
Incorrect ROS file structure
Unsafe joint values 

Outputs

report.txt
report.json
Simulation logs + image frames


