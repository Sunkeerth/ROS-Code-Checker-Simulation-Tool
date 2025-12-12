#!/usr/bin/env python3
"""
FAULTY ROS PACKAGE WITH INTENTIONAL ERRORS
This package should fail validation checks
"""

import time  # WRONG: Should use rospy.sleep, not time.sleep

# MISSING ROS NODE INITIALIZATION - ERROR
# rospy.init_node('unsafe_node')  # This line is commented out intentionally

def dangerous_function():
    """Function with multiple safety violations"""
    
    # UNSAFE: Hardcoded joint values beyond limits - WARNING
    joint_positions = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]  # 10 rad is UNSAFE!
    
    # NO SAFETY CHECKS - WARNING
    # Missing validation of joint limits
    
    # DANGEROUS: Infinite loop without proper exit - WARNING
    counter = 0
    while True:  # INFINITE LOOP - no break condition
        # Using blocking sleep instead of ROS sleep - WARNING
        time.sleep(0.1)  # BLOCKING SLEEP - should use rospy.sleep
        
        # Hardcoded dangerous values
        position = [100.0, 200.0, 300.0]  # EXTREME VALUES
        
        counter += 1
        
        # Missing break condition
        if counter > 1000:
            pass  # Should have break or return here
    
    # UNREACHABLE CODE - WARNING
    print("This code is never reached")

# MISSING MAIN GUARD PROPERLY
dangerous_function()

# MULTIPLE STATEMENTS ON ONE LINE (bad style) - WARNING
x=1; y=2; z=x+y  # BAD STYLE

# UNUSED VARIABLES - WARNING
unused_variable = "This variable is never used"

# TOO LONG LINE THAT EXCEEDS TYPICAL STYLE GUIDES - WARNING
this_is_a_very_long_variable_name_that_exceeds_the_recommended_line_length_of_79_characters_by_quite_a_lot = "yes"

# MISSING ERROR HANDLING
result = 1 / 0  # POTENTIAL DIVISION BY ZERO - WARNING

print("This won't execute due to above error")