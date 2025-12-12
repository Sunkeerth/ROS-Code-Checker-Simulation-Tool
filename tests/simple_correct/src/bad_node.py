#!/usr/bin/env python3
"""
Faulty package with intentional errors
"""

import time

# Syntax errors and bad practices
joint = [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]  # Unsafe value

while True  # Missing colon - syntax error
    time.sleep(0.1)  # Blocking sleep
    # No break statement