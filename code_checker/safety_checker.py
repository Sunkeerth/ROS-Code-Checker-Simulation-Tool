import re

class SafetyChecker:
    @staticmethod
    def check_file(filepath):
        """Check a single file for safety issues"""
        issues = []
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
                
                # Check for joint values
                joint_patterns = [
                    r'joint.*=\s*\[[^\]]*([0-9]+(?:\.[0-9]+)?)',
                    r'position.*=\s*\[[^\]]*([0-9]+(?:\.[0-9]+)?)',
                    r'angle.*=\s*([0-9]+(?:\.[0-9]+)?)'
                ]
                
                for pattern in joint_patterns:
                    matches = re.finditer(pattern, content, re.IGNORECASE)
                    for match in matches:
                        # Extract numbers from the match
                        numbers = re.findall(r'[-+]?\d*\.\d+|\d+', match.group(0))
                        for num_str in numbers[:5]:  # Check first 5 numbers
                            try:
                                num = float(num_str)
                                if abs(num) > 3.2:  # Approximately π
                                    issues.append(f"Unsafe joint/position value: {num} (should be within ±3.14 rad)")
                            except:
                                pass
                
                # Check for improper sleep in loops
                if ('while' in content or 'for ' in content) and 'time.sleep(' in content:
                    if 'rospy.sleep' not in content and 'rclpy.sleep' not in content:
                        issues.append("Use rospy.sleep() or rclpy.sleep() instead of time.sleep() in loops")
                
                # Check for infinite loops
                if 'while True:' in content:
                    # Simple check: see if there's a break or return after while True
                    lines = content.split('while True:')
                    if len(lines) > 1:
                        after_while = lines[1][:300]  # Look at first 300 chars
                        if 'break' not in after_while and 'return' not in after_while:
                            issues.append("Potential infinite loop detected (while True: without break/return)")
                
                return issues
                
        except Exception as e:
            return [f"Could not analyze file: {str(e)[:100]}"]