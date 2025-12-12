import re
import ast

class ROSParser:
    @staticmethod
    def parse_python_file(filepath):
        """Parse Python file for ROS elements"""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
        except:
            return None
        
        elements = {
            "publishers": [],
            "subscribers": [],
            "services": [],
            "init_node": False,
            "imports": [],
            "node_name": None
        }
        
        # Check imports
        if 'import rospy' in content or 'from rospy' in content:
            elements["imports"].append("rospy")
        if 'import rclpy' in content or 'from rclpy' in content:
            elements["imports"].append("rclpy")
        
        # Find node initialization (ROS1)
        if 'rospy.init_node' in content:
            elements["init_node"] = True
            match = re.search(r'rospy\.init_node\([^,)]*[\'"]([^\'"]+)[\'"]', content)
            if match:
                elements["node_name"] = match.group(1)
        
        # Find node initialization (ROS2)
        if 'rclpy.init' in content:
            elements["init_node"] = True
        
        # Find publishers
        pub_patterns = [
            r'(\w+)\s*=\s*rospy\.Publisher\([\'"]([^\'"]+)[\'"]',
            r'create_publisher\([^,]+,\s*[\'"]([^\'"]+)[\'"]'
        ]
        
        for pattern in pub_patterns:
            matches = re.findall(pattern, content)
            for match in matches:
                if isinstance(match, tuple) and len(match) == 2:
                    elements["publishers"].append({
                        "variable": match[0],
                        "topic": match[1]
                    })
                elif isinstance(match, str):
                    elements["publishers"].append({"topic": match})
        
        # Find subscribers
        sub_patterns = [
            r'rospy\.Subscriber\([\'"]([^\'"]+)[\'"]',
            r'create_subscription\([^,]+,\s*[\'"]([^\'"]+)[\'"]'
        ]
        
        for pattern in sub_patterns:
            matches = re.findall(pattern, content)
            for topic in matches:
                if topic:
                    elements["subscribers"].append({"topic": topic})
        
        # Find services
        service_patterns = [
            r'rospy\.Service\([\'"]([^\'"]+)[\'"]',
            r'create_service\([^,]+,\s*[\'"]([^\'"]+)[\'"]'
        ]
        
        for pattern in service_patterns:
            matches = re.findall(pattern, content)
            for service in matches:
                if service:
                    elements["services"].append({"name": service})
        
        return elements