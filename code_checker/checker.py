import os
import zipfile
import tempfile
import subprocess
import json
import re
import shutil
from pathlib import Path

class ROSCodeChecker:
    def __init__(self):
        self.results = {
            "errors": [],
            "warnings": [],
            "info": [],
            "statistics": {
                "files_checked": 0,
                "python_files": 0,
                "cpp_files": 0,
                "ros_elements": {
                    "init_node": False,
                    "publishers": 0,
                    "subscribers": 0,
                    "services": 0
                }
            }
        }
    
    def validate_upload(self, zip_path):
        """Main validation entry point"""
        print(f"  Extracting and analyzing package...")
        
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                # Extract ZIP
                if not self._extract_zip(zip_path, tmp_dir):
                    self.results["errors"].append("Failed to extract ZIP file")
                    return self.results
                
                # Run all checks
                self._check_ros_structure(tmp_dir)
                self._check_syntax(tmp_dir)
                self._check_safety(tmp_dir)
                self._check_ros_elements(tmp_dir)
                
            return self.results
            
        except Exception as e:
            self.results["errors"].append(f"Validation failed: {str(e)}")
            return self.results
    
    def _extract_zip(self, zip_path, extract_to):
        try:
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall(extract_to)
                extracted_files = len(zip_ref.namelist())
                self.results["info"].append(f"Extracted {extracted_files} files")
                return True
        except zipfile.BadZipFile:
            self.results["errors"].append("❌ Invalid ZIP file format")
            return False
        except Exception as e:
            self.results["errors"].append(f"❌ Failed to extract ZIP: {str(e)}")
            return False
    
    def _check_ros_structure(self, dir_path):
        """Check ROS package structure"""
        path = Path(dir_path)
        
        # Look for package.xml
        package_files = list(path.rglob("package.xml"))
        if not package_files:
            self.results["errors"].append("❌ No ROS package.xml found")
        else:
            for pkg in package_files[:3]:  # Check first 3
                rel_path = str(pkg.relative_to(path))
                self.results["info"].append(f"✅ Found package.xml at {rel_path}")
                
                # Try to read package name
                try:
                    with open(pkg, 'r', encoding='utf-8') as f:
                        content = f.read()
                        # Simple XML parsing for name
                        name_match = re.search(r'<name>([^<]+)</name>', content)
                        if name_match:
                            pkg_name = name_match.group(1).strip()
                            self.results["info"].append(f"  Package name: {pkg_name}")
                except:
                    pass
        
        # Check for build files
        cmake_files = list(path.rglob("CMakeLists.txt"))
        setup_files = list(path.rglob("setup.py"))
        
        if not cmake_files and not setup_files:
            self.results["warnings"].append("⚠️ No build configuration (CMakeLists.txt or setup.py) found")
        else:
            if cmake_files:
                self.results["info"].append(f"✅ Found {len(cmake_files)} CMakeLists.txt file(s)")
            if setup_files:
                self.results["info"].append(f"✅ Found {len(setup_files)} setup.py file(s)")
    
    def _check_syntax(self, dir_path):
        """Check syntax of Python and C++ files"""
        path = Path(dir_path)
        
        # Python files
        py_files = list(path.rglob("*.py"))
        self.results["statistics"]["python_files"] = len(py_files)
        
        for py_file in py_files[:10]:  # Check first 10 Python files
            self.results["statistics"]["files_checked"] += 1
            rel_path = str(py_file.relative_to(path))
            
            try:
                # Basic Python syntax check using py_compile
                result = subprocess.run(
                    ["python", "-m", "py_compile", str(py_file)],
                    capture_output=True,
                    text=True,
                    timeout=5,
                    shell=True
                )
                
                if result.returncode != 0:
                    error_msg = result.stderr.split('\n')[0] if result.stderr else "Unknown syntax error"
                    self.results["errors"].append(f"❌ Python syntax error in {rel_path}: {error_msg[:100]}")
                else:
                    # Try flake8 for style if available
                    try:
                        flake8_result = subprocess.run(
                            ["python", "-m", "flake8", "--select=E", str(py_file)],
                            capture_output=True,
                            text=True,
                            timeout=5,
                            shell=True
                        )
                        if flake8_result.stdout:
                            warnings = flake8_result.stdout.split('\n')
                            for warn in warnings[:2]:  # Show first 2 warnings
                                if warn and len(warn) > 10:
                                    self.results["warnings"].append(f"⚠️ Style issue in {rel_path}: {warn[:150]}")
                    except:
                        pass  # Flake8 not available or failed
                        
            except subprocess.TimeoutExpired:
                self.results["warnings"].append(f"⚠️ Timeout checking {rel_path}")
            except Exception as e:
                self.results["errors"].append(f"❌ Failed to check {rel_path}: {str(e)[:100]}")
        
        # C++ files
        cpp_exts = ['*.cpp', '*.cxx', '*.cc', '*.c++']
        cpp_files = []
        for ext in cpp_exts:
            cpp_files.extend(list(path.rglob(ext)))
        
        self.results["statistics"]["cpp_files"] = len(cpp_files)
        
        for cpp_file in cpp_files[:5]:  # Check first 5 C++ files
            self.results["statistics"]["files_checked"] += 1
            rel_path = str(cpp_file.relative_to(path))
            
            try:
                # Simple check: see if file can be opened and contains C++ keywords
                with open(cpp_file, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                    if '#include' in content and ('int main' in content or 'void main' in content):
                        self.results["info"].append(f"✅ Found C++ main function in {rel_path}")
            except:
                self.results["warnings"].append(f"⚠️ Could not read C++ file: {rel_path}")
    
    def _check_safety(self, dir_path):
        """Basic safety checks for robotic code"""
        path = Path(dir_path)
        
        # Check Python files for safety issues
        py_files = list(path.rglob("*.py"))
        
        for py_file in py_files[:10]:  # Check first 10 files
            try:
                with open(py_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    rel_path = str(py_file.relative_to(path))
                    
                    # Check for hardcoded joint values
                    joint_patterns = [
                        r'joint.*=\s*\[[^\]]*([0-9]+(?:\.[0-9]+)?)',
                        r'position.*=\s*\[[^\]]*([0-9]+(?:\.[0-9]+)?)',
                        r'angle.*=\s*([0-9]+(?:\.[0-9]+)?)'
                    ]
                    
                    for pattern in joint_patterns:
                        matches = re.finditer(pattern, content, re.IGNORECASE)
                        for match in matches:
                            try:
                                # Try to extract numbers
                                import re as re2
                                numbers = re2.findall(r'[-+]?\d*\.\d+|\d+', match.group(0))
                                for num_str in numbers[:3]:  # Check first 3 numbers
                                    num = float(num_str)
                                    if abs(num) > 3.2:  # Safe range ±π (3.14)
                                        self.results["warnings"].append(f"⚠️ Potential unsafe value ({num}) in {rel_path}")
                            except:
                                pass
                    
                    # Check for improper sleep in loops
                    if 'while' in content or 'for ' in content:
                        if 'time.sleep(' in content and not ('rospy.sleep' in content or 'rclpy.sleep' in content):
                            self.results["warnings"].append(f"⚠️ Use rospy.sleep() instead of time.sleep() in {rel_path}")
                    
                    # Check for infinite loops without break
                    if 'while True:' in content:
                        lines = content.split('while True:')
                        if len(lines) > 1:
                            after_while = lines[1][:500]  # Look at 500 chars after while
                            if 'break' not in after_while and 'return' not in after_while:
                                self.results["warnings"].append(f"⚠️ Potential infinite loop in {rel_path}")
                    
                    # Check for print statements (should use rospy.log)
                    if 'print(' in content and not 'rospy.log' in content:
                        self.results["info"].append(f"ℹ️ Consider using rospy.log*() instead of print() in {rel_path}")
                        
            except Exception as e:
                self.results["warnings"].append(f"⚠️ Could not analyze safety of {py_file.name}: {str(e)[:50]}")
    
    def _check_ros_elements(self, dir_path):
        """Detect ROS elements in Python code"""
        path = Path(dir_path)
        py_files = list(path.rglob("*.py"))
        
        for py_file in py_files[:10]:  # Check first 10 files
            try:
                with open(py_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    rel_path = str(py_file.relative_to(path))
                    
                    # ROS1 initialization
                    if 'rospy.init_node' in content:
                        self.results["statistics"]["ros_elements"]["init_node"] = True
                        self.results["info"].append(f"✅ ROS1 node initialization found in {rel_path}")
                        
                        # Try to extract node name
                        match = re.search(r'rospy\.init_node\([^,)]*[\'"]([^\'"]+)[\'"]', content)
                        if match:
                            self.results["info"].append(f"  Node name: {match.group(1)}")
                    
                    # ROS2 initialization
                    if 'rclpy.init' in content:
                        self.results["statistics"]["ros_elements"]["init_node"] = True
                        self.results["info"].append(f"✅ ROS2 node initialization found in {rel_path}")
                    
                    # Publishers
                    pub_matches = re.findall(r'rospy\.Publisher|create_publisher', content)
                    if pub_matches:
                        count = len(pub_matches)
                        self.results["statistics"]["ros_elements"]["publishers"] += count
                        self.results["info"].append(f"✅ Found {count} publisher(s) in {rel_path}")
                    
                    # Subscribers
                    sub_matches = re.findall(r'rospy\.Subscriber|create_subscription', content)
                    if sub_matches:
                        count = len(sub_matches)
                        self.results["statistics"]["ros_elements"]["subscribers"] += count
                        self.results["info"].append(f"✅ Found {count} subscriber(s) in {rel_path}")
                    
                    # Services
                    service_matches = re.findall(r'rospy\.Service|create_service', content)
                    if service_matches:
                        count = len(service_matches)
                        self.results["statistics"]["ros_elements"]["services"] += count
                        self.results["info"].append(f"✅ Found {count} service(s) in {rel_path}")
                        
            except:
                pass  # Skip files we can't read
    
    def generate_report(self, results):
        """Generate JSON and text reports"""
        # JSON report
        json_report = json.dumps(results, indent=2)
        
        # Text report
        text_report = "📋 ROS CODE CHECKER REPORT\n"
        text_report += "=" * 60 + "\n\n"
        
        # Summary
        text_report += "SUMMARY:\n"
        text_report += f"  Files checked: {results['statistics']['files_checked']}\n"
        text_report += f"  Python files: {results['statistics']['python_files']}\n"
        text_report += f"  C++ files: {results['statistics']['cpp_files']}\n"
        text_report += f"  Errors: {len(results['errors'])}\n"
        text_report += f"  Warnings: {len(results['warnings'])}\n"
        text_report += f"  Info messages: {len(results['info'])}\n\n"
        
        # Errors
        if results["errors"]:
            text_report += "❌ ERRORS:\n"
            for i, error in enumerate(results["errors"][:5], 1):
                text_report += f"  {i}. {error}\n"
            if len(results["errors"]) > 5:
                text_report += f"  ... and {len(results['errors']) - 5} more errors\n"
            text_report += "\n"
        
        # Warnings
        if results["warnings"]:
            text_report += "⚠️ WARNINGS:\n"
            for i, warning in enumerate(results["warnings"][:5], 1):
                text_report += f"  {i}. {warning}\n"
            if len(results["warnings"]) > 5:
                text_report += f"  ... and {len(results['warnings']) - 5} more warnings\n"
            text_report += "\n"
        
        # Info
        if results["info"]:
            text_report += "ℹ️ INFO & RECOMMENDATIONS:\n"
            for i, info in enumerate(results["info"][:10], 1):
                text_report += f"  {i}. {info}\n"
            if len(results["info"]) > 10:
                text_report += f"  ... and {len(results['info']) - 10} more info messages\n"
            text_report += "\n"
        
        # ROS Elements found
        ros_elements = results["statistics"]["ros_elements"]
        if ros_elements["init_node"] or ros_elements["publishers"] > 0:
            text_report += "🤖 ROS ELEMENTS DETECTED:\n"
            if ros_elements["init_node"]:
                text_report += "  • Node initialization: YES\n"
            if ros_elements["publishers"] > 0:
                text_report += f"  • Publishers: {ros_elements['publishers']}\n"
            if ros_elements["subscribers"] > 0:
                text_report += f"  • Subscribers: {ros_elements['subscribers']}\n"
            if ros_elements["services"] > 0:
                text_report += f"  • Services: {ros_elements['services']}\n"
            text_report += "\n"
        
        # Final verdict
        if not results["errors"]:
            text_report += "✅ VALIDATION PASSED - Ready for simulation!\n"
            text_report += "   No critical errors found. Check warnings for improvements.\n"
        else:
            text_report += "❌ VALIDATION FAILED\n"
            text_report += f"   Found {len(results['errors'])} critical error(s). Fix before simulation.\n"
        
        text_report += "\n" + "=" * 60 + "\n"
        
        return json_report, text_report