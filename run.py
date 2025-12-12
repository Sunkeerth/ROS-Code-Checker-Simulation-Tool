#!/usr/bin/env python3
import sys
import os

# Add current directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import argparse
from code_checker.checker import ROSCodeChecker
from simulation.simulator import SimulationRunner

def main():
    parser = argparse.ArgumentParser(description="ROS Code Checker & Simulator")
    parser.add_argument("--zip", help="Path to ROS package ZIP")
    parser.add_argument("--simulate", action="store_true", help="Run simulation")
    parser.add_argument("--web", action="store_true", help="Launch web interface")
    
    args = parser.parse_args()
    
    if args.web:
        from web_interface.app import app
        print("\n" + "="*60)
        print("🤖 ROS Code Checker & Simulation Tool")
        print("="*60)
        print("🌐 Web interface: http://localhost:5000")
        print("📁 Upload folder: ./uploads")
        print("🛑 Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        # Create uploads directory if it doesn't exist
        os.makedirs("uploads", exist_ok=True)
        
        app.run(debug=True, host='0.0.0.0', port=5000)
    
    elif args.zip:
        if not os.path.exists(args.zip):
            print(f"❌ Error: File {args.zip} not found")
            return
        
        checker = ROSCodeChecker()
        print(f"\n🔍 Validating {args.zip}...")
        results = checker.validate_upload(args.zip)
        json_report, text_report = checker.generate_report(results)
        
        print("\n" + "="*60)
        print(text_report)
        print("="*60)
        
        if args.simulate and not results["errors"]:
            print("\n🚀 Launching simulation...")
            simulator = SimulationRunner()
            success = simulator.launch_simulation()
            
            if success:
                print("✅ Simulation completed successfully")
                simulator.cleanup()
            else:
                print("❌ Simulation failed or simulator not found")
                print("💡 Running mock simulation instead...")
                simulator._run_mock_simulation()
    
    else:
        print("Usage:")
        print("  python run.py --web                     # Launch web interface")
        print("  python run.py --zip package.zip         # Validate package")
        print("  python run.py --zip package.zip --simulate  # Validate and simulate")
        print("\nExample:")
        print("  python run.py --zip tests/correct_package.zip")
        print("  python run.py --zip tests/faulty_package.zip")
        print("  python run.py --web")

if __name__ == "__main__":
    main()