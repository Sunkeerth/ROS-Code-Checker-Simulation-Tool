from flask import Flask, render_template, request, jsonify, send_from_directory
import os
import tempfile
import uuid
import json
import shutil
from werkzeug.utils import secure_filename
import sys

# Add parent directory to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from code_checker.checker import ROSCodeChecker
from simulation.simulator import SimulationRunner

# Initialize Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ros-intern-task-secret-key-2024'
app.config['UPLOAD_FOLDER'] = 'uploads'
app.config['MAX_CONTENT_LENGTH'] = 50 * 1024 * 1024  # 50MB max

# Create upload folder if it doesn't exist
os.makedirs(app.config['UPLOAD_FOLDER'], exist_ok=True)

# Initialize components
checker = ROSCodeChecker()
active_simulations = {}

@app.route('/')
def index():
    """Serve the main web interface"""
    return render_template('index.html')

@app.route('/api/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        "status": "healthy",
        "service": "ROS Code Checker & Simulation Tool",
        "version": "1.0.0",
        "endpoints": {
            "upload": "/api/upload (POST)",
            "simulate": "/api/simulate (POST)",
            "results": "/api/results/<upload_id> (GET)",
            "cleanup": "/api/cleanup (POST)"
        }
    })

@app.route('/api/upload', methods=['POST'])
def upload_package():
    """Handle ROS package upload and validation"""
    if 'file' not in request.files:
        return jsonify({"success": False, "error": "No file uploaded"}), 400
    
    file = request.files['file']
    
    if file.filename == '':
        return jsonify({"success": False, "error": "No file selected"}), 400
    
    if not file.filename.endswith('.zip'):
        return jsonify({"success": False, "error": "File must be a ZIP archive"}), 400
    
    # Generate unique ID for this upload
    upload_id = str(uuid.uuid4())[:8]
    upload_dir = os.path.join(app.config['UPLOAD_FOLDER'], upload_id)
    os.makedirs(upload_dir, exist_ok=True)
    
    # Save uploaded file
    filename = secure_filename(file.filename)
    filepath = os.path.join(upload_dir, filename)
    file.save(filepath)
    
    print(f"[UPLOAD] Received: {filename} (ID: {upload_id})")
    
    try:
        # Validate the package
        results = checker.validate_upload(filepath)
        json_report, text_report = checker.generate_report(results)
        
        # Check validation status
        validation_passed = len(results["errors"]) == 0
        
        # Save results
        results_file = os.path.join(upload_dir, 'validation_results.json')
        with open(results_file, 'w', encoding='utf-8') as f:
            json.dump({
                "upload_id": upload_id,
                "filename": filename,
                "results": results,
                "text_report": text_report,
                "validation_passed": validation_passed,
                "timestamp": os.path.getctime(filepath)
            }, f, indent=2)
        
        response_data = {
            "success": True,
            "upload_id": upload_id,
            "filename": filename,
            "validation_passed": validation_passed,
            "error_count": len(results["errors"]),
            "warning_count": len(results["warnings"]),
            "info_count": len(results["info"]),
            "text_report": text_report,
            "json_report": json.loads(json_report)
        }
        
        print(f"[VALIDATION] {filename}: {len(results['errors'])} errors, {len(results['warnings'])} warnings")
        
        return jsonify(response_data)
        
    except Exception as e:
        error_msg = f"Validation error: {str(e)}"
        print(f"[ERROR] {error_msg}")
        return jsonify({"success": False, "error": error_msg}), 500

@app.route('/api/simulate', methods=['POST'])
def run_simulation():
    """Run simulation for a validated package"""
    data = request.json
    upload_id = data.get('upload_id')
    simulator_type = data.get('simulator', 'mock')  # Default to mock
    
    if not upload_id:
        return jsonify({"success": False, "error": "Missing upload_id"}), 400
    
    upload_dir = os.path.join(app.config['UPLOAD_FOLDER'], upload_id)
    if not os.path.exists(upload_dir):
        return jsonify({"success": False, "error": "Upload not found"}), 404
    
    # Check validation results
    results_file = os.path.join(upload_dir, 'validation_results.json')
    if not os.path.exists(results_file):
        return jsonify({"success": False, "error": "Package not validated"}), 400
    
    with open(results_file, 'r', encoding='utf-8') as f:
        validation_data = json.load(f)
    
    if not validation_data["validation_passed"]:
        return jsonify({
            "success": False,
            "error": "Package validation failed",
            "details": validation_data["results"]["errors"][:3]
        }), 400
    
    print(f"[SIMULATION] Starting for upload: {upload_id}")
    
    try:
        # Create simulation runner
        sim_runner = SimulationRunner(simulator=simulator_type)
        simulation_id = f"{upload_id}_sim_{int(time.time())}"
        
        # Store simulation runner
        active_simulations[simulation_id] = sim_runner
        
        # Run simulation
        success = sim_runner.launch_simulation()
        
        if success:
            # Get simulation report
            report = sim_runner.get_simulation_report()
            
            # Save report
            report_file = os.path.join(upload_dir, 'simulation_report.json')
            with open(report_file, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2)
            
            response_data = {
                "success": True,
                "simulation_id": simulation_id,
                "report": report,
                "message": "Simulation completed successfully"
            }
            
            print(f"[SIMULATION] Completed: {upload_id}")
            
            return jsonify(response_data)
        else:
            return jsonify({
                "success": False,
                "error": "Simulation failed to start",
                "details": sim_runner.simulation_data.get("errors", [])
            }), 500
            
    except Exception as e:
        error_msg = f"Simulation error: {str(e)}"
        print(f"[SIMULATION ERROR] {error_msg}")
        return jsonify({
            "success": False,
            "error": error_msg
        }), 500

@app.route('/api/results/<upload_id>', methods=['GET'])
def get_results(upload_id):
    """Get validation and simulation results"""
    upload_dir = os.path.join(app.config['UPLOAD_FOLDER'], upload_id)
    
    if not os.path.exists(upload_dir):
        return jsonify({"success": False, "error": "Results not found"}), 404
    
    results = {
        "upload_id": upload_id,
        "validation": None,
        "simulation": None
    }
    
    # Get validation results
    validation_file = os.path.join(upload_dir, 'validation_results.json')
    if os.path.exists(validation_file):
        with open(validation_file, 'r', encoding='utf-8') as f:
            results["validation"] = json.load(f)
    
    # Get simulation results
    simulation_file = os.path.join(upload_dir, 'simulation_report.json')
    if os.path.exists(simulation_file):
        with open(simulation_file, 'r', encoding='utf-8') as f:
            results["simulation"] = json.load(f)
    
    results["success"] = True
    return jsonify(results)

@app.route('/api/cleanup', methods=['POST'])
def cleanup_simulation():
    """Clean up simulation resources"""
    data = request.json
    simulation_id = data.get('simulation_id')
    
    if simulation_id and simulation_id in active_simulations:
        active_simulations[simulation_id].cleanup()
        del active_simulations[simulation_id]
    
    return jsonify({"success": True, "message": "Cleanup completed"})

@app.route('/api/demo', methods=['GET'])
def demo_data():
    """Provide demo data for testing"""
    return jsonify({
        "success": True,
        "demo": True,
        "message": "This is a demo of the ROS Code Checker API",
        "instructions": "Upload a ROS package ZIP file to get started",
        "sample_data": {
            "correct_package": {
                "errors": 0,
                "warnings": 2,
                "info": 8
            },
            "faulty_package": {
                "errors": 3,
                "warnings": 5,
                "info": 2
            }
        }
    })

@app.errorhandler(413)
def too_large(e):
    return jsonify({"success": False, "error": "File too large. Maximum size is 50MB"}), 413

@app.errorhandler(500)
def server_error(e):
    return jsonify({"success": False, "error": "Internal server error"}), 500

# Add time import at the top if needed
import time

if __name__ == '__main__':
    print("\n" + "="*60)
    print("🤖 ROS Code Checker & Simulation Tool - Web Interface")
    print("="*60)
    print(f"📁 Upload folder: {app.config['UPLOAD_FOLDER']}")
    print("🌐 Web interface: http://localhost:5000")
    print("📋 API endpoints:")
    print("   GET  /              - Web interface")
    print("   POST /api/upload    - Upload ROS package (ZIP)")
    print("   POST /api/simulate  - Run simulation")
    print("   GET  /api/results/<id> - Get results")
    print("   GET  /api/demo      - Demo data")
    print("="*60)
    print("🛑 Press Ctrl+C to stop the server")
    print("="*60 + "\n")
    
    app.run(debug=True, host='0.0.0.0', port=5000)