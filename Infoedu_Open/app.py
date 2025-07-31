from flask import Flask, render_template, request, send_file, jsonify
import serial
import numpy as np
import pyvista as pv
import threading
import os

app = Flask(__name__)

# Absolute path for scan.stl (in same folder as app.py)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
mesh_file_path = os.path.join(BASE_DIR, "scan.stl")

last_mesh = None  # global to reuse for STL export


def read_3d_points_from_serial(port="COM7", baudrate=115200):
    points = []

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"[INFO] Connected to {port} at {baudrate} baud.")
        print("[INFO] Reading points... Send 'END' to finish and generate mesh.")

        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                print("[DATA] Received:", line)
                if line.upper() == "END":
                    print("[INFO] End signal received. Stopping data collection.")
                    break
                try:
                    x, y, z = map(float, line.split(','))
                    points.append([x, y, z])
                except ValueError:
                    print("[WARN] Invalid point format, skipping line.")

    except serial.SerialException as e:
        print(f"[ERROR] Serial port error: {e}")
    finally:
        ser.close()

    return np.array(points)


def create_mesh_from_points(points):
    if len(points) < 4:
        print("[WARN] Not enough points to create a 3D mesh.")
        return None

    cloud = pv.PolyData(points)
    try:
        mesh = cloud.delaunay_3d()
        surface = mesh.extract_surface()
        return surface
    except Exception as e:
        print("[ERROR] Mesh generation failed:", e)
        return None


def visualize_mesh(mesh):
    if mesh is None:
        print("[WARN] No mesh to display.")
        return

    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color='lightblue', show_edges=True)
    plotter.show()


def run_scan():
    global last_mesh

    # Delete previous STL if exists
    if os.path.exists(mesh_file_path):
        os.remove(mesh_file_path)

    points = read_3d_points_from_serial()
    if points.size == 0:
        print("[WARN] No points collected.")
    else:
        mesh = create_mesh_from_points(points)
        if mesh:
            last_mesh = mesh
            print(f"[INFO] Saving STL to: {mesh_file_path}")
            mesh.save(mesh_file_path)
            visualize_mesh(mesh)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/start_scan', methods=['POST'])
def start_scan():
    threading.Thread(target=run_scan).start()
    return "Scan started."


@app.route('/download_stl')
def download():
    if os.path.exists(mesh_file_path):
        return send_file(mesh_file_path, as_attachment=True)
    return "No STL file found. Please run a scan first."


@app.route('/scan_status')
def scan_status():
    return jsonify({"ready": os.path.exists(mesh_file_path)})


if __name__ == "__main__":
    app.run(debug=True)
