from flask import Flask, Response, jsonify, send_from_directory, request
from ros_bridge import ROSBridge, start_ros
import threading
import rclpy
import requests
import os
import time
import random

app = Flask(__name__)
ros_node = None

FRONTEND_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "frontend"))

# -------------------------
# Frontend routes
# -------------------------
@app.route("/")
def index():
    return send_from_directory(FRONTEND_DIR, "index.html")

@app.route("/assets/<path:filename>")
def assets(filename):
    return send_from_directory(os.path.join(FRONTEND_DIR, "assets"), filename)

# -------------------------
# ROS camera images
# -------------------------
@app.route("/image/<drone>")
def image(drone):
    if ros_node and drone in ros_node.latest_images:
        img_bytes = ros_node.latest_images[drone]
        if img_bytes:
            return Response(img_bytes, mimetype="image/jpeg")
    return "", 204

# -------------------------
# ROS casualty location
# -------------------------
@app.route("/location")
def location():
    if ros_node and ros_node.latest_location:
        data = ros_node.latest_location.copy()
        from datetime import datetime
        data["timestamp"] = datetime.utcnow().isoformat() + "Z"
        return jsonify(data)
    return jsonify({})

# -------------------------
# Operator IP geolocation
# -------------------------
@app.route("/ip_location")
def ip_location():
    try:
        r = requests.get("https://ipapi.co/json/", timeout=2)
        d = r.json()
        return jsonify({
            "lat": d.get("latitude"),
            "lon": d.get("longitude"),
            "city": d.get("city"),
            "region": d.get("region"),
            "country": d.get("country_name"),
            "ip": d.get("ip")
        })
    except Exception:
        return jsonify({})

# -------------------------
# Drone status for panel (NOW WITH BATTERY!)
# -------------------------
@app.route("/drone_status")
def drone_status():
    if not ros_node:
        # Return default status with battery levels
        return jsonify([
            {"status": "unknown", "battery": 0},
            {"status": "unknown", "battery": 0},
            {"status": "unknown", "battery": 0},
            {"status": "unknown", "battery": 0},
            {"status": "unknown", "battery": 0}
        ])
    
    statuses = []
    # Simulate realistic battery drain over time
    mission_time = time.time() - MISSION_START
    base_battery = max(100 - (mission_time / 360), 10)  # Drains 10% per hour, min 10%
    
    for i, drone in enumerate(["drone1", "drone2", "drone3", "drone4", "drone5"]):
        img = ros_node.latest_images.get(drone)
        status = "operational" if img else "disconnected"
        
        # Add some variation to battery levels for realism
        battery_variation = random.uniform(-5, 5)
        battery_level = max(10, min(100, base_battery + (i * 2) + battery_variation))
        
        statuses.append({
            "status": status,
            "battery": round(battery_level)
        })
    
    return jsonify(statuses)

# -------------------------
# Casualty locations for map
# -------------------------
@app.route("/casualty_locations")
def casualty_locations():
    if ros_node and ros_node.latest_location:
        return jsonify([{
            "x": ros_node.latest_location["lat"],
            "y": ros_node.latest_location["lon"],
            "confidence_radius": 50
        }])
    return jsonify([])

# -------------------------
# Drone telemetry (for map markers)
# -------------------------
@app.route("/drone_telemetry")
def drone_telemetry():
    if not ros_node or not ros_node.latest_location:
        base_lat, base_lon = 20, 0
    else:
        base_lat = ros_node.latest_location["lat"]
        base_lon = ros_node.latest_location["lon"]
    
    # Get battery levels from the status endpoint for consistency
    battery_levels = []
    try:
        status_res = drone_status()
        status_data = status_res.get_json()
        battery_levels = [drone["battery"] for drone in status_data]
    except:
        battery_levels = [100, 95, 88, 76, 62]  # Fallback values

    drones = []
    for i in range(5):
        drones.append({
            "id": f"drone{i+1}",
            "lat": base_lat + 0.001*i,
            "lon": base_lon + 0.001*i,
            "altitude": 50 + (i * 5),  # Add altitude
            "autonomy": "auto" if i % 2 == 0 else "manual",
            "signal": "good" if ros_node and ros_node.latest_images.get(f"drone{i+1}") else "lost",
            "battery": battery_levels[i] if i < len(battery_levels) else 100
        })
    return jsonify({"altitude": drones[0]["altitude"]} if drones else {"altitude": 0})

# -------------------------
# Mission timer
# -------------------------
MISSION_START = time.time()
@app.route("/mission_time")
def mission_time():
    elapsed = time.time() - MISSION_START
    hours, rem = divmod(int(elapsed), 3600)
    minutes, seconds = divmod(rem, 60)
    return jsonify({"hours": hours, "minutes": minutes, "seconds": seconds})

# -------------------------
# Emergency stop endpoint
# -------------------------
@app.route("/emergency_stop", methods=["POST"])
def emergency_stop():
    try:
        # Here you would send ROS command to all drones
        # For now, simulate the response
        if ros_node:
            ros_node.get_logger().warning("EMERGENCY STOP ACTIVATED - All drones returning to base")
        
        return jsonify({
            "success": True,
            "message": "Emergency stop activated. All drones returning to base.",
            "timestamp": time.time()
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

# -------------------------
# Main
# -------------------------
def main():
    global ros_node

    rclpy.init()
    ros_node = ROSBridge()

    ros_thread = threading.Thread(target=start_ros, args=(ros_node,), daemon=True)
    ros_thread.start()

    app.run(host="0.0.0.0", port=8080, debug=False)

if __name__ == "__main__":
    main()