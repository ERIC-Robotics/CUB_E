#!/usr/bin/python3

from flask import Flask, jsonify
from flask_socketio import SocketIO
import threading
import cube_analysis
import time

app = Flask(__name__)
socketio = SocketIO(app)

analytics = {
    "Distance": 0, 
    "Velocity": 
        {
            "linear_velocity": 0.0, 
            "angular_velocity": 0.0
        },
    "Battery": 0,
    "Emergency_stop":
        {
            "software": 0,
            "hardware": 0
        },
    "Control_mode": ""
    }

def run_ros_node():
    cube_analysis.main()

@app.route("/cube_analytics", methods=["GET"])
def get_battery_voltage():
    return jsonify(analytics)


@socketio.on("connect")
def handle_connect():
    # socketio.emit("battery_update", battery_data)
    pass


def update_battery_data():
    while True:
        analytics["Distance"] = cube_analysis.cube_analytics.get_total_distance()
        analytics["Velocity"]["linear_velocity"] = cube_analysis.cube_analytics.get_linear_velocity()
        analytics["Velocity"]["angular_velocity"] = cube_analysis.cube_analytics.get_angular_velocity()
        analytics["Battery"] = cube_analysis.cube_analytics.get_battery_voltage()
        analytics["Emergency_stop"]["software"] = cube_analysis.cube_analytics.get_es_software()
        analytics["Emergency_stop"]["hardware"] = cube_analysis.cube_analytics.get_es_hardware()
        analytics["Control_mode"] = cube_analysis.cube_analytics.get_nav_feedback()
        # socketio.emit("battery_update", battery_data)

        socketio.sleep(1)


if __name__ == "__main__":
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.start()
    time.sleep(1)
    battery_update_thread = threading.Thread(target=update_battery_data)
    battery_update_thread.start()
    socketio.run(app)
