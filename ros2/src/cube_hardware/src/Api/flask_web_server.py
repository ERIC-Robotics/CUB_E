#!/usr/bin/python3

from flask import Flask, jsonify
from flask_socketio import SocketIO
import threading
import battery_sub
import time

app = Flask(__name__)
socketio = SocketIO(app)

battery_data = {'voltage': 0.0}

def run_ros_node():
    battery_sub.main()

@app.route('/battery_voltage', methods=['GET'])
def get_battery_voltage():
    return jsonify(battery_data)

@socketio.on('connect')
def handle_connect():
    socketio.emit('battery_update', battery_data)

def update_battery_data():
    while True:
        battery_data['voltage'] = battery_sub.battery_subscriber.get_voltage()
        socketio.emit('battery_update', battery_data)
        socketio.sleep(1)

if __name__ == '__main__':
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.start()
    time.sleep(1)
    battery_update_thread = threading.Thread(target=update_battery_data)
    battery_update_thread.start()
    socketio.run(app)
