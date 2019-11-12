#!api/bin/python
from flask import Flask, jsonify, request, abort, make_response

import rospy
import rospkg
import yaml
import robot_data 
from riptide_msgs.msg import ControlStatus
from riptide_msgs.msg import Depth
from riptide_msgs.msg import Dvl
from riptide_msgs.msg import Imu
from riptide_msgs.msg import Object
from riptide_msgs.msg import SwitchState
from darknet_ros_msgs.msg import BoundingBoxes
import os
import threading

from std_msgs.msg import String

app = Flask(__name__)

@app.route('/', methods=['POST'])
def respond():
    if not request.json or not 'request' in request.json:
        abort(400)
    output = []
    for json_input in request.json['request']:
        if json_input['data'] == 'Controls_Depth':
            arr = []
            arr.append({'reference' : 50})
            arr.append({'current' : 50})
            arr.append({'error' : 1})
            output.append({'Controls_Depth' : arr})

        if json_input['data'] == 'State_Depth':
            arr = []
            arr.append({'depth' : robot_data.depth})
            arr.append({'pressure' : robot_data.pressure})
            arr.append({'temp' : robot_data.temp})
            arr.append({'altitude' : robot_data.altitude})  
            output.append({'State_Depth' : arr})

        if json_input['data'] == 'Bboxes':
            arr = []
            arr.append({'enabled' : 50})
            arr.append({'pingFrequency' : 50})
            arr.append({'filename' : 'String'})
            output.append({'Bboxes' : arr})
            
        if json_input['data'] == 'Dvl':
            arr = []
            arr.append({'enabled' : 50})
            arr.append({'pingFrequency' : 50})
            arr.append({'filename' : 'String'})
            output.append({'Dvl' : arr})

        if json_input['data'] == 'Imu':
            arr = []
            arr.append({'enabled' : 50})
            arr.append({'pingFrequency' : 50})
            arr.append({'filename' : 'String'})
            output.append({'Imu' : arr})

        if json_input['data'] == 'Object':
            arr = []
            arr.append({'enabled' : 50})
            arr.append({'pingFrequency' : 50})
            arr.append({'filename' : 'String'})
            output.append({'Object' : arr})

        if json_input['data'] == 'Switches':
            arr = []
            arr.append({'enabled' : 50})
            arr.append({'pingFrequency' : 50})
            arr.append({'filename' : 'String'})
            output.append({'Switches' : arr})

    return jsonify({'data' : output}), 201

@app.errorhandler(400)
def invalid(error):
    return make_response(jsonify({'error':'Invalid JSON Request'}), 400)

rpack = rospkg.RosPack()
config_path = os.getcwd() + "/infoNode_cfg.yaml"
pubs = {}
cfg = {}

def controls_depth_callback(msg):
    reference = msg.reference
    current = msg.current
    error = msg.error

def state_depth_callback(msg):
    robot_data.depth = msg.depth
    robot_data.pressure = msg.pressure
    robot_data.temp = msg.temp
    robot_data.altitude = msg.altitude


def bboxes_callback(msg):
    top_left = msg.top_left
    bottom_right = msg.bottom_right

def dvl_callback(msg):
    time = msg.time
    dt1 = msg.dt1
    dt2 = msg.dt2
    velocity = msg.velocity
    vehicle_pos = msg.vehicle_pos
    figure_of_merit = msg.figureOfMerit
    beam_distance = msg.beamDistance
    battery_voltage = msg.batteryVoltage
    speed_sound = msg.speedSound
    pressure = msg.pressure
    temp = msg.temp


def imu_callback(msg):
    test = 1

def object_callback(msg):
    test = 1

def switches_callback(msg):
    test = 1

def loadConfig():
    global cfg
    with open(config_path, 'r') as stream:
        cfg = yaml.load(stream)

def main():
    loadConfig()

    threading.Thread(target=lambda: rospy.init_node('infoNode', disable_signals=True)).start()

    controls_depth_sub = rospy.Subscriber(cfg['controls_depth_topic'], ControlStatus, controls_depth_callback)
    state_depth_sub = rospy.Subscriber(cfg['state_depth_topic'], Depth, state_depth_callback)
    bboxes_sub = rospy.Subscriber(cfg['bboxes_topic'], BoundingBoxes, bboxes_callback)
    dvl_sub = rospy.Subscriber(cfg['dvl_topic'], Dvl, dvl_callback)
    imu_sub = rospy.Subscriber(cfg['imu_topic'], Imu, imu_callback)
    object_sub = rospy.Subscriber(cfg['object_topic'], Object, object_callback)
    switches_sub = rospy.Subscriber(cfg['switches_topic'], SwitchState, switches_callback)

    rospy.loginfo('Node started')

if __name__ == '__main__':

    main()
    rospy.loginfo('Flask starting')
    app.run(host='0.0.0.0', port=5000)

