#!api/bin/python
from flask import Flask, jsonify, request, abort, make_response

import rospy
import rospkg
import yaml 
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

global controls_depth_msg
global state_depth_msg
global bboxes_msg
global dvl_msg
global imu_msg
global object_msg
global switches_msg

app = Flask(__name__)

@app.route('/', methods=['OPTIONS'])
def opt_respond():
    response = jsonify('{"pass"}')
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', '*')
    return response

@app.route('/', methods=['POST'])
def respond():
    try:
        if not request.json or not 'request' in request.json:
            abort(400)
        output = []
        for json_input in request.json['request']:
            if json_input['data'] == 'Controls_Depth':
                output.append({'Controls_Depth' : controls_depth_msg})

            if json_input['data'] == 'State_Depth':
                output.append({'State_Depth' : state_depth_msg})

            if json_input['data'] == 'Bboxes':
                output.append({'Bboxes' : bboxes_msg})
                
            if json_input['data'] == 'Dvl':
                output.append({'Dvl' : dvl_msg})

            if json_input['data'] == 'Imu':
                output.append({'Imu' : imu_msg})

            if json_input['data'] == 'Object':
                output.append({'Object' : object_msg})

            if json_input['data'] == 'Switches':
                output.append({'Switches' : switches_msg})

        response = jsonify({'data' : output})
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Headers', '*')
        return response, 201
    except Exception as e:
        print(e)

@app.errorhandler(400)
def invalid(error):
    return make_response(jsonify({'error':'Invalid JSON Request'}), 400)

rpack = rospkg.RosPack()
config_path = os.getcwd() + "/api/infoNode_cfg.yaml"
pubs = {}
cfg = {}

def controls_depth_callback(msg):
    controls_depth_msg = yaml.load(str(msg))

def state_depth_callback(msg):
    state_depth_msg = yaml.load(str(msg))

def bboxes_callback(msg):
    bboxes_msg = yaml.load(str(msg))

def dvl_callback(msg):
    dvl_msg = yaml.load(str(msg))

def imu_callback(msg):
    imu_msg = yaml.load(str(msg))

def object_callback(msg):
    object_msg = yaml.load(str(msg))

def switches_callback(msg):
    switches_msg = yaml.load(str(msg))

def loadConfig():
    global cfg
    with open(config_path, 'r') as stream:
        cfg = yaml.load(stream)

def start_node():
    loadConfig()

    threading.Thread(target=lambda: rospy.init_node('infoNode', disable_signals=True)).start()

    controls_depth_sub = rospy.Subscriber(cfg['controls_depth_topic'], ControlStatus, controls_depth_callback)
    state_depth_sub = rospy.Subscriber(cfg['state_depth_topic'], Depth, state_depth_callback)
    bboxes_sub = rospy.Subscriber(cfg['bboxes_topic'], BoundingBoxes, bboxes_callback)
    dvl_sub = rospy.Subscriber(cfg['dvl_topic'], Dvl, dvl_callback)
    imu_sub = rospy.Subscriber(cfg['imu_topic'], Imu, imu_callback)
    object_sub = rospy.Subscriber(cfg['object_topic'], Object, object_callback)
    switches_sub = rospy.Subscriber(cfg['switches_topic'], SwitchState, switches_callback)


if __name__ == '__main__':
    start_node()
    app.run(host='0.0.0.0', port=5000)

