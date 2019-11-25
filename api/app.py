#!api/bin/python

"""
Authors:
Nicholas Iten
Noah Limes
Clark Godwin
Jerry Qiu
Seamus Scanlon
"""

from flask import Flask, jsonify, request, abort, make_response
import rospy
import rospkg
import yaml 
from riptide_msgs.msg import ControlStatus
from riptide_msgs.msg import Depth
from nortek_dvl.msg import Dvl
from riptide_msgs.msg import Imu
from riptide_msgs.msg import Object
from riptide_msgs.msg import SwitchState
from darknet_ros_msgs.msg import BoundingBoxes
import os
import threading
from std_msgs.msg import String

#Global variables that store objects representing the JSON response
global controls_depth_msg
global state_depth_msg
global bboxes_msg
global dvl_msg
global imu_msg
global object_msg
global switches_msg

#Starts the initial flask server
app = Flask(__name__)

#Needed to permit Angular to interact with the server
@app.route('/', methods=['OPTIONS'])
def opt_respond():
    response = jsonify('{"pass"}')
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', '*')
    return response

#Handles POST requests sent to the server
@app.route('/', methods=['POST'])
def respond():
    try:
        #Checks if the JSON is supposed to be here
        if not request.json or not 'request' in request.json:
            abort(400)
        
        #Goes through the request to find which data to send back
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

        #Creates the JSON request and headers
        response = jsonify({'data' : output})
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Headers', '*')
        return response, 201
    except Exception as e:
        print(e)

#Tells the requester if they messed up the JSON Request
@app.errorhandler(400)
def invalid(error):
    return make_response(jsonify({'error':'Invalid JSON Request'}), 400)

#some stuff for the node setup
rpack = rospkg.RosPack()
config_path = os.getcwd() + "/api/infoNode_cfg.yaml"
pubs = {}
cfg = {}

#callbacks for the node
def controls_depth_callback(msg):
    global controls_depth_msg
    controls_depth_msg = yaml.load(str(msg))

def state_depth_callback(msg):
    global state_depth_msg 
    state_depth_msg = yaml.load(str(msg))

def bboxes_callback(msg):
    global bboxes_msg
    bboxes_msg = yaml.load(str(msg))

def dvl_callback(msg):
    global dvl_msg
    dvl_msg = yaml.load(str(msg))

def imu_callback(msg):
    global imu_msg
    imu_msg = yaml.load(str(msg))

def object_callback(msg):
    global object_msg
    object_msg = yaml.load(str(msg))

def switches_callback(msg):
    global switches_msg
    switches_msg = yaml.load(str(msg))

#Gets the config
def loadConfig():
    global cfg
    with open(config_path, 'r') as stream:
        cfg = yaml.load(stream)

def start_node():
    #Gets the config
    loadConfig()

    #starts the node on a new thread
    threading.Thread(target=lambda: rospy.init_node('infoNode', disable_signals=True)).start()

    #Subscribes the node to all of the topics we want
    controls_depth_sub = rospy.Subscriber(cfg['controls_depth_topic'], ControlStatus, controls_depth_callback, queue_size = 1)
    state_depth_sub = rospy.Subscriber(cfg['state_depth_topic'], Depth, state_depth_callback, queue_size = 1)
    bboxes_sub = rospy.Subscriber(cfg['bboxes_topic'], BoundingBoxes, bboxes_callback, queue_size = 1)
    dvl_sub = rospy.Subscriber(cfg['dvl_topic'], Dvl, dvl_callback, queue_size = 1)
    imu_sub = rospy.Subscriber(cfg['imu_topic'], Imu, imu_callback, queue_size = 1)
    object_sub = rospy.Subscriber(cfg['object_topic'], Object, object_callback, queue_size = 1)
    switches_sub = rospy.Subscriber(cfg['switches_topic'], SwitchState, switches_callback, queue_size = 1)

#Start hte node and the server
if __name__ == '__main__':
    start_node()
    app.run(host='0.0.0.0', port=5000)

