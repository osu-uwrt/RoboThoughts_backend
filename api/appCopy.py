#!api/bin/python

from flask import Flask, jsonify, request, abort, make_response
import rospy
import rospkg
import yaml
import json 

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from riptide_msgs.msg import SwitchState

import os
import threading
from std_msgs.msg import String

#Global variables that store objects representing the JSON response
global depth
global orientation
global position
global switches
global video

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
            if json_input['data'] == 'depth':
                output.append({'depth' : depth})

            if json_input['data'] == 'orientation':
                output.append({'orientation' : orientation})

            if json_input['data'] == 'position':
                output.append({'position' : position})
                
            if json_input['data'] == '':
                output.append({'Dvl' : dvl_msg})

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
def pose_callback(msg):
    global position
    global depth
    global orientation  
    msg = yaml.load(str(msg))['pose']['pose']

    # store other data contained within the pose_gt topic
    position = msg['position']
    depth = position['z']
    orientation = msg['orientation']

def video_callback(msg):
    global video 
    video = yaml.load(str(msg))


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
    pose_sub = rospy.Subscriber(cfg['pose_topic'], Odometry, pose_callback, queue_size = 1)
    video_sub = rospy.Subscriber(cfg['video_topic'], CompressedImage, video_callback, queue_size = 1)
    switches_sub = rospy.Subscriber(cfg['switches_topic'], SwitchState, switches_callback, queue_size = 1)

#Start hte node and the server
if __name__ == '__main__':
    start_node()
    app.run(host='0.0.0.0', port=5000)

