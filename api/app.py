#!api/bin/python

from flask import Flask, jsonify, request, abort, make_response
import rospy
import rospkg
import yaml
import json
import os
import threading
from std_msgs.msg import String 

# loads data types of the ROS topics we will subscribe to
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import CompressedImage
from riptide_msgs.msg import SwitchState

# Declares global variables which will be sent back in our responses
global depth
global orientation
global position
global switches
global video

#Starts the initial flask server
app = Flask(__name__)

# handles OPTIONS requests sent to the server
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

        #Creates the JSON response with correct headers
        response = jsonify({'data' : output})
        response.headers.add('Access-Control-Allow-Origin', '*')
        response.headers.add('Access-Control-Allow-Headers', '*')
        return response, 201
    except Exception as e:
        print(e)

@app.route('/video_feed')
def video_feed():
    output = 'http://0.0.0.0:8080/stream?topic=/puddles/stereo/left/image_rect_color&type=mjpeg&quality=25'
    response = jsonify({'data' : output})
    response.headers.add('Access-Control-Allow-Origin', '*')
    response.headers.add('Access-Control-Allow-Headers', '*')
    return response, 201

#Tells the requester if they messed up the JSON Request
@app.errorhandler(400)
def invalid(error):
    return make_response(jsonify({'error':'Invalid JSON Request'}), 400)

#some stuff to set up the ROS Node
rpack = rospkg.RosPack()
config_path = os.getcwd() + "/api/infoNode_cfg.yaml"
pubs = {}
cfg = {}

#callbacks which will be passed to subscribers on the Node:

def pose_callback(msg):
    global position
    global depth
    global orientation 

    # loads the pose_gt topic data 
    msg = yaml.load(str(msg))['pose']['pose']

    #  update global vars with new values located in the pose_gt topic
    position = msg['position']
    depth = position['z']
    orientation = msg['orientation']

def switches_callback(msg):
    global switches_msg
    switches_msg = yaml.load(str(msg))

# Gets the config
def loadConfig():
    global cfg
    with open(config_path, 'r') as stream:
        cfg = yaml.load(stream)

def start_node():
    # Gets the config
    loadConfig()

    # Starts the node on a new thread
    threading.Thread(target=lambda: rospy.init_node('infoNode', disable_signals=True)).start()

    # Subscribes the node to all of the topics we want
    pose_sub = rospy.Subscriber(cfg['pose_topic'], Odometry, pose_callback, queue_size = 1)
    switches_sub = rospy.Subscriber(cfg['switches_topic'], SwitchState, switches_callback, queue_size = 1)

#Start the node and the server
if __name__ == '__main__':
    start_node()
    # host='0.0.0.0' parameter opens the server on the network
    app.run(host='0.0.0.0', port=5000)
    # app.run() with no params starts to server on local host
    