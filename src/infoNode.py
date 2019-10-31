#!/usr/bin/env python
# infoNode.py
# This node subscribes to topics and posts them to the API server

import rospy
import rospkg
import yaml
from riptide_msgs.msg import BoundingBox
from riptide_msgs.msg import ControlStatus
from riptide_msgs.msg import Depth
from riptide_msgs.msg import Dvl
from riptide_msgs.msg import Imu
from riptide_msgs.msg import Object
from riptide_msgs.msg import SwitchState

rpack = rospkg.RosPack()
config_path = rpack.get_path('RoboThoughts') + "backend/src/cfg/infoNode_cfg.yml"
pubs = {}
cfg = {}

def bboxes_callback(msg):
    top_left = msg.top_left
    bottom_right = msg.bottom_right

def depth_callback(msg):
	depth = msg.depth
    pressure = msg.pressure
    temp = msg.temp
    altitude = msg.altitude

def dvl_callback(msg):
    time = msg.time
    dt1 = msg.dt1
    dt2 = msg.dt2
    velocity = msg.velocity
    vehicle_pos = msg.vehicle_pos
	figureOfMerit = msg.figureOfMerit
	beamDistance = msg.beamDistance
	batteryVoltage = msg.batteryVoltage
	speedSound = msg.speedSound
	pressure = msg.pressure
	temp = msg.temp

def imu_callback(msg):
	quaternion = msg.quaternion
	rpy_rad = msg.rpy_rad
	rpy_deg = msg.rpy_deg
	heading_alt = msg.heading_alt
	heading_lord = msg.heading_lord
	linear_accel = msg.linear_accel
	ang_vel_rad = msg.ang_vel_rad
	ang_vel_deg = msg.ang_vel_deg
	ang_accel_rad = msg.ang_accel_rad
	ang_accel_deg = msg.ang_accel_deg

def object_callback(msg):
	# name of object camera is looking at
	object_name = msg.object_name
	# bounding box in pixels
	bbox_width = msg.bbox_width
	bbox_height = msg.bbox_height
	# center position in pixels relative to camera center
	pos = msg.pos

def switches_callback(msg):
	kill = msg.kill
	sw1 = msg.sw1
	sw2 = msg.sw2
	sw3 = msg.sw3
	sw4 = msg.sw4
	sw5 = msg.sw5

def loadConfig():
    global cfg
    with open(config_path, 'r') as stream:
        cfg = yaml.load(stream)

def main():
    loadConfig()
	bbox_sub = rospy.Subscriber(cfg['bboxes_topic'], BoundingBoxes, bboxes_callback)
    depth_sub = rospy.Subscriber(cfg['depth_topic'], Depth, depth_callback)
	dvl_sub = rospy.Subscriber(cfg['dvl_topic'], Dvl, dvl_callback)
	imu_sub = rospy.Subscriber(cfg['imu_topic'], Imu, imu_callback)
	object_sub = rospy.Subscriber(cfg['object_topic'], Object, object_callback)
	switches_sub = rospy.Subscriber(cfg['switches_topic'], SwitchState, switches_callback)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("infoNode")
    main()
