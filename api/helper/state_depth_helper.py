from riptide_msgs.msg import Depth

global depth
global pressure
global temp
global altitude

def json_builder():
    arr = []
    arr.append({'depth' : state_depth_helper.depth})
    arr.append({'pressure' : state_depth_helper.pressure})
    arr.append({'temp' : state_depth_helper.temp})
    arr.append({'altitude' : state_depth_helper.altitude})
    return arr  

def state_depth_callback(msg):
    depth = msg.depth
    pressure = msg.pressure
    temp = msg.temp
    altitude = msg.altitude