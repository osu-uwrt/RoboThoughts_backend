from riptide_msgs.msg import Depth

global depth
global pressure
global temp
global altitude

def json_builder():
    arr = []
    arr.append({'depth' : depth})
    arr.append({'pressure' : pressure})
    arr.append({'temp' : temp})
    arr.append({'altitude' : altitude})
    return arr  