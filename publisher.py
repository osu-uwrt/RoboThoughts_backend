import rospy
from riptide_msgs.msg import Depth
import random


def talker():
    pub = rospy.Publisher('/state/depth', Depth, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        data = Depth()
        data.depth = random.randint(1,100)
        data.pressure = random.randint(1,100)
        data.altitude = random.randint(1,100)
        data.temp = random.randint(1,100)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
