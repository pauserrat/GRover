#!/usr/bin/evn python3

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Joy

def callback(data):
    global cam, canvi
    if data.buttons[0]==0:
        canvi=False
    if data.buttons[0]==1 and canvi==False:
        if cam==0:
            cam=1
        else:
            cam=0
        canvi=True
    pub.publish(cam)
if __name__ == '__main__':
    cam = Int8()
    cam=1
    canvi=False
    rospy.init_node('nodo_ruedas_pc')
    pub = rospy.Publisher('robot_vel',Int8,queue_size=100)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()