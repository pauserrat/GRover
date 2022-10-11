#!/usr/bin/evn python3


import rospy

from sensor_msgs.msg import Joy
#from std_msgs.msg import Int16
from geometry_msgs.msg import Twist


def callback(data):
    if abs(data.axes[0])>0.2:
        velocidades.linear.x=int(maxSpeed1*data.axes[0])
    else: velocidades.linear.x=0
    if abs(data.axes[1])>0.2:
        velocidades.linear.y=int(maxSpeed2*data.axes[1])
    else: velocidades.linear.y=0
    if abs(data.axes[3])>0.2:
        velocidades.linear.z=int(maxSpeed3*data.axes[3])
    else: velocidades.linear.z=0
    if abs(data.axes[4])>0.2:
        velocidades.angular.x=int(maxSpeed4*data.axes[4])
    else: velocidades.angular.x=0
    if data.buttons[0]==1:
        velocidades.angular.y=maxSpeed5
    elif data.buttons[2]==1:
        velocidades.angular.y=-maxSpeed5
    else: velocidades.angular.y=0
    if abs(data.axes[1])>0.2 and data.buttons[1]==1:
        velocidades.angular.z=maxSpeed6*data.axes[1]
    else: velocidades.angular.z=0
    
    pub.publish(velocidades)


if __name__ == '__main__':

    velocidades = Twist()
    maxSpeed1 =50
    maxSpeed2 =1000
    maxSpeed3 =1000
    maxSpeed4 =800
    maxSpeed5 =2000
    maxSpeed6 =100

    rospy.init_node('node_joy_pc')
    pub = rospy.Publisher('/joy_pc', Twist,queue_size=100)
    rospy.Subscriber("joy", Joy, callback)

    rospy.spin()

