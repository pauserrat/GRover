#!/usr/bin/evn python3

from cmath import cos, pi, sin, tan
from re import A
import rospy
import math

from sensor_msgs.msg import Joy
from brass_package.msg import VelSteppersBRASS
from brass_package.msg import PosSteppersBRASS

movent=False
endavant=False
enrere=False
adalt=False
abaix=False
q1=math.pi/2.0
q2=-3.05

def velQ1(maxQ2,vx,vy,q1,q2):
    return -maxQ2

def callbackPos(data):
    global q1
    global q2
    q1=(2*math.pi/20000)*data.posStepper2
    q2=(2*math.pi/20000)*data.posStepper3

def callback(data):
    global endavant
    global enrere
    global adalt
    global abaix
    global velAng1, velAng2, velAng3, velAng6, velAng5, velAng4
    if abs(data.axes[1])>0.1 and abs(data.axes[0])<0.1 and data.axes[5]==1: #endavant o enrere
        if data.axes[1]>0.1 and data.buttons[0]==0:
            endavant=True
            enrere=False
        elif data.axes[1]<-0.1 and data.buttons[0]==0:
            enrere=True
            endavant=False
    elif data.axes[0]>0.1 and abs(data.axes[1])<0.1:
        velAng1=50
    elif data.axes[0]<-0.1 and abs(data.axes[1])<0.1:
        velAng1=-50
    elif abs(data.axes[4])>0.1 and abs(data.axes[3])<0.1 and data.axes[5]==1:
        if data.axes[4]>0.1 and data.buttons[0]==0:
            adalt=True
            abaix=False
        elif data.axes[4]<0.1 and data.buttons[0]==0:
            abaix=True
            adalt=False
    elif data.buttons[13]==1:
        velAng2=0.2
    elif data.buttons[14]==1:
        velAng2=-0.2
    elif data.buttons[0]==1:
        velAng4=1500
    elif data.buttons[2]==1:
        velAng4=-1500
    elif data.buttons[15]==1:
        velAng3=0.2
    elif data.buttons[16]==1:
        velAng3=-0.2
    elif data.buttons[1]==1:
        velAng6=30
    elif data.buttons[3]==1:
        velAng6=-30
    elif data.buttons[4]==1:
        velAng5=300
    elif data.buttons[5]==1:
        velAng5=-300
    else:
        velAng1=0
        velAng2=0
        velAng3=0
        velAng4=0
        endavant= False
        enrere=False
        adalt=False
        abaix=False
        velAng6=0
        velAng5=0

if __name__ == '__main__':

    velAng1=0
    velAng2=0.0
    velAng3=0.0
    velAng4=0
    velAng5=0
    velAng6=0

    revStep1=400
    revStep2=24000
    revStep3=24000
    revStep4=24000
    revStep5=24000

    l1=0.38
    l2=0.31

    steppers = VelSteppersBRASS()

    rospy.init_node('node_joy_brass')
    pub = rospy.Publisher('/vel_steppers_BRASS', VelSteppersBRASS,queue_size=100)
    rospy.Subscriber("joy1", Joy, callback)
    rospy.Subscriber("posBRASS",PosSteppersBRASS,callbackPos)
    r=rospy.Rate(15)
    while(True):
        if endavant:
            velAng2=(0.01)/(l1*math.cos(q1)*math.tan(q1+q2)-l1*math.sin(q1)) #rad/s
            velAng3=0.0/(l2*math.cos(q1+q2))-velAng2*(1+((l1*math.cos(q1))/(l2*math.cos(q1+q2)))) #calcul inverse
            if q2>-0.04:
                velAng2=0
                velAng3=0
            elif abs(velAng2)>0.3 or abs(velAng3)>0.3:
                velAng2=(velAng2/abs(velAng2))*0.3
                velAng3=-velAng2*(1+((l1*math.cos(q1))/(l2*math.cos(q1+q2))))
                if abs(velAng3)>0.3:
                    velAng3=(velAng3/abs(velAng3))*0.3
                    velAng2=-velAng3/(1+(l1*math.cos(q1))/(l2*math.cos(q1+q2)))
        elif enrere:
            velAng2=(-0.01)/(l1*math.cos(q1)*math.tan(q1+q2)-l1*math.sin(q1)) #rad/s
            velAng3=-0.0/(l2*math.cos(q1+q2))-velAng2*(1+((l1*math.cos(q1))/(l2*math.cos(q1+q2))))
            if abs(velAng2)>0.3 or abs(velAng3)>0.3:
                velAng2=(velAng2/abs(velAng2))*0.3
                velAng3=-velAng2*(1+((l1*math.cos(q1))/(l2*math.cos(q1+q2))))
                if abs(velAng3)>0.3:
                    velAng3=(velAng3/abs(velAng3))*0.3
                    velAng2=-velAng3/(1+(l1*math.cos(q1))/(l2*math.cos(q1+q2)))
             #calcul inverse
        elif adalt:
            if (l1*math.cos(q1) -l2*math.cos(q1+q2)-1-(l1*math.sin(q1)/(l2*math.sin(q1+q2))))==0:
                velAng2=0
                velAng3=0
            else:
                velAng2=0.01/(l1*math.cos(q1) -l2*math.cos(q1+q2)-1-(l1*math.sin(q1)/(l2*math.sin(q1+q2))))
                velAng3=-velAng2/(l2*math.sin(q1+q2))*(l2*math.sin(q1+q2)+l1*math.sin(q1))
                if q2>-0.04:
                    velAng2=0
                    velAng3=0
                elif abs(velAng2)>0.3 or abs(velAng3)>0.3:
                    velAng2=(velAng2/abs(velAng2))*0.3
                    velAng3=-velAng2/(l2*math.sin(q1+q2))*(l2*math.sin(q1+q2)+l1*math.sin(q1))
                    if abs(velAng3)>0.3:
                        velAng3=(velAng3/abs(velAng3))*0.3
                        velAng2=-velAng3/(l2*math.sin(q1+q2)+l1*math.sin(q1))*(l2*math.sin(q1+q2))

        elif abaix:
            if (l1*math.cos(q1) -l2*math.cos(q1+q2)-1-(l1*math.sin(q1)/(l2*math.sin(q1+q2))))==0:
                velAng2=0
                velAng3=0
            else: 
                velAng2=-0.01/(l1*math.cos(q1) -l2*math.cos(q1+q2)-1-(l1*math.sin(q1)/(l2*math.sin(q1+q2))))
                velAng3=-velAng2/(l2*math.sin(q1+q2))*(l2*math.sin(q1+q2)+l1*math.sin(q1))
                if q2>-0.04:
                    velAng2=0
                    velAng3=0
                elif abs(velAng2)>0.25 or abs(velAng3)>0.25:
                    if velAng2>0:
                        velAng2=0.25
                    elif velAng2<0:
                        velAng2=-0.25
                    velAng3=-velAng2/(l2*math.sin(q1+q2))*(l2*math.sin(q1+q2)+l1*math.sin(q1))
                    if abs(velAng3)>0.25:
                        if velAng3>0:
                            velAng3=0.25
                        elif velAng3<0:
                            velAng3=-0.25
                        rospy.loginfo(velAng3)
                        velAng2=-velAng3/(l2*math.sin(q1+q2)+l1*math.sin(q1))*(l2*math.sin(q1+q2))
                        rospy.loginfo(velAng2)
          
        steppers.velStepper2=int(velAng2*(1.0/(2.0*math.pi))*(24000.0/1.0))
        steppers.velStepper3=int(velAng3*(1.0/(2.0*math.pi))*(24000.0/1.0))
        steppers.velStepper1=velAng1
        steppers.velStepper6=velAng6
        steppers.velStepper4=velAng4
        steppers.velStepper5=velAng5
        pub.publish(steppers)
        r.sleep()

    rospy.spin()