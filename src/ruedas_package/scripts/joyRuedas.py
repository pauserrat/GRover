#!/usr/bin/evn python3

from math import sqrt
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ruedas_package.msg import Motores
from std_msgs.msg import Int8

suma=False
resta=False
velLinMax=200.0

def _map(x, in_min, in_max, out_min, out_max):
    return float((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def callback(data):
    alfa=0.0
    beta=0.0
    stepsIzq=0
    stepsDer=0
    vIzq=0.0
    vDer=0.0
    velLin=0.0
    velAng=0.0
    radi=0.0
    global suma, resta, canvi, cam
    global velLinMax

    if data.buttons[2]==1 and suma:
        velLinMax=velLinMax+5
        suma=False
    elif data.buttons[2]==0:
        suma=True
    if data.buttons[0]==1 and resta:
        velLinMax=velLinMax-5
        resta=False
    elif data.buttons[0]==0:
        resta=True
    if velLinMax>253:
        velLinMax=253
    elif velLinMax<0:
        velLinMax=0

    if data.axes[3]>toleranciaJoy: #gir ESQUERRA
        radi=_map(data.axes[3],toleranciaJoy,1.0,radiMax,radiMin)
        alfa=math.atan(l1/(radi-l2)) #radians
        beta=math.atan(l1/(radi+l2))
        if abs(data.axes[1])>toleranciaJoy:
            if data.axes[1]>toleranciaJoy:
                velLin=_map(data.axes[1],toleranciaJoy,1.0,0.0,velLinMax) #quan axes[1] es positiu
            else:
                velLin=_map(data.axes[1],-toleranciaJoy,-1.0,0.0,-velLinMax)#quan es negatiu
            velAng=velLin/radi
            vIzq=velAng*sqrt(pow(l1,2)+pow(radi-l2,2))/1.9 #cm/s
            vDer=velAng*sqrt(pow(l2,2)+pow(radi+l2,2))/1.9#velLinRoda=wRobot*rDer
    elif data.axes[3]<-toleranciaJoy: #gir DRETA
        radi=_map(data.axes[3],-toleranciaJoy,-1,radiMax,radiMin)
        beta=-math.atan(l1/(radi-l2))
        alfa=-math.atan(l1/(radi+l2))
        if abs(data.axes[1])>toleranciaJoy:
            if data.axes[1]>toleranciaJoy:
                velLin=_map(data.axes[1],toleranciaJoy,1.0,0.0,velLinMax) #quan axes[1] es positiu
            else:
                velLin=_map(data.axes[1],-toleranciaJoy,-1.0,0.0,-velLinMax) #quan es negatiu
            velAng=velLin/radi
            vIzq=velAng*sqrt(pow(l1,2)+pow(radi+l2,2))/1.9
            vDer=velAng*sqrt(pow(l2,2)+pow(radi-l2,2))/1.9
    elif data.buttons[5]==1:
        alfa=-math.pi/2.0
        beta=math.pi/2.0
        vIzq=0
        vDer=0
        if abs(data.axes[1])>toleranciaJoy:
            if data.axes[1]>0:
                vIzq=_map(data.axes[1],toleranciaJoy,1.0,0.0,velLinMax)
                vDer=vIzq
            else:
                vIzq=_map(data.axes[1],-toleranciaJoy,-1.0,0.0,-velLinMax)
                vDer=vIzq

    elif data.buttons[4]==1:
        radi=0
        alfa=math.atan(l1/(radi-l2)) #radians
        beta=math.atan(l1/(radi+l2))
        vIzq=0
        vDer=0
        if abs(data.axes[1])>toleranciaJoy:
            if data.axes[1]>0:
                vIzq=_map(data.axes[1],toleranciaJoy,1.0,0.0,velLinMax)
                vDer=-vIzq
            else:
                vIzq=_map(data.axes[1],-toleranciaJoy,-1.0,0.0,-velLinMax)
                vDer=-vIzq
    else: #NO GIRA
        alfa=0
        beta=0
    
    if abs(data.axes[1])>toleranciaJoy and abs(data.axes[3])<toleranciaJoy and data.buttons[4]==0: #ENDAVANT SENSE GIR
        if data.axes[1]>toleranciaJoy:
            velLin=_map(data.axes[1],toleranciaJoy,1.0,0.0,velLinMax)
        else:
            velLin=_map(data.axes[1],-toleranciaJoy,-1.0,0.0,-velLinMax)
        vIzq=velLin #cm/s
        vDer=velLin
    elif abs(data.axes[1])<toleranciaJoy and abs(data.axes[3])<toleranciaJoy: #QUIET SENSE GIR
        vIzq=0
        vDer=0
    
    if data.buttons[1]==0:
        canvi=False
    if data.buttons[1]==1 and canvi==False:
        if cam==0:
            cam=1
        else:
            cam=0
        canvi=True
    pub1.publish(cam)

    

    #vIzq=60.0*vIzq/(2.0*3.1415*rRoda) #rpm 60s*rad/s*(1rev/2*pi*r) = rpm
    #vDer=60.0*vDer/(2.0*3.1415*rRoda)

    stepsIzq=_map(alfa,-2.0*3.1415,2.0*3.1415,-8000.0,8000.0)
    stepsDer=_map(beta,-2.0*3.1415,2.0*3.1415,-8000.0,8000.0)
    ruedas.posIzq=round(stepsIzq) #steps
    ruedas.posDer=round(stepsDer) #steps
    ruedas.rpmIzq=round(vIzq) #rpm
    ruedas.rpmDer=round(vDer) #rpm
    pub.publish(ruedas)

if __name__ == '__main__':
    ruedas = Motores()
    canvi=False
    cam = Int8()
    cam=1
    canvi=False
    radiMax=500.0 #cm
    radiMin=60.0 #cm
    l1=42.5 #cm
    l2=42.5 #cm
    rRoda=20.0 #cm
    toleranciaJoy=0.10
    
    rospy.init_node('nodo_ruedas_pc')
    pub = rospy.Publisher('/vel_motores',Motores,queue_size=100)
    pub1 = rospy.Publisher('/fpv',Int8,queue_size=100)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()