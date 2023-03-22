#!/usr/bin/env python

## récupère les obstacles de node obstacle_tracker
##en déduit la position du robot et l'envoit rossur "positions"

import rospy
from robot_ROS.msg import Table_description
from robot_ROS.msg import Object_position_description
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle
from std_msgs.msg import String
import time
import math
import numpy as np


def get_position(data):
    L = [None for _ in range(4)]
    obstacles = data.circles
    for obstacle in obstacles:
        if obstacle.id != 0:
            L[obstacle.id] = np.array([obstacle.center.x, obstacle.center.y ])
    
    if L[1].any() == None:
        G = Gtheo
    elif L[2].any() == None:
        G = Gtheo
    elif L[3].any() == None:
        G = Gtheo
    if L[1].all()!= None and L[2].all()!= None and L[3].all()!= None:
        teta = mod(angleDeg(B1-B2)-angleDeg(L[1]-L[2])) + mod(angleDeg(B1-B3)-angleDeg(L[1]-L[3])) + mod(angleDeg(B2-B3)-angleDeg(L[2]-L[3])) #TODO remplacer les constantes
        teta = teta/3
        G = L[1] + L[2] + L[3]
        G = 1/3 * G
        G = rotate(G, -teta)
        G = Gtheo - G
    
    pos = Object_position_description()
    pos.object = "self"
    pos.x = G[0]
    pos.y = G[1]
    pos.alpha = int(teta)
    
    pub = rospy.Publisher('positions', Object_position_description, queue_size=10)
    pub.publish(pos)




def listener():
    rospy.init_node('get_position', anonymous=True)

    rospy.Subscriber("obstacles", Obstacles, get_position)

    rospy.spin()


#positions des balises
B1 = np.array([0,0])
B2 = np.array([3,0])
B3 = np.array([1.5,2])

Gtheo = (B1+B2+B3)/3

def angleDeg(point):
    #envoi l'angle (en deg) que fait OP avec l'horizontale
    x = point[0]
    y = point[1]
    return math.atan2(y,x)*180/math.pi

def mod(angle):
    #renvoit la valeur (modulo 360°) entre -180° et 180°
    if angle >180: return mod(angle-360)
    if angle <= -180: return mod(angle+360)
    return angle

def rotate(point, teta):
    #pivote le vecteur OP de teta
    teta = teta*math.pi/180
    x = point[0]
    y = point[1]
    xprim = x*math.cos(teta) + y*math.sin(teta)
    yprim = y*math.cos(teta) - x*math.sin(teta)
    return np.array([xprim, yprim])




if __name__ == '__main__':
    listener()
