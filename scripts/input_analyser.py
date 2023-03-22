#!/usr/bin/env python

## récupère les obstacles de node obstacle_tracker
##en déduit la position du robot et l'envoit rossur "positions"

import rospy
from robot_ROS.msg import Table_description
from robot_ROS.msg import Object_position_description
from robot_ROS.msg import Static_positioned_objects
from obstacle_detector.msg import Obstacles
from obstacle_detector.msg import CircleObstacle
from std_msgs.msg import String
import time
import math
import numpy as np


def get_position(data):
    L = [None for _ in range(5)]    #1-3: les 3 balises, 4-5: le(s) robot(s) adverse(s)
    found = [0 for i in range(3)]
    obstacles = data.circles
    for obstacle in obstacles:
        if obstacle.id != 0:
            L[obstacle.id-1] = np.array([obstacle.center.x, obstacle.center.y ])
            found[obstacle.id-1] = 1
    
    if found[0] and found[1] and not found[2]:
        #TODO: calcul correct
        G = Gtheo
    elif found[0] and not found[1] and found[2]:
        #TODO: calcul correct
        G = Gtheo
    elif not found[0] and not found[1] and found[2]:
        #TODO: calcul correct avec 2 balises
        G = Gtheo
    elif found[0] and found[1] and found[2]:
        teta = mod(angleDeg(B1-B2)-angleDeg(L[1]-L[2])) + mod(angleDeg(B1-B0)-angleDeg(L[1]-L[0])) + mod(angleDeg(B2-B0)-angleDeg(L[2]-L[0])) #TODO remplacer les constantes
        teta = teta/3
        G = L[0] + L[1] + L[2]
        G = 1/3 * G
        G = rotate(G, -teta)
        G = Gtheo - G
    
    pos = Object_position_description()
    pos.object = "self"
    pos.x = G[0]
    pos.y = G[1]
    pos.alpha = int(teta)

    pub1 = rospy.Publisher('positions', Object_position_description, queue_size=10)
    pub1.publish(pos)
    

    table_descr = Table_description(itself = Object_position_description(object="main", x=pos.x, y=pos.y, alpha=pos.alpha),
                                    annex = Object_position_description(object="annex", x=0, y=0, alpha=0),
                                    opp_main = Object_position_description(object="opp_main", x=0, y=0, alpha=0),
                                    opp_annex = Object_position_description(object="opp_annex", x=0, y=0, alpha=0),
                                    other = Static_positioned_objects(q1_value = 2,
                                                                      q2_value = 2,
                                                                      q3_value = 1,
                                                                      q4_value = 2,
                                                                      q1 = [Object_position_description(object = "ingredient rose", x = 575, y = 2000-225, alpha = 0),
                                                                            Object_position_description(object = "ingredient yellow", x = 775, y = 2000-225, alpha = 0),
                                                                            Object_position_description(object = "ingredient brown", x = 1125, y = 2000-725, alpha = 0)],
                                                                      q2 = [Object_position_description(object = "ingredient rose", x = 3000-575, y = 2000-225, alpha = 0),
                                                                            Object_position_description(object = "ingredient yellow", x = 3000-775, y = 2000-225, alpha = 0),
                                                                            Object_position_description(object = "ingredient brown", x = 3000-1125, y = 2000-725, alpha = 0)],
                                                                      q3 = [Object_position_description(object = "ingredient rose", x = 3000-575, y = 225, alpha = 0),
                                                                            Object_position_description(object = "ingredient yellow", x = 3000-775, y = 225, alpha = 0),
                                                                            Object_position_description(object = "ingredient brown", x = 3000-1125, y = 725, alpha = 0)],
                                                                      q4 = [Object_position_description(object = "ingredient rose", x = 575, y = 225, alpha = 0),
                                                                            Object_position_description(object = "ingredient yellow", x = 775, y = 225, alpha = 0),
                                                                            Object_position_description(object = "ingredient brown", x = 1125, y = 725, alpha = 0)]))



    pub2 = rospy.Publisher("update_data", Table_description, queue_size=10)
    pub2.publish(table_descr)




def listener():
    rospy.init_node('get_position', anonymous=True)

    rospy.Subscriber("obstacles", Obstacles, get_position)

    rospy.spin()


#positions des balises
B0 = np.array([0,0])
B1 = np.array([3,0])
B2 = np.array([1.5,2])

Gtheo = (B0+B1+B2)/3

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
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
