#!/usr/bin/env python

## Ce programme gère la prise de décision globale et la communication avec les différents actionneurs.

import rospy
from robot_ROS.msg import Table_description
from robot_ROS.msg import Object_position_description
from std_msgs import String
import math
import time

processing = False
moving = False
time_left = 0
avancement = 0

def change_coordinates(new_origin, point):
    # projection dans le repère du robot
    projected = Object_position_description(object = point.object,
                                            x = math.cos(new_origin.alpha)*point.x + math.sin(new_origin.alpha)*point.y, 
                                            y = - math.sin(new_origin.alpha)*point.x + math.cos(new_origin.alpha)*point.y,
                                            alpha = 0)
    rot_origin = Object_position_description(objet=new_origin.object,
                                            x = math.cos(new_origin.alpha)*point.x + math.sin(new_origin.alpha)*point.y, 
                                            y = - math.sin(new_origin.alpha)*point.x + math.cos(new_origin.alpha)*point.y,
                                            alpha = 0)
    projected.x -= rot_origin.x
    projected.y -= rot_origin.y
    return projected

def make_decision(data):
    #make decision
    #input : Table description, intern state, time left, moving, past actions
    #output : first move
    #publish for actioners to act
    #l'ordre peut être de stopper tout mouvement jusqu'à validation par les actionneurs internes (état stop moteur)
    #V1 : follow a predefined path
    #V2 : remember position of objects; take into consideration disruption from opponent
    #V3 : assess the situation
    table_description = data.data
    
def get_next_point(x,y, table_description):
    #prend en entrée l'objectif final et renvoie le prochain point à atteindre
    pass

def get_move_type(x, y, self_pos):
     #prend en entrée un point atteignable et renvoie les instructions de rotation + translation correspondants
     pass

def talker_motors(type, x, y, rotation):
    # format des données à envoyer aux moteurs principaux : (int type, float x, float y, int rotation).
    # type = (0:stop, 1:translation; 2:rotation)
    # coordonnées dans la base du robot
    pub = rospy.Publisher('order_move', list, queue_size=10)
    message = [type, x, y, rotation]
    pub.publish(message) #publish order to move again
    moving = True
    
def talker_actioners(grab):
    # donne l'ordre aux actionneurs de se mettre en route pour ramasser les palets
    # type = (true: grab, false: release)
    pub = rospy.Publisher('order_actioners', String, queue_size=10)
    message = ("true" if grab else "false")
    pub.publish(message) #publish order to grab slices or to release them
    processing = True
    
def feedback_move(data):
    #reçoit le retour de l'arduino moteur (par ex si le mvmt a été effectué avec succès)
    #lance un nouvel ordre instantanément
    #reçoit "success" si c'est un succès, autre chose si c'est une erreur
    if data.data == "success":
        moving = False
        pub = rospy.Publisher('ask_data', Table_description, queue_size=10)
        message = "true"
        pub.publish(message)
    else:
        pass
        #debug here
    
def intern_state(data):
    #reçoit l'état interne du robot (actionneurs internes)
    #lorsque les palets ont été ramassés avec succès, on peut lancer le prochain ordre de déplacement
    #reçoit "success" si c'est un succès, autre chose si c'est une erreur
    if data.data == "success":
        processing = False
        pub = rospy.Publisher('ask_data', Table_description, queue_size=10)
        message = "true"
        pub.publish(message)
    else:
        pass
        #debug here

def listener():
    rospy.init_node('decision_maker', anonymous=True)

    rospy.Subscriber("send_data", Table_description, make_decision)
    rospy.Subscriber("feedback_move", String, feedback_move)
    rospy.Subscriber("intern_state", String, intern_state)

    while not rospy.is_shutdown():
        pub = rospy.Publisher('ask_data', Table_description, queue_size=10)
        rate = rospy.Rate(0.2) # 0.2hz
        message = "true"
        pub.publish(message)


if __name__ == '__main__':
    listener()
