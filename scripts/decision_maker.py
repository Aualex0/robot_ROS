#!/usr/bin/env python

## Ce programme gère la prise de décision globale et la communication avec les différents actionneurs.

import rospy
from robot_ROS.msg import Table_description
from robot_ROS.msg import Object_position_description
from std_msgs import String
import math

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
    #publish for actioners to act
    #V1 : follow a predefined path
    #V2 : remember position of objects; take into consideration disruption from opponent
    #V3 : assess the situation
    True

def talker_motors():
    # format des données à envoyer aux moteurs principaux : (int type, float x, float y, int rotation).
    # type = (0:stop, 1:translation; 2:rotation)
    # coordonnées dans la base du robot
    True

def listener():
    rospy.init_node('decision_maker', anonymous=True)

    rospy.Subscriber("send_data", Table_description, make_decision)

    while not rospy.is_shutdown():
        pub = rospy.Publisher('ask_data', Table_description, queue_size=10)
        rate = rospy.Rate(0.2) # 0.2hz
        message = "true"
        pub.publish(message)


if __name__ == '__main__':
    listener()
