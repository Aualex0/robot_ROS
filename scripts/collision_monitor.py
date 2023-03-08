#!/usr/bin/env python

## Ce programme conserve map en mémoire.
## Il la met à jour et maintient une veille d'évitement

import rospy
from robot_ROS.msg import Table_description
from robot_ROS.msg import Object_position_description
from std_msgs.msg import String
import time
import math

current_map = Table_description()
current_map.itself.object = "false"
current_map.annex.object = "false"
current_map.opp_main.object = "false"
current_map.opp_annex.object = "false"

stop = False

#TODO : init position of static objects



def update_map(data):
    # Mise à jour de la map avec comparaison à la map actuelle

    trust_factor = 0.3 # facteur de confiance envers les données du Lidar
    #TODO : rechercher le trust_factor optimal -- à voir si la bibliothèque le fait bcp mieux
    #TODO : comparer la nouvelle position et l'ancienne pour éliminer les données aberrantes
    #TODO : update status of static objects that become useless (by tracking position of opponent robots)

    if current_map.itself.object == "false" :
        current_map.itself = data.itself
    else :
        current_map.itself.x += trust_factor*(data.itself.x - current_map.itself.x)

    if current_map.annex.object == "false" :
        current_map.annex = data.annex
    else :
        current_map.annex.x += trust_factor*(data.annex.x - current_map.annex.x)

    if current_map.opp_main.object == "false" :
        current_map.opp_main = data.opp_main
    else :
        current_map.opp_main.x += trust_factor*(data.opp_main.x - current_map.opp_main.x)

    if current_map.opp_annex.object == "false" :
        current_map.opp_annex = data.opp_annex
    else :
        current_map.opp_annex.x += trust_factor*(data.opp_annex.x - current_map.opp_annex.x)

        
def update_objective(data):
    current_map = data


def send_data(data):
    # Envoi de la map au decision_maker sur demande
    if data.data == "true":
        pub = rospy.Publisher('send_data', Table_description, queue_size=10)
        pub.publish(current_map)



def check():
    # Cette fonction renvoie true en cas de risque de collision
    #TODO : collision with walls ?
    #TODO : edit pour mettre des cercles au lieu des carrés ?

    safety_distance = 200
    #TODO : calculate optimal safety distance

    if (current_map.annex.object != "false" and abs(current_map.itself.x - current_map.annex.x) < safety_distance
                                            and abs(current_map.itself.y - current_map.annex.y) < safety_distance) :
        return True
    if (current_map.opp_main.object != "false" and abs(current_map.itself.x - current_map.opp_main.x) < safety_distance
                                            and abs(current_map.itself.y - current_map.opp_main.y) < safety_distance) :
        return True
    if (current_map.opp_annex.object != "false" and abs(current_map.itself.x - current_map.opp_annex.x) < safety_distance
                                            and abs(current_map.itself.y - current_map.opp_annex.y) < safety_distance) :
        return True

    return False



def listener():
    global stop
    rospy.init_node('collision_monitor', anonymous=True)
    
    
    rospy.Subscriber("update_objective", Table_description, update_objective)
    rospy.Subscriber("update_data", Table_description, update_map)
    rospy.Subscriber("ask_data", String, send_data)

    while not rospy.is_shutdown():
        # Veille évitement
        if check():
            if not stop:
                stop = True
                #TODO : envoyer directement à order_move l'ordre de s'arrêter sans passer par le decision_maker ?
                #pub = rospy.Publisher('send_data', Table_description, queue_size=10)
                pub = rospy.Publisher('order_move', String, queue_size=10)
                #pub.publish() publish order to stop
        elif stop:
            stop = False;
            pub = rospy.Publisher('order_move', String, queue_size=10)
            #pub.publish() publish order to move again
        time.sleep(0.05)


if __name__ == '__main__':
    listener()
