#!/usr/bin/env python

## Ce programme gère la prise de décision globale et la communication avec les différents actionneurs.

import rospy
from robot_ROS.msg import Table_description
from robot_ROS.msg import Object_position_description
from std_msgs import String
import math
import time

processing = False          #True si action interne en cours qui nécessite l'arrêt du robot (grab ou release de gateaux)
moving = False              #True si le robot est actuellement en mouvement
depose = False             #True à partir du moment où on a récupéré les 18 couches
init_time = time.time()     #initialisation du temps à t = 0
blocks_grabbable = 3        #nombre de block récupérable dans la pile de récupération
blocks_grabbed = 0          #nombre de blocs récupérés (un bloc est un ensemble de 3 couches)
cakes_released = 0          #nombre de gateaux relâchés (normalement bien formés)
zones_availables = [2, 2, 2, 2]        #zones de blocs encore disponibles (non défoncés par un robot adverse) (sens horaire, en partant des paniers)(2=True,1=target,0=False)
zones_plats = [(0,0)]*5     #coordonnées du centre des 5 plats par ordre d'importance (=les plus éloignés de la safezone) pour la fin de la partie (TODO)
current_goal = [(0,0)]      #coordonnées de l'objectif actuel, à initialiser avec le premier point (1er bloc de ressources)
equipe = 3                  #3:bleu et 0:vert

#variables à ajuster au cours des tests
time_to_pdp = 10            #temps nécessaire pour mettre les pieds dans le plat (pdp)
calcul_frequency = 0.2      #Hertz

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
    #output : first move (order to motors or order to actioners)
    #publish for actioners to act
    #l'ordre peut être de stopper tout mouvement jusqu'à validation par les actionneurs internes (état stop moteur)
    
    table_description = data.data
    #moves possibles : aller chercher la ressource suivante, aller déposer les gateaux, mettre les pieds dans le plat
    #si on est déjà en récupération de ressources, on privilégie de terminer la recup
    time_left = 100 - (time.time() - init_time())
    if time_left < time_to_pdp:
        pieds_dans_le_plat(table_description)
    else:
        if not (processing or moving):
            if not depose:
                if blocks_grabbable == 0:
                    if blocks_grabbed == 6:
                        depose = True
                    else:
                        #réordonner les blocks
                        pass
                else:
                    for i in range(4):
                        if zones_availables[i]==1:
                            #go take the closest available block inside the target part and update status
                            hastarget = True
                            pass
                    if not hastarget :
                        if zones_availables[1] == 2 and zones_availables[2] == 2 :
                            #go to closest
                            #switch to ongoing
                            pass
                        else:
                            if zones_availables[equipe] == 2 :
                                #go there
                                #switch to ongoing
                                pass
                            elif zones_availables[3-equipe]==2 :
                                #go there
                                #switch to ongoing
                                pass
                            else:
                                #go to available from 1 and 2
                                pass

            else:
                #aller poser dans les zones_plats en remplissant par le bas puis se placer en protection
                pass
        
def pieds_dans_le_plat(table_description):
    #effectue l'action de mettre les pieds dans le plat
    for i in range(5):
        if zone_libre(i):
            type, x, y, rotation = get_move(zones_plats[i][0], zones_plats[i][1], table_description) #TODO
            talker_motors(type, x, y, rotation)
            break

    
def zone_libre(zone):
    #regarde si la zone selectionnée est libre
    #facile en regardant si les robots adverses intersectent la zone étudiée
    pass
    
def get_path(x, y, table_description):
    #aller chercher l'info dans le fichier de pathfinding
    #variable retournée : liste des coordonnées auxquelles il faut aller successivement
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
    pub.publish(message) #publish order to move
    moving = True
    
def talker_actioners(grab):
    # donne l'ordre aux actionneurs de se mettre en route pour ramasser les palets
    # type = (true: grab, false: release)
    pub = rospy.Publisher('order_actioners', String, queue_size=10)
    message = ("true" if grab else "false")
    pub.publish(message) #publish order to grab slices or to release them and put a cherry on it
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
        rate = rospy.Rate(calcul_frequency) 
        message = "true"
        pub.publish(message)


if __name__ == '__main__':
    listener()
