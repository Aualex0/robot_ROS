#!/usr/bin/env python

## Ce programme gère la prise de décision globale et la communication avec les différents actionneurs.

from pathfinding import *
import rospy
from robot_ROS.msg import Table_description
from robot_ROS.msg import Object_position_description
from std_msgs.msg import String
import math
import time

#debuggage
debug = True

processing = False          #True si action interne en cours qui nécessite l'arrêt du robot (grab ou release de gateaux)
moving = False              #True si le robot est actuellement en mouvement
depose = False             #True à partir du moment où on a récupéré les 18 couches
hastarget = True
init_time = time.time()     #initialisation du temps à t = 0
blocks_grabbable = 3        #nombre de block récupérable dans la pile de récupération
blocks_grabbed = 0          #nombre de blocs récupérés (un bloc est un ensemble de 3 couches)
cakes_released = 0          #nombre de gateaux relâchés (normalement bien formés)
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
    rot_origin = Object_position_description(object=new_origin.object,
                                            x = math.cos(new_origin.alpha)*point.x + math.sin(new_origin.alpha)*point.y, 
                                            y = - math.sin(new_origin.alpha)*point.x + math.cos(new_origin.alpha)*point.y,
                                            alpha = 0)
    projected.x -= rot_origin.x
    projected.y -= rot_origin.y
    return projected

def distance(origin, point):
    #TODO : calcul de distance effective via algorithmithme de pathfinding pour amélioration
    return math.sqrt((origin.x - point.x)**2 + (origin.y - point.y)**2)
  
def position_closest(list_objects, self_pos):
    #careful: this function removes said objects from database 
    dist = 3000
    point = None
    for i in range(len(list_objects)):
        if distance(self_pos, list_objects[i]) <= dist:
            point = i
            dist = distance(self_pos, list_objects[i])
    point = list_objects.pop(point)
    return point
        

def make_decision(data):
    global depose, hastarget
    #input : Table description, intern state, time left, moving, past actions
    #output : first move (order to motors or order to actioners)
    #publish for actioners to act
    #l'ordre peut être de stopper tout mouvement jusqu'à validation par les actionneurs internes (état stop moteur)
    
    table_description = data
    zones_availables = [[table_description.other.q1_value,     
                         table_description.other.q1],
                        [table_description.other.q2_value,
                         table_description.other.q2],
                        [table_description.other.q3_value,
                         table_description.other.q3],
                        [table_description.other.q4_value,
                         table_description.other.q4]]        #zones de blocs encore disponibles (non défoncés par un robot adverse) (sens horaire, en partant des paniers)(2=True,1=target,0=False)
    #moves possibles : aller chercher la ressource suivante, aller déposer les gateaux, mettre les pieds dans le plat
    #si on est déjà en récupération de ressources, on privilégie de terminer la recup
    time_left = 100 - (time.time() - init_time)
    if time_left < time_to_pdp:
        pieds_dans_le_plat(table_description)
    else:
        if not depose:
            if blocks_grabbable == 0:
                if blocks_grabbed == 6:
                    depose = True
                else:
                    #réordonner les blocks
                    pass
            else:
                for i in range(4):
                    if zones_availables[i][0]==1:
                        objective = position_closest(zones_availables[i][1], data.itself)
                        move_steps = pathfinding.main(objective.x, objective.y, data, equipe)
                        if debug:
                            message = str(move_steps[1][0]) + str(move_steps[1][0])
                            pub_debug.publish(message)
                        start_move(move_steps[1][0], move_steps[1][1], data.itself)
                        table_description.itself.x = move_steps[1][0]
                        table_description.itself.y = move_steps[1][1]
                        update_database(table_description)
                        pass
                        if len(zones_availables[i][1])==0:
                            hastarget = False
                            zones_availables[i][0]=0
                            if i == 0:
                                table_description.other.q1_value = 0
              
                            elif i == 1:
                                table_description.other.q2_value = 0
                        
                            elif i == 2:
                                table_description.other.q3_value = 0
                           
                            elif i == 3:
                                table_description.other.q4_value = 0
                
                            update_database(table_description)

                if not hastarget:
                    if zones_availables[1][0] == 2 and zones_availables[2][0] == 2 :
                        #go to closest
                        #switch to ongoing
                        pass
                    #TODO: cas une seule zone available
                    else:
                        if zones_availables[equipe][0] == 2 :
                            #go there
                            zones_availables[equipe][0] = 1
                            pass
                        elif zones_availables[3-equipe][0] == 2 :
                            #go there
                            zones_availables[3-equipe][0] = 1
                            pass
                        else:
                            #go to available from 1 and 2
                            pass

        else:
            #aller poser dans les zones_plats en remplissant par le bas puis se placer en protection (passage en phase 3 de test plus tard)
            pass
              
    message = "true"
    pub_ask_data.publish(message)
        
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


def start_move(x, y, self_pos, data):
    #prend en entrée un point atteignable et renvoie les instructions de rotation + translation correspondants
    objective = Object_position_description(object = "objective", x=x, y=y, alpha=0)
    talker_motors(1, 0, 0, 180/math.pi*math.atan2(objective.y - self_pos.y, objective.x - self_pos.x) - self_pos.alpha)
    talker_motors(2, distance(data.itself, objective), 0, 0)

def talker_motors(type, x, y, rotation):
    # format des données à envoyer aux moteurs principaux : (int type, float x, float y, int rotation).
    # type = (0:stop, 2:translation; 1:rotation)
    # coordonnées dans la base du robot
    # rotation sens trigo
    
    message = str(type) + "," + str(x) + "," + str(y) + "," + str(rotation)
    pub_order_move.publish(message) #publish order to move
    moving = True
    
    if debug:
        message = "commande envoyee au moteur : " + message
        pub_debug.publish(message)
    
def talker_actioners(grab):
    # donne l'ordre aux actionneurs de se mettre en route pour ramasser les palets
    # type = (true: grab, false: release)
    message = ("true" if grab else "false")
    pub_order_actioners.publish(message) #publish order to grab slices or to release them and put a cherry on it
    processing = True
    
def feedback_move(data):
    #reçoit le retour de l'arduino moteur (par ex si le mvmt a été effectué avec succès)
    #lance un nouvel ordre instantanément
    #reçoit "success" si c'est un succès, autre chose si c'est une erreur
    
    if debug:
        message = "feedback moteurs : " + data.data
        pub_debug.publish(message)
				
    if data.data == "success":
        moving = False
        message = "true"
        pub_ask_data.publish(message)
    else:
        pass
        #debug here
    
def intern_state(data):
    #reçoit l'état interne du robot (actionneurs internes)
    #lorsque les palets ont été ramassés avec succès, on peut lancer le prochain ordre de déplacement
    #reçoit "success" si c'est un succès, autre chose si c'est une erreur
    if data.data == "success":
        processing = False
        message = "true"
        pub_ask_data.publish(message)
    else:
        pass
        #debug here
        
def test_fct(data):
    if debug:
        message = "test starting"
        pub_debug.publish(message)
    table_description = data
    zones_availables = [[table_description.other.q1_value,     
                         table_description.other.q1],
                        [table_description.other.q2_value,
                         table_description.other.q2],
                        [table_description.other.q3_value,
                         table_description.other.q3],
                        [table_description.other.q4_value,
                         table_description.other.q4]]
    
    objective = position_closest(zones_availables[3][1], data.itself)
    update_database(table_description)
    
    print(zones_availables[3][1])
    
    move_steps = pathfinder(objective.x, objective.y, data, equipe)
    print(move_steps)
    if debug:
        message = "coordonnees a atteindre : x = " + str(move_steps[1][0]) + ", y = " + str(move_steps[1][1])
        pub_debug.publish(message)
        start_move(move_steps[1][0], move_steps[1][1], data.itself, data)
       
def starter_test(data):
    message = "true"
    pub_ask_data.publish(message)

def listener():

    while not rospy.is_shutdown():
    	if debug:
            rate = rospy.Rate(100)
            rate.sleep()
    		
    	else:
            rate = rospy.Rate(calcul_frequency) 
            message = "true"
            pub_ask_data.publish(message)
  
 
                
def update_database(data):
    pub_update.publish(data)
             


if __name__ == '__main__':
    global pub_ask_data, pub_debug

    pub_ask_data = rospy.Publisher('ask_data', String, queue_size=10)
    pub_debug = rospy.Publisher('debug', String, queue_size=10)
    pub_update = rospy.Publisher('update_objective', Table_description, queue_size=10)
    pub_order_move = rospy.Publisher('order_move', String, queue_size=10)
    pub_order_actioners = rospy.Publisher('order_actioners', String, queue_size=10)

    rospy.init_node('decision_maker', anonymous=True)

    if debug:
        rospy.Subscriber("send_data", Table_description, test_fct)
    else:
        rospy.Subscriber("send_data", Table_description, make_decision)
    rospy.Subscriber("feedback_move", String, feedback_move)
    rospy.Subscriber("intern_state", String, intern_state)
    rospy.Subscriber("starter", String, starter_test)

    listener()
