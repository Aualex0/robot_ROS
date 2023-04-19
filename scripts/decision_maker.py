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
depose = False              #True à partir du moment où on a récupéré les 18 couches
grab = False                #True si la prochaine action est de grab
hastarget = False           #True si ibjectif en cours
init_time = time.time()     #initialisation du temps à t = 0
blocks_grabbable = 3        #nombre de block récupérable dans la pile de récupération
blocks_grabbed = 0          #nombre de blocs récupérés (un bloc est un ensemble de 3 couches)
cakes_released = 0          #nombre de gateaux relâchés (normalement bien formés)
zones_plats = [(1000,1000)]*5     #coordonnées du centre des 5 plats par ordre d'importance (=les plus éloignés de la safezone) pour la fin de la partie (TODO)
current_goal = (0,0,0)        #coordonnées de l'objectif actuel de dépose (safe_zone, puis 2e zone, puis pdp)
equipe = 3                  #3:bleu et 0:vert
pdp = False                 #phase de pdp initiée

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
    obj = list_objects.pop(point)
    return obj
        

def make_decision(data):
    global depose, grab, processing, hastarget, current_goal, blocks_grabbable, blocks_grabbed, pdp
    #input : Table description, intern state, time left, moving, past actions
    #output : first move (order to motors or order to actioners)
    #publish for actioners to act
    #l'ordre peut être de stopper tout mouvement jusqu'à validation par les actionneurs internes (état stop moteur)
    
    if debug:
        message = "decision maker called"
        pub_debug.publish(message)
    
    table_description = data
    
    print("itself pos : " + str(table_description.itself.x) + ", " + str(table_description.itself.y))
    
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
    print("time left : " + str(time_left))
    print("blocks grabbed : " + str(blocks_grabbed))
    print("blocks grabbable : " + str(blocks_grabbable))
    if time_left < time_to_pdp and pdp == False:
        pieds_dans_le_plat(table_description)
    else:
        if not depose:
            if blocks_grabbable == 0:
                if blocks_grabbed == 6:
                    message = "stade de depose atteint"
                    pub_debug.publish(message)
                    processing = False
                    depose = True
                    make_decision(data)
                else:
                    message = "stade de reordonnement atteint"
                    pub_debug.publish(message)
                    message = ("reorder")
                    pub_order_actioners.publish(message)
                    processing = True
                    processing = False #TODO à enlever après le 2e arduino
                    blocks_grabbable = 3 #TODO à enlever après le 2e arduino
                    
            elif not processing and not hastarget: #on peut bouger vers une autre zone
                       
                for i in range(4):
                    if zones_availables[i][0]==1:
                        
                        objective = position_closest(zones_availables[i][1], data.itself)
                        hastarget = True
                        current_goal = (objective.x, objective.y, i)
                        
                        move_steps = pathfinder(objective.x, objective.y, data, equipe)
                        
                        if move_steps == []:
                            return False
                        
                        if move_steps[1][0] == objective.x and move_steps[1][1] == objective.y:
                            grab = True
                            processing = True  
                            hastarget = False
                            blocks_grabbable -= 1
                            blocks_grabbed += 1       
                        
                        print(zones_availables[i][1])
                        print("move steps : " + str(move_steps))
                        
                        start_move(move_steps[1][0], move_steps[1][1], data.itself)
                        
                        if debug:
                            message = "coordonnees a atteindre : x = " + str(move_steps[1][0]) + ", y = " + str(move_steps[1][1])
                            pub_debug.publish(message)
                        
                        table_description.itself.x = move_steps[1][0]
                        table_description.itself.y = move_steps[1][1]
                        update_database(table_description)
                        
                        if len(zones_availables[i][1])==0:
                            zones_availables[i][0]=0
                            if i < 3 and zones_availables[i+1][0] != 0:
                                zones_availables[i+1][0] = 1
                            if i == 0:
                                table_description.other.q1_value = 0
                                table_description.other.q2_value = 1
                                update_database(table_description)
                                break
                                
                            elif i == 1:
                                table_description.other.q2_value = 0
                                update_database(table_description)
                                print("toutes les zones ont été explorées")
                                depose = True
                        
                            elif i == 2:
                                table_description.other.q3_value = 0
                                table_description.other.q4_value = 1
                                update_database(table_description)
                                break
                           
                            elif i == 3:
                                table_description.other.q4_value = 0
                                table_description.other.q1_value = 1
                                update_database(table_description)
                                break
                
                            
                            
            elif hastarget and not processing:
                
                i = current_goal[2]
                move_steps = pathfinder(current_goal[0], current_goal[1], data, equipe)
                
                if move_steps == []:
                    return False
                
                if move_steps[1][0] == current_goal[0] and move_steps[1][1] == current_goal[1]:
                    grab = True
                    processing = True
                    hastarget = False
                    blocks_grabbable -= 1
                    blocks_grabbed += 1 
                
                print(zones_availables[i][1])
                print("move steps : " + str(move_steps))
                        
                if True:
                    if True:
                        
                        if debug:
                            message = "coordonnees a atteindre : x = " + str(move_steps[1][0]) + ", y = " + str(move_steps[1][1])
                            pub_debug.publish(message)
                            start_move(move_steps[1][0], move_steps[1][1], data.itself)
                        
                        table_description.itself.x = move_steps[1][0]
                        table_description.itself.y = move_steps[1][1]
                        update_database(table_description)
                        
                        if len(zones_availables[i][1])==0:
                            zones_availables[i][0]=0
                            if i < 3 and zones_availables[i+1][0] != 0:
                                zones_availables[i+1][0] = 1
                            if i == 0:
                                table_description.other.q1_value = 0
                                table_description.other.q2_value = 1
                                
                            elif i == 1:
                                table_description.other.q2_value = 0
                                table_description.other.q3_value = 1
                        
                            elif i == 2:
                                table_description.other.q3_value = 0
                                table_description.other.q4_value = 1
                           
                            elif i == 3:
                                table_description.other.q4_value = 0
                                depose = True
                
                            update_database(table_description)
                
            else: 
                talker_actioners()
                processing = False #TODO à enlever après le 2e arduino

        else: #depose est à True
            depose_ressources(table_description)

def depose_ressources(data):
    #utiliser current_goal
    #TODO grab la dernière ressource puis aller déposer aux 2 endroits
    for i in range(5) :
	if zone_libre(i, data):
	    #TODO : if in zone  then depose, reset blocks grabbed; else call pathfinding
	    break
    print("dépos de ressources")
    
def pieds_dans_le_plat(data):
    #effectue l'action de mettre les pieds dans le plat
    global hastarget, current_goal, blocks_grabbable, pdp, depose, grab
    
    message = "temps limite ecoule, phase des pieds dans le plat initiee"
    pub_debug.publish(message)
    
    for i in range(5):
        if zone_libre(i, data):
            hastarget = True
            depose = False
            pdp = True
            blocks_grabbable = -1
            current_goal = (zones_plats[i][0], zones_plats[i][1], 3)
            make_decision(data)
            break


global libre = [True,True,True,True,True]
def zone_libre(zone, data):
    #regarde si la zone selectionnée est libre
    #TODO calculer en dur avec data.opponent et data.opp_annex
    return libre[zone]


def start_move(x, y, self_pos):
    #prend en entrée un point atteignable et renvoie les instructions de rotation + translation correspondants
    objective = Object_position_description(object = "objective", x=x, y=y, alpha=0)
    talker_motors(1, 0, 0, 180/math.pi*math.atan2(objective.y - self_pos.y, objective.x - self_pos.x) - self_pos.alpha)
    talker_motors(2, distance(self_pos, objective), 0, 0)

def talker_motors(type, x, y, rotation):
    # format des données à envoyer aux moteurs principaux : (int type, float x, float y, int rotation).
    # type = (0:stop, 2:translation; 1:rotation)
    # coordonnées dans la base du robot
    # rotation sens trigo
    
    global moving
    
    message = str(type) + "," + str(x) + "," + str(y) + "," + str(rotation)
    pub_order_move.publish(message) #publish order to move
    moving = True
    
    if debug:
        message = "commande envoyee au moteur : " + message
        pub_debug.publish(message)
    
def talker_actioners():
    # donne l'ordre aux actionneurs de se mettre en route pour ramasser les palets
    # type = (true: grab, false: release)
    
    global processing, grab
    
    message = ("grab" if grab else "depose")
    
    if debug:
        message = "commande envoyee aux actioneurs : " + message
        pub_debug.publish(message)
    
    pub_order_actioners.publish(message) #publish order to grab slices or to release them and put a cherry on it
    processing = True
    grab = False
    
def feedback_move(data):
    #reçoit le retour de l'arduino moteur (par ex si le mvmt a été effectué avec succès)
    #lance un nouvel ordre instantanément si c'est une translation
    #reçoit "success" si c'est un succès, autre chose si c'est une erreur
    
    global moving
    
    if debug:
        message = "feedback moteurs : " + data.data
        pub_debug.publish(message)
				
    if data.data == "translation ok":
        moving = False
        message = "true"
        pub_ask_data.publish(message)
    else:
        pass
        #debug here
    
def feedback_actioners(data):
    #reçoit l'état interne du robot (actionneurs internes)
    #lorsque les palets ont été ramassés avec succès, on peut lancer le prochain ordre de déplacement
    #reçoit "success" si c'est un succès, autre chose si c'est une erreur
    
    global processing
    
    if debug:
        message = "feedback moteurs : " + data.data
        pub_debug.publish(message)
    
    if data.data != "error":
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
    
    print(zones_availables[3][1])
    
    move_steps = pathfinder(objective.x, objective.y, data, equipe)
    print(move_steps)
    if debug:
        message = "coordonnees a atteindre : x = " + str(move_steps[1][0]) + ", y = " + str(move_steps[1][1])
        pub_debug.publish(message)
        start_move(move_steps[1][0], move_steps[1][1], data.itself)
       
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
        #rospy.Subscriber("send_data", Table_description, test_fct)
        rospy.Subscriber("send_data", Table_description, make_decision)
    else:
        rospy.Subscriber("send_data", Table_description, make_decision)
    rospy.Subscriber("feedback_move", String, feedback_move)
    rospy.Subscriber("feedback_actioners", String, feedback_actioners)
    rospy.Subscriber("starter", String, starter_test)

    listener()
