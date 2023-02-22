#!/usr/bin/env python

## Ce programme traite les données arrivant du Lidar et les formatte pour la base de donnée.
## Il les transmets ensuite au node collision_monitor, qui met à jour la base de données.

import rospy
from robot_ROS.msg import Table_description
from robot_ROS.msg import Object_position_description

def talker():
    pub = rospy.Publisher('update_data', Table_description, queue_size=10)
    rospy.init_node('input_analyser', anonymous=True)
    rate = rospy.Rate(100) # 100hz
    table_descr = Table_description(itself = Object_position_description(object="main", x=3000-140, y=225, alpha=180),
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
    
    while not rospy.is_shutdown():
        #update here the database
        #may need to add a listener
        
        pub.publish(table_descr)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
