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
    while not rospy.is_shutdown():
        table_descr = Table_description(itself = Object_position_description(object="main", x=0, y=0, alpha=0),
                                        annex = Object_position_description(object="annex", x=0, y=0, alpha=0),
                                        opp_main = Object_position_description(object="opp_main", x=0, y=0, alpha=0),
                                        opp_annex = Object_position_description(object="opp_annex", x=0, y=0, alpha=0))
        pub.publish(table_descr)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
