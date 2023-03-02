#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("Set_Point_Generator")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    signal_pub = rospy.Publisher("/set_point",set_point, queue_size=1)
    #time_pub = rospy.Publisher("set_point",set_point, queue_size=1)

    print("The Set Point Generator is Running")

    input = rospy.get_param("/input",10)

    #Run the node
    while not rospy.is_shutdown():

        ini_input = set_point()

        ini_input.time = rospy.get_time()
        ini_input.input = input*np.sin(rospy.get_time())
        #rospy.loginfo(ini_input)
        # Imprimir en la terminal lo que sale aqui en cada nodo
        signal_pub.publish(ini_input)

        rate.sleep()
