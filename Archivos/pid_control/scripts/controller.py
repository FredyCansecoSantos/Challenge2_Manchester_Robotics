#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import motor_output
from pid_control.msg import motor_input
from pid_control.msg import set_point

#Setup parameters, vriables and callback functions here (if required)
kp = rospy.get_param("/kp",0.0)
kd = rospy.get_param("/kd",1.0)
ki = rospy.get_param("/ki",0.5)

#global set_point_nuevo, set_point_tiempo, status_nuevo, output_nuevo, output_time_nuevo,cons_int , dt


#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__=='__main__':
    #Initialise and Setup node
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #global set_point_recibido, set_point_tiempo, status_nuevo, output_nuevo, output_time_nuevo, cons_int , dt
    #global set_point_recibido, output_nuevo, error, error_previo, cons_int , dt
    global error, error_previo, cons_int , dt

    set_point_recibido = 0
    #set_point_tiempo = 0
    #status_nuevo = ""
    #output_nuevo = 0
    #output_time = 0
    error = 0
    error_previo = 0
    cons_int = 0.01
    dt = 0.01
    output_nuevo = 0

    set_point_recibido = set_point()
    output_nuevo = motor_output()
    #set_point_recibido = 0

    #set_point_nuevo = set_point()

    def callback_output(msg):
        global output_nuevo
        output_nuevo = msg

        #global output_nuevo
        #global output_time
        #global output_status
        #output_msg = msg
        #$output_nuevo = output_msg.output
        #output_time = output_msg.time
        #output_status = output_msg.status

    def callback_input(msg):
        global set_point_recibido
        set_point_recibido = msg

        #global set_point_tiempo_nuevo
        #input_msg = msg
        #set_point_recibido = input_msg.input
        #set_point_tiempo_nuevo = input_msg.time

    #Setup Publishers and subscribers here
    rospy.Subscriber("/motor_output", motor_output, callback_output)
    rate = rospy.Rate(100)
    rospy.Subscriber("/set_point", set_point, callback_input)
    rate = rospy.Rate(100)

    control_msg = motor_input()
    control_pub = rospy.Publisher("/motor_input", motor_input, queue_size=10)

    print("The Controller is Running")
    #Run the node


    while not rospy.is_shutdown():
        #set_point_tiempo_dif = rospy.get_time()
    #cont = 0
    #    if cont == 0:
    #        control_pub.publish(set_point_recibido)
    #        cont+=1
    #    else:
        error = set_point_recibido.input - output_nuevo.output
            #tiempo = set_point_tiempo_dif - set_point_tiempo_nuevo

        derivative = kd * (error - error_previo) / dt

        proportional = kp * error

        integral =  ki * (cons_int + error * dt)
        # Salidad del sistema PID
        Respuesta_PID = derivative + proportional + integral

        error_previo = error

        control_msg.input = Respuesta_PID
        control_msg.time = rospy.get_time()
        #rospy.loginfo(control_msg)
        #rospy.loginfo("Set_Point_Recibido = %f" % set_point_recibido)
        #rospy.loginfo("Out_Put_Recibido = %f" % output_nuevo)
        #rospy.loginfo("Publicacion PID = %f" % control_msg)
        control_pub.publish(control_msg)

        rate.sleep()
