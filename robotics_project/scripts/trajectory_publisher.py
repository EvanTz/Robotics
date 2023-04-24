#!/usr/bin/python3

# Tzortzis Evangelos 3088
# Mpakolas Theodoros 3035
# Zamparas Nikolaos  2969

import math
import movement_function
import odom_recorder
import rospy
from geometry_msgs.msg import Twist

def publisher():
    # rospy.init_node('velocity_pub_node',anonymous=False)
    pub = rospy.Publisher('pioneer/cmd_vel', Twist, queue_size=100)
    
    rate = rospy.Rate(100)

    velocity_msg = Twist()
	
    print('Velocity publisher started.')

    ########################

    ###### SET DIFFERENT COORDINATES AND ANGLES HERE FOR TESTING ######

    ###### !!!!!!!!!!!! an allaksoun ta shmeia, tha ginei overwrite h apothikeysh tou odometry me ta nea shmeia 
    ###### !!!!!!!!!!!! ektos an ginei comment to odom_recorder.odom_rec_node() sth grammh 97 

    # initial point
    x_0 = 0
    y_0 = 0
    theta_0 = 0

    # intermediate point
    # round(3088/200) = 15 
    # round(3088/400) = 8
    x_i = 15
    y_i = 8

    # final point
    # round(3088/200) = 15 
    # -round(3088/400) = -8
    x_f = 15
    y_f = -8

    # 3088/2000 = 1.544 radians
    theta_f = 1.544

    ###################################################################

    theta_i= math.atan2(y_i-y_0,x_i-x_0)
    
    theta_temp = math.atan2(y_f-y_i,x_f-x_i)

    # time segment durations
    t_0,t_1,t_2,t_3,t_4,t_5 = movement_function.calc_time_segments(x_0,y_0,theta_0,
                                                                    x_i,y_i,theta_i,
                                                                    x_f,y_f,theta_f)


    ########################
	
	
	
    # xreiazetai to sleep giati alliws to start_time einai 0
    rate.sleep()
    start_time = rospy.get_time()
    print('start_time: ',start_time)
    
    with open('start_time.txt', 'w') as start_time_file:
        start_time_file.write('start_time = '+str(start_time))
        start_time_file.close()


    while not rospy.is_shutdown():
        current_time = rospy.get_time()
        # print('current_time: ',current_time)

        true_time = current_time-start_time
        # print('true_current_time: ',true_time)

        data = movement_function.trajectory_calc(true_time) # douleuei kai auto me tis standard times

        #data = movement_function.trajectory_calc(true_time,
        #                                        t_0=t_0,t_1=t_1+4,t_2=t_2,t_3=t_3,t_4=t_4,t_5=t_5,  # added 4 in the first turn because of too much angular accelaration since tb = tf*10%
        #                                        x_0=x_0,y_0=y_0,theta_0=theta_0,
        #                                        x_1=x_i,y_1=y_i,theta_1=theta_i, 
        #                                        x_f=x_f,y_f=y_f,theta_final=theta_f)
        

        vel = math.sqrt(data[3]**2 + data[4]**2)
        

        velocity_msg.linear.x = vel
        velocity_msg.angular.z = data[5]
        
        pub.publish(velocity_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('velocity_pub_node',anonymous=False)

        odom_recorder.odom_rec_node() # comment this line to run odom_rec_node separately
        
        publisher()
    except rospy.ROSInterruptException:
        pass

