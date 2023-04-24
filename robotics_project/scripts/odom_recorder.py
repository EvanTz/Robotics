#!/usr/bin/python3

# Tzortzis Evangelos 3088
# Mpakolas Theodoros 3035
# Zamparas Nikolaos  2969

import math
import csv
import rospy
from nav_msgs.msg import Odometry


def odom_rec(msg):
    # here write to csv file for comparison of the simulation with the calculations
    x = round(msg.pose.pose.position.x,3)
    y = round(msg.pose.pose.position.y,3)
    theta = round(msg.pose.pose.orientation.z,3)*2
    
    angular_speed = round(msg.twist.twist.angular.z,3)
    linear_speed = round(msg.twist.twist.linear.x,3)
    
    if abs(angular_speed) < 0.001:
    	angular_speed = 0.0
    if abs(linear_speed) < 0.001:
    	linear_speed = 0.0
	
    timestamp = round(msg.header.stamp.secs)+round(msg.header.stamp.nsecs/(10e8),4)

    row = [x, y, theta, linear_speed, angular_speed, timestamp]
    # print('x:',x)
    # print('y:',y)
    # print('theta:',theta)
    # print('timestamp: ',timestamp,' secs')
    # print('timestamp: ',msg.header.stamp.secs,' secs, ',msg.header.stamp.nsecs,' nsecs, ')
    
    with open('data_odom.csv', 'a') as csvFile:
        writer = csv.writer(csvFile)
        writer.writerow(row)
        csvFile.close()


def odom_rec_node_local():
    rospy.init_node('odom_recorder',anonymous=False)
    rospy.Subscriber('pioneer/odom',Odometry,callback=odom_rec)
    print('Odometry recorder started.')
    
    # while not rospy.is_shutdown():
    #     pass
    
    # clear previous data
    save_file = open("data_odom.csv", "w")
    save_file.truncate()
    save_file.close()

    rospy.spin() # kanei to idio me to while apo panw


# called from trajectory_publisher as a topic of the same node
def odom_rec_node():
    # rospy.init_node('odom_recorder',anonymous=False)
    rospy.Subscriber('pioneer/odom',Odometry,callback=odom_rec)
    print('Odometry recorder started.')

    # clear previous data
    save_file = open("data_odom.csv", "w")
    save_file.truncate()
    save_file.close()


if __name__ == '__main__':
    try:
        odom_rec_node_local()
    except rospy.ROSInterruptException:
        pass

