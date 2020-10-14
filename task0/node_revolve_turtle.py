#!/usr/bin/env python

'''**********************************
E-yantra
Theme: Vitran Drone
Task: task0
Purpose: To Move the turtle in circular motion
Team ID : 583
Team name : !ABHIMANYU 
**********************************'''

#Importing required modules
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import time
from std_srvs.srv import Empty

# declare global variables to store pose
x=0
y=0
yaw = 0


'''**********************************
Function name   :   poseCallback
Functionality   :   callback function to store the pose in global variables
Arguments       :   pose_message
Return Value    :   None
***********************************'''
def poseCallback(pose_message):     
    global x , y, yaw

    x = pose_message.x
    y = pose_message.y
    yaw = pose_message.theta


'''**********************************
Function name   :   revolve
Functionality   :   move the turtle in circle
Arguments       :   radius, linear speed, no of revolutions
Return Value    :   None
***********************************'''
def revolve(r, speed, n):
    velocity_message = Twist()

    global x, y, yaw

    yaw0 = yaw
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # setting loop rate

    # dectareing the cmd_vel publisher
    cmd_vel_topic = '/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

    while True:

        delta_theta = abs(yaw-yaw0)  # change in yaw, i,e. delta theta
        yaw0 = yaw
        if(delta_theta>6):           # this will work when yaw will change from 3.14 to -3.14, so here it will take 2*pi extra change
            delta_theta-=2*np.pi     # here we subtract the 2*pi extra change

        distance_moved += r*abs(delta_theta)    # radius*angle gives the length of arc

        rospy.loginfo("distance_moved : " + str(distance_moved))    # logging the distance moved
        
        if(distance_moved >= n*2*np.pi*r):  # terminating the loop after n revolution
            break

        velocity_message.linear.x = speed
        velocity_message.angular.z = speed/r
        velocity_publisher.publish(velocity_message)    # publishing the velocity message
        loop_rate.sleep()

    velocity_message.linear.x = 0
    velocity_message.angular.z = 0
    velocity_publisher.publish(velocity_message)    # publishing the zero velocity to stop
    rospy.loginfo("destination reached")


#main function
def main():    
    
    # Making the script a ROS Node.
    rospy.init_node('node_turtle_revolve', anonymous = True)

    # dectareing the pose subscriber
    position_topic = '/turtle1/pose'
    pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
    time.sleep(2)

    # Print info on console.
    rospy.loginfo(" task0 VD E-yantra, Team ID : 583, Team name : !ABHIMANYU ")
    
    radius = 2
    linear_speed = 2
    revolution = 1
    rospy.loginfo("moving the turtle in radius : " + str(radius)+ " unit , linear speed : " + str(linear_speed)+ " unit, revolution : " + str(revolution))
    
    # calling function to move turtle in a circle
    revolve(radius, linear_speed , revolution)


if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass
