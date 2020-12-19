#!/usr/bin/env python

'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
from pyzbar.pyzbar import decode
from std_msgs.msg import String,Float64, Int32MultiArray
import matplotlib.pyplot as plt
import rospkg

class image_detection():

    # Initialise everything
    def __init__(self):
        rospy.init_node('marker_detect')  # Initialise rosnode
        self.img = np.empty([])
        # This will contain your image frame from camera
        self.bridge = CvBridge()
        # self.pub = rospy.Publisher('qrValue',String,queue_size=1)
        # self.pub = rospy.Publisher('/edrone/marker_data',Float64,queue_size=1)
        self.pub = rospy.Publisher('pixValue',Int32MultiArray,queue_size=1)
        self.rate = rospy.Rate(1)
        # Subscribing to the camera topic
        self.image_sub = rospy.Subscriber(
            "/edrone/camera/image_raw", Image, self.image_callback)

        rospack = rospkg.RosPack()
        filepath = rospack.get_path('vitarana_drone')+'/data/cascade.xml'
        self.logo_cascade = cv2.CascadeClassifier(filepath)

        rospy.spin()



    # Callback function of camera topic
    def image_callback(self, data):
        try:
            # Converting the image to OpenCV standard image
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
            logo = self.logo_cascade.detectMultiScale(gray, scaleFactor=1.05,minNeighbors=5)

            for (x, y, w, h) in logo:
                cv2.rectangle(self.img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                #print(x + w/2)
                #print(y + h/2)
                centre_x = x + w/2
                centre_y = y + h/2
                data_for_publishing = Int32MultiArray(data = [centre_x, centre_y])
                self.pub.publish(data_for_publishing)


            # plt.imshow(cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB))
            cv2.imshow('image',self.img)
            cv2.waitKey(1)
            # plt.show()
            # plt.clf()

        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    try:
        image_dec_obj = image_detection()
    except rospy.ROSInterruptException:
        pass
