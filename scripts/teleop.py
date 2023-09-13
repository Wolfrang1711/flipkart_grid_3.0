#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from geometry_msgs.msg import Twist

class controller:

    def __init__(self):

        rospy.init_node('controller', anonymous=True)
        self.pub = rospy.Publisher("grid_robot/cmd_vel", Twist, queue_size=10)

        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown(): 

            self.move = Twist()

            self.Teleop()           

            self.pub.publish(self.move) 
            rate.sleep()

    def Teleop(self):

        screen = np.zeros((512,512,3),np.uint8)

        cv.putText(screen, "Teleop Controller", (35,40), cv.FONT_HERSHEY_PLAIN, 3, (255,255,255), 1) 
        cv.putText(screen, "Click Here to Begin......", (60,80), cv.FONT_HERSHEY_PLAIN, 2, (255,255,255), 1) 
        cv.putText(screen, "Press and Hold W A S D", (50,160), cv.FONT_HERSHEY_PLAIN, 2, (255,255,255), 1)
        cv.putText(screen, "to control the robot", (70,200), cv.FONT_HERSHEY_PLAIN, 2, (255,255,255), 1)
        
        screen = cv.resize(screen, None, fx=0.8, fy=0.8, interpolation=cv.INTER_LINEAR)
        
        cv.imshow("Teleop",screen)

        if cv.waitKey(100) & 0xFF == ord('w'):  #Forward
            self.move.linear.x = 0.5
        if cv.waitKey(100) & 0xFF == ord('a'):  #left turn
            self.move.angular.z = 1.0  
        if cv.waitKey(100) & 0xFF == ord('s'):  #backward
            self.move.linear.x = -0.5          
        if cv.waitKey(100) & 0xFF == ord('d'):  #right turn
            self.move.angular.z = -1.0       

if __name__ == '__main__':

    try:
        controller()        
    except rospy.ROSInterruptException:
        pass