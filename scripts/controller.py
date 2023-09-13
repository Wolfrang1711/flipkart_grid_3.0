#!/usr/bin/env python3
from ast import Try
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import apriltag
import math
from geometry_msgs.msg import Twist

class controller:

    def __init__(self):

        rospy.init_node('controller', anonymous=True)
        self.sub = rospy.Subscriber("head/image_raw", Image, self.callback)
        self.pub = rospy.Publisher("grid_robot/cmd_vel", Twist, queue_size=10)

        self.VarAssign()

        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown(): 

            self.ImageProcess()            

            error = 0
            error = self.angle
            rectify = self.PID(error)
            # print(rectify)

            self.Orient(rectify)            
            self.Move(rectify)

            self.pub.publish(self.move) 
            rate.sleep()

    def VarAssign(self):

        self.img = None
        self.target_or = 0
        self.robo_or = 0  
        self.dist = 0
        self.angle = 0
        
        self.last_error = 0
        self.intg = 0
        self.diff = 0
        self.prop = 0
        self.kp = 0.39
        self.ki = 0
        self.kd = 0.01

        self.options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(self.options)     

        self.move = Twist()

    def callback(self, data):

        bridge = CvBridge()  

        try:
            self.img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
    
    def ImageProcess(self):

        gray = cv.cvtColor(self.img, cv.COLOR_BGR2GRAY)        
        results = self.detector.detect(gray)

        for r in results:

            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            cv.line(self.img, ptA, ptB, (0, 255, 0), 2)
            cv.line(self.img, ptB, ptC, (0, 255, 0), 2)
            cv.line(self.img, ptC, ptD, (0, 255, 0), 2)
            cv.line(self.img, ptD, ptA, (0, 255, 0), 2)

            (cx, cy) = (int(r.center[0]), int(r.center[1]))

            cv.circle(self.img, (cx, cy), 4, (0, 0, 255), -1)
            cv.putText(self.img, "Pose : ", (1100,90), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv.putText(self.img, str(cx), (1160,90), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv.putText(self.img, str(cy), (1200,90), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            (midx, midy) = int((ptB[0] + ptC[0]) / 2), int((ptB[1] + ptC[1]) / 2)

            cv.circle(self.img, (midx,midy), 3, (0, 0, 255), -1)            
            cv.circle(self.img, (x,y), 3, (0, 0, 255), -1)

            self.dist = np.linalg.norm(np.array((x,y)) - np.array((cx,cy))) 

            self.target_or = math.atan2(x-midx, y-midy)
            self.robo_or = math.atan2(midx-cx, midy-cy) 
            self.angle = self.target_or - self.robo_or              

            cv.putText(self.img, "Target Orientation : ", (998,120), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv.putText(self.img, str(round(self.target_or,4)), (1160,120), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv.putText(self.img, "Robot Orientation : ", (1000,150), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv.putText(self.img, str(round(self.robo_or,4)), (1160,150), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv.putText(self.img, "Target Distance : ", (1015,180), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv.putText(self.img, str(round(self.dist,4)), (1160,180), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv.line(self.img, (midx, midy), (x, y), (0, 0, 255), 2)
            
        cv.imshow("Image Feed",self.img)
        cv.waitKey(1)   

    def Orient(self, rectify):

        self.move.angular.z = rectify
        self.move.linear.x = 0

    def Move(self, rectify):

        if (self.dist <= 20):
                
            self.move.angular.z = 0
            self.move.linear.x = 0

        if (self.dist > 20 and abs(self.angle) < 0.1):
            
            self.move.angular.z = rectify 
            self.move.linear.x = 0.1

    def PID(self, error):

        self.prop = error
        self.intg = error + self.intg
        self.diff = error - self.last_error
        balance = (self.kp*self.prop) + (self.ki*self.intg) + (self.kd*self.diff)
        self.last_error = error
        return balance        

if __name__ == '__main__':

    x = int(input("Enter X Co-ordinate : "))
    y = int(input("Enter Y Co-ordinate : "))    

    if (x <= 1279 and y <= 767):   
        try:
            controller()
        except rospy.ROSInterruptException:
            pass

    else:
        print("Error!! Out of Range")
        exit()    
    

    