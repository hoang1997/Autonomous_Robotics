#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback

class find_object:

    def __init__(self):
        
        # Windows to display image output
        cv2.startWindowThread()
        
  
        # Current position of the robot
        self.robotposx = 0
        self.robotposy = 0

        
        # Specifying the colours and whether or not they have been found
        self.red = False
        self.yellow = False
        self.blue = False
        self.green = False
        
        
        # Waypoints for the turtlebot to follow when it is unable 
        # to see any coloured objects, and whether they have been reached
        self.waypoint1x = 1.564
        self.waypoint1y = -4.437
        self.waypoint1sent = False
        self.waypoint1reached = False
        
        self.waypoint2x = 2.000
        self.waypoint2y = 0.000
        self.waypoint2sent = False
        self.waypoint2reached = False
        
        self.waypoint3x = -0.500
        self.waypoint3y = -1.400
        self.waypoint3sent = False
        self.waypoint3reached = False
        
        self.waypoint4x = 0
        self.waypoint4y = 4.461
        self.waypoint4sent = False
        self.waypoint4reached = False
        
        self.waypoint5x = -4.4
        self.waypoint5y = 1.528
        self.waypoint5sent = False
        self.waypoint5reached = False
        
        self.waypointsReached = 0
        self.obstaclePresent = False

        
        # Declaring Subscriber and Publisher variables through which info of
        # the area around the robot can be determined and manipulated
        self.bridge = CvBridge()
                                          
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",
                                          Image, self.callback)
                                         
        self.robotPos = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.closeWaypoint)
        
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)



    def callback(self, data):
        cv2.namedWindow("Image Raw", 2)
        cv2.namedWindow("Image window", 1)
        try:
            # Convert image data from robot sensors to a type readable by cv
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e
                                 
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, d = cv_image.shape
        
        
        # Pre-declaring necessary variables in case all if-statements fail        
        hsv_thresh = cv2.inRange(hsv_img,
                                     numpy.array((255, 255, 255)),
                                     numpy.array((255, 255, 205)))

        hsv_pixels = cv2.moments(hsv_thresh)
        r = hsv_pixels
        y = hsv_pixels
        b = hsv_pixels
        g = hsv_pixels

        
        # If statements creating thresholded images for each colour as well as
        # pixel counts for each 
        if self.red == False: 
            red_thresh = cv2.inRange(hsv_img,
                                     numpy.array((0, 200, 30)),
                                     numpy.array((5, 255, 150)))
            r = cv2.moments(red_thresh)

        elif self.yellow == False: 
            yellow_thresh = cv2.inRange(hsv_img,
                                     numpy.array((30, 200, 30)),
                                     numpy.array((40, 255, 200)))
            y = cv2.moments(yellow_thresh)
        elif self.blue == False: 
            blue_thresh = cv2.inRange(hsv_img,
                                     numpy.array((90, 200, 30)),
                                     numpy.array((120, 255, 230)))
            b = cv2.moments(blue_thresh)
        elif self.green == False:
            green_thresh = cv2.inRange(hsv_img,
                                     numpy.array((60, 200, 30)),
                                     numpy.array((80, 255, 200)))
            g = cv2.moments(green_thresh)


        
        # Draw red circle on currently targetted object        
        hsv_thresh, hsv_contours, hierachy = cv2.findContours(hsv_thresh.copy(),
                                                  cv2.RETR_TREE,
                                                  cv2.CHAIN_APPROX_SIMPLE)
                                                     
        for c in hsv_contours:
            a = cv2.contourArea(c)
            if a > 100.0:
                cv2.drawContours(cv_image, c, -1, (255, 0, 0))


        # End condition - if all beacons are found, cancel current goal and shut down program
        if self.red == True and self.yellow == True and self.blue == True and self.green == True:
            ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            ac.cancel_goal()            
            rospy.loginfo("All objects found!")
            rospy.signal_shutdown("Found all objects")
        
        
        
        # Check which of the beacons has yet to be found, and whether or not
        # they take up a sufficient area of the screen to not drag the robot 
        # into obstacles. After the first beacon, check to see if the previous
        # beacon has been found.
        if r['m00'] > 650000 and self.red == False: 
            hsv_thresh = red_thresh
            self.red = self.moveToBeacon(cv_image, hsv_thresh)
        elif y['m00'] > 650000 and self.yellow == False and self.red == True:
            hsv_thresh = yellow_thresh
            self.yellow = self.moveToBeacon(cv_image, hsv_thresh)
        elif b['m00'] > 650000 and self.blue == False and self.yellow == True:
            hsv_thresh = blue_thresh
            self.blue = self.moveToBeacon(cv_image, hsv_thresh)
        elif g['m00'] > 650000 and self.green == False and self.blue == True:
            hsv_thresh = green_thresh
            self.green = self.moveToBeacon(cv_image, hsv_thresh)
        else:
            # If robot is unable to see any of the beacons, begin traveling
            # towards each of the 5 waypoints in turn
            if self.waypoint1reached == False and self.waypoint1sent == False:
                self.moveToWaypoint(self.waypoint1x, self.waypoint1y, 1)
            elif self.waypoint2reached == False and self.waypoint1reached == True and self.waypoint2sent == False:
                self.moveToWaypoint(self.waypoint2x, self.waypoint2y, 2)
            elif self.waypoint3reached == False and self.waypoint2reached == True and self.waypoint3sent == False:
                self.moveToWaypoint(self.waypoint3x, self.waypoint3y, 3)
            elif self.waypoint4reached == False and self.waypoint3reached == True and self.waypoint4sent == False:
                self.moveToWaypoint(self.waypoint4x, self.waypoint4y, 4)
            elif self.waypoint5reached == False and self.waypoint4reached == True and self.waypoint5sent == False:
                self.moveToWaypoint(self.waypoint5x, self.waypoint5y, 5)
            
        # Display current coloured thresholded image as well as the 
        # raw image for inspection    
        cv2.imshow("Image window", hsv_thresh)
        cv2.waitKey(1)
        cv2.imshow("Image", cv_image)  
        cv2.waitKey(1)
            
        
        
        
    # Define function for moving the robot towards an identified beacon
    def moveToBeacon(self, cv_image, hsv_thresh):    
        h, w, d = cv_image.shape
        hsv_test = hsv_thresh
        search_top = h*0.5
        search_bot = h * 0.79
        limit = h * 0.80
        
        # Action client to cancel the current goal on the navigation stack
        # so that finding beacons takes priority over waypoints
        ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        ac.cancel_goal()
        
        # If any of the thresholded object is directly in front of the robot
        # return true to change the status of the beacon to found
        if hsv_test[limit:h, 0:w].any():
            rospy.loginfo("Found object!")
            return True
        
        
        hsv_thresh[0:search_top, 0:w] = 0
        hsv_thresh[search_bot:h, 0:w] = 0
        M = cv2.moments(hsv_thresh)

        # Check that object is present in thresholded image    
        if M['m00'] > 0:
            
            # Determine the distance of the object from the centre of the screen
            # in order to correct the path of the robot if it's off course
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 20, (0,0,255), -1)
            # Build the twist message to send to the robot telling it to move
            twist_msg = Twist();
            err = cx - w/2
            twist_msg.linear.x = 0.3
            twist_msg.angular.z = -float(err) / 100
            self.vel_pub.publish(twist_msg)
          
        # Once finished, return False stating that the object hasn't currently
        # been located
        return False
        
    # Defining function to move the robot between waypoints around the map     
    def moveToWaypoint(self, x, y, goalNum):
       
       # Create an action client to send goals to the navigation stack
       ac = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

       # Wait for the action client to be available
       while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
           rospy.loginfo("Waiting for the move_base action server to come up")
       
       # Begin setting up the goal message to send to the stack
       goal = MoveBaseGoal()
       goal.target_pose.header.frame_id = "map"
       goal.target_pose.header.stamp = rospy.Time.now()
       
       # Specify each of the goal positions
       goal.target_pose.pose.position.x = x
       goal.target_pose.pose.position.y = y
       goal.target_pose.pose.orientation.x = 0.0
       goal.target_pose.pose.orientation.y = 0.0
       goal.target_pose.pose.orientation.z = 0.0
       if goalNum == 5:
           goal.target_pose.pose.orientation.z = -0.5
       goal.target_pose.pose.orientation.w = 1.0
       
       # Send the waypoint to the navigation stack
       ac.send_goal(goal)


       # Change the corresponding waypoint status to "sent" to stop the program
       # from flooding the stack with the same message when another callback is called
       if goalNum == 1:
           self.waypoint1sent = True
           print goalNum
       elif goalNum == 2:
           self.waypoint2sent = True
           print goalNum
       elif goalNum == 3:
           self.waypoint3sent = True
           print goalNum
       elif goalNum == 4:
           self.waypoint4sent = True
           print goalNum
       elif goalNum == 5:
           self.waypoint5sent = True
           print goalNum
#           

    # Define function to close the waypoint once the robot has reached it
    def closeWaypoint(self, data):

        # Take data from move_base/feedback to find the current position of the
        # robot
        self.robotposx = data.feedback.base_position.pose.position.x
        self.robotposy = data.feedback.base_position.pose.position.y
        self.robotorx = data.feedback.base_position.pose.orientation.x
        self.robotory = data.feedback.base_position.pose.orientation.y
        self.robotorz = data.feedback.base_position.pose.orientation.z
        error = 0.50     
        
        
        # Check whether the robot's current position is within the error region 
        # specified, if so set the waypoint's status to found and increment 
        # the number of waypoints found.
        if self.waypoint1sent == True and self.waypoint1reached == False:
            if self.robotposx >= (self.waypoint1x - error) and self.robotposx <= (self.waypoint1x + error) and self.robotposy >= (self.waypoint1y - error) and self.robotposy <= (self.waypoint1y + error):
                self.waypoint1reached = True
                self.waypointsReached += 1
        elif self.waypoint2sent == True and self.waypoint2reached == False:
            if self.robotposx >= (self.waypoint2x - error) and self.robotposx <= (self.waypoint2x + error) and self.robotposy >= (self.waypoint2y - error) and self.robotposy <= (self.waypoint2y + error):
                self.waypoint2reached = True
                self.waypointsReached += 1
        elif self.waypoint3sent == True and self.waypoint3reached == False:
            if self.robotposx >= (self.waypoint3x - error) and self.robotposx <= (self.waypoint3x + error) and self.robotposy >= (self.waypoint3y - error) and self.robotposy <= (self.waypoint3y + error):
                self.waypoint3reached = True
                self.waypointsReached += 1
        elif self.waypoint4sent == True and self.waypoint4reached == False:
            if self.robotposx >= (self.waypoint4x - error) and self.robotposx <= (self.waypoint4x + error) and self.robotposy >= (self.waypoint4y - error) and self.robotposy <= (self.waypoint4y + error):
                self.waypoint4reached = True
                self.waypointsReached += 1
        elif self.waypoint5sent == True and self.waypoint5reached == False:
            if self.robotposx >= (self.waypoint5x - error) and self.robotposx <= (self.waypoint5x + error) and self.robotposy >= (self.waypoint5y - error) and self.robotposy <= (self.waypoint5y + error):
                self.waypoint5reached = True
                self.waypointsReached += 1

     
if __name__ == '__main__':
    # Initialise a node of the find_objects class
    rospy.init_node("find_objects", anonymous=True)
    cv = find_object()
    
    rospy.spin()
    cv2.destroyAllWindows()
