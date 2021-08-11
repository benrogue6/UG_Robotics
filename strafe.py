#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class dodge_cube:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    
    self.image_sub = rospy.Subscriber('/camera/color/image_raw',Image, self.image_callback)

    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

   #convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,127,255,0)

   #find contours in the binary image
    gray_image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    lower_black = numpy.array([ 0, 0, 0])
    upper_black = numpy.array([180, 255, 30])

    lower_white = numpy.array([0,0,180])
    upper_white = numpy.array([255,255,255])

    mask1 = cv2.inRange(hsv, lower_black, upper_black)
    mask2 = cv2.inRange(hsv, lower_white, upper_white)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20
 
    mask1[0:search_top, 0:w] = 0
    mask1[search_bot:h, 0:w] = 0
    mask2[0:search_top, 0:w] = 0
    mask2[search_bot:h, 0:w] = 0

    M1 = cv2.moments(mask1)
    M2 = cv2.moments(mask2)

    for c in contours:
    # calculate moments for each contour
     M3 = cv2.moments(c)  
    if M3["m00"] > 0:
      cx3 = int(M3["m10"] / M3["m00"])
      cy3 = int(M3["m01"] / M3["m00"])
    else:
      cx, cy = 0, 0

 
    #follow the sidewalk 
    if M1['m00'] > 0:
      cx1 = int(M1['m10']/M1['m00'])
      cy1 = int(M1['m01']/M1['m00'])
      err = cx1 - w/2
      
      self.twist.linear.x = 0.45
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)

    elif M1['m00'] > 0:
      cx1 = int(M1['m10']/M1['m00'])
      cy1 = int(M1['m01']/M1['m00'])
      err = cx1 - w/2

      #track for change in the x position of black mask blob on the white sidewalk
      if cx1 > 400:
        self.twist.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twist)

      elif cx1 < 250:
        self.twist.angular.z = float(err) / 100

        self.twist.linear.x = 0.7
        self.cmd_vel_pub.publish(self.twist)

      #track for change in the x position of the black mask blob on the middle dark sidewalk
      if M1 == gray_image_black:
    #  self.twist.linear.x = 0.3
       self.twist.angular.z = -0.4
       self.cmd_vel_pub.publish(self.twist)

      elif M1 != gray_image_black:
     # self.twist.angular.z = -float(err) / 100

       self.twist.linear.x = 0.7
       self.cmd_vel_pub.publish(self.twist)

      
   #cv2.imshow("white_mask",mask2)
    cv2.imshow("black_mask",mask1)
    cv2.imshow("Gray_image",gray_image)
   #cv2.imshow("output", image)
    cv2.waitKey(3)

rospy.init_node('roll')
dodge_cube = dodge_cube()
rospy.spin()
