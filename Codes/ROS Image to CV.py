#!/usr/bin/env python
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys

bridge =CvBridge()  # initialise the object

def imgcallback(ros_image):
    global bridge
    #convert ros image to opencv convertible image 
    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image,'bgr8')
    except CvBridgeError as e:
        print(e)
    # Therefore image is converted to opencv image - the following code does image preocessing
    (rows,columns,channels)=cv_image.shape

    cv2.imshow("Image Window",cv_image)
    cv2.waitKey(1000)



rospy.init_node('imgConverter',anonymous=True)
# include topic of the image 
#initialise image callback function also specify the topic you have to receive data from
image_sub=rospy.Subscriber("/camera/rgb/image_raw",Image,imgcallback)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")

cv2.destroyAllWindows()
