# crockeries_detection
One of the NODE for a crockeries detection project 

Utilized ROS OpenCV to detect crockeries
1. Hough Circle Transform [Circle Detection]
2. Center Pixel [Types of crockeries]

---------------------------------------------------------------

	#!/usr/bin/env python

	import rospy
	from sensor_msgs.msg import Image
	from cv_bridge import CvBridge, CvBridgeError
	from std_msgs.msg import Float64MultiArray
	import cv2
	import numpy as np

	class CircleDetector:
    	def __init__(self):
        	  	rospy.init_node('circle_detector', anonymous=True)
        	  	self.bridge = CvBridge()
        	  	self.circle_info_pub = rospy.Publisher('circle_info', Float64MultiArray, queue_size=10)
        	  	self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

    	def image_callback(self, data):
        		try:
            		cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        		except CvBridgeError as e:
            		print(e)
            		return

        		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        		circles = cv2.HoughCircles(
            		gray,
            		cv2.HOUGH_GRADIENT,
            		dp=1,
            		minDist=50,
            		param1=50,
            		param2=30,
            		minRadius=10,
            		maxRadius=1000
        		)

        		if circles is not None:
            		circle_info_msg = Float64MultiArray()
            		for i, circle in enumerate(circles[0, :]):
              			center_pixel_color = cv_image[int(circle[1]), int(circle[0])]
              			circle_info = [circle[0], circle[1], center_pixel_color[0], center_pixel_color[1], center_pixel_color[2]]
              			circle_info_msg.data.extend(circle_info)
            		self.circle_info_pub.publish(circle_info_msg)

	def main():
    	try:
        		circle_detector = CircleDetector()
        		rospy.spin()
    	except KeyboardInterrupt:
        		pass  

	if __name__ == '__main__':
    	main()
