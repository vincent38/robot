#!/usr/bin/env python

import cv2
import sys
import rospy as rp
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import UInt8, Int8
from cv_bridge import CvBridge, CvBridgeError
import time
import traceback
import random

"""
TODO:

    - Improve the lookup pattern, to find a face in the range reachable by the robot

    - Change the face detection process if possible and compute the time taken by both to compare them ?

    - Implement Tilt lookup (first find a person via tilt, then lift the head until tilt is at 90 degrees. Problem: no idea where the verin is and what is its "range") => Is it even possible ?

    - Improve primitive tracking of a person (if face lost, process the move on same direction for 2 seconds, and either catch back the face or swap to lookup) 
    
"""

"""
This class manages the camera feed processing and the sending of commands to the robot.

Needs to be connected on the same network than the robot to work, if running on a separate platform.
Please ensure that ROS is correctly configured on both platforms in this case.
"""

LOOKUP_MODE = 1
DETECTION_MODE = 2

class CamCtrl:
    """
    Handles the initialization of the node, the registration of the different publishers,
    and the registration on the webcam feed sent by the tablet.
    """
    def __init__(self):
	"""
	Configuration variables for the mode switch and the tracking feature
	"""
	random.seed()
	self.hasData = False
	#
	self.mode = LOOKUP_MODE
	self.lastRot = 0 #-1 left 1 right 0 IDLE
	self.lastUD = 0 #-1 down 1 up 0 IDLE
	self.iTrack = 0
	self.padding = 5
	"""
	Node initialization
	"""
        print("[INFO] Initializing class...")
	rp.init_node("Test", anonymous=True)
        self.bridge = CvBridge()
        self.pV = rp.Publisher('verin_speed', UInt8, queue_size=10)
        self.pH = rp.Publisher('cmdhead', Int8, queue_size=10)
	print("[DEBUG] Loading CV2 classifier ?")
        self.face_cascade = cv2.CascadeClassifier('/home/robair/RobAIR/catkin_ws/src/face_alignment/src/haarcascade_frontalface_default.xml')
	print("done.")
        self.s = rp.Subscriber('usb_cam/image_raw/compressed', CompressedImage, self.printIt, queue_size=1)
        self.sH = rp.Subscriber('head', Int8, self.updateHead)
        self.head = 0
        self.rate = rp.Rate(25)
        print("[INFO] Init done.")

    """
    Callback for current head position setting
    """
    def updateHead(self, data):
        self.head = data.data
        #print("[INFO] New head position:", self.head)

    """
    Callback used when we receive data on the subscribed topic
    """
    def printIt(self, data):
        #rp.loginfo("==========CALLED=========")
        print("called")
	self.hasData = True
	self.gData = data
        #time.sleep(2)

    """
    Main loop , checks what's happening on ros and process the callbacks and data publishing
    """
    def run(self):
        print("[INFO] Running application.")
        while not rp.is_shutdown():
	    try:
		start = time.time()
		# Do the process only if we have received an image		
		if self.hasData:
			# Always consider that no face is detected on the newly caught image. This is handled later on.
			hasFace = False
			bigw = 0
			bigh = 0

			bigx = 320
			bigy = 240

			try:
			    # Try to read the image as a cv2 data
			    cvi = self.bridge.compressed_imgmsg_to_cv2(self.gData, "bgr8")
			except CvBridgeError as e:
			    print(e)
			    exit()
			# Use the legacy face detection
			gray = cv2.cvtColor(cvi, cv2.COLOR_BGR2GRAY)
			faces = self.face_cascade.detectMultiScale(gray, 1.1, 4)
			# For all potential faces found, loop and only keep the biggest rectangle detected (i.e. the closest)
			# In any case, consider that a face is found as soon as condition is triggered and swap to detection mode
			for (x, y, w, h) in faces:
			    cv2.rectangle(cvi, (x, y), (x+w, y+h), (255, 0, 0), 2)
			    if w > bigw and h  > bigh:
				bigw = w
				bigh = h
				bigx = x + w/2
				bigy = y + h/2
				hasFace = True
				self.mode = DETECTION_MODE
			# If a face is found on the camera output
			if self.mode == DETECTION_MODE and hasFace: # Follow the face by rotating and translating the face
			    """
			    We log the movements done by the robot to follow the face. This is used on the tracking step later.
			    """
			    rp.loginfo("FACE FOLLOW")
			    if bigx < 300:
			        # Left
			        self.head = self.head - 5
				self.lastRot = -1
			        if self.head < -128:
				    self.head = -128
   			    elif bigx > 340:
			        # Right
			        self.head = self.head + 5
				self.lastRot = 1
			        if self.head > 127:
				    self.head = 127
			    else:
				self.lastRot = 0
			    self.pH.publish(self.head)
			    # Up/Down on h
			    if bigy < 220:
			        #Command up
			        self.pV.publish(127)
				self.lastUD = 1
			    elif bigy > 260:
			        #Command down
			        self.pV.publish(129)
				self.lastUD = -1
			    else:
			        self.pV.publish(128)
				self.lastUD = 0
			elif self.mode == DETECTION_MODE and not hasFace: # Switch to tracking mode
			    """
			    When the face is lost, swapping to tracking step (not done when coming from lookup).
			    Use the last movements recorded by the logging system on the last follow step, so that the robot tries to continue
			    the previous moments for a few iterations.
			    Outcomes: face detected and we return to previous step, otherwise we swap to the lookup phase.
			    """
			    rp.loginfo("FACE TRACK")
			    # x-Iteration track before lookup
			    if self.lastRot == -1:
			        # Left
			        self.head = self.head - 5
			        if self.head < -128:
				    self.head = -128
   			    elif self.lastRot == 1:
			        # Right
			        self.head = self.head + 5
			        if self.head > 127:
				    self.head = 127
			    self.pH.publish(self.head)
			    # Up/Down on h
			    if self.lastUD == 1:
			        #Command up
			        self.pV.publish(127)
			    elif self.lastUD == -1:
			        #Command down
			        self.pV.publish(129)
			    else:
			        self.pV.publish(128)
			    self.iTrack = self.iTrack + 1
			    if self.iTrack > 40:
				self.iTrack = 0
				self.mode = LOOKUP_MODE
			elif self.mode == LOOKUP_MODE:
			    """
			    Lookup mode, the robot wanders in its authorized rotation and translation space. It continues until the face detection
			    grabs a face and throws us back in the detection phase.
			    The head rotates around its authorized range, translation is a unmanaged random speed seting being sent.
			    """
			    rp.loginfo("LOST IN THOUGHTS")
			    print("Lookup")
			    self.lastRot = 0
			    self.lastUD = 0
			    self.head = self.head + self.padding
			    if self.head < -128:
				self.head = -128
				self.padding = 5
			    if self.head > 127:
				self.head = 127
				self.padding = -5
			    v = random.randint(127,129)
			    self.pH.publish(self.head)
			    #self.pV.publish(v)
			cv2.imshow("Webcam feed", cvi)
			cv2.waitKey(3)
			self.hasData = False
		end = time.time()
		print("Spent", end - start, "seconds on iteration") 
	        self.rate.sleep()
	    except KeyboardInterrupt:
	        print("[INFO] Called keyboard interruption...")
	        cv2.destroyAllWindows()

if __name__ == '__main__':
    print("[INFO] Starting application...")
    #print("App has loaded the following modules:")
    #for i in sys.modules:
    #	print("\t"+i)
    try:
        app = CamCtrl()
        app.run()
    except:
        print("[ERROR] Application stopped. Reason:",sys.exc_info()[0])
	print(sys.exc_info())
        print(traceback.print_stack())
        exit()
