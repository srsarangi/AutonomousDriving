import os
import sys
import numpy as np
from PIL import Image
from io import BytesIO
import base64
import traceback

# try block is for debugging, handle specific exception if needed
try:
	sys.path.insert(1, 'Scripts\\interProcessCommunication\\PetersonLock')
	from PetersonLock import *

	# for memory mappped files
	import mmap
	# for locks
	import win32event
	import win32con

	sys.path.insert(1, 'Scripts\\externalScripts')
	from cv_lane_detector import *
	detector = LaneDetector()
	# from lanenet_detector import *
	# os.chdir(".\\Scripts\\externalScripts")
	# print("--------starting session---------")
	# start_sess()
	# print("--------started session---------")

	# input("Press Enter to start.") 
	mmf = mmap.mmap(0, 1000000, "Local\\"+sys.argv[1])

	lock = PetersonLock(sys.argv[2], 2, True)

	while True:
		lock.acquire()
		# need to read the file as required here image is a string
		mmf.seek(0)
		img_str = mmf.readline()
		# setting cursor in file to begining to read from mmf's beginning agin
		lock.release()

		# Standard Image processing procedure
		image = Image.open(BytesIO(base64.b64decode(img_str)))
		image = np.asarray(image)
		# Execute image processing
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		try:
			cv2.imshow("shared img", image)
			res_image = detector.process(image)
			# res_image, res2 = lanenet_process(image)
			cv2.imshow("img", res_image)
			# cv2.imshow("img2", res2)
			cv2.waitKey(1)
		except Exception as e:
			cv2.imshow("img", image)
			print(e)
			cv2.waitKey(1)

except Exception as e:
	print(traceback.format_exc())
	print(e)
	# enable for debugging
	input()
