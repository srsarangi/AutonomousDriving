import os
import sys
import numpy as np
from PIL import Image
from io import BytesIO
import base64
import traceback

try:
	sys.path.insert(1, 'Scripts/interProcessCommunication/PetersonLock')
	from PetersonLock import *

	# for memory mappped files
	import mmap
	# for locks
	import win32event
	import win32con

	sys.path.insert(1, 'Scripts/carDrivingAlgo/darknet')
	from darknet import unity_processing

# try block is for debugging, handle specific exception if needed
	
	mmf = mmap.mmap(0, 1000000, "Local\\imageTransferDarknet")

	lock = PetersonLock("MMFDarknet", 2, True)

	while True:
		lock.acquire()
		img_str = mmf.readline()
		# setting cursor in file to begining to read from mmf's beginning agin
		mmf.seek(0)
		lock.release()
		
		# Standard Image processing procedure
		image = Image.open(BytesIO(base64.b64decode(img_str)))
		image = np.asarray(image)
		# Execute image processing
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		try:
			res_image = unity_processing(image)
			cv2.imshow("img", res_image)
			cv2.waitKey(1)
		except Exception as e:
			cv2.imshow("img", image)
			cv2.waitKey(1)

except Exception as e:
	print(traceback.format_exc())
	print(e)
	# enable for debugging
	input()
