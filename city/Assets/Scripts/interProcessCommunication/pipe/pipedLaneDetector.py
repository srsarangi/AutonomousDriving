import sys
import os
import traceback
import win32file
import cv2
import numpy as np
from PIL import Image
from io import BytesIO
import base64
sys.path.insert(1, 'Scripts')
from cv_lane_detector import *

print("Begining")
# Pipe name to connect to is set here, in this case name is imageSendPipe
fileHandle = win32file.CreateFile("\\\\.\\pipe\\imageSendPipe", win32file.GENERIC_READ, 0, None, win32file.OPEN_EXISTING, 0, None)

detector = LaneDetector()
try:
	while (True):
		# First argument the size of image in bytes being transmitted over the pipe
		left1, data1 = win32file.ReadFile(fileHandle, 11)
		size = int(data1)/2
		data2 = b""
		i = size
		# Set size of data to be fetched per ReadFile call
		bufferSize = 4096
		# Collecting the bytes for image untill it matches fixed size
		while(True):
			left2, s2 = win32file.ReadFile(fileHandle, bufferSize)
			data2 += s2
			i -= len(s2)
			if (i < bufferSize):
				bufferSize = i
			if (i <= 0):
				break
		# removing end of line for this image frame from pipe 
		left3, data3 =  win32file.ReadFile(fileHandle, 2)
		image = Image.open(BytesIO(base64.b64decode(data2)))
		image = np.asarray(image)
		# Execute image processing
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		try:
			res_image = detector.process(image)
			cv2.imshow("img", res_image)
			cv2.waitKey(1)
		except Exception as e:
			cv2.imshow("img", image)
			cv2.waitKey(1)
except Exception as e:
	print(traceback.format_exc())
	print(e)
	# enable for debugging
	# input()
