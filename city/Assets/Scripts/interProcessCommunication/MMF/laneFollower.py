import os
import sys
import numpy as np
from PIL import Image
from io import BytesIO
import base64
import traceback
import time
import struct

sys.path.insert(0, 'Scripts\\interProcessCommunication\\PetersonLock')
from PetersonLock import *

# for memory mappped files
import mmap
# for locks
import win32event
import win32con
import cv2

sys.path.insert(0, 'Scripts\\externalScripts')
from lane_steering import lane_follower

# try block is for debugging, handle specific exception if needed
try:
	mmfImageTransfer = mmap.mmap(0, 1000000, "Local\\" + sys.argv[1])

	lockImageTransfer = PetersonLock(sys.argv[2], 2, True)

	mmfRetrievedInfo = mmap.mmap(0, 8,  "Local\\" + sys.argv[3])
	lockRetrievedInfo = PetersonLock(sys.argv[4], 2, True)

	mmfServerState = mmap.mmap(0, 4,  "Local\\" + sys.argv[5])
	mmfClientState = mmap.mmap(0, 4,  "Local\\" + sys.argv[6])

	start_time = time.time_ns()
	frames = -1

	mmfClientState.seek(0)
	mmfClientState.write(PetersonLock.intToBytes(1))
	print("READY!")

	serverState = 0
	while serverState == 0:
		mmfServerState.seek(0)
		serverState = PetersonLock.bytesToInt(mmfServerState.read(4))

	while True:
		# current_time = time.time_ns()
		# if current_time - start_time > 1000000000:
		# 	print("fps: " + str(frames))
		# 	frames = 0
		# 	start_time = current_time
		# frames += 1

		mmfServerState.seek(0)
		serverState = PetersonLock.bytesToInt(mmfServerState.read(4))

		if serverState == 0:
			break

		lockImageTransfer.acquire()
		# need to read the file as required here image is a string
		img_str = mmfImageTransfer.readline()
		# setting cursor in file to begining to read from mmf's beginning agin
		mmfImageTransfer.seek(0)
		lockImageTransfer.release()
		
		# Standard Image processing procedure
		image = Image.open(BytesIO(base64.b64decode(img_str)))
		image = np.asarray(image)
		# Execute image processing
		# image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		try:
			cv2.imshow('Shared image', image)
			lane_img, steering_angle, lane_offset = lane_follower(image)
			cv2.imshow('Lane Fit', lane_img)
			# print("Steering Angle: ", steering_angle)
			# print("Lane Offset: ", lane_offset)
			cv2.waitKey(1)
			
			# Store in endian convension of the system
			byte_array_steering_angle = None
			byte_array_lane_offset = None

			if sys.byteorder == 'little':
				byte_array_steering_angle = struct.pack('<f', steering_angle)
				byte_array_lane_offset = struct.pack('<f', lane_offset)
			else:
				byte_array_steering_angle = struct.pack('>f', steering_angle)
				byte_array_lane_offset = struct.pack('>f', lane_offset)

			lockRetrievedInfo.acquire()
			mmfRetrievedInfo.seek(0)
			mmfRetrievedInfo.write(byte_array_steering_angle)
			mmfRetrievedInfo.seek(4)
			mmfRetrievedInfo.write(byte_array_lane_offset)
			lockRetrievedInfo.release()
		except Exception as e:
			print(traceback.format_exc())
			print(e)
			input()

	mmfClientState.seek(0)
	mmfClientState.write(PetersonLock.intToBytes(0))

except Exception as e:
	print(traceback.format_exc())
	print(e)
	# enable for debugging
	input()
