"""
	Author: Mudit Soni
"""
import socketio
from flask import Flask
import eventlet

import numpy as np
import cv2
from PIL import Image
from io import BytesIO
import base64
import argparse

# from cv_lane_detector import *
# model= 'cv_lane_detector'

# def init_args():
# 	parser = argparse.ArgumentParser()
# 	parser.add_argument('--models', type=str, nargs='+')
# 	parser.add_argument('--display_result', action='store_true', default= False)
# 	return parser.parse_args()

# Define server
sio = socketio.Server()
app= Flask(__name__)

@sio.on('connect')
def connect(sid, environ):
	print(sid, " connected")
	start()

def start():
	sio.emit("instr",{"instructions":"connected"})

@sio.event
def disconnect(sid):
	cv2.destroyAllWindows()
	print(sid, ' disconnected')

@sio.on("image")
def input(sid, data):
	process_inp(data)

def process_inp(data):
	imgString = data["img"]
	image = Image.open(BytesIO(base64.b64decode(imgString)))
	image = np.asarray(image)
	image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

	# if model== 'cv_lane_detector':
	# 	try:
	# 		res_image= detector.process(image)
	# 	except:
	# 		res_image= image
	# 	data_dict= {"instructions":"do something"}

	# else:
	# 	res_image= lanenet_process(image)
	# 	#res_image= image
	# 	data_dict= {"instructions":"do something"}
	
	# if args.display_result==True:
	cv2.imshow("img",res_image)
	cv2.waitKey(1)

	sio.emit("instr",data_dict)


if __name__ == '__main__':
	# args = init_args()
	# if args.models!= None: 
	# 	model= args.models[0] 

	# if model== 'cv_lane_detector':
	# 	detector= LaneDetector()
	# else:
	# 	from lanenet_detector import *
	# 	start_sess()
	
	app = socketio.Middleware(sio, app)
	eventlet.wsgi.server(eventlet.listen(('', 5000)), app)

