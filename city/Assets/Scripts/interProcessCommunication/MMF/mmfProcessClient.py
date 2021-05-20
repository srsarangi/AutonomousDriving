import os
import sys
import numpy as np
from PIL import Image
from io import BytesIO
import base64
import binascii
import traceback
from matplotlib import pyplot as plt
# for memory mappped files
import mmap
import struct
# for locks
import win32event
import win32con
import cv2
from test_PINet import init_testing, Testing
lane_agent=init_testing()
steering_angle=0
#All these are different data points made for calulcating homography matrix. the uncommented one is the latest and accurate of all
#prev pts=np.array([[437, 699], [1034, 699], [985, 620], [512, 620]], dtype=np.float32)
#new pts = np.array([[434, 699], [1034, 699], [978, 620], [503, 620]], dtype=np.float32)
#im_pts=np.array([[594, 699], [934, 699], [934, 620], [594, 620]], dtype=np.float32)
#new im_pts = np.array([[594, 699], [934, 699], [934, 460], [594, 460]], dtype=np.float32)
#im_pts=np.array([[594, 699], [934, 699], [934, 460], [594, 460]], dtype=np.float32)
#pts=np.array([[727, 493], [828, 493], [803, 421], [854, 421]], dtype=np.float32)
#im_pts=np.array([[726, 577], [817, 577], [816, 258], [898, 259]], dtype=np.float32)
pts=np.array([[727, 493], [828, 493], [803, 421], [854, 421]], dtype=np.float32) #points used to calculate homogrpahy matrix These are points in origional image
im_pts=np.array([[710, 577], [815, 577], [810, 258], [915, 259]], dtype=np.float32) #points used to calculate homography matrix these are points in top view of image
M = cv2.getPerspectiveTransform(pts, im_pts) # The homography matrix
Minv = cv2.getPerspectiveTransform(im_pts, pts) # Inverse homography
# try block is for debugging, handle specific exception if needed

try:
	
	mmf = mmap.mmap(0, 1000000, "Local\\imageTransfer") #mmf for image from the camera
	mmf2=mmap.mmap(0, 8, "Local\\steerAngle") #mmf for sharing steering angle
	mmf3=mmap.mmap(0, 4, "Local\\numLeft") #number of lanes detected
	mmf4=mmap.mmap(0, 4, "Local\\numRight") #total number of points detected
	# Arguments - mutex security (SYNCHRONIZE or MUTEX_ALL_ACCESS), initial owner bool, name of mutex
	lock = win32event.OpenMutex(win32con.SYNCHRONIZE, False, "lockForMMF") #locks for accessing mmf files
	lock2 = win32event.OpenMutex(win32con.SYNCHRONIZE, False, "lockSteer")
	#lock3 = win32event.OpenMutex(win32con.SYNCHRONIZE, False, "lockNum")
	while True: #infinite loop
		# wait forever (win32event.INFINITE) to access lock
		win32event.WaitForSingleObject(lock, win32event.INFINITE) #opening lock
		img_str = mmf.readline()
		#print(img_str)
		# setting cursor in file to begining to read from mmf's beginning agin
		mmf.seek(0)
		win32event.ReleaseMutex(lock)
		
		# Standard Image processing procedure
		image = Image.open(BytesIO(base64.b64decode(img_str)))
		image = np.asarray(image)
		# Execute image processing
		image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
		#try:
			#image=cv2.warpPerspective(image, M, image.shape[:2][::-1])
		in_x, in_y, res_img = Testing(lane_agent, image)
		#res_img=cv2.warpPerspective(res_img, M, res_img.shape[:2][::-1]) #obtain top view of image for given M
		lane_left=[] #appends indices of lines left to center of image here
		lane_right=[] #appends indices of lines right to center of image here
		max_xL=0  
		max_iL=-1 #max index in lane_left corresponds to left traffic line of current lane 
		min_xR=res_img.shape[1]
		min_iR=-1 #min index in right_left corresponds to right traffic line of current lane
		mid_pt=res_img.shape[1]/2 #center of image corresponds to car position
		total_pts=0
		for i in range(len(in_x)):
			x1=np.array(in_x[i])*res_img.shape[1]/512 #scaling of keypoints to origional size of image
			total_pts+=len(x1) #total points
			t=np.sum(x1[:5])/5
			if t <=mid_pt: #detects set of points to the left of image center
				lane_left.append(i)
				if(max_xL<t): #calculates index of traffic line just left of image center
					max_xL=t
					max_iL=i
					
			else:
				lane_right.append(i) #detects set of points to the right of image center
				if(min_xR>t): #calculates index of traffic line just left of image center
					min_xR=t
					min_iR=i
		if min_iR!=-1 and max_iL!=-1: #IF lane lines are present or not
				xL=np.matmul(M, np.vstack((np.array(in_x[max_iL])*res_img.shape[1]/512.0, np.array(in_y[max_iL])*res_img.shape[0]/256.0, np.ones(np.array(in_y[max_iL]).shape)))) #obtains key point in top view for left line
				xL=xL/xL[2, :] #converting from homogenous to non-homogenous coordinates for traffic line
				#same stes for right line
				xR=np.matmul(M, np.vstack((np.array(in_x[min_iR])*res_img.shape[1]/512.0, np.array(in_y[min_iR])*res_img.shape[0]/256.0, np.ones(np.array(in_y[min_iR]).shape))))
				xR=xR/xR[2, :]
				
				#Fitting polynomials
				b1=np.polyfit(xL[1, :], xL[0, :], 3)
				b2=np.polyfit(xR[1, :], xR[0, :], 3)
				yb1=b1[0]*xL[1, :]**3+b1[1]*xL[1, :]**2+b1[2]*xL[1, :]+b1[3] #evaluating bottom most coordinate for the traffic line left
				yb2=b2[0]*xR[1, :]**3+b2[1]*xR[1, :]**2+b2[2]*xR[1, :]+b2[3] #evaluating bottom most coordinate for the traffic line right
				#calculating derivative of polynomials
				dy1=3*b1[0]*xL[1, 1]**2+2*b1[1]*xL[1, 1]+b1[2]
				dy2=3*b2[0]*xR[1, 1]**2+2*b2[1]*xR[1, 1]+b2[2]
		
				steering_angle=0.2666667*180*(np.arctan((dy1+dy2)*0.5))/np.pi+0.6*steering_angle #estimating steering angle from slopes of polynomial. Taking wieghted average with prev one to remove noise
				#print(steering_angle)
				if(abs(steering_angle)<4): #thresholding to remove noise
					steering_angle=0
				
				cen=(yb1[0]+yb2[0])/2 #center of the current lane
				offset=res_img.shape[1]/2-cen #offset from lane centrer
				#adjusting steering angles for offset
				if(offset>40):
					steering_angle=10#np.arctan(offset/(res_img.shape[0]-(xL[1, 0]+xR[1, 0])*0.5))*180/np.pi#25*offset/250
				elif(offset<-40):
					steering_angle=-10#steering_angle=np.arctan(offset/(res_img.shape[0]-(xL[1, 0]+xR[1, 0])*0.5))*180/np.pi#25*offset/250
				#Converting to bytes and writing file to mmf 
				ba=bytearray(struct.pack("d", steering_angle))
				ba1=bytearray(struct.pack("i", total_pts))
				ba2=bytearray(struct.pack("i", len(lane_left)*10+len(lane_right)))
				win32event.WaitForSingleObject(lock2, win32event.INFINITE)
				mmf2.seek(0)
				mmf2.write(ba)
				mmf3.seek(0)
				mmf3.write(ba1)
				mmf4.seek(0)
				mmf4.write(ba2)
				win32event.ReleaseMutex(lock2)
				#print(steering_angle)
				#print(offset)
				#for i in range(len(yb1)): #can be used for plotting on image
				#	res_img=cv2.circle(res_img, (int(yb1[i]), int(xL[1, i])), 10, (255, 255, 0), -1)
				#for i in range(len(yb2)):
				#	res_img=cv2.circle(res_img, (int(yb2[i]), int(xR[1, i])), 10, (0, 255, 255), -1)
				#fig, ax = plt.subplots(1, 1, figsize=(50, 30))
				#ax.imshow(res_img)
				#ax.scatter(yb1, xL[1, :])
				#ax.scatter(yb2, xR[1, :])
				#ax.set_xlim((0, res_img.shape[1]))
				#ax.set_ylim((res_img.shape[0], 0))
				#plt.show()
				#plt.close()
		else: #if no lane line detected then just write total num points detected 
				ba1=bytearray(struct.pack("i", total_pts))
				ba2=bytearray(struct.pack("i", len(lane_left)*10+len(lane_right))) #number of left traffic lines and number of right traffic lines combined togehter
			
				win32event.WaitForSingleObject(lock2, win32event.INFINITE)
				mmf3.seek(0)
				mmf3.write(ba1)
				mmf4.seek(0)
				mmf4.write(ba2)
				win32event.ReleaseMutex(lock2)
		#cv2.imshow('res', res_img) #can be used to show image
		#cv2.waitKey(1)
		#cv2.imwrite('newimg.jpg', res_img)
			
		#except Exception as e:
		#	cv2.imshow("imges", image)
		#	cv2.waitKey(1)

except Exception as e:
	print(traceback.format_exc())
	print(e)
	# enable for debugging
	input()
