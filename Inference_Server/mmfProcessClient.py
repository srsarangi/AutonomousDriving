import os
import sys
import numpy as np
import cupy as cp
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
pts=np.array([[163, 253], [346, 253], [294, 181], [216, 181], [229, 164], [281, 164], [274, 154], [237, 154]], dtype=np.float32)
im_pts=np.array([[217, 253], [293, 253], [293, 177], [217, 177], [218, 115], [293, 115], [293, 40], [218, 40]], dtype=np.float32)
#im_pts=np.array([[726, 577], [817, 577], [816, 258], [898, 259]], dtype=np.float32)
#pts=np.array([[229, 224], [297, 223], [288, 202], [279, 182], [241, 182], [236, 202]], dtype=np.float32) #points used to calculate homogrpahy matrix These are points in origional image
#im_pts=np.array([[236, 230], [287, 230], [287, 201], [287, 151], [236, 151], [236, 201]], dtype=np.float32) #points used to calculate homography matrix these are points in top view of image
M, _= cv2.findHomography(pts, im_pts) # The homography matrix
Mcp=cp.array(M)
offset=0
prevoffset=0
laneChangeMode=True;
x_sp=cp.linspace(start=-480, stop=256)
xtsp=np.round(x_sp, decimals=0).astype(int)+int(480)
#Minv = cv2.getPerspectiveTransform(im_pts, pts) # Inverse homography
# try block is for debugging, handle specific exception if needed
def process2(cLy, cRy, num_left, num_right, bL, bR):
    xL=np.linspace(start=cp.min(cLy).get(), stop=cp.max(cLy).get())
    xR=np.linspace(start=cp.min(cRy).get(), stop=cp.max(cRy).get())
    yL=bL[0]*xL**3+bL[1]*xL**2+bL[2]*xL+bL[3]
    yR=bR[0]*xR**3+bR[1]*xR**2+bR[2]*xR+bR[3]
    width=yR[0]-yL[0]
    left_lim=yL[yL.shape[0]-1]-330
    #print("left_lim", left_lim)
    print("width", width)
    yL=yL-left_lim
    yR=yR-left_lim
    Lx=[]
    Rx=[]
    for j in range(num_left):
         Lx.append(yL-j*width)
    for j in range(num_right):
         Rx.append(yR+j*width)
    canvas=np.zeros((1000, 800), dtype='uint8')
    canvas[:, :]=128
    for x in Lx:
         for i in range(x.shape[0]):
              canvas=cv2.circle(canvas,(int(round(x[i])), int(round(xL[i]))+480), 5, (255,255,255), -1)
    for x in Rx:
         for i in range(x.shape[0]):
              canvas=cv2.circle(canvas,(int(round(x[i])), int(round(xR[i]))+480), 5, (255,255,255), -1)
    car_pos=cp.matmul(Mcp, cp.array([256, 256, 1])[:, cp.newaxis])
    car_pos=car_pos/car_pos[2, :]
    car_pos[0, 0]=car_pos[0, 0]-left_lim
    car_pos[1, 0]=car_pos[1, 0]+480
    canvas=cv2.rectangle(canvas, (car_pos[0, 0]-25, car_pos[1, 0]-25), (car_pos[0, 0]+25, car_pos[1, 0]+25), (255, 255, 255), -1)
    cv2.imshow('res', canvas)
    cv2.waitKey(1)

def process(in_x, in_y):
    total_pts=0
    for x in in_x:
	        total_pts+=len(x)
    B=[]
    Lx=[]
    Ly=[]
    if total_pts>50:
            for i in range(len(in_x)):
                    x1=cp.array(in_x[i])
                    x2=cp.array(in_y[i])
                    xL=cp.matmul(Mcp, cp.vstack((x1, x2, cp.ones(x2.shape)))) #obtains key point in top view for left line
                    xL=xL/xL[2, :]
                    Lx.append(xL[0])
                    Ly.append(xL[1])
                    #B.append(np.polyfit(xL[1, :].get(), xL[0, :].get(), 3))
            #B=cp.array(B)
            #x=cp.repeat(cp.array([256**3, 256**2, 256, 1])[cp.newaxis, :], len(in_x), axis=0)
            #interc=cp.sum(x*B, axis=1)
            #sorted_ind=cp.argsort(interc)
            #interc=interc[sorted_ind]
            #x=cp.array([-480**3, 480**2, -480, 1])
            #left_lim=min(cp.sum(x*B[sorted_ind[0], :]), interc[0])-10
            #right_lim=max(cp.sum(x*B[sorted_ind[sorted_ind.shape[0]-1], :]), interc[sorted_ind.shape[0]-1])
            #print(left_lim, right_lim)
            canvas=np.zeros((736, 550), dtype='uint8')
            canvas[:, :]=128
            #B[:, B.shape[1]-1]=B[:, B.shape[1]-1]-left_lim
            L=[]
            for i in range(len(Lx)):
                    #y=B[i, 0]*x_sp**3+B[i, 1]*x_sp**2+B[i, 2]*x_sp+B[i, 3]
                    #y=np.round(y, decimals=0).astype(int)
                    for j in range(len(Lx[i])):
                            #canvas=cv2.circle(canvas,(y[i], xtsp[i]), 5, (255,255,255), -1)
                            canvas=cv2.circle(canvas,(Lx[i][j], Ly[i][j]+480), 5, (255,255,255), -1)
                    #L.append(np.round(cp.hstack((y, x_sp)).get(), decimals=0).astype(int))
            #res_img=cv2.polylines(canvas, L, True, (255, 255, 255))
            cv2.imshow('res', canvas)
            cv2.waitKey(1)


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
		#process(in_x, in_y)
		for i in range(len(in_x)):
			x1=cp.array(in_x[i])#*res_img.shape[1]/512 #scaling of keypoints to origional size of image
			total_pts+=len(x1) #total points
			t=cp.sum(x1[:5])/5
			if t <=mid_pt: #detects set of points to the left of image center
				lane_left.append(i)
				if(max_xL<t and in_y[i][0]>220): #calculates index of traffic line just left of image center
					max_xL=t
					max_iL=i
					
			else:
				lane_right.append(i) #detects set of points to the right of image center
				if(min_xR>t and in_y[i][0]>220): #calculates index of traffic line just left of image center
					min_xR=t
					min_iR=i
		if min_iR!=-1 and max_iL!=-1: #IF lane lines are present or not
				xL=cp.matmul(Mcp, cp.vstack((cp.array(in_x[max_iL]), cp.array(in_y[max_iL]), cp.ones(cp.array(in_y[max_iL]).shape)))) #obtains key point in top view for left line
				xL=xL/xL[2, :] #converting from homogenous to non-homogenous coordinates for traffic line
				#same stes for right line
				xR=cp.matmul(Mcp, cp.vstack((cp.array(in_x[min_iR]), cp.array(in_y[min_iR]), cp.ones(cp.array(in_y[min_iR]).shape))))
				xR=xR/xR[2, :]
				#print("Here is your maximums")
				#print(cp.min(xL[1, :]))
				#print(cp.min(xR[1, :]))
				#Fitting polynomials
				b1=np.polyfit(xL[1, :].get(), xL[0, :].get(), 3)
				b2=np.polyfit(xR[1, :].get(), xR[0, :].get(), 3)
				#yb1=b1[0]*xL[1, 0]**3+b1[1]*xL[1, 0]**2+b1[2]*xL[1, 0]+b1[3] #evaluating bottom most coordinate for the traffic line left
				#yb2=b2[0]*xR[1, 0]**3+b2[1]*xR[1, 0]**2+b2[2]*xR[1, 0]+b2[3] #evaluating bottom most coordinate for the traffic line right
				#calculating derivative of polynomials
				dy1=3*b1[0]*xL[1, 1]**2+2*b1[1]*xL[1, 1]+b1[2]
				dy2=3*b2[0]*xR[1, 1]**2+2*b2[1]*xR[1, 1]+b2[2]
				steering_angle=54*(cp.arctan((dy1+dy2)*0.5))/cp.pi+0.7*steering_angle
				#steering_angle=0.2666667*180*(cp.arctan((dy1+dy2)*0.5))/np.pi+0.6*steering_angle #estimating steering angle from slopes of polynomial. Taking wieghted average with prev one to remove noise
				#print(steering_angle)
				if(abs(steering_angle)<4): #thresholding to remove noise
					steering_angle=0
				
				cen=(xL[0, 0]+xR[0, 0])/2 #center of the current lane
				#offset=0.3*(res_img.shape[1]/2-cen)+0.7*offset #offset from lane centrer
				offset=res_img.shape[1]/2-cen
				#adjusting steering angles for offset
				#print(offset)
				if(offset>15):
					steering_angle=10#np.arctan(offset/(res_img.shape[0]-(xL[1, 0]+xR[1, 0])*0.5))*180/np.pi#25*offset/250
				elif(offset<-15):
					steering_angle=-10#steering_angle=np.arctan(offset/(res_img.shape[0]-(xL[1, 0]+xR[1, 0])*0.5))*180/np.pi#25*offset/250
				#if laneChangeMode:
				#	steering_angle=10
				#	if(prevoffset*offset<0):
				#		laneChangeMode=False
				#prevoffset=offset
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
				#process2(xL[1, :], xR[1, :], len(lane_left), len(lane_right), b1, b2)#process2(cLy, cRy, num_left, num_right, bL, bR)
				#print(steering_angle)
				#print(offset)
				#for i in range(len(yb1)): #can be used for plotting on image
				#	res_img=cv2.circle(res_img, (int(yb1[i]), int(xL[1, i])), 10, (255, 255, 0), -1)
				#for i in range(len(yb2)):
				#	res_img=cv2.circle(res_img, (int(yb2[i]), int(xR[1, i])), 10, (0, 255, 255), -1)
				#for i in range(len(in_x[max_iL])): #can be used for plotting on image
				#	res_img=cv2.circle(res_img, (int(in_x[max_iL][i]), int(in_y[max_iL][i])), 10, (255, 255, 0), -1)
				#for i in range(len(in_x[min_iR])):
				#	res_img=cv2.circle(res_img, (int(in_x[min_iR][i]), int(in_y[min_iR][i])), 10, (0, 255, 255), -1)
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
		cv2.imshow('res2', res_img) #can be used to show image
		cv2.waitKey(1)
		cv2.imwrite('newimg.jpg', res_img)
			
		#except Exception as e:
		#	cv2.imshow("imges", image)
		#	cv2.waitKey(1)

except Exception as e:
	print(traceback.format_exc())
	print(e)
	# enable for debugging
	input()
