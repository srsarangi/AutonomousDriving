import cv2
while True:
	img=cv2.imread('imgtp.jpg')
	try:
		cv2.imshow('fus', img)
		cv2.waitKey(100)
	except:
		continue
