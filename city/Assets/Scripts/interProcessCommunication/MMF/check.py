from test_PINet import init_testing, Testing
import cv2
lane_agent=init_testing()
img=cv2.imread('test4.jpg')
im2=Testing(lane_agent, img)
cv2.imshow('h', im2)
cv2.waitKey(1)
