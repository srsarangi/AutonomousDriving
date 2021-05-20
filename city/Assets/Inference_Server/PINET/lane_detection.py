import numpy as np
import cupy as cp
import cv2
from PINET.test_PINet import init_testing, Testing


def getLaneAgent():
	lane_agent = init_testing()
	return lane_agent


def getHomographyMatrix():
	# points used to calculate homography matrix These are points in original image
	pts = np.array([[229, 224], [297, 223], [288, 202], [279, 182], [241, 182], [236, 202]], dtype=np.float32)
	# points used to calculate homography matrix these are points in top view of image
	im_pts = np.array([[236, 230], [287, 230], [287, 201], [287, 151], [236, 151], [236, 201]], dtype=np.float32)
	homography_matrix, _ = cv2.findHomography(pts, im_pts)  # The homography matrix
	homography_matrix_cp = cp.array(homography_matrix)
	return homography_matrix, homography_matrix_cp  # numpy and cupy homography matrices


homography_matrix, homography_matrix_cp = getHomographyMatrix()


def inference(lane_agent, image):
	in_x, in_y, res_img = Testing(lane_agent, image)
	lane_left = []   # appends indices of lines left to center of image here
	lane_right = []  # appends indices of lines right to center of image here
	max_xL = 0
	max_iL = -1  # max index in lane_left corresponds to left traffic line of current lane
	min_xR = res_img.shape[1]
	min_iR = -1  # min index in right_left corresponds to right traffic line of current lane
	mid_pt = res_img.shape[1]/2  # center of image corresponds to car position
	total_pts = 0
	for i in range(len(in_x)):
		x1 = cp.array(in_x[i])  # *res_img.shape[1]/512 #scaling of keypoints to origional size of image
		total_pts += len(x1)  # total points
		t = cp.sum(x1[:5])/5
		if t <= mid_pt:  # detects set of points to the left of image center
			lane_left.append(i)
			if max_xL < t and in_y[i][0] > 220:  # calculates index of traffic line just left of image center
				max_xL = t
				max_iL = i
		else:
			lane_right.append(i)  # detects set of points to the right of image center
			if min_xR > t and in_y[i][0] > 220:  # calculates index of traffic line just left of image center
				min_xR = t
				min_iR = i

	if min_iR != -1 and max_iL != -1: #IF lane lines are present or not
		xL = cp.matmul(homography_matrix_cp, cp.vstack((cp.array(in_x[max_iL]), cp.array(in_y[max_iL]), cp.ones(cp.array(in_y[max_iL]).shape)))) #obtains key point in top view for left line
		xL = xL/xL[2, :]  # converting from homogenous to non-homogenous coordinates for traffic line
		# same stes for right line
		xR = cp.matmul(homography_matrix_cp, cp.vstack((cp.array(in_x[min_iR]), cp.array(in_y[min_iR]), cp.ones(cp.array(in_y[min_iR]).shape))))
		xR = xR/xR[2, :]
		
		# Fitting polynomials
		b1 = np.polyfit(xL[1, :].get(), xL[0, :].get(), 3)
		b2 = np.polyfit(xR[1, :].get(), xR[0, :].get(), 3)
		# calculating derivative of polynomials
		dy1 = 3*b1[0]*xL[1, 1]**2+2*b1[1]*xL[1, 1]+b1[2]
		dy2 = 3*b2[0]*xR[1, 1]**2+2*b2[1]*xR[1, 1]+b2[2]
		steering_angle = 180*(cp.arctan((dy1+dy2)*0.5))/cp.pi
		steering_angle = float(steering_angle)

		cen = (xL[0, 0]+xR[0, 0])/2  # center of the current lane
		offset = res_img.shape[1]/2-cen
		# adjusting steering angles for offset
		if offset > 15:
			steering_angle = 10
		elif offset < -15:
			steering_angle = -10
		return steering_angle, total_pts, len(lane_left), len(lane_right)
	else:  # if no lane line detected then just write total num points detected
		return -1, total_pts, len(lane_left), len(lane_right)
