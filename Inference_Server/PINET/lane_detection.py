import numpy as np
import cupy as cp
import cv2
from PINET.test_PINet import init_testing, Testing


def getLaneAgent():
	lane_agent = init_testing()
	return lane_agent


def getHomographyMatrix():
	# points used to calculate homography matrix These are points in original image
	pts=np.array([[163, 253], [346, 253], [294, 181], [216, 181], [229, 164], [281, 164], [274, 154], [237, 154]], dtype=np.float32)
	im_pts=np.array([[217, 253], [293, 253], [293, 177], [217, 177], [218, 115], [293, 115], [293, 40], [218, 40]], dtype=np.float32)
	# points used to calculate homography matrix these are points in top view of image
	# im_pts = np.array([[236, 230], [287, 230], [287, 201], [287, 151], [236, 151], [236, 201]], dtype=np.float32)
	homography_matrix, _ = cv2.findHomography(pts, im_pts)  # The homography matrix
	homography_matrix_cp = cp.array(homography_matrix)
	return homography_matrix, homography_matrix_cp  # numpy and cupy homography matrices


homography_matrix, homography_matrix_cp = getHomographyMatrix()


history_left = [3, 3, 3, 3, 3]
history_right = [3, 3, 3, 3, 3]
weight = [1/15, 2/15, 3/15, 4/15, 5/15]


def denoise(num, side='left'):
	global history_left
	global history_right
	if side == 'left':
		history_left = history_left[1:]
		history_left.append(num)
		return int(sum(i*j for i, j in zip(history_left, weight)))
	else:
		history_right = history_right[1:]
		history_right.append(num)
		return int(sum(i * j for i, j in zip(history_right, weight)))



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

	num_left = len(lane_left)
	num_right = len(lane_right)
	num_left = denoise(num_left)
	num_right = denoise(num_right)
	if min_iR != -1 and max_iL != -1:  # IF lane lines are present or not
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
		return steering_angle, total_pts, num_left, num_right, xL[1, :], xR[1, :], b1, b2, offset
	else:  # if no lane line detected then just write total num points detected
		return -100, total_pts, num_left, num_right, 0, 0, 0, 0, 0
