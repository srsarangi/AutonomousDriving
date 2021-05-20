# Reference (implemented with modification, idea is same): https://github.com/dventimi/CarND-Advanced-Lane-Lines
from collections import deque
import cProfile
from itertools import groupby, islice, zip_longest, cycle, filterfalse
import numpy as np
import cv2
import matplotlib
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import math

M = None
Minv = None

camera_perspective_save_file_name = 'camera_perspective.txt'
num_windows = 10
skip_window_thresh = 3
box_diff_thresh = 6

window_margin = 100
minpix_recenter = 10
"""
Manual Lane length in unity's cordinate space needs to be done
lane width: 3.722, 3.74
lane height: according to picture (rectangle sheet used was 8 units)
"""

lane_width_units = 3.73
y_per_pix = None
x_per_pix = None
lane_width_px = None
lane_height_units = 8
lane_height_px = None

src_quad_temp = None
src_quad = None
dst_quad = None
state_perspective_measurement = None

point_registered = None

lane_side_1 = None
lane_side_2 = None

def perspective_M(x):
	# return cv2.warpPerspective(x, M, (src_quad[3][0], src_quad[3][1]-src_quad[2][1]), flags=cv2.INTER_LINEAR)
	# return cv2.warpPerspective(x, M, (1000, 1000), flags=cv2.INTER_LINEAR)
	return cv2.warpPerspective(x, M, x.shape[:2][::-1], flags=cv2.INTER_LINEAR)

def quad_points_to_str(points):
	s = ''
	for i in range(4):
		for j in range(2):
			s += str(points[i][j]) + ' '
	return s[:-1]

def string_to_quad_points(input_string):
	l = input_string.split(' ')
	r = []
	count = 0
	for i in range(4):
		r.append((int(l[count]), int(l[count+1])))
		count += 2
	return r

"""
Interactive perspective mesurement tool
"""
def perspective_measurement(img):
	plt.ion()

	fig = plt.figure()
	plt.imshow(img)

	global point_registered
	global src_quad_temp
	global src_quad
	global dst_quad
	global state_perspective_measurement


	src_quad_temp = [[-1,-1], [-1, -1]]
	src_quad = [[-1, -1], [-1, -1], [-1, -1], [-1, -1]]

	point_registered = 0
	state_perspective_measurement = 0

	def trapezium_extender(x_diff=None):
		global src_quad_temp

		x_mid = int(img.shape[1]/2)
		if (src_quad_temp[0][0] < x_mid):
			x_diff_thresh = x_diff = src_quad_temp[0][0]
			if x_diff is None or x_diff > x_diff_thresh:
				x_diff = x_diff_thresh
			src_quad_temp[0][0] -= x_diff
			src_quad_temp[1][0] -= x_diff
		else:
			x_diff_thresh = 2 * x_mid - src_quad_temp[0][0]
			if x_diff is None or x_diff > x_diff_thresh:
				x_diff = x_diff_thresh
			src_quad_temp[0][0] += x_diff
			src_quad_temp[1][0] += x_diff

	def trapezium_builder():
		global src_quad

		x_mid = int(img.shape[1]/2)
		src_quad[0:2] = src_quad_temp
		if (src_quad_temp[0][0] < x_mid):
			x_diff = x_mid - src_quad_temp[1][0]
			src_quad[2] = [src_quad_temp[1][0]+2*x_diff, src_quad_temp[1][1]]
			x_diff = x_mid - src_quad_temp[0][0]
			src_quad[3] = [src_quad_temp[0][0]+2*x_diff, src_quad_temp[0][1]]
		else:
			x_diff = src_quad_temp[1][0] - x_mid
			src_quad[2] = [src_quad_temp[1][0]-2*x_diff, src_quad_temp[1][1]]
			x_diff = src_quad_temp[0][0] - x_mid
			src_quad[3] = [src_quad_temp[0][0]-2*x_diff, src_quad_temp[0][1]]
		
		print(src_quad_temp)
	
	def trapezium_show():
		for i in range(4):
			plt.plot((src_quad[i][0], src_quad[(i+1)%4][0]), (src_quad[i][1], src_quad[(i+1)%4][1]), color='r')
	
	def lane_width_show():
		plt.plot((lane_side_1, lane_side_2), (img.shape[0], img.shape[0]), color='r')

	def add_points_handler(e):
		if e.key != 'z':
			return
		
		if not (state_perspective_measurement == 0 or state_perspective_measurement == 1):
			return

		global point_registered
		global lane_side_1
		global lane_side_2

		if state_perspective_measurement == 0:
			if point_registered == 0:
				lane_side_1 = int(e.xdata)
				point_registered = 1
			else:
				lane_side_2 = int(e.xdata)
				point_registered = 2
				plt.clf()
				plt.imshow(img)
				lane_width_show()
		elif state_perspective_measurement == 1:
			if point_registered == 0:
				src_quad_temp[0] = [int(e.xdata), img.shape[0]]
				point_registered = 1
			else:
				point_registered = 2
				src_quad_temp[1] = [int(e.xdata), int(e.ydata)]
				plt.clf()
				plt.imshow(img)
				trapezium_builder()
				trapezium_show()
	
	def dst_quad_builder():
		global dst_quad
		img_shape_y = int(img.shape[0])
		img_shape_x = int(img.shape[1])
		dst_quad = [[0, img_shape_y], [0, 0], [img_shape_x, 0], [img_shape_x, img_shape_y]]
		
	def state_progess_handler(e):
		global point_registered
		global src_quad_temp
		global src_quad
		global state_perspective_measurement
		global lane_height_units
		global lane_height_px
		global lane_width_px
		global lane_side_1
		global lane_side_2

		global M
		global Minv

		if e.key == 'x':
			if state_perspective_measurement == 0:
				if point_registered == 2:
					state_perspective_measurement = 1
					lane_width_px = abs(lane_side_2 - lane_side_1)
					point_registered = 0

					plt.clf()
					plt.imshow(img)
			elif state_perspective_measurement == 1:
				if point_registered == 2:
					state_perspective_measurement = 2
					lane_height_px = src_quad[0][1] - src_quad[1][1]
					trapezium_extender(100)
					trapezium_builder()
					dst_quad_builder()

					M = cv2.getPerspectiveTransform(np.asfarray(src_quad, np.float32), np.asfarray(dst_quad, np.float32))
					Minv = cv2.getPerspectiveTransform(np.asfarray(dst_quad, np.float32), np.asfarray(src_quad, np.float32))
					x_per_pix = lane_width_units/lane_width_px
					y_per_pix = lane_height_units/lane_height_px

					topview_img = perspective_M(img)
					plt.clf()
					plt.imshow(topview_img)
			elif state_perspective_measurement == 2:
				# Write config file
				camera_perspective_save_file = open(camera_perspective_save_file_name, 'w+')
				camera_perspective_save_file.write(quad_points_to_str(src_quad)+'\n')
				camera_perspective_save_file.write(quad_points_to_str(dst_quad)+'\n')

				camera_perspective_save_file.write(str(lane_width_px)+'\n')
				camera_perspective_save_file.write(str(lane_width_units)+'\n')
				camera_perspective_save_file.write(str(lane_height_px)+'\n')
				camera_perspective_save_file.write(str(lane_height_units)+'\n')
				camera_perspective_save_file.close()
				e.canvas.stop_event_loop()
				plt.ioff()
				plt.close()

		elif e.key == 'c':
			if state_perspective_measurement == 1:
				plt.clf()
				state_perspective_measurement = 0
				point_registered = 0
				lane_width_px = None
				lane_side_1 = None
				lane_side_2 = None
				plt.imshow(img)
			elif state_perspective_measurement == 2:
				plt.clf()
				state_perspective_measurement = 1
				src_quad_temp = [[-1,-1], [-1, -1]]
				src_quad = [[-1, -1], [-1, -1], [-1, -1], [-1, -1]]
				point_registered = 0
				plt.imshow(img)

	def end_handler(e):
		e.canvas.stop_event_loop()
		plt.ioff()
		lane_width_px = None
		lane_height_units = None
		lane_height_px = None

		src_quad_temp = None
		src_quad = None
		dst_quad = None
		state_perspective_measurement = None

		point_registered = None

		lane_side_1 = None
		lane_side_2 = None

	cid1 = fig.canvas.mpl_connect('key_press_event', add_points_handler)
	cid2 = fig.canvas.mpl_connect('key_press_event', state_progess_handler)
	cid3 = fig.canvas.mpl_connect('close_event', end_handler)
	fig.canvas.start_event_loop(timeout=-1)

def scale(img, factor=255.0):
	scale_factor = np.max(img)/factor
	return (img/scale_factor).astype(np.uint8)

def derivative(img, sobel_kernel=3):
	derivx = np.absolute(cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=sobel_kernel))
	derivy = np.absolute(cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
	gradmag = np.sqrt(derivx**2 + derivy**2)
	absgraddir = np.arctan2(derivy, derivx)
	return scale(derivx), scale(derivy), scale(gradmag), absgraddir

def grad(img, k1=3, k2=15):
	_,_,g,_ = derivative(img, sobel_kernel=k1)
	_,_,_,p = derivative(img, sobel_kernel=k2)
	return g,p

def hls_select(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float)
	h = hsv[:,:,0]
	l = hsv[:,:,1]
	s = hsv[:,:,2]
	return h,l,s

def rgb_select(img):
	rgb = img
	r = rgb[:,:,0]
	g = rgb[:,:,1]
	b = rgb[:,:,2]
	return r,g,b

def threshold(img, thresh_min=0, thresh_max=255):
	binary_output = np.zeros_like(img)
	binary_output[(img >= thresh_min) & (img <= thresh_max)] = 1
	return binary_output

def highlight(img):
	land = lambda *x: np.logical_and.reduce(x)
	lor = lambda *x: np.logical_or.reduce(x)
	r,g,b = rgb_select(img)
	h,l,s = hls_select(img)
	o01 = threshold(r, 200, 255)
	o02 = threshold(g, 200, 255)
	o03 = threshold(s, 200, 255)
	return scale(lor(land(o01,o02),o03))

def detect_lanes_sliding_window(img):
	histogram = np.sum(img[int(img.shape[0] * (num_windows-skip_window_thresh)/10):, :], axis=0)
	# Create an output image to draw on and  visualize the result
	out_img = np.dstack((img, img, img))*25

	# midpoint = np.int(histogram.shape[0]/2)
	midpoint = int(img.shape[0]/2)
	leftx_base = np.argmax(histogram[:midpoint])
	rightx_base = np.argmax(histogram[midpoint:]) + midpoint

	window_height = np.int(img.shape[0]/num_windows)

	nonzero = img.nonzero()
	nonzeroy = np.array(nonzero[0])
	nonzerox = np.array(nonzero[1])

	# Current positions to be updated for each window
	leftx_current = leftx_base
	rightx_current = rightx_base
   
	# Create empty lists to receive left and right lane pixel indices
	left_lane_inds = []
	right_lane_inds = []

	left_not_found = 0
	right_not_found = 0

	skip_left = False
	skip_right = False

	left_boxes = 0
	right_boxes = 0

	left_closest_lane_point = 0
	right_closest_lane_point = 0

	for window in range(num_windows):
		# Identify window boundaries in x and y (and right and left)
		win_y_low = img.shape[0] - (window+1)*window_height
		win_y_high = img.shape[0] - window*window_height

		if not skip_left:
			left_boxes += 1
			win_xleft_low = leftx_current - window_margin
			win_xleft_high = leftx_current + window_margin

			# Draw the windows on the visualization image
			cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),(0,255,0), 2) 

			# Identify the nonzero pixels in x and y within the window
			good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]

			# If you found > minpix pixels, recenter next window on their mean position
			if len(good_left_inds) > minpix_recenter:
				leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
				left_not_found = 0
			else:
				left_not_found += 1
				if left_not_found >= skip_window_thresh:
					skip_left = True

			if left_boxes == 1:
				left_closest_lane_point = leftx_current
			# Append these indices to the lists
			left_lane_inds.append(good_left_inds)

		if not skip_right:
			right_boxes += 1
			win_xright_low = rightx_current - window_margin
			win_xright_high = rightx_current + window_margin

			cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),(0,255,0), 2)

			good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

			if len(good_right_inds) > minpix_recenter:        
				rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
				right_not_found = 0
			else:
				right_not_found += 1
				if skip_right >= skip_window_thresh:
					skip_right = True
			
			if right_boxes == 1:
				right_closest_lane_point = rightx_current
			right_lane_inds.append(good_right_inds)

	ploty = np.linspace(0, img.shape[0]-1, img.shape[0])

	# Concatenate the arrays of indices
	left_lane_inds = np.concatenate(left_lane_inds)
	right_lane_inds = np.concatenate(right_lane_inds)

	# Extract left and right line pixel positions
	leftx = nonzerox[left_lane_inds]
	lefty = nonzeroy[left_lane_inds] 

	left_slope = None
	if (left_lane_inds.size > skip_window_thresh*minpix_recenter):
		# Fit a second order polynomial to each
		left_fit,left_res,_,_,_ = np.polyfit(lefty, leftx, 2, full=True)
		# Generate x and y values for plotting
		left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
		out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
		out_img[ploty.astype('int'),left_fitx.astype('int')] = [0, 255, 255]
		left_slope = 2*left_fit[0]*left_closest_lane_point + left_fit[1]

	rightx = nonzerox[right_lane_inds]
	righty = nonzeroy[right_lane_inds] 

	right_slope = None
	if (right_lane_inds.size > skip_window_thresh*minpix_recenter):
		right_fit,right_res,_,_,_ = np.polyfit(righty, rightx, 2, full=True)
		right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
		out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
		out_img[ploty.astype('int'),right_fitx.astype('int')] = [0, 255, 255]
		right_slope = 2*right_fit[0]*right_closest_lane_point + right_fit[1]

	steering_angle = 0
	offset = 0

	if abs(left_boxes - right_boxes) >= box_diff_thresh or (right_slope is None) or (left_slope is None):
		if (left_boxes > right_boxes) or left_slope is not None:
			steering_angle = (y_per_pix/x_per_pix)*left_slope
			offset = lane_width_units/-4
		elif right_slope is not None:
			steering_angle = (y_per_pix/x_per_pix)*right_slope
			offset = lane_width_units/4
		else:
			steering_angle = 0.0
			offset = 0.0
	else:
		steering_angle = (y_per_pix/x_per_pix)*(left_slope+right_slope)/2
		offset = x_per_pix * (left_closest_lane_point+right_closest_lane_point-img.shape[1])/2

	# radian to degree conversion
	steering_angle = math.atan(steering_angle)*57.2958

	return out_img, steering_angle, offset

"""
Image should be undistorted
"""

prev_out_img = None
prev_steering_angle = 0.0
prev_lane_offset = 0.0
def lane_follower(img):
	global M
	global Minv
	global src_quad
	global dst_quad
	global lane_width_units
	global lane_width_px
	global lane_height_units
	global lane_height_px
	global x_per_pix
	global y_per_pix

	global prev_out_img
	global prev_steering_angle
	global prev_lane_offset

	if M is None:
		try:
			camera_perspective_save_file = open(camera_perspective_save_file_name, 'r')
			src_quad = string_to_quad_points(camera_perspective_save_file.readline())
			dst_quad = string_to_quad_points(camera_perspective_save_file.readline())
			lane_width_px = int(camera_perspective_save_file.readline())
			lane_width_units = float(camera_perspective_save_file.readline())
			lane_height_px = int(camera_perspective_save_file.readline())
			lane_height_units = float(camera_perspective_save_file.readline())

			x_per_pix = lane_width_units/lane_width_px
			y_per_pix = lane_height_units/lane_height_px

			M = cv2.getPerspectiveTransform(np.float32(src_quad), np.float32(dst_quad))
			Minv = cv2.getPerspectiveTransform(np.float32(dst_quad), np.float32(src_quad))
		except FileNotFoundError:
			print("No saved file: " + camera_perspective_save_file_name)
			print("Complete the lane perspective measurement first")
			perspective_measurement(img)
	
	try:
		prev_out_img, prev_steering_angle, prev_lane_offset = detect_lanes_sliding_window(perspective_M(highlight(img)))
	except Exception as e:
		print(e)
	
	return prev_out_img, prev_steering_angle, prev_lane_offset
