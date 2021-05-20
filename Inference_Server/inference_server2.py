import io
import base64
from PIL import Image
from io import BytesIO
import time
import cv2
import numpy as np
import time
import cupy as cp
import struct
import socketserver
import json
import threading
from ctypes import *
from PINET import getLaneAgent, inference, getHomographyMatrix
# from YOLO.traffic_light_detection import get_traffic_light_status
#from utils.vector_serializer import string_to_vector, pvrcnn_output_to_json
#from PVRCNN.pointCloudInference import pc_inference
from voxels import processTopView
from WorldState import WorldState

sx = (293 - 217) / 2
sy = (252 - 176) / 2
# lane_agent = None
# current_steering_angle = 0
so_file = "./sem.so"
sem = CDLL(so_file)
sem.readMMF.restype = c_char_p


def process3(cLy, cRy, num_left, num_right, bL, bR):
    xL = np.linspace(start=cp.min(cLy).get(), stop=cp.max(cLy).get())
    xR = np.linspace(start=cp.min(cRy).get(), stop=cp.max(cRy).get())
    yL = bL[0] * xL ** 3 + bL[1] * xL ** 2 + bL[2] * xL + bL[3]
    yR = bR[0] * xR ** 3 + bR[1] * xR ** 2 + bR[2] * xR + bR[3]
    width = yR[0] - yL[0]
    left_lim = yL[yL.shape[0] - 1] - 330
    # print("left_lim", left_lim)
    # print("width", width)
    yL = yL - left_lim
    yR = yR - left_lim
    Lx = []
    Rx = []
    for j in range(num_left):
        Lx.append(yL - j * width)
    for j in range(num_right):
        Rx.append(yR + j * width)
    canvas = np.zeros((1000, 800), dtype='uint8')
    canvas[:, :] = 128
    for x in Lx:
        for i in range(x.shape[0]):
            canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xL[i])) + 480), 5, (255, 255, 255), -1)
    for x in Rx:
        for i in range(x.shape[0]):
            canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xR[i])) + 480), 5, (255, 255, 255), -1)
    car_pos = cp.matmul(Mcp, cp.array([256, 256, 1])[:, cp.newaxis])
    car_pos = car_pos / car_pos[2, :]
    car_pos[0, 0] = car_pos[0, 0] - left_lim
    car_pos[1, 0] = car_pos[1, 0] + 480
    canvas = cv2.rectangle(canvas, (car_pos[0, 0] - 25, car_pos[1, 0] - 25), (car_pos[0, 0] + 25, car_pos[1, 0] + 25),
                           (255, 255, 255), -1)
    # print("width2", width)
    # cv2.imshow('res34', canvas)
    # print("width3", width)
    # cv2.waitKey(0)
    for box in pred_dicts:
        a = -box[:, 0] * sx + car_pos[0, 0].get()
        b = -box[:, 1] * sy + car_pos[1, 0].get()
        pt0 = (int(np.round(a[0], decimals=0)), int(np.round(b[0], decimals=0)))
        pt1 = (int(np.round(a[1], decimals=0)), int(np.round(b[1], decimals=0)))
        pt2 = (int(np.round(a[2], decimals=0)), int(np.round(b[2], decimals=0)))
        pt3 = (int(np.round(a[3], decimals=0)), int(np.round(b[3], decimals=0)))
        canvas = cv2.line(canvas, pt0, pt1, (255, 255, 255), 3)
        canvas = cv2.line(canvas, pt1, pt2, (255, 255, 255), 3)
        canvas = cv2.line(canvas, pt2, pt3, (255, 255, 255), 3)
        canvas = cv2.line(canvas, pt3, pt0, (255, 255, 255), 3)
    cv2.imwrite('imgtp.jpg', canvas)


def process2(cLy, cRy, num_left, num_right, bL, bR):
    xL = np.linspace(start=cp.min(cLy).get(), stop=cp.max(cLy).get())
    xR = np.linspace(start=cp.min(cRy).get(), stop=cp.max(cRy).get())
    yL = bL[0] * xL ** 3 + bL[1] * xL ** 2 + bL[2] * xL + bL[3]
    yR = bR[0] * xR ** 3 + bR[1] * xR ** 2 + bR[2] * xR + bR[3]
    width = yR[0] - yL[0]
    left_lim = yL[yL.shape[0] - 1] - 330
    # print("left_lim", left_lim)
    print("width", width)
    yL = yL - left_lim
    yR = yR - left_lim
    Lx = []
    Rx = []
    for j in range(num_left):
        Lx.append(yL - j * width)
    for j in range(num_right):
        Rx.append(yR + j * width)
    canvas = np.zeros((1000, 800), dtype='uint8')
    canvas[:, :] = 128
    for x in Lx:
        for i in range(x.shape[0]):
            canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xL[i])) + 480), 5, (255, 255, 255), -1)
    for x in Rx:
        for i in range(x.shape[0]):
            canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xR[i])) + 480), 5, (255, 255, 255), -1)
    car_pos = cp.matmul(Mcp, cp.array([256, 256, 1])[:, cp.newaxis])
    car_pos = car_pos / car_pos[2, :]
    car_pos[0, 0] = car_pos[0, 0] - left_lim
    car_pos[1, 0] = car_pos[1, 0] + 480
    canvas = cv2.rectangle(canvas, (car_pos[0, 0] - 25, car_pos[1, 0] - 25), (car_pos[0, 0] + 25, car_pos[1, 0] + 25),
                           (255, 255, 255), -1)
    print("width2", width)
    # cv2.imshow('res34', canvas)
    print("width3", width)
    # cv2.waitKey(0)
    if len(pred_dicts) > 0:
        bb = pred_dicts[0]['pred_boxes'].cpu().detach().tolist()
        lab = pred_dicts[0]['pred_labels'].cpu().detach().tolist()
        for i in range(len(bb)):
            # x0=-bb[i][1]*sx+car_pos[0, 0]
            # y0=-bb[i][0]*sy+car_pos[1, 0]
            xcord = np.array(
                [[-bb[i][3], -bb[i][4]], [bb[i][3], -bb[i][4]], [bb[i][3], bb[i][4]], [-bb[i][3], bb[i][4]]]).T / 2
            print("xcord before rotation", xcord)
            rot = np.array([[np.cos(bb[i][6]), -np.sin(bb[i][6])], [np.sin(bb[i][6]), np.cos(bb[i][6])]])
            org = np.repeat(np.array([bb[i][0], bb[i][1]])[:, np.newaxis], 4, axis=1)
            xcord = np.matmul(rot, xcord) + org
            print("xcord after rotation", xcord)
            print("car_pos", car_pos[0, 0].get(), car_pos[1, 0].get())
            a = -xcord[1, :] * sx + car_pos[0, 0].get()
            b = -xcord[0, :] * sy + car_pos[1, 0].get()
            print("final xcord", xcord)
            pt0 = (int(np.round(a[0], decimals=0)), int(np.round(b[0], decimals=0)))
            pt1 = (int(np.round(a[1], decimals=0)), int(np.round(b[1], decimals=0)))
            pt2 = (int(np.round(a[2], decimals=0)), int(np.round(b[2], decimals=0)))
            pt3 = (int(np.round(a[3], decimals=0)), int(np.round(b[3], decimals=0)))
            # pt1=(int(np.round(xcord[0, 1], decimals=0)), int(np.round(xcord[1, 1], decimals=0)))
            # pt2=(int(np.round(xcord[0, 2], decimals=0)), int(np.round(xcord[1, 2], decimals=0)))
            # pt3=(int(np.round(xcord[0, 3], decimals=0)), int(np.round(xcord[1, 3], decimals=0)))
            print(pt0, pt1, pt2, pt3)
            canvas = cv2.line(canvas, pt0, pt1, (255, 255, 255), 3)
            canvas = cv2.line(canvas, pt1, pt2, (255, 255, 255), 3)
            canvas = cv2.line(canvas, pt2, pt3, (255, 255, 255), 3)
            canvas = cv2.line(canvas, pt3, pt0, (255, 255, 255), 3)
            # canvas=cv2.circle(canvas,(int(np.round(x0.get(), decimals=0)), int(np.round(y0.get(), decimals=0))), 10, (255,255,255), -1)
    cv2.imwrite('imgtp.jpg', canvas)



# shm_fd = sem.shared_mem_open(bytes("imageTransfer", encoding='utf-8'), sem.getO_CREAT_ORDWR())
# sem.ftrunc(shm_fd, 1000000)
# mmf = sem.mmap_obj(1000000, shm_fd)

# shm_fd = sem.shared_mem_open(bytes("steerAngle", encoding='utf-8'), sem.getO_CREAT_ORDWR())
# sem.ftrunc(shm_fd, 20)
# mmf2 = sem.mmap_obj(20, shm_fd)
#
# shm_fd = sem.shared_mem_open(bytes("total_pts", encoding='utf-8'), sem.getO_CREAT_ORDWR())
# sem.ftrunc(shm_fd, 20)
# mmf3 = sem.mmap_obj(20, shm_fd)
#
# shm_fd = sem.shared_mem_open(bytes("numLane", encoding='utf-8'), sem.getO_CREAT_ORDWR())
# sem.ftrunc(shm_fd, 20)
# mmf4 = sem.mmap_obj(20, shm_fd)

# shm_fd = sem.shared_mem_open(bytes("objects", encoding='utf-8'), sem.getO_CREAT_ORDWR())
# sem.ftrunc(shm_fd, 1000000)
# mmf5 = sem.mmap_obj(1000000, shm_fd)

# lock3 = sem.semaphore_open(bytes("point_sem", encoding='utf-8'), sem.getO_Creat(), 1)
# lock2 = sem.semaphore_open(bytes("lockSteer", encoding='utf-8'), sem.getO_Creat(), 1)
# lock = sem.semaphore_open(bytes("lockForMMF", encoding='utf-8'), sem.getO_Creat(), 1)
# sem.post(lock2)
# sem.post(lock)
# sem.post(lock3)

world_state = WorldState(num_lanes=2)


def image_reader():
    shm_fd = sem.shared_mem_open(bytes("imageTransfer", encoding='utf-8'), sem.getO_CREAT_ORDWR())
    sem.ftrunc(shm_fd, 1000000)
    mmf = sem.mmap_obj(1000000, shm_fd)

    lock = sem.semaphore_open(bytes("lockForMMF", encoding='utf-8'), sem.getO_Creat(), 1)
    sem.post(lock)

    lane_agent = getLaneAgent()

    while True:
        current_steering_angle = world_state.steering_angle
        sem.wait(lock)
        img_str = sem.readMMF(mmf, 1000000)
        sem.post(lock)
        # try:
        image = Image.open(BytesIO(base64.b64decode(img_str)))
        image = np.asarray(image)
        # cv2.imshow('see', image)
        # cv2.waitKey(1)
        # except:
        # continue
        new_steering_angle, num_points, num_left_lane, num_right_lane, cLy, cRy, b1, b2 = inference(lane_agent, image)

        # numsend = num_left_lane * 10 + num_right_lane
        if new_steering_angle != -100:
            if new_steering_angle == 10 or new_steering_angle == -10:
                current_steering_angle = new_steering_angle
            elif abs(current_steering_angle) < 4:
                current_steering_angle = 0
            else:
                current_steering_angle = 0.3 * new_steering_angle + 0.7 * current_steering_angle

            # Delegate writing to the separate thread
            # sem.wait(lock2)
            # struct.pack("i", current_steering_angle)
            world_state.steering_angle = current_steering_angle
            # sem.WriteInt(round(current_steering_angle), mmf2)
            # sem.writeMMF(base64.b64encode(struct.pack("d", current_steering_angle)), mmf2)
            world_state.num_points = num_points
            # sem.WriteInt(num_points, mmf3)
            # sem.writeMMF(base64.b64encode(struct.pack("i", num_points)), mmf3)
            world_state.num_lanes_right = num_right_lane
            world_state.num_lanes_left = num_left_lane
            # sem.WriteInt(numsend, mmf4)
            # sem.writeMMF(base64.b64encode(struct.pack("i", numsend)), mmf4)
            # sem.post(lock2)
        else:
            # sem.wait(lock2)
            # sem.WriteInt(num_points, mmf3)
            world_state.num_points = num_points
            world_state.num_lanes_right = num_right_lane
            world_state.num_lanes_left = num_left_lane
            # sem.WriteInt(numsend, mmf4)
            # sem.writeMMF(base64.b64encode(struct.pack("i", num_points)), mmf3)
            # sem.writeMMF(base64.b64encode(struct.pack("i", numsend)), mmf4)
            # sem.post(lock2)
        print(current_steering_angle)


def point_cloud_reader():
    shm_fd = sem.shared_mem_open(bytes("objects", encoding='utf-8'), sem.getO_CREAT_ORDWR())
    sem.ftrunc(shm_fd, 1000000)
    mmf5 = sem.mmap_obj(1000000, shm_fd)

    lock3 = sem.semaphore_open(bytes("point_sem", encoding='utf-8'), sem.getO_Creat(), 1)
    sem.post(lock3)

    while True:
        sem.wait(lock3)
        img_str = sem.readMMF(mmf5, 1000000)
        sem.post(lock3)
        image = Image.open(BytesIO(base64.b64decode(img_str)))
        image = np.asarray(image)
        processTopView(image)
        # print(image.shape)
        time.sleep(0.05)
        # TODO Do something with the image


def control_writer():
    shm_fd = sem.shared_mem_open(bytes("steerAngle", encoding='utf-8'), sem.getO_CREAT_ORDWR())
    sem.ftrunc(shm_fd, 20)
    mmf2 = sem.mmap_obj(20, shm_fd)

    shm_fd = sem.shared_mem_open(bytes("total_pts", encoding='utf-8'), sem.getO_CREAT_ORDWR())
    sem.ftrunc(shm_fd, 20)
    mmf3 = sem.mmap_obj(20, shm_fd)

    shm_fd = sem.shared_mem_open(bytes("numLane", encoding='utf-8'), sem.getO_CREAT_ORDWR())
    sem.ftrunc(shm_fd, 20)
    mmf4 = sem.mmap_obj(20, shm_fd)

    lock2 = sem.semaphore_open(bytes("lockSteer", encoding='utf-8'), sem.getO_Creat(), 1)
    sem.post(lock2)

    while True:
        if world_state.can_change_lane():
            # TODO
            pass
        else:
            sem.wait(lock2)
            sem.WriteInt(round(world_state.steering_angle), mmf2)
            sem.WriteInt(world_state.num_points, mmf3)
            # HACK:- to send number of lanes in single variable
            sem.WriteInt(world_state.num_lanes_left * 10 + world_state.num_lanes_right, mmf4)
            sem.post(lock2)


# lane_detection_reader()
if __name__ == '__main__':
    lane_detection_thread = threading.Thread(target=image_reader)
    point_cloud_thread = threading.Thread(target=point_cloud_reader)
    controller_thread = threading.Thread(target=control_writer)
    lane_detection_thread.start()
    point_cloud_thread.start()
    lane_detection_thread.join()
    point_cloud_thread.join()
