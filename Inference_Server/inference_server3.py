import numpy as np
import os
from ctypes import *
import io
import base64
from PIL import Image
from io import BytesIO
import time
import traceback
import sys
# import os
import cv2
# import numpy as np
from matplotlib import pyplot as plt
# import cv2
# import time
from skimage import morphology
# import numpy as np
import time

from typing import List

so_file = "./sem.so"
sem = CDLL(so_file)
sem.readMMF.restype = c_char_p
# from YOLO.traffic_light_detection import get_traffic_light_status
# from utils.vector_serializer import string_to_vector, pvrcnn_output_to_json
# from PVRCNN.pointCloudInference import pc_inference
# from voxels import processTopView
# from . import WorldState
old_objs = []
maxind = 1
sx = (293 - 217) / 2
sy = (252 - 176) / 2


# sx=60
# sy=60


def object_tracking(old_objs, new_objs, thresh, vel):
    m = len(new_objs) 
    new_time = time.time()
    dists = np.zeros(m * len(old_objs))
    i = 0
    for o1 in old_objs: # calculates distance between every new object and every old object
        for o2 in new_objs:
            dists[i] = abs(o1.center[0] - o2.center[0]) + abs(o1.center[1] - o2.center[1])
            i += 1
    tmp = np.argsort(dists) # sorts the distances in ascending order
    for ent in tmp:
        if dists[ent] < thresh: #if distance is less than threshold then the new object an old object can be same objet
            ix = ent % m  # index of new object
            jx = ent // m #index of old object
            if new_objs[ix].idind == -1: # if id of new object is not adssigned then assign the attributes of corresponding old object to new object
                new_objs[ix].idind = old_objs[jx].idind #id assigned
                new_objs[ix].vx = (new_objs[ix].center[0] - old_objs[jx].center[0]) / (new_time - old_objs[jx].t) #assigning relative velocty along x axis
                new_objs[ix].vy = (new_objs[ix].center[1] - old_objs[jx].center[1]) / (new_time - old_objs[jx].t) #assigning relative velocty along y axis
                new_objs[ix].t = new_time #assigning time stamp
                #print(new_objs[ix].idind, new_objs[ix].vx, new_objs[ix].vy, vel)
        else:
            break
    global maxind
    #assinging new ids to the new objects which dont have any assigned id
    for o in new_objs:
        if o.idind == -1:
            o.idind = maxind
            maxind += 1
            o.t = new_time


#obstacle class sotring information for each object
class Obstacle:
    idind = -1
    vx = 0
    vy = 0
    lane = -10
    t = 0

    def __init__(self, BB):
        self.center = (np.sum(BB[:, 0]) / 4, np.sum(BB[:, 1]) / 4) #calculating centorid for object
        self.bb = BB #bounding box for the object 4 coordinates of vertices
        self.ymax=np.max(BB[:, 1])  #y-value nearest to the self-driving car

    def setLane(self, Lane):
        self.lane = Lane #lane number of the object


# lane_agent = None
# current_steering_angle = 0

#currently not being used. Satyam was working on high level decision ,aing at intersections
def wait_or_go(car_location, obstacles: List[Obstacle]):
    car_loc_x = car_location[0]
    car_loc_y = car_location[1]
    for obstacle in obstacles:
        pass


bboxesG = []

#applies object detection on bird's eye veiw from voxel
def processTopView(img, selem=np.ones((3, 3), dtype='bool'), res=0.5):
    count = 2
    img = img == 255 #image binarizing
    img = img[:, :, 0] #image binarizing
    img[img.shape[0] // 2, img.shape[1] // 2] = 0 #image binarizing
    img = morphology.binary_closing(img, selem) #image closing
    img2 = np.zeros(img.shape, dtype='int32')
    img2[img] = 1
    yn, xn = np.nonzero(img2) #getting non-zero pixels
    minarrx = []
    minarry = []
    maxarrx = []
    maxarry = []
    #BFS on image to find connected components
    for i in range(xn.shape[0]):
        if img2[yn[i], xn[i]] == 1:
            qu = [(xn[i], yn[i])]
            img2[yn[i], xn[i]] = count
            minarrx.append(xn[i])
            minarry.append(yn[i])
            maxarrx.append(xn[i])
            maxarry.append(yn[i])
            ind = count - 2
            while len(qu) > 0:
                curr = qu.pop(0)
                p = curr[0] + 1
                q = curr[1] + 1
                if p < img2.shape[1] and q < img2.shape[0] and img2[q, p] == 1:
                    img2[q, p] = count
                    qu.append((p, q))
                    maxarrx[ind] = max(p, maxarrx[ind])
                    maxarry[ind] = max(q, maxarry[ind])
                    minarrx[ind] = min(p, minarrx[ind])
                    minarry[ind] = min(q, minarry[ind])
                p = curr[0] - 1
                q = curr[1] - 1
                if p >= 0 and q >= 0 and img2[q, p] == 1:
                    img2[q, p] = count
                    qu.append((p, q))
                    maxarrx[ind] = max(p, maxarrx[ind])
                    maxarry[ind] = max(q, maxarry[ind])
                    minarrx[ind] = min(p, minarrx[ind])
                    minarry[ind] = min(q, minarry[ind])
                q = curr[1] - 1
                if q >= 0 and img2[q, curr[0]] == 1:
                    img2[q, curr[0]] = count
                    qu.append((curr[0], q))
                    maxarrx[ind] = max(curr[0], maxarrx[ind])
                    maxarry[ind] = max(q, maxarry[ind])
                    minarrx[ind] = min(curr[0], minarrx[ind])
                    minarry[ind] = min(q, minarry[ind])
                p = curr[0] - 1
                if p >= 0 and img2[curr[1], p] == 1:
                    img2[curr[1], p] = count
                    qu.append((p, curr[1]))
                    maxarrx[ind] = max(p, maxarrx[ind])
                    maxarry[ind] = max(curr[1], maxarry[ind])
                    minarrx[ind] = min(p, minarrx[ind])
                    minarry[ind] = min(curr[1], minarry[ind])
                p = curr[0] + 1
                q = curr[1] - 1
                if p < img.shape[1] and q >= 0 and img2[q, p] == 1:
                    img2[q, p] = count
                    qu.append((p, q))
                    maxarrx[ind] = max(p, maxarrx[ind])
                    maxarry[ind] = max(q, maxarry[ind])
                    minarrx[ind] = min(p, minarrx[ind])
                    minarry[ind] = min(q, minarry[ind])
                p = curr[0] - 1
                q = curr[1] + 1
                if p >= 0 and q < img.shape[0] and img2[q, p] == 1:
                    img2[q, p] = count
                    qu.append((p, q))
                    maxarrx[ind] = max(p, maxarrx[ind])
                    maxarry[ind] = max(q, maxarry[ind])
                    minarrx[ind] = min(p, minarrx[ind])
                    minarry[ind] = min(q, minarry[ind])
                p = curr[0] + 1
                if p < img.shape[1] and img2[curr[1], p] == 1:
                    img2[curr[1], p] = count
                    qu.append((p, curr[1]))
                    maxarrx[ind] = max(p, maxarrx[ind])
                    maxarry[ind] = max(curr[1], maxarry[ind])
                    minarrx[ind] = min(p, minarrx[ind])
                    minarry[ind] = min(curr[1], minarry[ind])
                q = curr[1] + 1
                if q < img.shape[0] and img2[q, curr[0]] == 1:
                    img2[q, curr[0]] = count
                    qu.append((curr[0], q))
                    maxarrx[ind] = max(curr[0], maxarrx[ind])
                    maxarry[ind] = max(q, maxarry[ind])
                    minarrx[ind] = min(curr[0], minarrx[ind])
                    minarry[ind] = min(q, minarry[ind])
            count += 1
    i = count - 3
    bboxes = []
    #finding the tightest bounding boxe for each connected component
    while (i >= 0):
        yp, xp = np.nonzero(img2[minarry[i]:maxarry[i] + 1, minarrx[i]:maxarrx[i] + 1])
        yp = yp + minarry[i]
        xp = xp + minarrx[i]
        cnt = np.concatenate((xp[:, np.newaxis], yp[:, np.newaxis]), axis=1)
        rect = cv2.minAreaRect(cnt) #finds convex hull and minimum area bounding box 
        # print(rect)
        box = cv2.boxPoints(rect) # obtains vertices from output of cv2.minAreaRect
        bboxes.append(box)
        #box = np.int0(box)
        #img2 = cv2.drawContours(img2, [box], 0, (6, 6, 6), 1)
        i -= 1
    # print(img2.shape)
    # cv2.imshow(np.repeat(img2[:, :, np.newaxis], 3, axis=2))
    # cv.waitKey(1)
    #plt.imshow(img2)
    #plt.pause(0.001)
    # cv2.waitKey(100)
    return (np.array(bboxes) - img2.shape[0] / 2) * res # convert from bird's eye view back to 3D LIDAR  coordinates
    # global bboxesG
    # bboxesG= bboxes

#This is used for visulizing the output if of snesor fusion
def process3(cLy, cRy, num_left, num_right, bL, bR):
    xL = np.linspace(start=cp.min(cLy).get(), stop=cp.max(cLy).get()) #values for plotting lanes
    xR = np.linspace(start=cp.min(cRy).get(), stop=cp.max(cRy).get()) #values for plotting lanes
    yL = bL[0] * xL ** 3 + bL[1] * xL ** 2 + bL[2] * xL + bL[3] #values for plotting current lane
    yR = bR[0] * xR ** 3 + bR[1] * xR ** 2 + bR[2] * xR + bR[3] #values for plotting current lane
    width = yR[0] - yL[0] #width of current lane
    left_lim = yL[yL.shape[0] - 1] - 330 #point from where the urrent lane starts as seen on image canvas
    # print("left_lim", left_lim)
    # print("width", width)
    yL = yL - left_lim #shifting the current lane for plotting purpose only
    yR = yR - left_lim #shifting the current lane for plotting purpose only
    Lx = []
    Rx = []
    #parallel line assumpption. finding side traffic lines assuming parallel lanes
    for j in range(num_left):
        Lx.append(yL - j * width)
    for j in range(num_right):
        Rx.append(yR + j * width)
    canvas = np.zeros((1000, 800), dtype='uint8') #canvas blank for plotting
    canvas[:, :] = 128
    for x in Lx:
        for i in range(x.shape[0]):
            canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xL[i])) + 480), 5, (255, 255, 255), -1) #plotting all lanes on left of current lane and left traffic lines of current lane
    for x in Rx: #plotting all lanes on right of current lane and right traffic lines of current lane
        for i in range(x.shape[0]):
            canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xR[i])) + 480), 5, (255, 255, 255), -1)
    car_pos = cp.matmul(Mcp, cp.array([256, 256, 1])[:, cp.newaxis]) #finding position of our own car
    car_pos = car_pos / car_pos[2, :] #finding position of our own car
    car_pos[0, 0] = car_pos[0, 0] - left_lim #finding position of our own car as on canvas
    car_pos[1, 0] = car_pos[1, 0] + 480 #finding position of our own car as on canvas
    canvas = cv2.rectangle(canvas, (car_pos[0, 0] - 25, car_pos[1, 0] - 25), (car_pos[0, 0] + 25, car_pos[1, 0] + 25),
                           (255, 255, 255), -1) #plotting our own self driving car
    # print("width2", width)
    # cv2.imshow('res34', canvas)
    # print("width3", width)
    # cv2.waitKey(0)
    for box in pred_dicts: #plots other vehicles based as bounding boxes
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
    cv2.imwrite('imgtp.jpg', canvas) #writes image on to hard disk


#This used for sensor fusion without plotting it. Steps for plotting have been removed to reduce computation
def process(arr, bL, bR, myb, vel):  # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
    num_left = arr[2] // 10 #number of traffic lines to the left of car
    num_right = arr[2] % 10 #number of traffic lines to the right of car
    # xL = np.linspace(start=arr[3], stop=arr[5])
    # xR = np.linspace(start=arr[4], stop=arr[6])
    yL = bL[0] * arr[5] ** 3 + bL[1] * arr[5] ** 2 + bL[2] * arr[5] + bL[3] #bottom most coordinate of left traffic line of current lane
    yR = bR[0] * arr[6] ** 3 + bR[1] * arr[6] ** 2 + bR[2] * arr[6] + bR[3] #bottom most coordinate of right traffic line of current lane
    width = yR - yL #wifth of current lane
    bL3 = []
    bR3 = []
    #parallel lane assumption. finding other traffic lines under paralle lane assumption
    for j in range(num_left):
        # Lx.append(yL - j * width)
        bL3.append(bL[3] - j * width)
    for j in range(num_right):
        # Rx.append(yR + j * width)
        bR3.append(bR[3] + j * width)


    objs = []
    immleft = []
    currlane = []
    immright = []
    #This assigns lanes to object detected
    for box in myb:
        obj = Obstacle(box * sx + mat_car)
        objs.append(obj)
        fLx = bL[0] * obj.center[1] ** 3 + bL[1] * obj.center[1] ** 2 + bL[2] * obj.center[1] #putting object coordinate in the lane equation baring the constant term
        fRx = bR[0] * obj.center[1] ** 3 + bR[1] * obj.center[1] ** 2 + bR[2] * obj.center[1] #putting object coordinate in the lane equation baring the constant term
        flag = False
        if len(bL3) > 0 and fLx + bL3[0] < obj.center[0] and len(bR3)>0 and fRx + bR3[0] > obj.center[0]: #checks if object is ins current lane
            obj.setLane(0)
            currlane.append(obj)
            continue

        if len(bL3) > 1 and fLx + bL3[1] < obj.center[0] and fLx + bL3[0] > obj.center[0]: #check for object in immediate left lane
            obj.setLane(-1)
            immleft.append(obj)
            continue

        for i in range(2, len(bL3)): #check for object in other left lane
            if fLx + bL3[i] < obj.center[0] and fLx + bL3[i - 1] > obj.center[0]:
                flag = True
                obj.setLane(-i)
                break
        if flag:
            continue
        if len(bR3) > 1 and fRx + bR3[1] > obj.center[0] and len(bR3)>0 and fRx + bR3[0] < obj.center[0]: #check for object in immediate right lane
            obj.setLane(1)
            immright.append(obj)
            continue
        for i in range(2, len(bR3)): #check for object in other right lanes
            if fRx + bR3[i] > obj.center[0] and fRx + bR3[i - 1] < obj.center[0]:
                flag = True
                obj.setLane(i)
                break

        
    global old_objs, maxind
    if len(old_objs) > 0:
        object_tracking(old_objs, objs, 100, vel) #object tracking is applied given old_objs are available
    else: #if no old objects avilable then assign new id to each new object
        new_time = time.time()
        for o in objs:
            o.idind = maxind
            o.t = new_time
            maxind += 1
    old_objs = objs
    return objs, immleft, currlane, immright

#This is same as that in process(...) except extra plotting steps have been added to plot the objects and lanes
def process2(arr, bL, bR, myb):  # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
    num_left = arr[2] // 10
    num_right = arr[2] % 10
    xL = np.linspace(start=arr[3], stop=arr[5])
    xR = np.linspace(start=arr[4], stop=arr[6])
    yL = bL[0] * xL ** 3 + bL[1] * xL ** 2 + bL[2] * xL + bL[3]
    yR = bR[0] * xR ** 3 + bR[1] * xR ** 2 + bR[2] * xR + bR[3]
    width = yR[yR.shape[0] - 1] - yL[yL.shape[0] - 1]
    # print("width", width)
    left_lim = yL[yL.shape[0] - 1] - 330
    # print("left_lim", left_lim)
    # print("width", width)
    yL = yL - left_lim
    yR = yR - left_lim
    Lx = []
    Rx = []
    bL3 = []
    bR3 = []
    for j in range(num_left):
        Lx.append(yL - j * width)
        bL3.append(bL[3] - j * width)
    for j in range(num_right):
        Rx.append(yR + j * width)
        bR3.append(bR[3] + j * width)

    canvas = np.zeros((3000, 800), dtype='uint8')
    canvas[:, :] = 128
    for x in Lx:
        for i in range(x.shape[0]):
            canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xL[i])) + 960), 5, (255, 255, 255), -1)
    for x in Rx:
        for i in range(x.shape[0]):
            canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xR[i])) + 960), 5, (255, 255, 255), -1)
    car_pos = cp.matmul(Mcp, cp.array([256, 256, 1])[:, cp.newaxis])
    car_pos = car_pos / car_pos[2, :]
    car_pos[0, 0] = car_pos[0, 0] - left_lim
    car_pos[1, 0] = car_pos[1, 0] + 960 + 2.702 * sy
    canvas = cv2.rectangle(canvas, (car_pos[0, 0] - 25, car_pos[1, 0] - 25), (car_pos[0, 0] + 25, car_pos[1, 0] + 25),
                           (255, 255, 255), -1)
    #    print(myb)
    objs = []
    immleft = []
    currlane = []
    immright = []
    x_car = car_pos[0, 0].get()
    y_car = car_pos[1, 0].get()
    mat_car = np.array([[x_car, y_car], [x_car, y_car], [x_car, y_car], [x_car, y_car]])
    for box in myb:
        obj = Obstacle(box * sx + mat_car)
        objs.append(obj)
        fLx = bL[0] * obj.center[1] ** 3 + bL[1] * obj.center[1] ** 2 + bL[2] * obj.center[1]
        fRx = bR[0] * obj.center[1] ** 3 + bR[1] * obj.center[1] ** 2 + bR[2] * obj.center[1]
        flag = False
        if fLx + bL3[0] < obj.center[0] and fRx + bR3[0] > obj.center[0]:
            obj.setLane(0)
            currlane.append(obj)
            continue

        if len(bL3) > 1 and fLx + bL3[1] < obj.center[0] and fLx + bL3[0] > obj.center[0]:
            obj.setLane(-1)
            immleft.append(obj)
            continue

        for i in range(2, len(bL3)):
            if fLx + bL3[i] < obj.center[0] and fLx + bL3[i - 1] > obj.center[0]:
                flag = True
                obj.setLane(-i)
                break
        if flag:
            continue
        if len(bR3) > 1 and fRx + bR3[1] > obj.center[0] and fRx + bR3[0] < obj.center[0]:
            obj.setLane(1)
            immright.append(obj)
            continue
        for i in range(2, len(bR3)):
            if fRx + bR3[i] > obj.center[0] and fRx + bR3[i - 1] < obj.center[0]:
                flag = True
                obj.setLane(i)
                break
    global old_objs, maxind
    if len(old_objs) > 0:
        object_tracking(old_objs, objs, 100)
    else:
        for o in objs:
            o.idind = maxind
            maxind += 1
    old_objs = objs
    for o in objs:
        pt0 = (int(np.round(o.bb[0, 0], decimals=0)), int(np.round(o.bb[0, 1], decimals=0)))
        pt1 = (int(np.round(o.bb[1, 0], decimals=0)), int(np.round(o.bb[1, 1], decimals=0)))
        pt2 = (int(np.round(o.bb[2, 0], decimals=0)), int(np.round(o.bb[2, 1], decimals=0)))
        pt3 = (int(np.round(o.bb[3, 0], decimals=0)), int(np.round(o.bb[3, 1], decimals=0)))
        canvas = cv2.line(canvas, pt0, pt1, (255, 255, 255), 3)
        canvas = cv2.line(canvas, pt1, pt2, (255, 255, 255), 3)
        canvas = cv2.line(canvas, pt2, pt3, (255, 255, 255), 3)
        canvas = cv2.line(canvas, pt3, pt0, (255, 255, 255), 3)
        cv2.putText(canvas, str(o.idind), pt0, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # canvas=cv2.circle(canvas,(int(np.round(x0.get(), decimals=0)), int(np.round(y0.get(), decimals=0))), 10, (255,255,255), -1)
    cv2.namedWindow('imgtp', cv2.WINDOW_NORMAL)
    cv2.imshow('imgtp', canvas)
    cv2.waitKey(1)
    return objs, immleft, currlane, immright

#Declaring shared memory objects
shm_fd = sem.shared_mem_open(bytes("imageTransfer", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 1000000)
mmf = sem.mmap_obj(1000000, shm_fd)

shm_fd = sem.shared_mem_open(bytes("steerAngle", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 20)
mmf2 = sem.mmap_obj(20, shm_fd)

shm_fd = sem.shared_mem_open(bytes("total_pts", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 20)
mmf3 = sem.mmap_obj(20, shm_fd)

shm_fd = sem.shared_mem_open(bytes("numLane", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 20)
mmf4 = sem.mmap_obj(20, shm_fd)

shm_fd = sem.shared_mem_open(bytes("objects", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 1000000)
mmf5 = sem.mmap_obj(1000000, shm_fd)

shm_fd = sem.shared_mem_open(bytes("arr", encoding='utf-8'),
                             sem.getO_CREAT_ORDWR())  # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
sem.ftrunc(shm_fd, 100)
mmf_arr = sem.mmap_obj(100, shm_fd)

shm_fd = sem.shared_mem_open(bytes("b1", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 50)
mmf_b1 = sem.mmap_obj(50, shm_fd)

shm_fd = sem.shared_mem_open(bytes("b2", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 50)
mmf_b2 = sem.mmap_obj(50, shm_fd)

shm_fd = sem.shared_mem_open(bytes("speed", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 20)
mmf_speed = sem.mmap_obj(20, shm_fd)

shm_fd = sem.shared_mem_open(bytes("curr_lane", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 20)
mmf_curr_lane = sem.mmap_obj(20, shm_fd)

# new_, num_points, num_left_lane, num_right_lane, cLy, cRy, b1, b2
#Declaring mutex locks for mshared memory
lock5=sem.semaphore_open(bytes("curr_lane_lock", encoding='utf-8'), sem.getO_Creat(), 1)
lock4 = sem.semaphore_open(bytes("lockarr", encoding='utf-8'), sem.getO_Creat(), 1)
lock3 = sem.semaphore_open(bytes("point_sem", encoding='utf-8'), sem.getO_Creat(), 1)
lock2 = sem.semaphore_open(bytes("lockSteer", encoding='utf-8'), sem.getO_Creat(), 1)
lock = sem.semaphore_open(bytes("lockForMMF_img", encoding='utf-8'), sem.getO_Creat(), 1)


# world_state = WorldState.WorldState(num_lanes=2)

#This function reads image from shared memroy, processes it using PiNet and writes the result back on to shared memory
def lane_detection_reader():
    lane_agent = getLaneAgent()
    current_steering_angle = 0
    while True:
        try:
          #sem.getVal(lock)
          sem.wait(lock)
          #sem.getVal(lock)
          img_str = sem.readMMF(mmf, 1000000) #reading image from camera oin car's hood
          sem.post(lock)
        except KeyboardInterrupt:
          sem.post(lock)
            # os._exit()
          break
        except:
          traceback.print_exception(*sys.exc_info())
          print("Exception")
          sem.post(lock)
        #sem.getVal(lock)
        # try:
        image = Image.open(BytesIO(base64.b64decode(img_str))) #string to image
        image2 = np.array(image)
        # cv2.imshow('see', image)
        # cv2.waitKey(1)
        #Getting lane key points from PINet and computing steering angle, num lane, equation of lanes, offset, etc
        new_steering_angle, num_points, num_left_lane, num_right_lane, cLy, cRy, b1, b2, offset = inference(lane_agent,
                                                                                                            image2)
        if new_steering_angle != -100:
            if new_steering_angle == 10 or new_steering_angle == -10:
                current_steering_angle = new_steering_angle
            elif abs(current_steering_angle) <= 1:
                current_steering_angle = 0
            else:
                current_steering_angle = 0.3 * new_steering_angle + 0.7 * current_steering_angle #weighted average of steering angle to remove noise 
            numsend = num_left_lane * 10 + num_right_lane #encoding left lane and right lane numbers into single integer
            arr_to_send = base64.b64encode(np.array( 
                [round(current_steering_angle), num_points, numsend, int(cp.min(cLy).get()), int(cp.min(cRy).get()),
                 int(cp.max(cLy).get()), int(cp.max(cRy).get()), round(offset.get() * 100)],
                dtype=int).tobytes())  # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy 
            # print(len(arr_to_send))
            b1_to_send = base64.b64encode(b1.tobytes())
            b2_to_send = base64.b64encode(b2.tobytes())
            #sem.getVal(lock4)
            try:
              #sem.getVal(lock4)
              sem.wait(lock4)
              #sem.getVal(lock4)
              sem.writeMMF(arr_to_send, mmf_arr) #writing arrays on to mmf
              sem.writeMMF(b1_to_send, mmf_b1) #writing arrays on to mmf
              sem.writeMMF(b2_to_send, mmf_b2) #writing arrays on to mmf
              sem.post(lock4)
            except KeyboardInterrupt:
              sem.post(lock4)
            # os._exit()
              break
            except:
              traceback.print_exception(*sys.exc_info())
              print("Exception")
              sem.post(lock4)
            # process2(cLy, cRy, num_left_lane, num_right_lane, b1, b2)
        else:
            numsend = num_left_lane * 10 + num_right_lane
            # sem.wait(lock2)
            # sem.WriteInt(num_points, mmf3)
            # sem.WriteInt(numsend, mmf4)

            # -> sem.writeMMF(base64.b64encode(struct.pack("i", num_points)), mmf3)
            # -> sem.writeMMF(base64.b64encode(struct.pack("i", numsend)), mmf4)
            # sem.post(lock2)
            arr_to_send = base64.b64encode(np.array([-100, num_points, numsend, 0, 0, 0, 0, 0],
                                                    dtype=int).tobytes())  # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
            #sem.getVal(lock4)
            try:
              #sem.getVal(lock4)
              sem.wait(lock4)
              #sem.getVal(lock4)
              sem.writeMMF(arr_to_send, mmf_arr)
              sem.post(lock4)
            except KeyboardInterrupt:
              sem.post(lock4)
            # os._exit()
              break
            except:
              traceback.print_exception(*sys.exc_info())
              print("Exception")
              sem.post(lock4)
            #sem.getVal(lock4)

        # if time.time()-start_time>20:
        #    break
        # print(current_steering_angle)
        # print(num_left_lane, num_right_lane)
        # time.sleep(0.05)


def point_cloud_reader():
    while True:
        #sem.getVal(lock3)
        try:
          #sem.getVal(lock3)
          sem.wait(lock3)
          #sem.getVal(lock3)
          img_str = sem.readMMF(mmf5, 1000000) # reading bird's eye view from voxel
          speed = sem.ReadInt(mmf_speed, 20) #getting speed of our car
          sem.post(lock3)
        except KeyboardInterrupt:
            sem.post(lock3)
            # os._exit()
            break
        except:
            traceback.print_exception(*sys.exc_info())
            print("Exception")
            sem.post(lock3)
        #sem.getVal(lock3)
        speed = speed * sy / 100
        # print(speed)
        image = Image.open(BytesIO(base64.b64decode(img_str)))
        image = np.array(image)
        bb = processTopView(image) #object detection on birds eye view
        try:
            #sem.getVal(lock4)
            #sem.getVal(lock4)
            sem.wait(lock4)
            #sem.getVal(lock4)
            x1 = base64.b64decode(sem.readMMF(mmf_arr, 56)) #reading lane detection output
            x2 = base64.b64decode(sem.readMMF(mmf_b1, 20)) #reading lane detection output
            x3 = base64.b64decode(sem.readMMF(mmf_b2, 20)) #reading lane detection output
            sem.post(lock4)
        except KeyboardInterrupt:
            sem.post(lock4)
            # os._exit()
            break
        except:
            traceback.print_exception(*sys.exc_info())
            print("Exception")
            sem.post(lock4)
        try:    
            #sem.getVal(lock4)
            arr = np.copy(np.frombuffer(x1, dtype=int))
            # print(arr)
            b1 = np.frombuffer(x2)
            b2 = np.frombuffer(x3)
            # process2(arr, b1, b2, bb)
            global prev_offset, move_right_lane, move_left_lane, curr_lane
            if move_right_lane: #if move_right_lane flag is true then lane changing to right lane is happening
                arr[0] = -10
                if prev_offset * arr[7] < 0 and prev_offset - arr[7] > 6000: #check for change in sign and discontunity of offset indicating lane changing has finished
                    move_right_lane = False
                    #prev_offset=0
                    curr_lane-=1
                    arr[0] = 12
                if prev_offset!=arr[7] and arr[7]!=0: #update prev_offset
                  #print(prev_offset, arr[7])
                  prev_offset=arr[7]
            elif move_left_lane: #if move_left_lane flag is true then lane changing to left lane is happening
                arr[0] = 10
                if prev_offset * arr[7] < 0 and arr[7] - prev_offset > 6000: #check for change in sign and discontunity of offset indicating lane changing has finished
                    move_left_lane = False
                    #prev_offset=0
                    curr_lane+=1
                    arr[0] = -12
                if prev_offset!=arr[7] and arr[7]!=0: #update prev_offset
                  #print(prev_offset, arr[7])
                  prev_offset=arr[7]
            elif arr[1] > 50:  # arr: steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy, offset
                #lanes are detected do usual task of lane centering 
                objs, left, curr, right = process(arr, b1, b2, bb, speed)
                # print("left", left)
                # print("curr", curr)
                # print("right", right)
                # Lane change Mode part
                flag = False
                # global move_right_lane, move_left_lane
                for o in curr: #check for object in front of self-driving car
                    if o.ymax < car_pos[1, 0] and  abs(
                            o.ymax - car_pos[1, 0]) < 300 and abs(
                            o.ymax - car_pos[1, 0]) > 250 and o.vy > 0: #condition for initiating lane changing
                        flag = True
                        break
                if flag:
                    if curr_lane!=4:
                      if len(right) == 0 and arr[2] % 10 > 1: #if right lane detected and no objects in right lane
                          move_right_lane = True
                      for o in right:  #check for space in right lane
                          if o.center[1] < car_pos[1, 0] and abs(
                                  o.center[1] - car_pos[1, 0]) > 600:
                              move_right_lane = True
                              break
                    if not move_right_lane and curr_lane!=3: #if right lane was not empty check if we can go to left lane
                        if len(left) == 0 and arr[2] // 10 > 1: #if left lane detected and no objects in left lane
                            move_left_lane = True
                        for o in left: #check for space in left lane
                            if o.center[1] < car_pos[1, 0] and abs(
                                    o.center[1] - car_pos[1, 0]) > 600:
                                move_left_lane = True
                                break
                #print(flag, move_right_lane, move_left_lane)
                if prev_offset * arr[7] < 0:  #if by accident car goes off lane bring it back
                  if prev_offset - arr[7] > 6000:
                    move_left_lane = True
                    curr_lane-=1
                  elif arr[7]-prev_offset > 6000:
                    move_right_lane = True
                    curr_lane+=1
                if prev_offset!=arr[7] and arr[7]!=0:
                  #print(prev_offset, arr[7])
                  prev_offset=arr[7]

            elif arr[0]==-100: #indiicates lane are not detected. This means we are at intersection or roundabout
                try:
                  sem.wait(lock5)
                  curr_lane=4-sem.ReadInt(mmf_curr_lane, 20) #get lane infromation from Unity 
                  sem.post(lock5)
                except KeyboardInterrupt:
                  sem.post(lock5)
                # os._exit()
                  break
                except:
                  traceback.print_exception(*sys.exc_info())
                  print("Exception")
                  sem.post(lock5)
                prev_offset=0
            #print("curr_lane", curr_lane)

            # time.sleep(0.2)
        except KeyboardInterrupt:
            #sem.post(lock4)
            # os._exit()
            break
        except:
            traceback.print_exception(*sys.exc_info())
            print("Exception")
            #sem.post(lock4)
        try:
            if arr[0] != -100:
              try:
                #sem.getVal(lock2)
                #sem.getVal(lock2)
                sem.wait(lock2)  # arr: steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy, offset
                #sem.getVal(lock2)
                # struct.pack("i", current_steering_angle)
                sem.WriteInt(int(arr[0]), mmf2) #write seteering angle and other values to Unity
                # sem.writeMMF(base64.b64encode(struct.pack("d", current_steering_angle)), mmf2)
                sem.WriteInt(int(arr[1]), mmf3)
                # sem.writeMMF(base64.b64encode(struct.pack("i", num_points)), mmf3)
                sem.WriteInt(int(arr[2]), mmf4)
                # sem.writeMMF(base64.b64encode(struct.pack("i", numsend)), mmf4)
                sem.post(lock2)
              except KeyboardInterrupt:
                sem.post(lock2)
                # os._exit()
                break
              except:
                traceback.print_exception(*sys.exc_info())
                print("Exception")
                sem.post(lock2)
                #sem.getVal(lock2)
            else:
              try:
                #sem.getVal(lock2)
                #sem.getVal(lock2)
                sem.wait(lock2)  # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
                #sem.getVal(lock2)
                # struct.pack("i", current_steering_angle)
                # sem.writeMMF(base64.b64encode(struct.pack("d", current_steering_angle)), mmf2)
                sem.WriteInt(int(arr[1]), mmf3)
                # sem.writeMMF(base64.b64encode(struct.pack("i", num_points)), mmf3)
                sem.WriteInt(int(arr[2]), mmf4)
                # sem.writeMMF(base64.b64encode(struct.pack("i", numsend)), mmf4)
                sem.post(lock2)
              except KeyboardInterrupt:
                sem.post(lock2)
                # os._exit()
                break
              except:
                traceback.print_exception(*sys.exc_info())
                print("Exception")
                sem.post(lock2)
                #sem.getVal(lock2)
        except KeyboardInterrupt:
            # os._exit()
            break
        except:
            traceback.print_exception(*sys.exc_info())
            print("Exception")
        time.sleep(0.01)
        # if(time.time()-start_time>20):
        #  break
        # print(image.shape)
        # print("hi")
        # time.sleep(0.05)
        # TODO Do something with the image


#function not being used
def car_controller():
    while True:
        process2(cLy, cRy, num_left_lane, num_right_lane, b1, b2)
        time.sleep(0.1)


# point_cloud_reader()
# lane_detection_reader()
if __name__ == '__main__':
    sem.getVal(lock)
    sem.getVal(lock3)
    sem.getVal(lock2)
    sem.getVal(lock4)
    cid = os.fork()
    if cid > 0:
        import cupy as cp
        import struct
        import socketserver
        import json
        # import threading
        # from ctypes import *
        from PINET import getLaneAgent, inference, getHomographyMatrix

        # so_file = "./sem.so"
        # sem = CDLL(so_file)
        # sem.readMMF.restype = c_char_p
        lane_detection_reader() #lane detection process
        sem.reset()
    
    else:
        import cupy as cp
        from PINET import getHomographyMatrix
        curr_lane=4
        M, Mcp = getHomographyMatrix()
        move_right_lane = False
        move_left_lane = False
        prev_offset = 0
        car_pos = np.matmul(M, np.array([256, 256, 1])[:, np.newaxis]) #out self-driving car position
        car_pos = car_pos / car_pos[2, :]
        mat_car = np.array([[car_pos[0, 0], car_pos[1, 0] + 2.702 * sy], [car_pos[0, 0], car_pos[1, 0] + 2.702 * sy],
                            [car_pos[0, 0], car_pos[1, 0] + 2.702 * sy], [car_pos[0, 0], car_pos[1, 0] + 2.702 * sy]]) #out self-driving car position
        point_cloud_reader() #object detection and sensor fusion process
        sem.reset()
        # while True:
        # try:
        #  sem.wait(lock4)
        # x1=base64.b64decode(sem.readMMF(mmf_arr, 56))
        # x2=base64.b64decode(sem.readMMF(mmf_b1, 20))
        # x3=base64.b64decode(sem.readMMF(mmf_b2, 20))
        # sem.post(lock4)
        # print(np.frombuffer(x1, dtype=int))
        # print(np.frombuffer(x2))
        # print(np.frombuffer(x3))
        # time.sleep(0.2)
        # except KeyboardInterrupt:
        # sem.post(lock4)
        # os._exit()
        # break
        # except:
        # print("Exception")
        #  sem.post(lock4)

        # time.sleep(0.2)
#    lane_detection_thread = mp.Process(target=lane_detection_reader)
#   point_cloud_thread = mp.Process(target=point_cloud_reader)
#  controller_thread = mp.Process(target=car_controller)
# lane_detection_thread.start()
#    point_cloud_thread.start()
#   lane_detection_reader()
# lane_detection_thread.join()
# point_cloud_thread.join()
