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
        #import os
import cv2
        #import numpy as np
from matplotlib import pyplot as plt
        #import cv2
        #import time
from skimage import morphology
        #import numpy as np
import time
so_file = "./sem.so"
sem = CDLL(so_file)
sem.readMMF.restype = c_char_p
# from YOLO.traffic_light_detection import get_traffic_light_status
#from utils.vector_serializer import string_to_vector, pvrcnn_output_to_json
#from PVRCNN.pointCloudInference import pc_inference
#from voxels import processTopView
#from . import WorldState
old_objs=[]
maxind=1
sx = (293 - 217) / 2
sy = (252 - 176) / 2
#sx=60
#sy=60

def object_tracking(old_objs, new_objs, thresh, vel):
  m=len(new_objs)
  new_time=time.time()
  dists=np.zeros(m*len(old_objs))
  i=0
  for o1 in old_objs:
    for o2 in new_objs:
      dists[i]=abs(o1.center[0]-o2.center[0])+abs(o1.center[1]-o2.center[1])
      i+=1
  tmp=np.argsort(dists)
  for ent in tmp:
    if dists[ent]<thresh:
      ix = ent%m
      jx = ent//m
      if new_objs[ix].idind == -1:
        new_objs[ix].idind=old_objs[jx].idind
        new_objs[ix].vx = (new_objs[ix].center[0]-old_objs[jx].center[0])/(new_time-old_objs[jx].t)
        new_objs[ix].vy = (new_objs[ix].center[1]-old_objs[jx].center[1])/(new_time-old_objs[jx].t)-vel
        new_objs[ix].t = new_time
    else:
      break
  global maxind
  for o in new_objs:
    if o.idind==-1:
      o.idind=maxind
      maxind+=1
      o.t=new_time


class obstacle:
  idind=-1
  vx=0
  vy=0
  lane=-10
  t=0
  def __init__(self, BB):
    self.center=(np.sum(BB[:, 0])/4, np.sum(BB[:, 1])/4)
    self.bb=BB

  def setLane(self, Lane):
    self.lane=Lane
    

# lane_agent = None
# current_steering_angle = 0

bboxesG=[]


def processTopView(img, selem=np.ones((3, 3), dtype='bool'), res=0.5):
  count=2
  img=img==255
  img=img[:, :, 0]
  img[img.shape[0]//2, img.shape[1]//2]=0
  img=morphology.binary_closing(img, selem)
  img2=np.zeros(img.shape, dtype='int32')
  img2[img]=1
  yn, xn=np.nonzero(img2)
  minarrx=[]
  minarry=[]
  maxarrx=[]
  maxarry=[]
  for i in range(xn.shape[0]):
    if img2[yn[i], xn[i]]==1:
      qu=[(xn[i], yn[i])]
      img2[yn[i], xn[i]]=count
      minarrx.append(xn[i])
      minarry.append(yn[i])
      maxarrx.append(xn[i])
      maxarry.append(yn[i])
      ind=count-2
      while len(qu)>0:
        curr=qu.pop(0)
        p=curr[0]+1
        q=curr[1]+1
        if p<img2.shape[1] and q<img2.shape[0] and img2[q, p]==1:
          img2[q, p]=count
          qu.append((p, q))
          maxarrx[ind]=max(p, maxarrx[ind])
          maxarry[ind]=max(q, maxarry[ind])
          minarrx[ind]=min(p, minarrx[ind])
          minarry[ind]=min(q, minarry[ind])
        p=curr[0]-1
        q=curr[1]-1
        if p>=0 and q>=0 and img2[q, p]==1:
          img2[q, p]=count
          qu.append((p, q))
          maxarrx[ind]=max(p, maxarrx[ind])
          maxarry[ind]=max(q, maxarry[ind])
          minarrx[ind]=min(p, minarrx[ind])
          minarry[ind]=min(q, minarry[ind])     
        q=curr[1]-1
        if q>=0 and img2[q, curr[0]]==1:
          img2[q, curr[0]]=count
          qu.append((curr[0], q))
          maxarrx[ind]=max(curr[0], maxarrx[ind])
          maxarry[ind]=max(q, maxarry[ind])
          minarrx[ind]=min(curr[0], minarrx[ind])
          minarry[ind]=min(q, minarry[ind])
        p=curr[0]-1
        if p>=0 and img2[curr[1], p]==1:
          img2[curr[1], p]=count
          qu.append((p, curr[1]))
          maxarrx[ind]=max(p, maxarrx[ind])
          maxarry[ind]=max(curr[1], maxarry[ind])
          minarrx[ind]=min(p, minarrx[ind])
          minarry[ind]=min(curr[1], minarry[ind])
        p=curr[0]+1
        q=curr[1]-1
        if p<img.shape[1] and q>=0 and img2[q, p]==1:
          img2[q, p]=count
          qu.append((p, q))
          maxarrx[ind]=max(p, maxarrx[ind])
          maxarry[ind]=max(q, maxarry[ind])
          minarrx[ind]=min(p, minarrx[ind])
          minarry[ind]=min(q, minarry[ind])
        p=curr[0]-1
        q=curr[1]+1
        if p>=0 and q<img.shape[0] and img2[q, p]==1:
          img2[q, p]=count
          qu.append((p, q))
          maxarrx[ind]=max(p, maxarrx[ind])
          maxarry[ind]=max(q, maxarry[ind])
          minarrx[ind]=min(p, minarrx[ind])
          minarry[ind]=min(q, minarry[ind])
        p=curr[0]+1
        if p<img.shape[1] and img2[curr[1], p]==1:
          img2[curr[1], p]=count
          qu.append((p, curr[1]))
          maxarrx[ind]=max(p, maxarrx[ind])
          maxarry[ind]=max(curr[1], maxarry[ind])
          minarrx[ind]=min(p, minarrx[ind])
          minarry[ind]=min(curr[1], minarry[ind])
        q=curr[1]+1
        if q<img.shape[0] and img2[q, curr[0]]==1:
          img2[q, curr[0]]=count
          qu.append((curr[0], q))
          maxarrx[ind]=max(curr[0], maxarrx[ind])
          maxarry[ind]=max(q, maxarry[ind])
          minarrx[ind]=min(curr[0], minarrx[ind])
          minarry[ind]=min(q, minarry[ind])
      count+=1
  i=count-3
  bboxes=[]
  while(i>=0):
    yp, xp=np.nonzero(img2[minarry[i]:maxarry[i]+1, minarrx[i]:maxarrx[i]+1])
    yp=yp+minarry[i]
    xp=xp+minarrx[i]
    cnt=np.concatenate((xp[:, np.newaxis], yp[:, np.newaxis]), axis=1)
    rect = cv2.minAreaRect(cnt)
    #print(rect)
    box = cv2.boxPoints(rect)
    bboxes.append(box)
    #box = np.int0(box)
    #img2=cv2.drawContours(img2,[box],0,(6,6,6),1)
    i-=1
  #print(img2.shape)
  #cv2.imshow(np.repeat(img2[:, :, np.newaxis], 3, axis=2))
  #cv.waitKey(1)
  #plt.imshow(img2)
  #plt.pause(0.001)
  #cv2.waitKey(100)
  return (np.array(bboxes)-img2.shape[0]/2)*res
  #global bboxesG
  #bboxesG= bboxes


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

def process(arr, bL, bR, myb, vel): # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
    num_left=arr[2]//10
    num_right=arr[2]%10
    #xL = np.linspace(start=arr[3], stop=arr[5])
    #xR = np.linspace(start=arr[4], stop=arr[6])
    yL = bL[0] * arr[5] ** 3 + bL[1] * arr[5] ** 2 + bL[2] * arr[5] + bL[3]
    yR = bR[0] * arr[6] ** 3 + bR[1] * arr[6] ** 2 + bR[2] * arr[6] + bR[3]
    width = yR - yL
    #print(width)
    #left_lim = yL[yL.shape[0] - 1] - 330
    # print("left_lim", left_lim)
    #print("width", width)
    #yL = yL - left_lim
    #yR = yR - left_lim
    #Lx = []
    #Rx = []
    bL3=[]
    bR3=[]
    for j in range(num_left):
        #Lx.append(yL - j * width)
        bL3.append(bL[3]-j*width)
    for j in range(num_right):
        #Rx.append(yR + j * width)
        bR3.append(bR[3]+j*width)

    #canvas = np.zeros((1000, 800), dtype='uint8')
    #canvas[:, :] = 128
    #for x in Lx:
     #   for i in range(x.shape[0]):
      #      canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xL[i])) + 480), 5, (255, 255, 255), -1)
    #for x in Rx:
     #   for i in range(x.shape[0]):
      #      canvas = cv2.circle(canvas, (int(round(x[i])), int(round(xR[i])) + 480), 5, (255, 255, 255), -1)
#    car_pos = np.matmul(M, np.array([256, 256, 1])[:, np.newaxis])
 #   car_pos = car_pos / car_pos[2, :]
#    print(myb)
    objs=[]
    immleft=[]
    currlane=[]
    immright=[]
#    x_car=car_pos[0, 0]
 #   y_car=car_pos[1, 0]+2.702*sy
  #  mat_car=np.array([[x_car, y_car], [x_car, y_car], [x_car, y_car], [x_car, y_car]])
    for box in myb:  
      obj=obstacle(box*sx+mat_car)
      objs.append(obj)
      fLx= bL[0] * obj.center[1] ** 3 + bL[1] * obj.center[1] ** 2 + bL[2] * obj.center[1]
      fRx= bR[0] * obj.center[1] ** 3 + bR[1] * obj.center[1] ** 2 + bR[2] * obj.center[1]
      flag=False
      if fLx+bL3[0]<obj.center[0] and fRx+bR3[0]>obj.center[0]:
        obj.setLane(0)
        currlane.append(obj)
        continue
        
      if len(bL3)>1 and fLx+bL3[1]<obj.center[0] and fLx+bL3[0]>obj.center[0]:
        obj.setLane(-1)
        immleft.append(obj)
        continue
        
      for i in range(2, len(bL3)):
        if fLx+bL3[i]<obj.center[0] and fLx+bL3[i-1]>obj.center[0]:
          flag=True
          obj.setLane(-i)
          break
      if flag:
        continue
      if len(bR3)>1 and fRx+bR3[1]>obj.center[0] and fRx+bR3[0]<obj.center[0]:
        obj.setLane(1)
        immright.append(obj)
        continue
      for i in range(2, len(bR3)):
        if fRx+bR3[i]>obj.center[0] and fRx+bR3[i-1]<obj.center[0]:
          flag=True
          obj.setLane(i)
          break
    global old_objs, maxind
    if len(old_objs)>0:
      object_tracking(old_objs, objs, 100, vel)
    else:
      new_time=time.time()
      for o in objs:
        o.idind=maxind
        o.t=new_time
        maxind+=1
    old_objs=objs
    #for o in objs:
    #  pt0 = (int(np.round(o.bb[0, 0], decimals=0)), int(np.round(o.bb[0, 1], decimals=0)))
    #  pt1 = (int(np.round(o.bb[1, 0], decimals=0)), int(np.round(o.bb[1, 1], decimals=0)))
     # pt2 = (int(np.round(o.bb[2, 0], decimals=0)), int(np.round(o.bb[2, 1], decimals=0)))
     # pt3 = (int(np.round(o.bb[3, 0], decimals=0)), int(np.round(o.bb[3, 1], decimals=0)))
#      canvas = cv2.line(canvas, pt0, pt1, (255, 255, 255), 3)
 #     canvas = cv2.line(canvas, pt1, pt2, (255, 255, 255), 3)
  #    canvas = cv2.line(canvas, pt2, pt3, (255, 255, 255), 3)
   #   canvas = cv2.line(canvas, pt3, pt0, (255, 255, 255), 3)
    #  cv2.putText(canvas, str(o.idind), pt0, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
            # canvas=cv2.circle(canvas,(int(np.round(x0.get(), decimals=0)), int(np.round(y0.get(), decimals=0))), 10, (255,255,255), -1)
   # cv2.imshow('imgtp', canvas)
   # cv2.waitKey(1)
    return objs, immleft, currlane, immright

def process2(arr, bL, bR, myb): # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
    num_left=arr[2]//10
    num_right=arr[2]%10
    xL = np.linspace(start=arr[3], stop=arr[5])
    xR = np.linspace(start=arr[4], stop=arr[6])
    yL = bL[0] * xL ** 3 + bL[1] * xL ** 2 + bL[2] * xL + bL[3]
    yR = bR[0] * xR ** 3 + bR[1] * xR ** 2 + bR[2] * xR + bR[3]
    width = yR[yR.shape[0]-1] - yL[yL.shape[0]-1]
    #print("width", width)
    left_lim = yL[yL.shape[0] - 1] - 330
    # print("left_lim", left_lim)
    #print("width", width)
    yL = yL - left_lim
    yR = yR - left_lim
    Lx = []
    Rx = []
    bL3=[]
    bR3=[]
    for j in range(num_left):
        Lx.append(yL - j * width)
        bL3.append(bL[3]-j*width)
    for j in range(num_right):
        Rx.append(yR + j * width)
        bR3.append(bR[3]+j*width)

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
    car_pos[1, 0] = car_pos[1, 0] + 960+2.702*sy
    canvas = cv2.rectangle(canvas, (car_pos[0, 0] - 25, car_pos[1, 0] - 25), (car_pos[0, 0] + 25, car_pos[1, 0] + 25),
                           (255, 255, 255), -1)
#    print(myb)
    objs=[]
    immleft=[]
    currlane=[]
    immright=[]
    x_car=car_pos[0, 0].get()
    y_car=car_pos[1, 0].get()
    mat_car=np.array([[x_car, y_car], [x_car, y_car], [x_car, y_car], [x_car, y_car]])
    for box in myb:  
      obj=obstacle(box*sx+mat_car)
      objs.append(obj)
      fLx= bL[0] * obj.center[1] ** 3 + bL[1] * obj.center[1] ** 2 + bL[2] * obj.center[1]
      fRx= bR[0] * obj.center[1] ** 3 + bR[1] * obj.center[1] ** 2 + bR[2] * obj.center[1]
      flag=False
      if fLx+bL3[0]<obj.center[0] and fRx+bR3[0]>obj.center[0]:
        obj.setLane(0)
        currlane.append(obj)
        continue
        
      if len(bL3)>1 and fLx+bL3[1]<obj.center[0] and fLx+bL3[0]>obj.center[0]:
        obj.setLane(-1)
        immleft.append(obj)
        continue
        
      for i in range(2, len(bL3)):
        if fLx+bL3[i]<obj.center[0] and fLx+bL3[i-1]>obj.center[0]:
          flag=True
          obj.setLane(-i)
          break
      if flag:
        continue
      if len(bR3)>1 and fRx+bR3[1]>obj.center[0] and fRx+bR3[0]<obj.center[0]:
        obj.setLane(1)
        immright.append(obj)
        continue
      for i in range(2, len(bR3)):
        if fRx+bR3[i]>obj.center[0] and fRx+bR3[i-1]<obj.center[0]:
          flag=True
          obj.setLane(i)
          break
    global old_objs, maxind
    if len(old_objs)>0:
      object_tracking(old_objs, objs, 100)
    else:
      for o in objs:
        o.idind=maxind
        maxind+=1
    old_objs=objs
    for o in objs:
      pt0 = (int(np.round(o.bb[0, 0], decimals=0)), int(np.round(o.bb[0, 1], decimals=0)))
      pt1 = (int(np.round(o.bb[1, 0], decimals=0)), int(np.round(o.bb[1, 1], decimals=0)))
      pt2 = (int(np.round(o.bb[2, 0], decimals=0)), int(np.round(o.bb[2, 1], decimals=0)))
      pt3 = (int(np.round(o.bb[3, 0], decimals=0)), int(np.round(o.bb[3, 1], decimals=0)))
      canvas = cv2.line(canvas, pt0, pt1, (255, 255, 255), 3)
      canvas = cv2.line(canvas, pt1, pt2, (255, 255, 255), 3)
      canvas = cv2.line(canvas, pt2, pt3, (255, 255, 255), 3)
      canvas = cv2.line(canvas, pt3, pt0, (255, 255, 255), 3)
      cv2.putText(canvas, str(o.idind), pt0, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255),2)
            # canvas=cv2.circle(canvas,(int(np.round(x0.get(), decimals=0)), int(np.round(y0.get(), decimals=0))), 10, (255,255,255), -1)
    cv2.namedWindow('imgtp', cv2.WINDOW_NORMAL)
    cv2.imshow('imgtp', canvas)
    cv2.waitKey(1)
    return objs, immleft, currlane, immright
    #print("width2", width)
    # cv2.imshow('res34', canvas)
    #print("width3", width)
    # cv2.waitKey(0)
#    if len(pred_dicts) > 0:
#        bb = pred_dicts[0]['pred_boxes'].cpu().detach().tolist()
#        lab = pred_dicts[0]['pred_labels'].cpu().detach().tolist()
#        for i in range(len(bb)):
#            # x0=-bb[i][1]*sx+car_pos[0, 0]
#            # y0=-bb[i][0]*sy+car_pos[1, 0]
#            xcord = np.array(
#                [[-bb[i][3], -bb[i][4]], [bb[i][3], -bb[i][4]], [bb[i][3], bb[i][4]], [-bb[i][3], bb[i][4]]]).T / 2
#            print("xcord before rotation", xcord)
#            rot = np.array([[np.cos(bb[i][6]), -np.sin(bb[i][6])], [np.sin(bb[i][6]), np.cos(bb[i][6])]])
#            org = np.repeat(np.array([bb[i][0], bb[i][1]])[:, np.newaxis], 4, axis=1)
#            xcord = np.matmul(rot, xcord) + org
#            print("xcord after rotation", xcord)
#            print("car_pos", car_pos[0, 0].get(), car_pos[1, 0].get())
#            a = -xcord[1, :] * sx + car_pos[0, 0].get()
#           b = -xcord[0, :] * sy + car_pos[1, 0].get()
#            print("final xcord", xcord)
#            pt0 = (int(np.round(a[0], decimals=0)), int(np.round(b[0], decimals=0)))
 #           pt1 = (int(np.round(a[1], decimals=0)), int(np.round(b[1], decimals=0)))
  #          pt2 = (int(np.round(a[2], decimals=0)), int(np.round(b[2], decimals=0)))
   #         pt3 = (int(np.round(a[3], decimals=0)), int(np.round(b[3], decimals=0)))
    #        # pt1=(int(np.round(xcord[0, 1], decimals=0)), int(np.round(xcord[1, 1], decimals=0)))
     #       # pt2=(int(np.round(xcord[0, 2], decimals=0)), int(np.round(xcord[1, 2], decimals=0)))
      #      # pt3=(int(np.round(xcord[0, 3], decimals=0)), int(np.round(xcord[1, 3], decimals=0)))
#            print(pt0, pt1, pt2, pt3)
 #           canvas = cv2.line(canvas, pt0, pt1, (255, 255, 255), 3)
  #          canvas = cv2.line(canvas, pt1, pt2, (255, 255, 255), 3)
   #         canvas = cv2.line(canvas, pt2, pt3, (255, 255, 255), 3)
    #        canvas = cv2.line(canvas, pt3, pt0, (255, 255, 255), 3)
            # canvas=cv2.circle(canvas,(int(np.round(x0.get(), decimals=0)), int(np.round(y0.get(), decimals=0))), 10, (255,255,255), -1)
    #cv2.imshow('imgtp', canvas)
    #cv2.waitKey(1)



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

shm_fd = sem.shared_mem_open(bytes("arr", encoding='utf-8'), sem.getO_CREAT_ORDWR()) # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
sem.ftrunc(shm_fd, 56)
mmf_arr=sem.mmap_obj(56, shm_fd)

shm_fd = sem.shared_mem_open(bytes("b1", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 50)
mmf_b1=sem.mmap_obj(50, shm_fd)

shm_fd = sem.shared_mem_open(bytes("b2", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 50)
mmf_b2=sem.mmap_obj(50, shm_fd)

shm_fd = sem.shared_mem_open(bytes("speed", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 20)
mmf_speed = sem.mmap_obj(20, shm_fd)

#new_, num_points, num_left_lane, num_right_lane, cLy, cRy, b1, b2
lock4=sem.semaphore_open(bytes("lockarr", encoding='utf-8'), sem.getO_Creat(), 1)
lock3=sem.semaphore_open(bytes("point_sem", encoding='utf-8'), sem.getO_Creat(), 1)
lock2 = sem.semaphore_open(bytes("lockSteer", encoding='utf-8'), sem.getO_Creat(), 1)
lock = sem.semaphore_open(bytes("lockForMMF", encoding='utf-8'), sem.getO_Creat(), 1)
sem.post(lock2)
sem.post(lock)
sem.post(lock3)
sem.post(lock4)
# world_state = WorldState.WorldState(num_lanes=2)


def lane_detection_reader():
    lane_agent = getLaneAgent()
    current_steering_angle = 0
    while True:
        sem.wait(lock)
        img_str = sem.readMMF(mmf, 1000000)
        sem.post(lock)
        # try:
        image = Image.open(BytesIO(base64.b64decode(img_str)))
        image2 = np.array(image)
        #cv2.imshow('see', image)
        #cv2.waitKey(1)
        # except:
        # continue
        #global num_left_lane, num_right_lane, cLy, cRy, b1, b2
        new_steering_angle, num_points, num_left_lane, num_right_lane, cLy, cRy, b1, b2 = inference(lane_agent, image2)
        if new_steering_angle != -100:
            if new_steering_angle == 10 or new_steering_angle == -10:
                current_steering_angle = new_steering_angle
            elif abs(current_steering_angle) < 4:
                current_steering_angle = 0
            else:
                current_steering_angle = 0.3 * new_steering_angle + 0.7 * current_steering_angle
            numsend = num_left_lane * 10 + num_right_lane
            arr_to_send=base64.b64encode(np.array([round(current_steering_angle), num_points, numsend, int(cp.min(cLy).get()), int(cp.min(cRy).get()), int(cp.max(cLy).get()), int(cp.max(cRy).get())], dtype=int).tobytes()) # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
            b1_to_send=base64.b64encode(b1.tobytes())
            b2_to_send=base64.b64encode(b2.tobytes())
            sem.wait(lock4)
            sem.writeMMF(arr_to_send, mmf_arr)
            sem.writeMMF(b1_to_send, mmf_b1)
            sem.writeMMF(b2_to_send, mmf_b2)
            sem.post(lock4)
            #process2(cLy, cRy, num_left_lane, num_right_lane, b1, b2)
        else:
            numsend = num_left_lane * 10 + num_right_lane
            #sem.wait(lock2)
            #sem.WriteInt(num_points, mmf3)
            #sem.WriteInt(numsend, mmf4)
            
            #-> sem.writeMMF(base64.b64encode(struct.pack("i", num_points)), mmf3)
            #-> sem.writeMMF(base64.b64encode(struct.pack("i", numsend)), mmf4)
            #sem.post(lock2)
            arr_to_send=base64.b64encode(np.array([-100, num_points, numsend, 0, 0, 0, 0], dtype=int).tobytes()) # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
            sem.wait(lock4)
            sem.writeMMF(arr_to_send, mmf_arr)
            sem.post(lock4)
        
        #if time.time()-start_time>20:
        #    break
        #print(current_steering_angle)
        #print(num_left_lane, num_right_lane)
        #time.sleep(0.05)


def point_cloud_reader():
    while True:
        sem.wait(lock3)
        img_str = sem.readMMF(mmf5, 1000000)
        speed=sem.ReadInt(mmf_speed, 20)
        sem.post(lock3)
        speed=speed/100
        #print(speed)
        image = Image.open(BytesIO(base64.b64decode(img_str)))
        image = np.array(image)
        bb=processTopView(image)
        try:
          sem.wait(lock4)
          x1=base64.b64decode(sem.readMMF(mmf_arr, 56))
          x2=base64.b64decode(sem.readMMF(mmf_b1, 20))
          x3=base64.b64decode(sem.readMMF(mmf_b2, 20))
          sem.post(lock4)
          arr=np.frombuffer(x1, dtype=int)
          b1=np.frombuffer(x2)
          b2=np.frombuffer(x3)
          #process2(arr, b1, b2, bb)
          _, left, curr, right=process(arr, b1, b2, bb, speed*sy)
          print("left", left)
          print("curr", curr)
          print("right", right)
           # time.sleep(0.2)
        except KeyboardInterrupt:
          sem.post(lock4)
            #os._exit()
          break
        except:
          traceback.print_exception(*sys.exc_info())
          print("Exception")
          sem.post(lock4)
        try:
          if arr[0]!=-100:
            sem.wait(lock2) # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
            # struct.pack("i", current_steering_angle)
            sem.WriteInt(int(arr[0]), mmf2)
            #sem.writeMMF(base64.b64encode(struct.pack("d", current_steering_angle)), mmf2)
            sem.WriteInt(int(arr[1]), mmf3)
            #sem.writeMMF(base64.b64encode(struct.pack("i", num_points)), mmf3)
            sem.WriteInt(int(arr[2]), mmf4)
            #sem.writeMMF(base64.b64encode(struct.pack("i", numsend)), mmf4)
            sem.post(lock2)
          else:
            sem.wait(lock2) # steering_angle, num_pts, numsend, mincLy, mincRy, maxcLy, maxcRy
            # struct.pack("i", current_steering_angle)
            #sem.writeMMF(base64.b64encode(struct.pack("d", current_steering_angle)), mmf2)
            sem.WriteInt(int(arr[1]), mmf3)
            #sem.writeMMF(base64.b64encode(struct.pack("i", num_points)), mmf3)
            sem.WriteInt(int(arr[2]), mmf4)
            #sem.writeMMF(base64.b64encode(struct.pack("i", numsend)), mmf4)
            sem.post(lock2)
        except KeyboardInterrupt:
          sem.post(lock2)
            #os._exit()
          break
        except:
          traceback.print_exception(*sys.exc_info())
          print("Exception")
          sem.post(lock2)
        time.sleep(0.05)
        #if(time.time()-start_time>20):
        #  break
        #print(image.shape)
        #print("hi")
        #time.sleep(0.05)
        # TODO Do something with the image


def car_controller():
  while True:
    process2(cLy, cRy, num_left_lane, num_right_lane, b1, b2)
    time.sleep(0.1)

#point_cloud_reader()
#lane_detection_reader()
if __name__ == '__main__':
    cid=os.fork()
    if cid>0:
        import cupy as cp
        import struct
        import socketserver
        import json
        #import threading
        #from ctypes import *
        from PINET import getLaneAgent, inference, getHomographyMatrix
        #so_file = "./sem.so"
        #sem = CDLL(so_file)
        #sem.readMMF.restype = c_char_p
        try:
          lane_detection_reader()
        except KeyboardInterrupt:
          sem.post(lock)
          sem.post(lock2)
          sem.post(lock4)
          #os._exit()
        except:
          traceback.print_exception(*sys.exc_info())
          sem.post(lock)
          sem.post(lock2)
          sem.post(lock4)
    else:
      import cupy as cp
      from PINET import getHomographyMatrix
      M, Mcp=getHomographyMatrix()
      car_pos = np.matmul(M, np.array([256, 256, 1])[:, np.newaxis])
      car_pos = car_pos / car_pos[2, :]
      mat_car=np.array([[car_pos[0, 0], car_pos[1, 0]+2.702*sy], [car_pos[0, 0], car_pos[1, 0]+2.702*sy], [car_pos[0, 0], car_pos[1, 0]+2.702*sy], [car_pos[0, 0], car_pos[1, 0]+2.702*sy]])
      point_cloud_reader()
        #while True:
         # try:
          #  sem.wait(lock4)
           # x1=base64.b64decode(sem.readMMF(mmf_arr, 56))
            #x2=base64.b64decode(sem.readMMF(mmf_b1, 20))
            #x3=base64.b64decode(sem.readMMF(mmf_b2, 20))
           # sem.post(lock4)
           # print(np.frombuffer(x1, dtype=int))
           # print(np.frombuffer(x2))
           # print(np.frombuffer(x3))
           # time.sleep(0.2)
          #except KeyboardInterrupt:
           # sem.post(lock4)
            #os._exit()
           # break
          #except:
           # print("Exception")
          #  sem.post(lock4)
            
          #time.sleep(0.2)
#    lane_detection_thread = mp.Process(target=lane_detection_reader)
 #   point_cloud_thread = mp.Process(target=point_cloud_reader)
  #  controller_thread = mp.Process(target=car_controller)
    #lane_detection_thread.start()
#    point_cloud_thread.start()
 #   lane_detection_reader()
    #lane_detection_thread.join()
    #point_cloud_thread.join()
