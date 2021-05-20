import io
import PIL.Image as Image
import cv2
import numpy as np
import base64
import time
import cupy as cp
from ctypes import *
import struct
so_file="./sem.so"
sem=CDLL(so_file)
sem.readMMF.restype=c_char_p
shm_fd=sem.shared_mem_open(bytes("imageTransfer", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 1000000)
mmf=sem.mmap_obj(1000000, shm_fd)
shm_fd=sem.shared_mem_open(bytes("steerAngle", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 12)
mmf2=sem.mmap_obj(12, shm_fd)
shm_fd=sem.shared_mem_open(bytes("total_pts", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 8)
mmf3=sem.mmap_obj(8, shm_fd)
shm_fd=sem.shared_mem_open(bytes("numLane", encoding='utf-8'), sem.getO_CREAT_ORDWR())
sem.ftrunc(shm_fd, 8)
mmf4=sem.mmap_obj(8, shm_fd)
lock2=sem.semaphore_open(bytes("lockSteer", encoding='utf-8'), sem.getO_Creat(), 1)
lock=sem.semaphore_open(bytes("lockForMMF", encoding='utf-8'), sem.getO_Creat(), 1)
sem.post(lock2)
sem.post(lock)
while True:
  sem.wait(lock2)
  print(sem.ReadInt(mmf3, 20));
  print(sem.ReadInt(mmf4, 20));
  #print(struct.unpack("d", base64.b64decode(sem.readMMF(mmf2, 12))))
  #print(struct.unpack("i", base64.b64decode(sem.readMMF(mmf3, 8))))
  #print(struct.unpack("i", base64.b64decode(sem.readMMF(mmf4, 8))))
  sem.post(lock2)