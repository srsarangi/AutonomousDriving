import io
import PIL.Image as Image
import cv2
import numpy as np
import time
import cupy as cp
import socketserver
import json
import threading
from PINET import getLaneAgent, inference, getHomographyMatrix
# from YOLO.traffic_light_detection import get_traffic_light_status
from utils.vector_serializer import string_to_vector, pvrcnn_output_to_json
from PVRCNN.pointCloudInference import pc_inference
from voxels import findbb
sx = (293 - 217) / 2
sy = (252 - 176) / 2
pred_dicts = []
M, Mcp = getHomographyMatrix()
lane_agent = None
current_steering_angle = 0


def process3(cLy, cRy, num_left, num_right, bL, bR):
    xL = np.linspace(start=cp.min(cLy).get(), stop=cp.max(cLy).get())
    xR = np.linspace(start=cp.min(cRy).get(), stop=cp.max(cRy).get())
    yL = bL[0] * xL ** 3 + bL[1] * xL ** 2 + bL[2] * xL + bL[3]
    yR = bR[0] * xR ** 3 + bR[1] * xR ** 2 + bR[2] * xR + bR[3]
    width = yR[0] - yL[0]
    left_lim = yL[yL.shape[0] - 1] - 330
    # print("left_lim", left_lim)
    #print("width", width)
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
    #print("width2", width)
    # cv2.imshow('res34', canvas)
    #print("width3", width)
    # cv2.waitKey(0)
    for box in pred_dicts:
      a=-box[:, 0] * sx + car_pos[0, 0].get()
      b=-box[:, 1] * sy + car_pos[1, 0].get()
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
    if (len(pred_dicts) > 0):
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


def image_to_byte_array(image: Image):
    img_bytes = io.BytesIO()
    image.save(img_bytes, format=image.format)
    img_bytes = img_bytes.getvalue()
    return img_bytes


class ThreadedInferenceRequestHandler(socketserver.BaseRequestHandler):

    def handle(self):
        request_type = b''
        while len(request_type) < 1:
            request_type += self.request.recv(1)
        request_type = request_type.decode("ascii")

        # Perform PointCloud Inference
        if request_type == 'P':
            point_cloud_size = b''
            while len(point_cloud_size) < 4:
                point_cloud_size += self.request.recv(4)
            point_cloud_size = int.from_bytes(point_cloud_size, "big")

            point_cloud = b''
            while len(point_cloud) < point_cloud_size:
                point_cloud += self.request.recv(point_cloud_size)
            point_cloud = point_cloud.decode("ascii")
            # with open("test.txt", 'w') as f:
            #     f.write(point_cloud)
            point_cloud = string_to_vector(point_cloud)
            # global pred_dicts
            # try:
            #   pred_dicts=findbb(point_cloud)
            #   print(pred_dicts)
            # except:
            #   print("Exception in P")
            # pred_dicts = pc_inference(point_cloud)
            # pred_json = pvrcnn_output_to_json(pred_dicts)
            # self.request.send(bytes(pred_json, 'ascii'))
            # Perform Inference

        # Perform Image Inference
        elif request_type == 'I':
            global current_steering_angle
            # start = time.time()
            img_size = b''
            while len(img_size) < 4:
                img_size += self.request.recv(4)

            img_size = int.from_bytes(img_size, "big")
            image = b''
            while len(image) < img_size:
                image += self.request.recv(img_size)

            # end = time.time()

            # time_to_display = str(start - end) + ", "

            start = time.time()
            image = Image.open(io.BytesIO(image))
            image = np.asarray(image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Perform PINET Inference
            new_steering_angle, num_points, num_left_lane, num_right_lane, cLy, cRy, b1, b2 = inference(lane_agent,
                                                                                                        image)
            if new_steering_angle != -100:
                print("Infered Steering Angle", new_steering_angle)
                if new_steering_angle == 10 or new_steering_angle == -10:
                    current_steering_angle = new_steering_angle
                else:
                    current_steering_angle = 0.3 * new_steering_angle + 0.7 * current_steering_angle
                    if abs(current_steering_angle) < 4:
                        current_steering_angle = 0
                # process3(cLy, cRy, num_left_lane, num_right_lane, b1, b2)

            inference_output = {
                "SteeringAngle": current_steering_angle,
                "TotalNumberOfPoints": num_points,
                "LeftLanes": num_left_lane,
                "RightLanes": num_right_lane,
            }
            # end = time.time()
            # time_to_display += str(start - end) + ", "

            # Perform YOLO Inference
            start = time.time()
            # yolo_image = cv2.resize(image, (416, 416))
            # inference_output["TrafficLightColor"] = get_traffic_light_status(yolo_image)
            # inference_output["TrafficLightColor"] = "r"
            # Send JSON as a string
            response_json = json.dumps(inference_output)
            end = time.time()
            print(end - start)
            # time_to_display += str(start - end)
            # print(time_to_display)
            print(inference_output)
            self.request.send(bytes(response_json, 'ascii'))


class ThreadedInferenceServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    pass


if __name__ == '__main__':
    lane_agent = getLaneAgent()

    address = ('localhost', 11111)
    server = ThreadedInferenceServer(address, ThreadedInferenceRequestHandler)

    t = threading.Thread(target=server.serve_forever)
    # t.setDaemon(True)  # don't hang on exit
    t.start()
    print('Server loop running in thread:', t.getName())
