import io
import base64
import PIL.Image as Image
import cv2
import numpy as np
import time

import socketserver
import json
import threading
from PINET import getLaneAgent, inference
from YOLO.traffic_light_detection import get_traffic_light_status

lane_agent = None
current_steering_angle = 0


def image_to_byte_array(image: Image):
    img_bytes = io.BytesIO()
    image.save(img_bytes, format=image.format)
    img_bytes = img_bytes.getvalue()
    return img_bytes


class ThreadedInferenceRequestHandler(socketserver.BaseRequestHandler):

    def handle(self):
        global current_steering_angle

        start = time.time()
        img_size = b''
        while len(img_size) < 4:
            img_size += self.request.recv(4)

        img_size = int.from_bytes(img_size, "big")
        image = b''
        while len(image) < img_size:
            image += self.request.recv(img_size)
            
        end = time.time()

        time_to_display = str(start-end)+", "
        
        start = time.time()
        image = Image.open(io.BytesIO(image))
        image = np.asarray(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Perform PINET Inference
        new_steering_angle, num_points, num_left_lane, num_right_lane = inference(lane_agent, image)
        print("Infered Steering Angle", new_steering_angle)
        if new_steering_angle == 10 or new_steering_angle == -10:
            current_steering_angle = new_steering_angle
        else:
            current_steering_angle = 0.3 * new_steering_angle + 0.7 * current_steering_angle
            if abs(current_steering_angle) < 4:
                current_steering_angle = 0
                
        inference_output = {
            "SteeringAngle": current_steering_angle,
            "TotalNumberOfPoints": num_points,
            "LeftLanes": num_left_lane,
            "RightLanes": num_right_lane,
        }
        end = time.time()
        time_to_display += str(start-end)+", "


        # Perform YOLO Inference
        start = time.time()
        yolo_image = cv2.resize(image, (416, 416))
        # inference_output["TrafficLightColor"] = get_traffic_light_status(yolo_image)
        inference_output["TrafficLightColor"] = "r"
        # Send JSON as a string
        response_json = json.dumps(inference_output)
        end = time.time()
        print(end - start)
        time_to_display += str(start-end)
        print(time_to_display)
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
