from os.path import dirname
from yolov4.tf import YOLOv4
import cv2
import numpy as np

yolo = YOLOv4(tiny=True)
font = cv2.FONT_HERSHEY_SIMPLEX
org = (50, 50)
fontScale = 1
color = (255, 0, 0)
thickness = 2
   
#yolo.classes = dirname(__file__)+"/coco.names"
yolo.classes="./coco.names"
yolo.input_size = (416, 416)
yolo.make_model()
#yolo.load_weights(dirname(__file__)+"/yolov4-tiny.weights", weights_type="yolo")
yolo.load_weights("./yolov4-tiny.weights", weights_type="yolo")
image=cv2.imread('newimg.jpg')
image=cv2.resize(image, (416, 416))
bboxes = yolo.predict(image)
print(bboxes)
height, width, _ = image.shape
bboxes[:, [0, 2]] = bboxes[:, [0, 2]] * width
bboxes[:, [1, 3]] = bboxes[:, [1, 3]] * height
for box in bboxes:
     x, y, w, h = int(box[0]), int(box[1]), int(box[2] / 2), int(box[3] / 2)
     #   crop_img = image[y - h:y + h, x - w:x + w]
     image=cv2.rectangle(image, (x-w, y-h), (x+w, y+h), (255, 255, 255), 3)
     image = cv2.putText(image, str(box[4]), (x-w, y-h), font, 
                   fontScale, color, thickness, cv2.LINE_AA)
cv2.imshow('img', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
#filter_traffic_light = []
#for bbox in bboxes:
 #       if yolo.classes[int(bbox[4])] == "traffic_light":
  #          filter_traffic_light.append(True)
   #     else:
   #         filter_traffic_light.append(False)
   # return bboxes[filter_traffic_light]
