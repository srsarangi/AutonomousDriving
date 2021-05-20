from os.path import dirname
from yolov4.tf import YOLOv4
import cv2
import numpy as np

yolo = YOLOv4(tiny=True)
yolo.classes = dirname(__file__)+"/coco.names"
yolo.input_size = (416, 416)
yolo.make_model()
yolo.load_weights(dirname(__file__)+"/yolov4-tiny.weights", weights_type="yolo")


def get_traffic_light_status(image):
    bboxes = get_traffic_light_box(image)
    # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    height, width, _ = image.shape
    bboxes[:, [0, 2]] = bboxes[:, [0, 2]] * width
    bboxes[:, [1, 3]] = bboxes[:, [1, 3]] * height

    light_color = {"Red": 0, "Green": 0}
    for box in bboxes:
        x, y, w, h = int(box[0]), int(box[1]), int(box[2] / 2), int(box[3] / 2)
        crop_img = image[y - h:y + h, x - w:x + w]
        light_color[detect_light(crop_img)] += 1

    if light_color["Red"] > light_color["Green"]:
        return 'r'
    else:
        return 'g'


def get_traffic_light_box(image):
    bboxes = yolo.predict(image)
    filter_traffic_light = []
    for bbox in bboxes:
        if yolo.classes[int(bbox[4])] == "traffic_light":
            filter_traffic_light.append(True)
        else:
            filter_traffic_light.append(False)
    return bboxes[filter_traffic_light]


def detect_light(img, threshold=0.01):
    desired_dim = (30, 90)  # width, height
    img = cv2.resize(np.array(img), desired_dim, interpolation=cv2.INTER_LINEAR)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # lower mask (0-10)
    lower_red = np.array([0, 70, 50])
    upper_red = np.array([10, 255, 255])
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

    # upper mask (170-180)
    lower_red1 = np.array([170, 70, 50])
    upper_red1 = np.array([180, 255, 255])
    mask1 = cv2.inRange(img_hsv, lower_red1, upper_red1)

    # defining the Range of yellow color
    lower_yellow = np.array([21, 39, 64])
    upper_yellow = np.array([40, 255, 255])
    mask2 = cv2.inRange(img_hsv, lower_yellow, upper_yellow)

    # red pixels' mask
    mask = mask0 + mask1 + mask2

    # Compare the percentage of red values
    rate = np.count_nonzero(mask) / (desired_dim[0] * desired_dim[1])

    if rate > threshold:
        return "Red"
    else:
        return "Green"


if __name__ == "__main__":
    i = cv2.imread("test_img.PNG")
    print(get_traffic_light_status(i))

