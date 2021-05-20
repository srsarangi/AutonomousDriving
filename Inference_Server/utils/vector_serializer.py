import numpy as np
import torch
import json


def string_to_vector(input_string: str):
    lines = input_string.split("\n")
    vector_array = np.empty((len(lines), 3))
    for i, line in enumerate(lines):
        try:
            x, y, z = map(float, line.split(","))
            vector_array[i] = np.array([x, y, z])
        except:
            pass
            # print("The line is", line)

    return vector_array


def pvrcnn_output_to_json(pvrcnn_output):
    # box = [x, y, z, dx, dy, dz, alignment]
    # dx, dy, dz is the distance from center to the face in the respective direction
    # alignment is angle in which
    output = {'c': [], 'p': [], 'b': []}
    boxes = pvrcnn_output[0]['pred_boxes']
    scores = pvrcnn_output[0]['pred_scores']
    labels = pvrcnn_output[0]['pred_labels']
    assert len(boxes) == len(scores) == len(labels)
    for i in range(len(boxes)):
        box = [boxes[i][0].item(), boxes[i][1].item(), boxes[i][2].item(), boxes[i][3].item(),
               boxes[i][4].item(), boxes[i][5].item(), boxes[i][6].item()]
        if labels[i] == 1:          # Car
            output['c'] += box
        elif labels[i] == 2:        # Bike
            output['b'] += box
        elif labels[i] == 3:        # Pedestrian
            output['p'] += box
    return json.dumps(output)


# Use for test only
if __name__ == '__main__':
    x = [{'pred_boxes': torch.Tensor([[13.9279, -0.1815, -1.8496, 4.0458, 1.6801, 1.5851, 1.4254]]),
          'pred_scores': torch.Tensor([0.2676]),
          'pred_labels': torch.Tensor([1])}]
    pvrcnn_output_to_json(x)
