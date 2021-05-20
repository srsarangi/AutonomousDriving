from pcdet.models import build_network, load_data_to_gpu
from pcdet.datasets.kitti.kitti_dataset import KittiDataset
import mayavi.mlab as mlab
from pcdet.config import cfg, cfg_from_yaml_file
from pcdet.utils import common_utils
import PVRCNN.visualize_utils as V
import numpy as np
import torch
import os

config = cfg_from_yaml_file(os.path.dirname(__file__)+"/pv_rcnn.yaml", cfg)
logger = common_utils.create_logger()
kitti = KittiDataset(dataset_cfg=config.DATA_CONFIG, class_names=config.CLASS_NAMES, training=False)
model = build_network(model_cfg=config.MODEL, num_class=len(config.CLASS_NAMES), dataset=kitti)
model.load_params_from_file(os.path.dirname(__file__)+"/pv_rcnn_8369.pth", to_cpu=False, logger=logger)
model.cuda()
model.eval()


def mask_points_and_boxes_outside_range(data_dict=None, point_cloud_range=None):
    if point_cloud_range is None:
        point_cloud_range = [0, -40, -3, 70.4, 40, 1]
    mask = mask_points_by_range(data_dict['points'], point_cloud_range)
    data_dict['points'] = data_dict['points'][mask]
    return data_dict


def mask_points_by_range(points, limit_range):
    mask = (points[:, 0] >= limit_range[0]) & (points[:, 0] <= limit_range[3]) \
            & (points[:, 1] >= limit_range[1]) & (points[:, 1] <= limit_range[4])
    return mask


def get_voxel_generator(point_cloud_range=None, VOXEL_SIZE=None, MAX_POINTS_PER_VOXEL=5, MAX_NUMBER_OF_VOXELS=40000):
    if VOXEL_SIZE is None:
        VOXEL_SIZE = [0.05, 0.05, 0.1]
    if point_cloud_range is None:
        point_cloud_range = [0, -40, -3, 70.4, 40, 1]
    try:
        from spconv.utils import VoxelGeneratorV2 as VoxelGenerator
    except:
        from spconv.utils import VoxelGenerator
    voxel_generator = VoxelGenerator(
        voxel_size=VOXEL_SIZE,
        point_cloud_range=point_cloud_range,
        max_num_points=MAX_POINTS_PER_VOXEL,
        max_voxels=MAX_NUMBER_OF_VOXELS
    )
    return voxel_generator


def transform_points_to_voxels(data_dict, voxel_generator):
        points = data_dict['points']
        voxel_output = voxel_generator.generate(points)
        if isinstance(voxel_output, dict):
            voxels, coordinates, num_points = \
                voxel_output['voxels'], voxel_output['coordinates'], voxel_output['num_points_per_voxel']
        else:
            voxels, coordinates, num_points = voxel_output

        if not data_dict['use_lead_xyz']:
            voxels = voxels[..., 3:]  # remove xyz in voxels(N, 3)

        data_dict['voxels'] = voxels
        data_dict['voxel_coords'] = coordinates
        data_dict['voxel_num_points'] = num_points
        return data_dict


def prepare_data(points, voxel_generator):
        data_dict= {'points': points, 'use_lead_xyz': [True]}
        data_dict=mask_points_and_boxes_outside_range(data_dict)
        data_dict=transform_points_to_voxels(data_dict, voxel_generator)
        data_dict['batch_size']=1
        data_dict['points']=np.pad(data_dict['points'], ((0, 0), (1, 0)), mode='constant', constant_values=0)
        data_dict['voxel_coords']=np.pad(data_dict['voxel_coords'], ((0, 0), (1, 0)), mode='constant', constant_values=0)
        return data_dict


def test(data_dict):
    with torch.no_grad():
        load_data_to_gpu(data_dict)
        pred_dicts, _ = model.forward(data_dict)
        return pred_dicts
        #V.draw_scenes(
         #  points=data_dict['points'][:, 1:], ref_boxes=pred_dicts[0]['pred_boxes'],
          # ref_scores=pred_dicts[0]['pred_scores'], ref_labels=pred_dicts[0]['pred_labels']
           #)
        #mlab.show(stop=True)


voxel_generator = get_voxel_generator()


c = 1
def pc_inference(points):
    global c
    points = np.pad(points, ((0, 0), (0, 1)), mode='constant', constant_values=0)
    # points = np.hstack((points, np.zeros((points.shape[0], 1), dtype=points.dtype)))
    if c%5==0: np.save('mydata_'+str(c)+'.npy', points)
    c += 1
    with torch.no_grad():
        data_dict = prepare_data(points, voxel_generator)
        load_data_to_gpu(data_dict)
        pred_dicts, _ = model.forward(data_dict)
        # global c
        # if(c==1):
         #     c=0
          #    V.draw_scenes(
           #   points=data_dict['points'][:, 1:], ref_boxes=pred_dicts[0]['pred_boxes'],
            #  ref_scores=pred_dicts[0]['pred_scores'], ref_labels=pred_dicts[0]['pred_labels']
             # )
              # mlab.show(stop=True)
        print(pred_dicts)
        return pred_dicts


if __name__ == "__main__":
    # Testing code
    print("-------Model successfully loaded--------")
    # voxel_generator = get_voxel_generator()
    points = np.load('my_data.npy')
    data_dict = prepare_data(points, voxel_generator)
    test(data_dict)
