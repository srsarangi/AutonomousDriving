"""
	Model: https://github.com/MaybeShewill-CV/lanenet-lane-detection
"""
import tensorflow as tf
import numpy as np
import cv2
import sys

sys.path.insert(1, 'Scripts\\externalScripts\\lanenet')

from config import global_config
from lanenet_model import lanenet
from lanenet_model import lanenet_postprocess
CFG = global_config.cfg

def minmax_scale(input_arr):
	output_arr = (input_arr - np.min(input_arr)) * 255.0 / (np.max(input_arr) - np.min(input_arr))
	return output_arr

def start_sess():
	weights_path= 'lanenet/weights/tusimple_lanenet_vgg.ckpt'
	global binary_seg_ret, instance_seg_ret, input_tensor
	input_tensor = tf.placeholder(dtype=tf.float32, shape=[1, 256, 512, 3], name='input_tensor')

	net = lanenet.LaneNet(phase='test', net_flag='vgg')
	binary_seg_ret, instance_seg_ret = net.inference(input_tensor=input_tensor, name='lanenet_model')

	global postprocessor
	postprocessor = lanenet_postprocess.LaneNetPostProcessor()

	saver = tf.train.Saver()

    # Set sess configuration
	sess_config = tf.ConfigProto()
	sess_config.gpu_options.per_process_gpu_memory_fraction = CFG.TEST.GPU_MEMORY_FRACTION
	sess_config.gpu_options.allow_growth = CFG.TRAIN.TF_ALLOW_GROWTH
	sess_config.gpu_options.allocator_type = 'BFC'

	global sess
	sess = tf.Session(config=sess_config)
	#sess = tf.Session()

	with sess.as_default():
		saver.restore(sess=sess, save_path=weights_path)

def lanenet_process(image):
	with sess.as_default():
		image_vis = image
		image = cv2.resize(image, (512, 256), interpolation=cv2.INTER_LINEAR)
		image = image / 127.5 - 1.0

		binary_seg_image, instance_seg_image = sess.run(
			[binary_seg_ret, instance_seg_ret],
			feed_dict={input_tensor: [image]}
		)

		postprocess_result = postprocessor.postprocess(
			binary_seg_result=binary_seg_image[0],
			instance_seg_result=instance_seg_image[0],
			source_image=image_vis,
		)
		mask_image = postprocess_result['mask_image']

		for i in range(CFG.TRAIN.EMBEDDING_FEATS_DIMS):
			instance_seg_image[0][:, :, i] = minmax_scale(instance_seg_image[0][:, :, i])
		embedding_image = np.array(instance_seg_image[0], np.uint8)

		print(postprocess_result['fit_params'])

	# return embedding_image
	return postprocess_result['source_image'], embedding_image