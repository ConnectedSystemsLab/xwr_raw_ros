#!/usr/bin/env python3

"""Simple visualizer of raw radar data.
"""

import os
import sys
import time
import pprint
import argparse
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import cv2
from collections import deque
import openvino.runtime as ov

import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from xwr_raw_ros.msg import RadarFrame
from xwr_raw_ros.msg import RadarFrameStamped
from xwr_raw_ros.msg import RadarFrameFull
from xwr_raw.radar_config import RadarConfig

from geometry_msgs.msg import Pose2D

import xwr_raw.dsp as dsp
import xwr_raw.image_tools as image_tools


def preprocess_2d_radar_6843ods(radar_cube,
                                angle_res=1, angle_range=90, 
                                range_subsampling_factor=2,
                                min_val=10.0, max_val=None,
                                resize_shape=(48,48)):
    """ Turn radar cube into x and y heatmaps for 6843isk-ods antenna array.
    """

    x_cube1 = np.stack([radar_cube[:,0,:],
                        radar_cube[:,3,:],
                        radar_cube[:,4,:],
                        radar_cube[:,7,:]], axis=1)
    x_cube2 = np.stack([radar_cube[:,1,:],
                        radar_cube[:,2,:],
                        radar_cube[:,5,:],
                        radar_cube[:,6,:]], axis=1)
    x_cube = x_cube1 + x_cube2

    y_cube1 = np.stack([radar_cube[:,4,:],
                        radar_cube[:,5,:],
                        radar_cube[:,8,:],
                        radar_cube[:,9,:]], axis=1)
    y_cube2 = np.stack([radar_cube[:,7,:],
                        radar_cube[:,6,:],
                        radar_cube[:,11,:],
                        radar_cube[:,10,:]], axis=1)
    y_cube = y_cube1 + y_cube2

    x_heatmap = dsp.compute_doppler_azimuth(x_cube, angle_res, angle_range, 
                                            range_subsampling_factor=range_subsampling_factor)
    y_heatmap = dsp.compute_doppler_azimuth(y_cube, angle_res, angle_range,
                                            range_subsampling_factor=range_subsampling_factor)

    x_heatmap = dsp.normalize(x_heatmap, min_val=min_val, max_val=max_val)
    y_heatmap = dsp.normalize(y_heatmap, min_val=min_val, max_val=max_val)

    x_heatmap = cv2.resize(x_heatmap, resize_shape, interpolation=cv2.INTER_AREA)
    y_heatmap = cv2.resize(y_heatmap, resize_shape, interpolation=cv2.INTER_AREA)

    return np.stack((x_heatmap, y_heatmap), axis=0)

def preprocess_2d_radar_6843aop(radar_cube,
                                angle_res=1, angle_range=90, 
                                range_subsampling_factor=2,
                                min_val=10.0, max_val=None,
                                resize_shape=(48,48)):
    """ Turn radar cube into x and y heatmaps for 6843aop antenna array.
    """

    x_cube1 = np.stack([radar_cube[:,6,:],
                        radar_cube[:,7,:],
                        radar_cube[:,9,:],
                        radar_cube[:,11,:]], axis=1)
    x_cube2 = np.stack([radar_cube[:,4,:],
                        radar_cube[:,6,:],
                        radar_cube[:,8,:],
                        radar_cube[:,10,:]], axis=1)
    x_cube = x_cube1 + x_cube2

    y_cube1 = np.stack([radar_cube[:,3,:],
                        radar_cube[:,2,:],
                        radar_cube[:,11,:],
                        radar_cube[:,10,:]], axis=1)
    y_cube2 = np.stack([radar_cube[:,1,:],
                        radar_cube[:,0,:],
                        radar_cube[:,9,:],
                        radar_cube[:,8,:]], axis=1)
    y_cube = y_cube1 + y_cube2

    x_heatmap = dsp.compute_doppler_azimuth(x_cube, angle_res, angle_range, 
                                            range_subsampling_factor=range_subsampling_factor)
    y_heatmap = dsp.compute_doppler_azimuth(y_cube, angle_res, angle_range,
                                            range_subsampling_factor=range_subsampling_factor)

    x_heatmap = dsp.normalize(x_heatmap, min_val=min_val, max_val=max_val)
    y_heatmap = dsp.normalize(y_heatmap, min_val=min_val, max_val=max_val)

    x_heatmap = cv2.resize(x_heatmap, resize_shape, interpolation=cv2.INTER_AREA)
    y_heatmap = cv2.resize(y_heatmap, resize_shape, interpolation=cv2.INTER_AREA)

    return np.stack((x_heatmap, y_heatmap), axis=0)

def preprocess_1d_radar_1843(radar_cube,
                             angle_res=1, angle_range=90, 
                             range_subsampling_factor=2,
                             min_val=10.0, max_val=None,
                             resize_shape=(48,48)):
    """ Turn radar cube into azimuthal heatmap for 1843 antenna array.
    """

    radar_cube = radar_cube[:,:8,:]

    heatmap = dsp.compute_doppler_azimuth(radar_cube, angle_res, angle_range,
                                          range_subsampling_factor=range_subsampling_factor)

    heatmap = dsp.normalize(heatmap, min_val=min_val, max_val=max_val)

    heatmap = cv2.resize(heatmap, resize_shape, interpolation=cv2.INTER_AREA)

    return heatmap 

def preprocess_2d_radar_1843aop(radar_cube,
                                angle_res=1, angle_range=90, 
                                range_subsampling_factor=2,
                                min_val=10.0, max_val=None,
                                resize_shape=(48,48)):
    """ Turn radar cube into x and y heatmaps for 1843aop antenna array.
    """

    x_cube =   radar_cube[:,:4,:]  \
           +   radar_cube[:,4:8,:] \
           +   radar_cube[:,8:12,:]

    y_cube = np.stack([radar_cube[:,0,:],
                       radar_cube[:,4,:],
                       radar_cube[:,8,:]], axis=1) \
           + np.stack([radar_cube[:,1,:],
                       radar_cube[:,5,:],
                       radar_cube[:,9,:]], axis=1) \
           + np.stack([radar_cube[:,2,:],
                       radar_cube[:,6,:],
                       radar_cube[:,10,:]], axis=1) \
           + np.stack([radar_cube[:,3,:],
                       radar_cube[:,7,:],
                       radar_cube[:,11,:]], axis=1)


    x_heatmap = dsp.compute_doppler_azimuth(x_cube, angle_res, angle_range, 
                                            range_subsampling_factor=range_subsampling_factor)
    y_heatmap = dsp.compute_doppler_azimuth(y_cube, angle_res, angle_range,
                                            range_subsampling_factor=range_subsampling_factor)

    x_heatmap = dsp.normalize(x_heatmap, min_val=min_val, max_val=max_val)
    y_heatmap = dsp.normalize(y_heatmap, min_val=min_val, max_val=max_val)

    x_heatmap = cv2.resize(x_heatmap, resize_shape, interpolation=cv2.INTER_AREA)
    y_heatmap = cv2.resize(y_heatmap, resize_shape, interpolation=cv2.INTER_AREA)

    return np.stack((x_heatmap, y_heatmap), axis=0)


def vis_doppler_azimuth(args, frame, flow_model=None, pub_flow=None):

    if not hasattr(vis_doppler_azimuth, 'radar_cubes'):
        vis_doppler_azimuth.radar_cubes = deque(maxlen=3)

    if 'xWR68xx' in frame.platform: 
        if 'AOP' not in frame.platform: #6843ISK-ODS
            radar_cube = dsp.reshape_frame(frame, flip_ods_phase=True)
        else: #6843AOP
            radar_cube = dsp.reshape_frame(frame, flip_aop_phase=True)

    else: # 1843/1843AOP
        radar_cube = dsp.reshape_frame(frame)

    # altitude = dsp.compute_altitude(radar_cube,
    #                                 frame.range_max/frame.shape[2],
    #                                 frame.range_bias)
    # print(altitude)

    vis_doppler_azimuth.radar_cubes.append(radar_cube)
    if len(vis_doppler_azimuth.radar_cubes) < vis_doppler_azimuth.radar_cubes.maxlen:
        return
    radar_cube = np.concatenate(vis_doppler_azimuth.radar_cubes, axis=0)
    # radar_cube = np.concatenate([x[::2] for x in vis_doppler_azimuth.radar_cubes], axis=0)

    if 'xWR68xx' in frame.platform:
        if not 'AOP' in frame.platform: #6843ISK-ODS
            heatmap = preprocess_2d_radar_6843ods(radar_cube,
                                                  angle_res=1, angle_range=90, 
                                                  range_subsampling_factor=2,
                                                  min_val=10.0, max_val=25.0,
                                                  resize_shape=(181,60))

            cv2.namedWindow('x_doppler_azimuth', cv2.WINDOW_KEEPRATIO)
            cv2.namedWindow('y_doppler_azimuth', cv2.WINDOW_KEEPRATIO)
            cv2.imshow('x_doppler_azimuth', image_tools.normalize_and_color(heatmap[0]))
            cv2.imshow('y_doppler_azimuth', image_tools.normalize_and_color(heatmap[1]))

        else: #6843AOP
            heatmap = preprocess_2d_radar_6843aop(radar_cube,
                                                  angle_res=1, angle_range=90, 
                                                  range_subsampling_factor=2,
                                                  min_val=10.0, max_val=25.0,
                                                  resize_shape=(181,60))

            cv2.namedWindow('x_doppler_azimuth', cv2.WINDOW_KEEPRATIO)
            cv2.namedWindow('y_doppler_azimuth', cv2.WINDOW_KEEPRATIO)
            cv2.imshow('x_doppler_azimuth', image_tools.normalize_and_color(heatmap[0]))
            cv2.imshow('y_doppler_azimuth', image_tools.normalize_and_color(heatmap[1]))
            
    elif 'xWR18xx' in frame.platform: # 1843
        if not 'AOP' in frame.platform:
            heatmap = preprocess_1d_radar_1843(radar_cube,
                                               angle_res=1, angle_range=90,
                                               range_subsampling_factor=8,
                                               min_val=10.0, max_val=25.0,
                                               resize_shape=(181, 60))
            heatmap = np.fliplr(heatmap) # 1843 antenna array is upside down

            cv2.namedWindow('doppler_azimuth', cv2.WINDOW_KEEPRATIO)
            cv2.imshow('doppler_azimuth', image_tools.normalize_and_color(heatmap))

        else: #1843AOP
            heatmap = preprocess_2d_radar_1843aop(radar_cube,
                                                  angle_res=1, angle_range=90,
                                                  range_subsampling_factor=4,
                                                  min_val=10.0, max_val=25.0,
                                                  resize_shape=(181, 60))

            cv2.namedWindow('x_doppler_azimuth', cv2.WINDOW_KEEPRATIO)
            cv2.namedWindow('y_doppler_azimuth', cv2.WINDOW_KEEPRATIO)
            cv2.imshow('x_doppler_azimuth', image_tools.normalize_and_color(heatmap[0]))
            cv2.imshow('y_doppler_azimuth', image_tools.normalize_and_color(heatmap[1]))

    else:
        raise ValueError('Unknown radar type')

    cv2.waitKey(1)

    if args.saved_model_path:
        heatmap = heatmap[np.newaxis, :]
        infer_request = flow_model.create_infer_request()
        infer_request.set_input_tensor(ov.Tensor(heatmap))
        infer_request.infer()
        output = np.asarray(infer_request.get_output_tensor().data)
        flow_x, flow_y = np.arctan2(output, altitude)[0].tolist()
        # print(f"flow: {flow_x:.3f} {flow_y:.3f}")
        pub_flow.publish(create_flow_msg(flow_x, flow_y, 1.0))


def create_flow_msg(flow_x, flow_y, d, dt=50e-3):
    msg = Pose2D()
    msg.theta = int(dt*1e6)
    msg.x = flow_x*dt
    msg.y = flow_y*dt
    return msg

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--saved_model_path', default=None, help='Model to load.')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('visualizer')

    # Flow model.
    if args.saved_model_path:
        core = ov.Core()
        rospack = rospkg.RosPack()
        flow_path = os.path.join(rospack.get_path('xwr_raw_ros'), 'models', args.saved_model_path)
        flow_model = core.compile_model(flow_path, "CPU")
        pub_flow = rospy.Publisher('flow', Pose2D, queue_size=1)
    else:
        flow_model = None
        pub_flow = None

    subscriber_radar = rospy.Subscriber('radar_data',
                                        numpy_msg(RadarFrameFull),
                                        lambda frame: vis_doppler_azimuth(args, frame, flow_model, pub_flow),
                                        queue_size=1)
    rospy.spin()
