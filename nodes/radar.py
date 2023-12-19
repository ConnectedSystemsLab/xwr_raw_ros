#!/usr/bin/env python3

"""Simple publisher of raw radar data.
"""
import os
import sys
import time
import socket
import serial
import argparse
import numpy as np

import rospy
import rospkg
from rospy.numpy_msg import numpy_msg
from xwr_raw_ros.msg import RadarFrame
from xwr_raw_ros.msg import RadarFrameStamped
from xwr_raw_ros.msg import RadarFrameFull
from xwr_raw.radar_config import RadarConfig
from xwr_raw.radar_pub import RadarPub

if __name__ == '__main__':

    # Read path to config file.
    parser = argparse.ArgumentParser()
    parser.add_argument('cfg', help="Path to configuration file for radar.")
    parser.add_argument('--cmd_tty',        default='/dev/ttyACM0',   help='CMD TTY of radar.')
    parser.add_argument('--dca_ip',         default='192.168.33.180', help='IP address of DCA1000.')
    parser.add_argument('--dca_cmd_port',   default=4096,             help='CMD port of DCA1000.')
    parser.add_argument('--host_ip',        default='192.168.33.30',  help='IP address of host.')
    parser.add_argument('--host_cmd_port',  default=4096,             help='CMD port of host.')
    parser.add_argument('--host_data_port', default=4098,             help='Data port of host.')
    args = parser.parse_args(rospy.myargv()[1:])

    # Initialize node and topic.
    rospy.init_node('xwr_radar')
    publisher = rospy.Publisher('radar_data',
                                numpy_msg(RadarFrameFull),
                                queue_size=1)

    # Parse and publish config file.
    rospack = rospkg.RosPack()
    with open(os.path.join(rospack.get_path('xwr_raw_ros'), args.cfg), 'r') as f:
        cfg = f.readlines()
    radar_config = RadarConfig(cfg)
    rospy.set_param('radar_config', dict(**radar_config))

    # Extract params from config.
    radar_params = radar_config.get_params()
    rospy.set_param('radar_params', dict(**radar_params))

    # Configure and start radar capture.
    radar = RadarPub(cfg,
                     cmd_tty        = args.cmd_tty,
                     dca_ip         = args.dca_ip,
                     dca_cmd_port   = int(args.dca_cmd_port),
                     host_ip        = args.host_ip,
                     host_cmd_port  = int(args.host_cmd_port),
                     host_data_port = int(args.host_data_port))
    radar.configure()
    radar.start_capture()

    rospy.on_shutdown(lambda : radar.close())

    # Publish radar data to topic.
    while True:
        frame_data, new_frame = radar.update_frame_buffer()
        if new_frame:
            # Publish raw radar frame only.
            # msg = RadarFrame()
            # msg.data = frame_data

            # Publish with timestamp.
            # msg = RadarFrameStamped()
            # msg.header.stamp = rospy.get_rostime()
            # msg.data = frame_data

            # Publish with all metadata.
            msg = RadarFrameFull()
            msg.platform       = radar_params['platform']
            msg.adc_output_fmt = radar_params['adc_output_fmt']
            msg.range_bias     = radar_params['range_bias']
            msg.rx_phase_bias  = radar_params['rx_phase_bias']

            msg.chirp_time   = radar_params['chirp_time']
            msg.chirp_slope  = radar_params['chirp_slope']
            msg.frame_time   = radar_params['frame_time']
            msg.velocity_max = radar_params['velocity_max']
            msg.velocity_res = radar_params['velocity_res']

            msg.sample_rate  = radar_params['sample_rate']
            msg.range_max    = radar_params['range_max']
            msg.range_res    = radar_params['range_res']

            msg.rx = radar_params['rx']
            msg.tx = radar_params['tx']
            msg.shape = (radar_params['n_chirps'],
                         radar_params['n_rx'],
                         radar_params['n_samples'])

            msg.data = frame_data

            publisher.publish(msg)

