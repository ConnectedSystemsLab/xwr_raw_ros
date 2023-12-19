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
    args = parser.parse_args(rospy.myargv()[1:])

    # Initialize node and topic.
    rospy.init_node('xwr_radar')

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
                     host_data_port = None)
    radar.configure()
    radar.start_capture()

    rospy.on_shutdown(lambda : radar.close())

    rate = rospy.Rate(1)
    while True:
        rate.sleep()
