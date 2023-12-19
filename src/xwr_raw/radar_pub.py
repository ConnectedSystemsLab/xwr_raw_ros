#!/usr/bin/env/python3

"""Radar data publisher.

Publisher requires a dedicated thread to receive UDP frames.
"""

import time
import socket
from typing import List

from xwr_raw.radar_config import RadarConfig
from xwr_raw.radar_cli import RadarCLI
from xwr_raw.dca1000 import DCA1000
from xwr_raw.frame_buffer import FrameBuffer


class RadarPub():
    """Radar data publisher. Consists of a DCA1000 and a RadarCLI.
    """
    def __init__(self,
                 cfg            : List[str],
                 cmd_tty        : str = '/dev/ttyUSB0',
                 dca_ip         : str = '192.168.33.181',
                 dca_cmd_port   : int = 5096,
                 host_ip        : str = '192.168.33.30',
                 host_cmd_port  : int = 5096,
                 host_data_port : int = 5098):

        self.config = RadarConfig(cfg)
        self.params = self.config.get_params()
        self.radar_cli = RadarCLI(cmd_tty)
        self.dca1000 = DCA1000(dca_ip,
                               dca_cmd_port,
                               host_ip,
                               host_cmd_port,
                               host_data_port)

        if hasattr(self.dca1000, 'data_socket'):
            self.dca1000.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131071*5)
        self.frame_buffer = FrameBuffer(2*self.params['frame_size'], self.params['frame_size'])

    def configure(self):
        self.dca1000.configure()
        self.radar_cli.configure(self.config)

    def start_capture(self):
        self.dca1000.start_capture()
        time.sleep(1)
        self.radar_cli.start()

    def stop_capture(self):
        self.radar_cli.stop()
        self.dca1000.stop_capture()

    def update_frame_buffer(self):
        seqn, bytec, msg = self.dca1000.recv_data()
        frame_data, new_frame = self.frame_buffer.add_msg(seqn, msg)
        return frame_data, new_frame

    def close(self):
        self.stop_capture()
        self.radar_cli.close()
        self.dca1000.close()

    def __del__(self):
        self.close()

class DCAPub():
    """Same as RadarPub but without RadarCLI. 
    """
    def __init__(self,
                 cfg            : List[str],
                 dca_ip         : str = '192.168.33.181',
                 dca_cmd_port   : int = 5096,
                 host_ip        : str = '192.168.33.30',
                 host_cmd_port  : int = 5096,
                 host_data_port : int = 5098):

        self.config = RadarConfig(cfg)
        self.params = self.config.get_params()
        self.dca1000 = DCA1000(dca_ip,
                               dca_cmd_port,
                               host_ip,
                               host_cmd_port,
                               host_data_port)

        if hasattr(self.dca1000, 'data_socket'):
            self.dca1000.data_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131071*5)
        self.frame_buffer = FrameBuffer(2*self.params['frame_size'], self.params['frame_size'])

    def configure(self):
        self.dca1000.configure()

    def start_capture(self):
        self.dca1000.start_capture()

    def stop_capture(self):
        self.dca1000.stop_capture()

    def update_frame_buffer(self):
        seqn, bytec, msg = self.dca1000.recv_data()
        frame_data, new_frame = self.frame_buffer.add_msg(seqn, msg)
        return frame_data, new_frame

    def close(self):
        self.stop_capture()
        self.dca1000.close()

    def __del__(self):
        self.close()

