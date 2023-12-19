#!/usr/bin/env/python3

""" DCA1000EVM socket client for radar data capture.
"""

import socket
import struct

class DCA1000():

    # dca1000evm configuration commands; only the ones used are filled in
    # TODO: hardcoded comand values should be changed
    cmds = { \
        'RESET_FPGA_CMD_CODE'               : b"", \
        'RESET_AR_DEV_CMD_CODE'             : b"", \
        'CONFIG_FPGA_GEN_CMD_CODE'          : b"\x5a\xa5\x03\x00\x06\x00\x01\x02\x01\x02\x03\x1e\xaa\xee", \
        'CONFIG_EEPROM_CMD_CODE'            : b"", \
        'RECORD_START_CMD_CODE'             : b"\x5a\xa5\x05\x00\x00\x00\xaa\xee", \
        'RECORD_STOP_CMD_CODE'              : b"\x5a\xa5\x06\x00\x00\x00\xaa\xee", \
        'PLAYBACK_START_CMD_CODE'           : b"", \
        'PLAYBACK_STOP_CMD_CODE'            : b"", \
        'SYSTEM_CONNECT_CMD_CODE'           : b"\x5a\xa5\x09\x00\x00\x00\xaa\xee", \
        'SYSTEM_ERROR_CMD_CODE'             : b"\x5a\xa5\x0a\x00\x01\x00\xaa\xee", \
        'CONFIG_PACKET_DATA_CMD_CODE'       : b"\x5a\xa5\x0b\x00\x06\x00\xc0\x05\x19\x00\x00\x00\xaa\xee", \
        'CONFIG_DATA_MODE_AR_DEV_CMD_CODE'  : b"", \
        'INIT_FPGA_PLAYBACK_CMD_CODE'       : b"", \
        'READ_FPGA_VERSION_CMD_CODE'        : b"\x5a\xa5\x0e\x00\x00\x00\xaa\xee", \
    }

    def __init__(self,
                 dca_ip         : str = '192.168.33.180',
                 dca_cmd_port   : int = 4096,
                 host_ip        : str = '192.168.33.30',
                 host_cmd_port  : int = 4096,
                 host_data_port : int = 4098):

        self.dca_cmd_addr   = (dca_ip,  dca_cmd_port)
        self.host_cmd_addr  = (host_ip, host_cmd_port)
        self.host_data_addr = (host_ip, host_data_port)

        # Setup command socket.
        self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_socket.bind(self.host_cmd_addr)
        self.cmd_socket.settimeout(10)

        # Setup data socket.
        if host_data_port:
            self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.data_socket.bind(self.host_data_addr)
            self.data_socket.setblocking(True)

        self.capturing = False

    def _recv_cmd(self):
        msg, _ = self.cmd_socket.recvfrom(2048)
        return ' '.join(msg.hex()[i:i+2] \
                for i in range(0,len(msg.hex()),2))

    def _send_cmd(self, cmd):
        self.cmd_socket.sendto(cmd, self.dca_cmd_addr)
        print('DCA1000:/>', ' '.join(cmd.hex()[i:i+2] \
                for i in range(0,len(cmd.hex()),2)))


    def configure(self):
        _cmds = [
            'SYSTEM_CONNECT_CMD_CODE',
            'READ_FPGA_VERSION_CMD_CODE',
            'CONFIG_FPGA_GEN_CMD_CODE',
            'CONFIG_PACKET_DATA_CMD_CODE',
        ]
        for cmd in _cmds:
            self._send_cmd(DCA1000.cmds[cmd])
            print(self._recv_cmd())

    def start_capture(self):
        if not self.capturing:
            self._send_cmd(DCA1000.cmds['RECORD_START_CMD_CODE'])
            print(self._recv_cmd())
            self.capturing = True

    def stop_capture(self):
        if self.capturing:
            self._send_cmd(DCA1000.cmds['RECORD_STOP_CMD_CODE'])
            print(self._recv_cmd())
            self.capturing = False

    def recv_data(self):
        msg, _ = self.data_socket.recvfrom(2048)
        seqn, bytec = struct.unpack('<IIxx', msg[:10])

        return seqn, bytec, msg[10:]

    def close(self):
        self.cmd_socket.close()
        if hasattr(self, 'data_socket'):
            self.data_socket.close()

