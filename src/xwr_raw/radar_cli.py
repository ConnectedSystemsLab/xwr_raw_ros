#!/usr/bin/env/python3

"""UART serial communication with radar EVM.
"""

import serial

class RadarCLI():

    cmds = [
        # 'configDataPort 921600 1',
        'sensorStop',
        'flushCfg',
        'sensorStart',
    ]

    def __init__(self,
                 cmd_tty    : str = '/dev/ttyACM0',
                 data_tty   : str = '/dev/ttyACM1'):
        self.cmd_tty, self.data_tty = cmd_tty, data_tty

        self.cmd_serial = serial.Serial(port=self.cmd_tty, 
                                        baudrate=115200, 
                                        bytesize=serial.EIGHTBITS,
                                        parity=serial.PARITY_NONE, 
                                        stopbits=serial.STOPBITS_ONE, 
                                        timeout=0.01)

        self.started = False

    def _recv(self):
        response = self.cmd_serial.read(size=32768)
        return response.decode()

    def _send(self, cmd):
        self.cmd_serial.write(cmd.encode('utf-8'))
        self.cmd_serial.write('\r'.encode())
        self.cmd_serial.reset_input_buffer()


    def configure(self, cfg):
        for cmd in RadarCLI.cmds[:2]:
            self._send(cmd)
            print(self._recv())

        _cmds = cfg.to_cfg()
        for cmd in _cmds:
            self._send(cmd)
            print(self._recv())

    def start(self):
        if not self.started:
            self._send(RadarCLI.cmds[2])
            print(self._recv())
            self.started = True

    def stop(self):
        if self.started:
            self._send(RadarCLI.cmds[0])
            print(self._recv())
            self.started = False

    def close(self):
        self.cmd_serial.close()

