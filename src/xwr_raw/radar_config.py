#!/usr/bin/env python3

"""Wrapper for .cfg file used to configure radar EVM.

Also computes derived parameters from the config using get_params().
"""

from collections import OrderedDict
import pprint

class RadarConfig(OrderedDict):
    """Container for EVM config that gets sent through UART.

    Attributes:
        cmds: List of valid commands (developed for SDK 3.5.0.4)
        multi_cmds: List of commands that can appear more than once per config file.
    """

    headers = [
        'Created for SDK',
        'Platform',
    ]

    cmds = [
        'dfeDataOutputMode',
        'channelCfg',
        'adcCfg',
        'adcbufCfg',
        # 'profileCfg',
        # 'chirpCfg',
        'lowPower',
        'frameCfg',
        'guiMonitor',
        # 'cfarCfg',
        'multiObjBeamForming',
        'calibDcRangeSig',
        'clutterRemoval',
        'aoaFovCfg',
        # 'cfarFovCfg',
        'compRangeBiasAndRxChanPhase',
        'measureRangeBiasAndRxChanPhase',
        'extendedMaxVelocity',
        'bpmCfg',
        # 'CQRxSatMonitor',
        # 'CQSigImgMonitor',
        'analogMonitor',
        'lvdsStreamCfg',
        'calibData',
    ]

    multi_cmds = [
        'profileCfg',
        'chirpCfg',
        'cfarCfg',
        'cfarFovCfg',
        'CQRxSatMonitor',
        'CQSigImgMonitor',
    ]

    def __init__(self):
        super(RadarConfig, self).__init__()

    def __init__(self, cfg):
        """Initialize RadarConfig from a list of config commands or a dict of config commands.

        Args:
            cfg: List of strings, where each string is a config command. Alternatively, a dict mapping config commands to parameters.
        """
        super(RadarConfig, self).__init__()
        if isinstance(cfg, list):
            self.from_cfg(cfg)
        elif isinstance(cfg, dict):
            for k,v in cfg.items():
                self[k] = v

    def from_cfg(self, cfg):
        """Add commands to RadarConfig from a list of config commands.

        Args:
            cfg: List of strings, where each string is a config command.
        """

        for line in cfg:
            for hdr in RadarConfig.headers:
                if hdr in line:
                    params = line.split(':')[1].strip()
                    self[hdr] = params
                    break

            for cmd in RadarConfig.cmds:
                if cmd in line:
                    params = [float(x) if '.' in x else int(x)
                              for x in line.split()[1:]]

                    self[cmd] = params
                    break

            for cmd in RadarConfig.multi_cmds:
                if cmd in line:
                    params = [float(x) if '.' in x else int(x)
                              for x in line.split()[1:]]

                    if cmd not in self.keys():
                        self[cmd] = [params]
                    else:
                        self[cmd].append(params)
                    break


    def to_cfg(self):
        """Convert RadarConfig into a list of config commands.

        Args:
            cfg: List of strings, where each string is a config command.
        """
        cfg = []

        for cmd, params in self.items():
            if isinstance(params[0], list):
                # Multi cmd case.
                for param in params:
                    cfg.append(' '.join([cmd] + [f'{x:.2f}' if type(x) is float else f'{x:}'
                                                 for x in param]))
            else:
                cfg.append(' '.join([cmd] + [f'{x:.2f}' if type(x) is float else f'{x:}'
                                             for x in params]))

        return cfg

    def get_params(self):
        """Returns number of samples, rx, tx, chirps, frame size, frame time, etc."""

        sdk = self['Created for SDK']
        platform = self['Platform']

        adc_output_fmt = int(self['adcCfg'][1]) # 0 - real, 1 - complex 1x, 2- complex 2x

        n_samples = int(self['profileCfg'][0][9])

        rx_str = self['channelCfg'][0]
        rx_bin = bin(int(rx_str))[2:]
        rx = [int(x) for x in reversed(rx_bin)]

        tx_str = self['channelCfg'][1]
        tx_bin = bin(int(tx_str))[2:]
        tx = [int(x) for x in reversed(tx_bin)]

        n_chirps = (int(self['frameCfg'][1]) - int(self['frameCfg'][0]) + 1)*self['frameCfg'][2]

        n_tx = sum(tx)
        n_rx = sum(rx)

        frame_size = n_samples*n_rx*n_chirps*2*(2 if adc_output_fmt > 0 else 1)
        frame_time = self['frameCfg'][4]

        range_bias = self['compRangeBiasAndRxChanPhase'][0]
        # rx_phase_bias = [a + 1j*b for a,b in zip(self['compRangeBiasAndRxChanPhase'][1::2],
        #                                          self['compRangeBiasAndRxChanPhase'][2::2])]
        rx_phase_bias = self['compRangeBiasAndRxChanPhase'][1:]

        operating_freq = self['profileCfg'][0][1]                         # Units in GHz
        chirp_time  = self['profileCfg'][0][2] + self['profileCfg'][0][4] # Idle time + ramp time, Units in usec
        velocity_max = (3e8/(operating_freq*1e9))/(4*(chirp_time*1e-6))   # Units in m
        velocity_res = velocity_max/n_chirps                              # Units in m

        chirp_slope = self['profileCfg'][0][7]*1e12                       # Units in MHz/usec
        sample_rate = self['profileCfg'][0][10]*1e3                       # Units in ksps
        range_max = (sample_rate*3e8)/(2*chirp_slope)                     # Units in m
        range_res = range_max/n_samples                                   # Units in m

        return OrderedDict([
            ('sdk', sdk),
            ('platform', platform),
            ('adc_output_fmt', adc_output_fmt),
            ('range_bias', range_bias),
            ('rx_phase_bias', rx_phase_bias),
            ('n_chirps', n_chirps),
            ('rx', rx),
            ('n_rx', n_rx),
            ('tx', tx),
            ('n_tx', n_tx),
            ('n_samples', n_samples),
            ('frame_size', frame_size),
            ('frame_time', frame_time),
            ('chirp_time', chirp_time),
            ('chirp_slope', chirp_slope),
            ('sample_rate', sample_rate),
            ('velocity_max', velocity_max),
            ('velocity_res', velocity_res),
            ('range_max' , range_max),
            ('range_res' , range_res),
        ])

if __name__ == '__main__':
    with open('configs/6843aop/6843aop_doppler_v1.cfg', 'r') as f:
        cfg = f.readlines()

    radar_config = RadarConfig(cfg)
    pprint.pprint(radar_config)
    pprint.pprint(radar_config.to_cfg())
    pprint.pprint(radar_config.get_params())

