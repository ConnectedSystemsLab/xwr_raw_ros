#!/usr/bin/env/python3

"""Framebuffer for UDP stream from DCA1000.

Needed because you must zero pad on missing frames to prevent entire data stream from being corrupted.
"""

import numpy as np
from numba import int8, int64
from numba.experimental import jitclass

spec = [
    ('buffer', int8[:]),
    ('frame_size', int64),
    ('curr_idx', int64),
    ('last_seqn', int64)
]

@jitclass(spec)
class FrameBuffer():
    def __init__(self, capacity, frame_size):

        assert (capacity % frame_size) == 0, 'Capacity must be a multiple of frame size.'

        self.buffer = np.zeros(capacity, dtype=np.int8)
        self.frame_size = frame_size
        self.curr_idx = 0
        self.last_seqn = 0

    def pad_zeros(self, n_msgs, msg_size):
        total_size = n_msgs*msg_size

        if self.curr_idx + total_size > self.buffer.size:
            self.buffer[self.curr_idx:] = 0
            self.buffer[:(self.curr_idx + total_size) % self.buffer.size] = 0
            self.curr_idx = (self.curr_idx + total_size) % self.buffer.size
        else:
            self.buffer[self.curr_idx:self.curr_idx + total_size] = 0

    def add_msg(self, seqn, msg):
        msg = np.frombuffer(msg, np.int8)

        if seqn > self.last_seqn + 1:
            # rospy.loginfo(f'Packet drop when recving seqn {seqn} (last seqn {self.last_seqn}')
            print('Packet drop')
            self.pad_zeros((seqn - self.last_seqn - 1), msg.size)

        self.last_seqn = seqn

        if self.curr_idx + msg.size > self.buffer.size:
            self.buffer[self.curr_idx:] = msg[:(self.buffer.size - self.curr_idx)]
            self.buffer[:msg.size - (self.buffer.size - self.curr_idx)] = msg[(self.buffer.size - self.curr_idx):]
        else:
            self.buffer[self.curr_idx : self.curr_idx + msg.size] = msg

        old_frame_idx = self.curr_idx // self.frame_size
        self.curr_idx = (self.curr_idx + msg.size) % self.buffer.size
        new_frame_idx = self.curr_idx // self.frame_size

        if old_frame_idx != new_frame_idx:
            return self.buffer[old_frame_idx*self.frame_size :
                               (old_frame_idx + 1)*self.frame_size].view(np.int16), True
        else:
            return self.buffer[old_frame_idx*self.frame_size :
                               (old_frame_idx + 1)*self.frame_size].view(np.int16), False

