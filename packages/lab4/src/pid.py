#! /usr/bin/env python3

import numpy as np


class PID():

    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.prev_error = None
        self.prev_time = None
        self.total_dist = 0
    def derivative_value(self, time, error):
        if(time == self.prev_time):
            return 0
        else:
            return (error - self.prev_error)/(time - self.prev_time)

    def integral_value(self, time, error):

        if(time == self.prev_time):
            return 0
        else:
            self.total_dist += (self.prev_error*(time-self.prev_time))
            return self.total_dist

    def get_control(self, time, error):
        if (self.prev_error is None or self.prev_time is None):
            self.prev_time = time
            self.prev_error = error

        p = self.k_p*error
        i = self.k_i*self.integral_value(time, error)
        d = self.k_d*self.derivative_value(time, error)
        self.prev_time = time
        self.prev_error = error
        return (p + i + d)
