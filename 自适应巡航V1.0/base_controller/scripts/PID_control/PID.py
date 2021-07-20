#!/usr/bin/python2
# coding: utf-8
#!/usr/bin/env python

class PID:
    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.clear()

    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 10.0
        self.output = 0.0

    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        delta_error = error - self.last_error
        self.PTerm =  error  # 比例
        self.ITerm += error   # 积分
        if (self.ITerm < -self.windup_guard):
            self.ITerm = -self.windup_guard
        elif (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard
        self.DTerm = 0.0

        self.DTerm = delta_error
        self.last_error = error
        self.output = (self.Kp *self.PTerm) + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


