from params import PARAMETERS

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self):
        self.kp = PARAMETERS['throttle']['kp']
        self.ki = PARAMETERS['throttle']['ki']
        self.kd = PARAMETERS['throttle']['kd']
        self.min = PARAMETERS['throttle']['min']
        self.max = PARAMETERS['throttle']['max']

        self.int_val = 0.
        self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        val = self.kp * error + self.ki * integral + self.kd * derivative

        self.last_error = error

        if val > self.max:
            return val
        
        if val < self.min:
            return val

        self.int_val = integral
        return val
