import numpy as np

class PIDControl:
    def __init__(self, p, i, d):
        self.err = 0
        self.target = 0
        self.integral = 0

        self.kP=p
        self.kI=i
        self.kD=d

    def setTarget(self, newTarget):
        if newTarget != self.target:
            self.target = newTarget
            #self.integral = 0

    def stepOutput(self, cur_val):
        # Diff current and target values (error)
        error = cur_val - self.target

        # PID (wikipedia.org/wiki/PID_Controller)
                 # Absolute position error - approach faster if we're further
                 # away
        output = (self.kP * error
                 # Integral is the cumulative error - area under the error graph
                 + self.kI * self.integral
                 # Differential is error 'slope', diff between current and last
                 # error values
                 + self.kD * (error - self.err))

        # Store error for diff
        self.err=error
        # Accumulate integral (remember, cumulative error)
        self.integral+=error

        return output
