import numpy as np

from util import *

class ArmVolume:
    def __init__(self, arm, res=100, offset=90, backSize=500):
        self.offset = offset
        self.res = res
        self.samples = np.ndarray(shape=(res,res), dtype=bool)
        self.size = (430-offset, backSize)

        self.y_step = self.size[1]/float(res)
        self.x_step = self.size[0]/float(res)
        for y in xrange(res):
            for x in xrange(res):
                pos = [0, (y-res/2)*self.y_step, offset + x*self.x_step]
                self.samples[x,y] = self.trueValid(arm, pos)

    def getRadii(self, height):
        y_ind = np.clip(int(height/self.y_step + self.res/2), 0, self.res-1)

        min = 0
        max = None
        for x in xrange(self.res):
            if max is None:
                if not self.samples[x, y_ind]:
                    min = self.offset + x*self.x_step
            if self.samples[x, y_ind]:
                max = self.offset + x*self.x_step

        return min,max

    def isValid(self, point):
        """Whether or not a potential end-effector position is likely to be
        valid. Point must be 2D (in IK plane)
        """
        x = int((point[0]-self.offset)/self.x_step)
        y = int(point[1]/self.y_step)
        return self.samples[x, y]

    def trueValid(self, arm, point):
        arm.setWristGoalPosition(point)
        if arm.ik.valid:
            pose = arm.getIKPose()
            if pose is not None:
                test = pose.checkClearance()
            else:
                test = False
        else:
            test = False
        return test


    def projectValid(self, arm, point):
        """Projects an invalid point to a valid one"""
        point = np.array(point)
        # 3D projection target point in plane
        td = normalize([point[0], point[2]])*300
        proj = np.array([td[0], 0, td[1]])
        point += (proj - point)*0.005

        iters = 0
        while not self.trueValid(arm, point) and iters < 1000:
            point += (proj - point)*0.005
            iters += 1
        return point
