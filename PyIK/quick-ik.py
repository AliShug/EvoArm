import numpy as np
from matplotlib import pyplot as plt

from src.util import *
from src.solvers import Circle
from src.litearm import ArmController

width = 300
height = 600
goal = np.zeros((width,height,2))
for j in xrange(height):
    for i in xrange(width):
        goal[i,j]=[i+40, -j+height/2]

origin=np.array([18.71,0])

elevator_length = 148.4
forearm_length = 160.0

valid=np.ones((width,height), dtype=bool)
centers = np.array(goal)
diff = centers - origin
dists = np.linalg.norm(diff, axis=2)

# close enough for intersection
valid &= dists < forearm_length + elevator_length
valid &= dists > 0
# intersect
a = (forearm_length**2 - elevator_length**2 + dists**2) / (dists*2)
h = np.sqrt(forearm_length**2 - a**2)
p2 = centers + (np.dstack([a,a])*(origin - centers)) / np.dstack([dists,dists])
i1 = np.array(p2)
# [:, :, ::-1] flips x and y coords
# dstack is a lazy way to get the scalar h/dists to multiply across the vectors
i1 += [1,-1] * np.dstack([h,h]) * (origin - centers)[:,:,::-1] / np.dstack([dists,dists])
i2 = np.array(p2)
i2 += [-1,1] * np.dstack([h,h]) * (origin - centers)[:,:,::-1] / np.dstack([dists,dists])

print('my dist={0}'.format(dists))
print('my a={0}'.format(a))
print('my h={0}'.format(h[0,0]))
print('my p2={0}'.format(p2[0,0]))
print('my i1={0}'.format(i1[0,0]))
print(origin-centers)
c1 = Circle(origin, elevator_length)
c2 = Circle([250,40], forearm_length)
c2.intersect(c1)
