# Author: Sida Liu 2020 (learner.sida.liu@gmail.com)
# This is a demo of using wrappers

import pybullet as p
from pybullet_wrapper.basic import pybullet_wrapper
from pybullet_wrapper.second import pybullet_2nd

p = pybullet_wrapper(p)
p = pybullet_2nd(p)
p.test1()
p.start()
while True:
    p.stepSimulation()
    p.sleep()