# Author: Sida Liu 2020 (learner.sida.liu@gmail.com)
# A convenient basic wrapper for pybullet
# so we can add some functionality to pybullet without modify others source code
# tested under pybullet 2.8.5 (installed via `pip install pybullet`)

import time

class pybullet_wrapper(object):
    def __init__(self, p):
        self.p = p

    # Basic mechanism: calling __getattribute__() will not trigger __getattr__(), so we override __getattr__, and calling __getattribute__ accordingly.
    def __getattr__(self, name):
        # Check for special member variable `p` and methods implemented in this class
        if name=='p' or name in type(self).__dict__:
            return self.__getattribute__(self, name)
        # Otherwise, call pybullet's function
        return getattr(self.p, name)

    # Handy shortcuts implemented here
    def start(self):
        self.p.connect(self.p.GUI)
    def sleep(self, n=0.1):
        time.sleep(n)

if __name__ == "__main__":
    import pybullet, time
    p = pybullet_wrapper(pybullet)
    p.start()
    while True:
        p.stepSimulation()
        time.sleep(0.1)
