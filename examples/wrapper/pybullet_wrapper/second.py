# Author: Sida Liu 2020 (learner.sida.liu@gmail.com)
# The second layer of wrapper for pybullet
# so we can add some functionality to pybullet without modify others source code
# tested under pybullet 2.8.5 (installed via `pip install pybullet`)

from .basic import pybullet_wrapper

class pybullet_2nd(pybullet_wrapper):
    def test1(self):
        print("\n\n")
        print("This is in pybullet_2nd::test1().")
        print(self.p)
        print("\n\n")
