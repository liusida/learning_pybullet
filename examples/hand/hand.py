import pybullet as p
import pybullet_data
from time import sleep
import random
import numpy as np

p.connect(p.GUI)
p.setGravity(0,0,-1)

p.setAdditionalSearchPath(pybullet_data.getDataPath())  
p.loadURDF("plane.urdf")
hands = []
for i in range(5):
    objects = p.loadMJCF("MPL/MPL.xml")
    hand = objects[0]
    hands.append(hand)
    p.resetBasePositionAndOrientation(hand, posObj=[0,0,0.5*i], ornObj=[0,0,0,1])
    for i in range(p.getNumJoints(hand)):
        p.setJointMotorControl2(hand, i, controlMode=p.VELOCITY_CONTROL, targetPosition=0)

for i in range(100000):
    
    for j,hand in enumerate(hands):
        if (i-j*10)%50==0:
            num_joints = p.getNumJoints(hand)
            # print(num_joints, " Joints.")
            jointIndices = np.arange(num_joints)
            targetPositions = np.random.random(size=[num_joints])
            forces = np.random.random(size=[num_joints])
            # targetPositions[::5] = 0
            p.setJointMotorControlArray(hand, controlMode=p.POSITION_CONTROL, jointIndices=jointIndices, targetPositions=targetPositions, forces=forces)

    p.stepSimulation()
    sleep(.003)