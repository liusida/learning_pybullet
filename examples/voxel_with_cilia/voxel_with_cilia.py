#%%

# voxel with cilia force
# collision

import pybullet as p
from time import sleep
import pybullet_data
import random, math
import numpy as np
import pyquaternion

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf", [0,0,-1], useMaximalCoordinates = True)

voxel1 = p.loadURDF("./cube_small.urdf", globalScaling=10, basePosition=[0,0,0])

for i in np.arange(start=-5, stop=5, dtype=np.int):
    for j in np.arange(start=-5, stop=5, dtype=np.int):
        voxel2 = p.loadURDF("./cube_small.urdf", globalScaling=5, basePosition=[i,j,-0.5])
        p.changeDynamics(voxel2, -1, mass=0.2)
        p.changeVisualShape(voxel2, -1, rgbaColor=[0,1,0,1])

p.changeVisualShape(voxel1, -1, rgbaColor=[1,0,0,1])
cilia_force = None

def random_ciliaforce():
    r = random.random()
    total_force = 2
    return (np.random.random(size=[6,3])*2-1) * total_force

cilia_force = random_ciliaforce()

initialPos1, initialOrn1 = p.getBasePositionAndOrientation(voxel1)

six_dim = np.concatenate((np.eye(3), -np.eye(3)))
six_dim = six_dim * 0.3

debugItem = None
iteration = 0
while True:
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == p.B3G_F5 and (v & p.KEY_WAS_TRIGGERED)):
            print("Reset to initial position.")
            cilia_force = random_ciliaforce()
            p.resetBasePositionAndOrientation(voxel1, initialPos1, initialOrn1)

    spherePos, orn = p.getBasePositionAndOrientation(voxel1)
    
    q = pyquaternion.Quaternion(orn[3], orn[0], orn[1], orn[2])

    cameraTargetPosition = spherePos
    
    if debugItem is not None:
        for i in debugItem:
            p.removeUserDebugItem(i)
    debugItem = []
    for i in range(6):
        force = q.rotate(cilia_force[i])
        facePos = np.array(spherePos) + q.rotate(six_dim[i])
        p.applyExternalForce(voxel1, -1, force, facePos, flags=p.WORLD_FRAME)
        line = p.addUserDebugLine(facePos, facePos+force)
        debugItem.append(line)

    p.stepSimulation()
    sleep(1./240.)
    iteration += 1