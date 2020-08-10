import random
import pybullet as p
from time import sleep
import numpy as np

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setGravity(0,0,-10)
# p.resetSimulation()

random.seed(10)
heightPerturbationRange = 0.2
numHeightfieldRows = 100
numHeightfieldColumns = 100
heightfieldData = np.zeros(
    shape=[numHeightfieldColumns, numHeightfieldRows], dtype=np.float)

# Calculate vertices accumutively
for i in range(int(numHeightfieldColumns/2)):
    for j in range(int(numHeightfieldRows)):
        n1 = 0
        n2 = 0
        if j > 0:
            n1 = heightfieldData[i, j-1]
        if i > 0:
            n2 = heightfieldData[i-1, j]
        else:
            n2 = n1
        noise = random.uniform(-heightPerturbationRange,
                                heightPerturbationRange)
        heightfieldData[i, j] = (n1+n2)/2 + noise

heightfieldData_inv = heightfieldData[::-1,:]
heightfieldData_2 = np.concatenate((heightfieldData_inv, heightfieldData))
print(heightfieldData_2)

col,row = heightfieldData_2.shape
heightfieldData_2 = heightfieldData_2.reshape(-1)

terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, heightfieldData=heightfieldData_2, meshScale=[0.5,0.5,1],
                                        numHeightfieldRows=row, numHeightfieldColumns=col)
terrain = p.createMultiBody(0, terrainShape)
p.resetBasePositionAndOrientation(terrain, [0, 0, 0], [0, 0, 0, 1])


# Create balls
balls = []
balls_init_pos = []
sphereRadius = 0.1
mass = 1
colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
for i in range(10):
  for j in range(10):
    sphereUid = p.createMultiBody(
        mass,
        colSphereId,
        -1, [i * 3 * sphereRadius, j * 3 * sphereRadius, 2],
        useMaximalCoordinates=True)
    balls.append(sphereUid)
    balls_init_pos.append([i * 3 * sphereRadius, j * 3 * sphereRadius, 2])

while True:
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == p.B3G_F5 and (v & p.KEY_WAS_TRIGGERED)):
            print("Reset to initial position.")
            for i,ball in enumerate(balls):
                p.resetBasePositionAndOrientation(ball, balls_init_pos[i], [0,0,0,1])

    p.stepSimulation()
    sleep(1./200.)
