import numpy as np
import pybullet as p
from time import sleep
p.connect(p.GUI, options='--background_color_red=0.96 --background_color_green=0.88 --background_color_blue=0.84')
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)

p.setAdditionalSearchPath("/home/liusida/thesis/code/3rdparty/pybullet/bullet3/data")
# Model downloaded from https://sketchfab.com/3d-models/matilda-7ddedfb652bd4ea091bc3de27f98fc02
# FBX convert into OBJ,MTL at http://www.greentoken.de/onlineconv/

girlColId = p.createCollisionShape(p.GEOM_MESH, fileName="Mathilda_Lando.obj", meshScale=[0.01,0.01,0.01], collisionFrameOrientation=[0.71,0,0,0.71])
girlVizId = p.createVisualShape(p.GEOM_MESH, fileName="Mathilda_Lando.obj", meshScale=[0.01,0.01,0.01], visualFrameOrientation=[0.71,0,0,0.71])
girl = p.createMultiBody(baseMass=1, baseVisualShapeIndex=girlVizId, baseCollisionShapeIndex=girlColId, basePosition=[0,0,0.5], baseInertialFramePosition=[0,0,0.5])

texture = p.loadTexture('color.png')
p.changeVisualShape(girl, -1, textureUniqueId=texture)

p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=-56, cameraPitch=0.6, cameraTargetPosition=[0.352,-0.28,0.676])

# Floor texture downloaded from https://www.pinterest.com/pin/565835140659425261/
planeId = p.loadURDF("plane.urdf")
texture_floor = p.loadTexture("floor.png")
p.changeVisualShape(planeId, -1, textureUniqueId=texture_floor, rgbaColor=[0.96,0.88,0.84,0.3])
p.setGravity(0,0,-10)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)

while True:
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k==97 and (v & p.KEY_WAS_RELEASED)):
            camInfo = p.getDebugVisualizerCamera()
            print("getDebugVisualizerCamera")
            print(camInfo[8:])

    p.stepSimulation()
    sleep(.006)