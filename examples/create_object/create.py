import pybullet as p
from time import sleep

p.connect(p.GUI)

print("isNumpyEnabled", p.isNumpyEnabled())
exit()
p.setGravity(0,0,-10)
planeId = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(baseCollisionShapeIndex=planeId,
                  baseVisualShapeIndex=planeId)

colBoxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, 1, 1])
colBoxId2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2, 1, 1])
mass = 1
link_Masses = [1]
linkCollisionShapeIndices = [colBoxId2]
linkVisualShapeIndices = [-1]
linkPositions = [[1, 0, 2.1]]
linkOrientations = [[0, 0, 0, 1]]
linkInertialFramePositions = [[0, 0, 0]]
linkInertialFrameOrientations = [[0, 0, 0, 1]]
linkParentIndices = [0]
jointTypes = [p.JOINT_REVOLUTE]
axis = [[0, 0, 1]]

sphereUid = p.createMultiBody(baseMass=mass,
                              baseCollisionShapeIndex=colBoxId,
                              baseVisualShapeIndex=-1,
                              basePosition=[0, 0, 1],
                              baseOrientation=[0, 0, 0, 1],
                              linkMasses=link_Masses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=linkParentIndices,
                              linkJointTypes=jointTypes,
                              linkJointAxis=axis)
# dyn_info = p.getDynamicsInfo(sphereUid, -1)

# print(dyn_info)
# p.changeDynamics(sphereUid, -1, linearDamping=0.0, jointDamping=0.0)
for joint in range(p.getNumJoints(sphereUid)):
    print("Setting joint", joint)
    p.setJointMotorControl2(sphereUid, joint, p.VELOCITY_CONTROL, targetVelocity=1, force=100)


while True:
    p.stepSimulation()
    sleep(.0025)
