import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])

radius = 0.2
mass = 1.0
position = [0,0,1]
color = [1,0,0,1]

colSphere = p.createCollisionShape(p.GEOM_SPHERE, radius = radius)

visSphere = p.createVisualShape(p.GEOM_SPHERE, radius = radius, rgbaColor = color)

sphereID = p.createMultiBody(baseMass = mass,
                             baseCollisionShapeIndex = colSphere,
                             baseVisualShapeIndex=visSphere,
                             basePosition = position)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

print(position)
p.disconnect()
