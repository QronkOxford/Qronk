import pybullet as p
import time
import pybullet_data
import os

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")

startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
p.setAdditionalSearchPath(os.environ['HOME']+"/qronk_ws/install/qronk_pybullet/share/qronk_pybullet/")
qronkId = p.loadURDF("qronk.urdf",startPos, startOrientation)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)

qronkPos, qronkOrn = p.getBasePositionAndOrientation(qronkId)
print(qronkPos,qronkOrn)
p.disconnect()
