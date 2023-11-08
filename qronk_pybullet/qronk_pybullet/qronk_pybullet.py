import pybullet as p
import time
import pybullet_data
import os
from ament_index_python import get_package_share_directory

def main(pathURDF=get_package_share_directory('qronk_pybullet')): #Gives option to change URDF file later
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")

    startPos = [0,0,1]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    p.setAdditionalSearchPath(pathURDF)
    qronkId = p.loadURDF("qronk.urdf",startPos, startOrientation)

    for i in range (10000):
        p.stepSimulation()
        time.sleep(1./240.)

    qronkPos, qronkOrn = p.getBasePositionAndOrientation(qronkId)
    print(qronkPos,qronkOrn)
    p.disconnect()

if __name__ == '__main__':
    main()