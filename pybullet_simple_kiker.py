import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
playerOnePos = [0,0,0]
playerTwoPos = [0,2,0]
spherePos = [0,0.48,0.5]
startOrientation = p.getQuaternionFromEuler([0,0,0])

sphereId = p.loadURDF("sphere2red.urdf", spherePos, startOrientation)

playerOneId = p.loadURDF("assets/biped_player.urdf", playerOnePos, startOrientation)
playerTwoId = p.loadURDF("assets/biped_player.urdf", playerTwoPos, startOrientation)

print(p.getLinkState(playerOneId, 1))

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
p.setJointMotorControl2(playerOneId,
                    1,
                    p.POSITION_CONTROL,
                    targetPosition=-1,
                    targetVelocity=0,
                    force=1000,
                    positionGain=1,
                    velocityGain=1,
                    maxVelocity=1)

jointFrictionForce = 0.1
p.setJointMotorControl2(playerTwoId, 1, p.POSITION_CONTROL, force=jointFrictionForce)

for i in range (20000):
        
    js = p.getJointState(playerOneId, 1)
    if js[0] + 1 < 10**(-3):
        p.setJointMotorControl2(playerOneId,
                    1,
                    p.POSITION_CONTROL,
                    targetPosition=2,
                    targetVelocity=0,
                    force=1000,
                    positionGain=1,
                    velocityGain=1,
                    maxVelocity=10)
    #print("position=", js[0], "velocity=", js[1])

    p.stepSimulation()
    time.sleep(1./240.)

#cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
#spherePos, sphereOrn = p.getBasePositionAndOrientation(sphereId)

p.disconnect()
