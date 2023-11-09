import pybullet as p
import time
import pybullet_data
import math
import re

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([math.pi / 2., 0, math.pi / 2.])
spot = p.loadURDF("motion_imitation/robots/urdf_robot/laikago_copy/laikago_toes_limits.urdf",
                  cubeStartPos, cubeStartOrientation)

# Góc nhìn ban đầu
# camera_distance = 1.0
# camera_yaw = -30
# camera_pitch = 0
# p.resetDebugVisualizerCamera(
#     cameraDistance=camera_distance,
#     cameraYaw=camera_yaw,
#     cameraPitch=camera_pitch,
#     cameraTargetPosition=[0, 0, 0]
# )

_CHASSIS_NAME_PATTERN = re.compile(r"\w+_chassis_\w+")
_MOTOR_NAME_PATTERN = re.compile(r"\w+_hip_motor_\w+")
_KNEE_NAME_PATTERN = re.compile(r"\w+_lower_leg_\w+")
_TOE_NAME_PATTERN = re.compile(r"jtoe\d*")

numberjoint = p.getNumJoints(spot)
infojoint = []
for i in range(numberjoint):
    temp = p.getJointInfo(spot, i)
    infojoint.append(temp[1])
zero_vec = [0] * 12
# L, A = p.calculateJacobian(spot, 0, (0, 0, 0), 3, zero_vec, zero_vec)
degrees = 15
radians = degrees * (math.pi / 180)
# p.resetJointState(spot, 0, radians)
# p.resetJointState(spot, 4, radians)
# p.resetJointState(spot, 8, radians)
# p.resetJointState(spot, 12, radians)
allow_knee_contact = False


def convert():
    num = p.getNumJoints(spot)
    joint_to_id = {}
    for t in range(num):
        jointinfo = p.getJointInfo(spot, t)
        joint_to_id[jointinfo[1].decode("UTF-8")] = jointinfo[0]
    return joint_to_id


joint_name_to_id = convert()


def buildurdfids():
    num_joints = p.getNumJoints(spot)

    chassis_link_ids = [-1]
    leg_link_ids = []
    motor_link_ids = []
    knee_link_ids = []
    foot_link_ids = []

    for k in range(num_joints):
        joint_info = p.getJointInfo(spot, k)
        joint_name = joint_info[1].decode("UTF-8")
        joint_id = joint_name_to_id[joint_name]
        if _CHASSIS_NAME_PATTERN.match(joint_name):
            chassis_link_ids.append(joint_id)
        elif _MOTOR_NAME_PATTERN.match(joint_name):
            motor_link_ids.append(joint_id)
        elif _KNEE_NAME_PATTERN.match(joint_name):
            knee_link_ids.append(joint_id)
        elif _TOE_NAME_PATTERN.match(joint_name):
            foot_link_ids.append(joint_id)
        else:
            raise ValueError("Unknown category of joint %s" % joint_name)
    leg_link_ids.extend(knee_link_ids)
    leg_link_ids.extend(foot_link_ids)
    if allow_knee_contact:
        foot_link_ids.extend(knee_link_ids)
    chassis_link_ids.sort()
    motor_link_ids.sort()
    foot_link_ids.sort()
    leg_link_ids.sort()
    return chassis_link_ids, motor_link_ids, foot_link_ids, leg_link_ids


print("----> TESTING <----")
print(f"Number of joint: {numberjoint}")
# for i in infojoint:
#     print(i)
# print(f"Joint state: {p.getJointState(spot, 0)}")
# print(f"Linear jacobian: {L}")
# print(f"Angular jacobian: {A}")
# print(joint_name_to_id)
print(buildurdfids())
print("----> TESTING <----")

for i in range(10000000):
    p.stepSimulation()
    time.sleep(1. / 240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(spot)
# print(cubePos, cubeOrn)
p.disconnect()
