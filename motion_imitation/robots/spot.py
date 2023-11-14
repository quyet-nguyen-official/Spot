import math
import re
import numpy as np
import pybullet as p

from motion_imitation.robots import spot_pose_utils
from motion_imitation.robots import spot_constants
from motion_imitation.robots import laikago_motor
from motion_imitation.robots import minitaur
from motion_imitation.robots import robot_config
from motion_imitation.envs import locomotion_gym_config

NUM_MOTORS = 12
NUM_LEGS = 4
MOTOR_NAMES = [
    "FR_hip_motor_2_chassis_joint",
    "FR_upper_leg_2_hip_motor_joint",
    "FR_lower_leg_2_upper_leg_joint",
    "FL_hip_motor_2_chassis_joint",
    "FL_upper_leg_2_hip_motor_joint",
    "FL_lower_leg_2_upper_leg_joint",
    "RR_hip_motor_2_chassis_joint",
    "RR_upper_leg_2_hip_motor_joint",
    "RR_lower_leg_2_upper_leg_joint",
    "RL_hip_motor_2_chassis_joint",
    "RL_upper_leg_2_hip_motor_joint",
    "RL_lower_leg_2_upper_leg_joint",
]
INIT_RACK_POSITION = [0, 0, 1]
INIT_POSITION = [0, 0, 0.48]
JOINT_DIRECTIONS = np.array([-1, 1, 1, 1, 1, 1, -1, 1, 1, 1, 1, 1])
HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = -0.6
KNEE_JOINT_OFFSET = 0.66
DOFS_PER_LEG = 3
JOINT_OFFSETS = np.array([HIP_JOINT_OFFSET, UPPER_LEG_JOINT_OFFSET, KNEE_JOINT_OFFSET] * 4)
PI = math.pi
TWO_PI = 2. * PI

MAX_MOTOR_ANGLE_CHANGE_PER_STEP = 0.2
_DEFAULT_HIP_POSITIONS = (
    (0.21, -0.1157, 0),
    (0.21, 0.1157, 0),
    (-0.21, -0.1157, 0),
    (-0.21, 0.1157, 0),
)

ABDUCTION_P_GAIN = 220.0
ABDUCTION_D_GAIN = 0.3
HIP_P_GAIN = 220.0
HIP_D_GAIN = 2.0
KNEE_P_GAIN = 220.0
KNEE_D_GAIN = 2.0

# Bases on the readings from Laikago's default pose.
INIT_MOTOR_ANGLES = np.array([
                                 spot_pose_utils.LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
                                 spot_pose_utils.LAIKAGO_DEFAULT_HIP_ANGLE,
                                 spot_pose_utils.LAIKAGO_DEFAULT_KNEE_ANGLE
                             ] * NUM_LEGS)

_CHASSIS_NAME_PATTERN = re.compile(r"\w+_chassis_\w+")
_MOTOR_NAME_PATTERN = re.compile(r"\w+_hip_motor_\w+")
_KNEE_NAME_PATTERN = re.compile(r"\w+_lower_leg_\w+")
_TOE_NAME_PATTERN = re.compile(r"jtoe\d*")

URDF_FILENAME = "urdf_robot/laikago_copy/laikago_toes_limits.urdf"

_BODY_B_FIELD_NUMBER = 2
_LINK_A_FIELD_NUMBER = 3

UPPER_BOUND = TWO_PI
LOWER_BOUND = -TWO_PI


class Spot(minitaur.Minitaur):

    MPC_BODY_MASS = 215 / 9.8
    MPC_BODY_INERTIA = (0.07335, 0, 0, 0, 0.25068, 0, 0, 0, 0.25447)
    MPC_BODY_HEIGHT = 0.42
    ACTION_CONFIG = [
        locomotion_gym_config.ScalarField(name="motor_angle_0",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_1",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_2",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_3",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_4",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_5",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_6",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_7",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_8",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_9",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_10",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND),
        locomotion_gym_config.ScalarField(name="motor_angle_11",
                                          upper_bound=UPPER_BOUND,
                                          lower_bound=LOWER_BOUND)
    ]

    def __init__(
            self,
            pybullet_client,
            motor_control_mode,
            urdf_filename=URDF_FILENAME,
            enable_clip_motor_commands=False,
            time_step=0.001,
            action_repeat=33,
            sensors=None,
            control_latency=0.002,
            on_rack=False,
            enable_action_interpolation=True,
            enable_action_filter=False,
            reset_time=-1,
            allow_knee_contact=False,
    ):
        self._urdf_filename = urdf_filename
        self._allow_knee_contact = allow_knee_contact
        self._enable_clip_motor_commands = enable_clip_motor_commands

        motor_kp = [
            ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN,
            HIP_P_GAIN, KNEE_P_GAIN, ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN,
            ABDUCTION_P_GAIN, HIP_P_GAIN, KNEE_P_GAIN
        ]
        motor_kd = [
            ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN,
            HIP_D_GAIN, KNEE_D_GAIN, ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN,
            ABDUCTION_D_GAIN, HIP_D_GAIN, KNEE_D_GAIN
        ]

        super().__init__(
            pybullet_client=pybullet_client,
            time_step=time_step,
            action_repeat=action_repeat,
            num_motors=NUM_MOTORS,
            dofs_per_leg=DOFS_PER_LEG,
            motor_direction=JOINT_DIRECTIONS,
            motor_offset=JOINT_OFFSETS,
            motor_control_mode=motor_control_mode,
            motor_model_class=laikago_motor.LaikagoMotorModel,
            sensors=sensors,
            motor_kp=motor_kp,
            motor_kd=motor_kd,
            control_latency=control_latency,
            on_rack=on_rack,
            enable_action_interpolation=enable_action_interpolation,
            enable_action_filter=enable_action_filter,
            reset_time=reset_time)

    def _load_robot_urdf_(self):
        laikago_urdf_path = self.get_urdf_file_()
        if self._self_collision_enabled:
            self.quadruped = self._pybullet_client.loadURDF(
                laikago_urdf_path,
                self._get_default_init_position_(),
                self._get_default_init_orientation_(),
                flags=self._pybullet_client.URDF_USE_SELF_COLLISION)
        else:
            self.quadruped = self._pybullet_client.loadURDF(
                laikago_urdf_path, self._get_default_init_position_(),
                self._get_default_init_orientation_())

    def _settle_down_for_reset_(self, default_motor_angles, reset_time):
        self._receive_observation()

        if reset_time <= 0:
            return

        for _ in range(500):
            self._step_internal(
                INIT_MOTOR_ANGLES,
                motor_control_mode=robot_config.MotorControlMode.POSITION)
        if default_motor_angles is not None:
            num_steps_to_reset = int(reset_time / self.time_step)
            for _ in range(num_steps_to_reset):
                self._step_internal(
                    default_motor_angles,
                    motor_control_mode=robot_config.MotorControlMode.POSITION)

    @staticmethod
    def get_hip_positions_in_base_frame():
        return _DEFAULT_HIP_POSITIONS

    def get_foot_contacts_(self):
        all_contacts = self._pybullet_client.getContactPoints(bodyA=self.quadruped)

        contacts = [False, False, False, False]
        for contact in all_contacts:
            # Ignore self contacts
            if contact[_BODY_B_FIELD_NUMBER] == self.quadruped:
                continue
            try:
                toe_link_index = self._foot_link_ids.index(
                    contact[_LINK_A_FIELD_NUMBER])
                contacts[toe_link_index] = True
            except ValueError:
                continue

        return contacts

    def compute_jacobian_(self, leg_id):
        """Compute the Jacobian for a given leg."""
        # Because of the default rotation in the Laikago URDF, we need to reorder
        # the rows in the Jacobian matrix.
        return super().compute_jacobian(leg_id)[(2, 0, 1), :]

    def reset_pose_(self, add_constraint):
        del add_constraint
        for name in self._joint_name_to_id:
            joint_id = self._joint_name_to_id[name]
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=joint_id,
                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0)
        for name, i in zip(MOTOR_NAMES, range(len(MOTOR_NAMES))):
            if "hip_motor_2_chassis_joint" in name:
                angle = INIT_MOTOR_ANGLES[i] + HIP_JOINT_OFFSET
            elif "upper_leg_2_hip_motor_joint" in name:
                angle = INIT_MOTOR_ANGLES[i] + UPPER_LEG_JOINT_OFFSET
            elif "lower_leg_2_upper_leg_joint" in name:
                angle = INIT_MOTOR_ANGLES[i] + KNEE_JOINT_OFFSET
            else:
                raise ValueError("The name %s is not recognized as a motor joint." %
                                 name)
            self._pybullet_client.resetJointState(self.quadruped,
                                                  self._joint_name_to_id[name],
                                                  angle,
                                                  targetVelocity=0)

    def get_urdf_file_(self):
        return self._urdf_filename

    def _build_urdf_ids_(self):
        """Build the link Ids from its name in the URDF file.

    Raises:
      ValueError: Unknown category of the joint name.
    """
        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        self._chassis_link_ids = [-1]
        self._leg_link_ids = []
        self._motor_link_ids = []
        self._knee_link_ids = []
        self._foot_link_ids = []

        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
            joint_name = joint_info[1].decode("UTF-8")
            joint_id = self._joint_name_to_id[joint_name]
            if _CHASSIS_NAME_PATTERN.match(joint_name):
                self._chassis_link_ids.append(joint_id)
            elif _MOTOR_NAME_PATTERN.match(joint_name):
                self._motor_link_ids.append(joint_id)
            # We either treat the lower leg or the toe as the foot link, depending on
            # the urdf version used.
            elif _KNEE_NAME_PATTERN.match(joint_name):
                self._knee_link_ids.append(joint_id)
            elif _TOE_NAME_PATTERN.match(joint_name):
                self._foot_link_ids.append(joint_id)
            else:
                raise ValueError("Unknown category of joint %s" % joint_name)

        self._leg_link_ids.extend(self._knee_link_ids)
        self._leg_link_ids.extend(self._foot_link_ids)
        if self._allow_knee_contact:
            self._foot_link_ids.extend(self._knee_link_ids)

        self._chassis_link_ids.sort()
        self._motor_link_ids.sort()
        self._foot_link_ids.sort()
        self._leg_link_ids.sort()

    @staticmethod
    def _get_motor_names_():
        return MOTOR_NAMES

    def _get_default_init_position_(self):
        if self._on_rack:
            return INIT_RACK_POSITION
        else:
            return INIT_POSITION

    @staticmethod
    def _get_default_init_orientation_():
        # The Laikago URDF assumes the initial pose of heading towards z axis,
        # and belly towards y-axis. The following transformation is to transform
        # the Laikago initial orientation to our commonly used orientation: heading
        # towards -x direction, and z axis is the up direction.
        init_orientation = p.getQuaternionFromEuler(
            [math.pi / 2.0, 0, math.pi / 2.0])
        return init_orientation

    def get_default_init_position(self):
        """Get default initial base position."""
        return self._get_default_init_position_()

    def get_default_init_orientation(self):
        """Get default initial base orientation."""
        return self._get_default_init_orientation_()

    @staticmethod
    def get_default_init_joint_pose():
        """Get default initial joint pose."""
        joint_pose = (INIT_MOTOR_ANGLES + JOINT_OFFSETS) * JOINT_DIRECTIONS
        return joint_pose

    def apply_action(self, motor_commands, motor_control_mode):
        """Clips and then apply the motor commands using the motor model.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).N
      motor_control_mode: A MotorControlMode enum.
    """
        if self._enable_clip_motor_commands:
            motor_commands = self._clip_motor_commands(motor_commands)

        super()._apply_action(motor_commands, motor_control_mode)

    def _clip_motor_commands(self, motor_commands):
        """Clips motor commands.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands,
        or motor pwms (for Minitaur only).

    Returns:
      Clipped motor commands.
    """

        # clamp the motor command by the joint limit, in case weired things happens
        max_angle_change = MAX_MOTOR_ANGLE_CHANGE_PER_STEP
        current_motor_angles = self.get_motor_angles()
        motor_commands = np.clip(motor_commands,
                                 current_motor_angles - max_angle_change,
                                 current_motor_angles + max_angle_change)
        return motor_commands

    @classmethod
    def get_constants_(cls):
        del cls
        return spot_constants
