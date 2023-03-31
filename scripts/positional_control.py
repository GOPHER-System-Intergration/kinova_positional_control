#!/usr/bin/env python
"""

"""

import rospy
import math
import numpy as np
import transformations

from std_msgs.msg import (Bool)
from geometry_msgs.msg import (Pose)

from kortex_driver.msg import (BaseCyclic_Feedback)
from kortex_driver.srv import (
    Stop,
    ApplyEmergencyStop,
    Base_ClearFaults,
)
from kinova_positional_control.srv import (PidVelocityLimit)
from relaxed_ik_ros1.msg import (EEPoseGoals)


class KinovaPositionalControl:
    """
    
    """

    def __init__(
        self,
        name='my_gen3',
        mounting_angles_deg=(0.0, -48.2, 90.0),
    ):
        """
        
        """

        # Public constants:
        self.ROBOT_NAME = name

        # Rotation matrices around axes. If the arm is mounted on the table,
        # mounting angles should all be zeros.
        self.ROTATE_X = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[0]),
            (1, 0, 0),
        )
        self.ROTATE_Y = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[1]),
            (0, 1, 0),
        )
        self.ROTATE_Z = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[2]),
            (0, 0, 1),
        )

        # Rotation matrices from Global Coordinate System (parallel to the
        # floor, X facing forward globally) to Relaxed IK Coordinate System
        # (parallel to Kinova Arm base, X facing according to Kinova Arm) and
        # back. If the arm is mounted on the table, no rotations will be
        # applied.
        self.ROTATE_GCS_TO_RIKCS = transformations.concatenate_matrices(
            self.ROTATE_X,
            self.ROTATE_Y,
            self.ROTATE_Z,
        )
        self.ROTATE_RIKCS_TO_GCS = transformations.inverse_matrix(
            self.ROTATE_GCS_TO_RIKCS
        )

        # Private constants:

        # Public variables:
        self.is_initialized = False

        # For an input to be executed by the robot tracking should be set to
        # True. When input_is_tracked turns True, on_tracking_start becomes
        # False to calculate input_relaxed_ik_difference, but once
        # input_is_tracked turns False, it turns on_tracking_start True again.
        self.input_is_tracked = False
        self.on_tracking_start = True

        # Input position and orientation in Global and Relaxed IK coordinate
        # systems.
        self.input_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'rikcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                }
        }
        self.relaxed_ik_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'rikcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                }
        }

        # This difference is calculated each time the tracking is started and
        # subracted from future inputs during current tracking to compensate for
        # linear misalignment between the input and relaxed_ik coordinate
        # systems.
        self.input_relaxed_ik_difference = {
            'gcs': np.array([0.0, 0.0, 0.0]),
            'rikcs': np.array([0.0, 0.0, 0.0]),
        }

        # Private variables:

        rospy.init_node(f'{self.ROBOT_NAME}_positional_control', anonymous=True)
        rospy.on_shutdown(self.__node_shutdown)

        # Service provider:

        # Service subscriber:
        self.__pid_velocity_limit = rospy.ServiceProxy(
            'pid_vel_limit',
            PidVelocityLimit,
        )

        self.__stop_arm = rospy.ServiceProxy(
            f'{self.ROBOT_NAME}/base/stop',
            Stop,
        )
        self.__estop_arm = rospy.ServiceProxy(
            f'{self.ROBOT_NAME}/base/apply_emergency_stop',
            ApplyEmergencyStop,
        )
        self.__clear_arm_faults = rospy.ServiceProxy(
            f'{self.ROBOT_NAME}/base/clear_faults',
            Base_ClearFaults,
        )

        # Topic publisher:

        # Topic subscriber:
        rospy.Subscriber(
            f'{self.ROBOT_NAME}/input/tracking',
            Bool,
            self.__input_tracking_callback,
        )
        rospy.Subscriber(
            f'{self.ROBOT_NAME}/input/pose',
            Pose,
            self.__input_pose_callback,
        )

        rospy.Subscriber(
            'pid/motion_finished',
            Bool,
            pid_motion_finished_callback,
        )
        rospy.Subscriber(
            f'{self.ROBOT_NAME}/base_feedback',
            BaseCyclic_Feedback,
            ee_callback,
        )

    def __input_tracking_callback(self, msg):
        """
        
        """

        self.input_is_tracked = msg.data

        if not self.input_is_tracked:
            self.on_tracking_start = True
            return

        if self.on_tracking_start:
            self.on_tracking_start = False

            # Calculate the compensation for coordinate systems
            # misalignment.
            self.input_relaxed_ik_difference['gcs'] = (
                self.input_pose['gcs']['position'] -
                self.relaxed_ik_pose['gcs']['position']
            )

        # # Publish target position and orientation in relaxed IK Coordinate system (Kinova)
        # def relaxed_ik_pub_target_rikcs(target_position, target_orientation):

        #     # Form a message for relaxedIK (right arm)
        #     pose_r = Pose()
        #     pose_r.position.x = target_position[0]
        #     pose_r.position.y = target_position[1]
        #     pose_r.position.z = target_position[2]

        #     pose_r.orientation.w = target_orientation[0]
        #     pose_r.orientation.x = target_orientation[1]
        #     pose_r.orientation.y = target_orientation[2]
        #     pose_r.orientation.z = target_orientation[3]

        #     # TODO: Form a message for relaxedIK (left arm)
        #     pose_l = Pose()
        #     pose_l.position.x = 0
        #     pose_l.position.y = 0
        #     pose_l.position.z = 0

        #     pose_l.orientation.w = 1
        #     pose_l.orientation.x = 0
        #     pose_l.orientation.y = 0
        #     pose_l.orientation.z = 0

        #     # Form full message for relaxedIK
        #     ee_pose_goals = EEPoseGoals()
        #     ee_pose_goals.ee_poses.append(pose_r)
        #     ee_pose_goals.ee_poses.append(pose_l)
        #     ee_pose_goals.header.seq = 0

        #     # Publish
        #     setpoint_pub.publish(ee_pose_goals)

        # # Publish target position and orientation in World Coordinate system (same as Global CS, but Z contains a chest height)
        # def relaxed_ik_pub_target_wcs(
        #     target_position_wcs, target_orientation_wcs
        # ):
        #     global R_gcs_to_kcs, R_kcs_to_gcs
        #     global relaxed_ik_pos_gcs, chest_pos

        #     # Update relaxed_ik global variable
        #     relaxed_ik_pos_gcs = target_position_wcs.copy()
        #     relaxed_ik_pos_gcs[2] = target_position_wcs[2] - chest_pos / 1000

        #     # Recalculate into relaxed IK CS
        #     relaxed_ik_pos_rikcs = np.matmul(
        #         R_gcs_to_kcs[0:3, 0:3], relaxed_ik_pos_gcs
        #     )

        #     # TODO: orientation
        #     relaxed_ik_pub_target_rikcs(
        #         relaxed_ik_pos_rikcs, target_orientation_wcs
        #     )

        # # Publish last commanded target position and orientation in a World Coordinate System
        # def relaxed_ik_pub_commanded_wcs(
        #     current_position_wcs, current_orientation_wcs
        # ):

        #     # Combine into a single array
        #     msg = Float32MultiArray()
        #     msg.data = current_position_wcs.tolist() + current_orientation_wcs

        #     # Publish
        #     relaxed_ik_target_wcs_pub.publish(msg)

        # # Publish target position and orientation in Global Coordinate system (parallel to the ground: X - forw, Y - left, Z - up)
        # def relaxed_ik_pub_target_gcs(
        #     target_position_gcs, target_orientation_gcs
        # ):
        #     global R_gcs_to_kcs, R_kcs_to_gcs
        #     global relaxed_ik_pos_gcs, chest_pos
        #     global relaxed_ik_boundary

        #     # Update relaxed_ik global variable
        #     relaxed_ik_pos_gcs = target_position_gcs.copy()

        #     # Apply limits
        #     if relaxed_ik_pos_gcs[0] < relaxed_ik_boundary['x_min']:
        #         relaxed_ik_pos_gcs[0] = relaxed_ik_boundary['x_min']

        #         calculate_controller_ee_diff()

        #     elif relaxed_ik_pos_gcs[0] > relaxed_ik_boundary['x_max']:
        #         relaxed_ik_pos_gcs[0] = relaxed_ik_boundary['x_max']

        #         calculate_controller_ee_diff()

        #     if relaxed_ik_pos_gcs[1] < relaxed_ik_boundary['y_min']:
        #         relaxed_ik_pos_gcs[1] = relaxed_ik_boundary['y_min']

        #         calculate_controller_ee_diff()

        #     elif relaxed_ik_pos_gcs[1] > relaxed_ik_boundary['y_max']:
        #         relaxed_ik_pos_gcs[1] = relaxed_ik_boundary['y_max']

        #         calculate_controller_ee_diff()

        #     if relaxed_ik_pos_gcs[2] < relaxed_ik_boundary['z_min']:
        #         relaxed_ik_pos_gcs[2] = relaxed_ik_boundary['z_min']

        #         calculate_controller_ee_diff()

        #     elif relaxed_ik_pos_gcs[2] > relaxed_ik_boundary['z_max']:
        #         relaxed_ik_pos_gcs[2] = relaxed_ik_boundary['z_max']

        #         calculate_controller_ee_diff()

        #     # Recalculate into relaxed IK CS
        #     relaxed_ik_pos_rikcs = np.matmul(
        #         R_gcs_to_kcs[0:3, 0:3], relaxed_ik_pos_gcs
        #     )

        #     # TODO: orientation
        #     relaxed_ik_pub_target_rikcs(
        #         relaxed_ik_pos_rikcs, target_orientation_gcs
        #     )

        # # Publish target position and orientation in relaxed IK Coordinate system (Kinova)
        # def relaxed_ik_pub_target_rikcs(target_position, target_orientation):

        #     # Form a message for relaxedIK (right arm)
        #     pose_r = geom_msgs.Pose()
        #     pose_r.position.x = target_position[0]
        #     pose_r.position.y = target_position[1]
        #     pose_r.position.z = target_position[2]

        #     pose_r.orientation.w = target_orientation[0]
        #     pose_r.orientation.x = target_orientation[1]
        #     pose_r.orientation.y = target_orientation[2]
        #     pose_r.orientation.z = target_orientation[3]

        #     # TODO: Form a message for relaxedIK (left arm)
        #     pose_l = geom_msgs.Pose()
        #     pose_l.position.x = 0
        #     pose_l.position.y = 0
        #     pose_l.position.z = 0

        #     pose_l.orientation.w = 1
        #     pose_l.orientation.x = 0
        #     pose_l.orientation.y = 0
        #     pose_l.orientation.z = 0

        #     # Form full message for relaxedIK
        #     ee_pose_goals = EEPoseGoals()
        #     ee_pose_goals.ee_poses.append(pose_r)
        #     ee_pose_goals.ee_poses.append(pose_l)
        #     ee_pose_goals.header.seq = 0

        #     # Publish
        #     setpoint_pub.publish(ee_pose_goals)

        # def kinova_mapping():
        #     global input_pos_gcs, relaxed_ik_pos_gcs
        #     global oculus_kinova_pos_diff_gcs, relaxedik_kinova_pos_diff_kcs
        #     global onTrackingStart, isInitialized
        #     global isTracking

        #     if isInitialized:

        #         # Tracking has started
        #         if isTracking == True:

        #             # On first press recalculate transition (difference) from oculus to
        #             if onTrackingStart['right_arm'] == True:
        #                 calculate_controller_ee_diff()

        #                 # Remove the flag
        #                 onTrackingStart['right_arm'] = False

        #             # Compesantion for controller and GCS misalignment
        #             relaxed_ik_pos_gcs = input_pos_gcs - oculus_kinova_pos_diff_gcs

        #             relaxed_ik_pub_target_gcs(
        #                 relaxed_ik_pos_gcs,
        #                 [1, 0, 0, 0],
        #             )

        #         # If tracking has stopped
        #         else:
        #             # Reset the flag
        #             if CHEST_CONTROL_MODE != 2:
        #                 onTrackingStart['right_arm'] = True

        #             # TODO: stop any robot motion
