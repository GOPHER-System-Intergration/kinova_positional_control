#!/usr/bin/env python
"""

"""

import rospy
import math

from std_msgs.msg import (Float64, Bool)
from sensor_msgs.msg import JointState

from kortex_driver.msg import (
    Base_JointSpeeds,
    JointSpeed,
    JointAngles,
)
from kortex_driver.srv import Stop
from kinova_positional_control.srv import PidVelocityLimit


class KinovaJointsControl:
    """
    
    """

    def __init__(
        self,
        name='my_gen3',
        continous_joints_indices=(0, 2, 4, 6),
        max_speeds=[1.396, 1.396, 1.396, 1.396, 1.222, 1.222, 1.222],
    ):
        """
        
        """

        # Public constants
        self.ROBOT_NAME = name
        self.MAX_SPEEDS = max_speeds  # Rad/s
        self.CONTINOUS_JOINTS_INDICES = continous_joints_indices
        self.JOINTS_NUMBER = 7

        # Private constants

        # Public variables
        self.is_initialized = False
        self.velocity_fraction_limit = 1.0
        self.motion_finished_threshold = 0.01

        # Private variables
        # Absolute joint positions at the moment of setting a new goal. Relative
        # current joint positions are relative to these positions
        self.__start_absolute_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Absolute current (feedback) joint positions.
        self.__current_absolute_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # These velocities are the output of PIDs, which will be sent to Kinova
        # joint velocity topic.
        self.__goal_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        rospy.init_node(f'{self.ROBOT_NAME}_joints_control', anonymous=True)
        rospy.on_shutdown(self.__node_shutdown)

        # Service provider
        rospy.Service(
            'pid_vel_limit',
            PidVelocityLimit,
            self.__pid_velocity_limit_handler,
        )

        # Service subscriber
        self.__stop_arm_srv = rospy.ServiceProxy(
            'my_gen3/base/stop',
            Stop,
        )

        # Topic publisher
        self.__joint_velocity = rospy.Publisher(
            f'{self.ROBOT_NAME}/in/joint_velocity',
            Base_JointSpeeds,
            queue_size=1,
        )

        self.__pid_motion_finished = rospy.Publisher(
            'pid/motion_finished',
            Bool,
            queue_size=1,
        )

        self.__state_1 = rospy.Publisher(
            'joint_1/state',
            Float64,
            queue_size=1,
        )
        self.__state_2 = rospy.Publisher(
            'joint_2/state',
            Float64,
            queue_size=1,
        )
        self.__state_3 = rospy.Publisher(
            'joint_3/state',
            Float64,
            queue_size=1,
        )
        self.__state_4 = rospy.Publisher(
            'joint_4/state',
            Float64,
            queue_size=1,
        )
        self.__state_5 = rospy.Publisher(
            '/joint_5/state',
            Float64,
            queue_size=1,
        )
        self.__state_6 = rospy.Publisher(
            'joint_6/state',
            Float64,
            queue_size=1,
        )
        self.__state_7 = rospy.Publisher(
            'joint_7/state',
            Float64,
            queue_size=1,
        )

        self.__setpoint_1 = rospy.Publisher(
            'joint_1/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_2 = rospy.Publisher(
            'joint_2/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_3 = rospy.Publisher(
            'joint_3/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_4 = rospy.Publisher(
            'joint_4/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_5 = rospy.Publisher(
            'joint_5/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_6 = rospy.Publisher(
            'joint_6/setpoint',
            Float64,
            queue_size=1,
        )
        self.__setpoint_7 = rospy.Publisher(
            'joint_7/setpoint',
            Float64,
            queue_size=1,
        )

        # Topic subscriber
        rospy.Subscriber(
            f'{self.ROBOT_NAME}/base_feedback/joint_state',
            JointState,
            self.__absolute_feedback_callback,
        )

        rospy.Subscriber(
            'relaxed_ik/joint_angle_solutions',
            JointAngles,
            self.__absolute_setpoint_callback,
        )

        rospy.Subscriber(
            'joint_1/control_effort',
            Float64,
            self.__control_effort_1_callback,
        )
        rospy.Subscriber(
            'joint_2/control_effort',
            Float64,
            self.__control_effort_2_callback,
        )
        rospy.Subscriber(
            'joint_3/control_effort',
            Float64,
            self.__control_effort_3_callback,
        )
        rospy.Subscriber(
            'joint_4/control_effort',
            Float64,
            self.__control_effort_4_callback,
        )
        rospy.Subscriber(
            'joint_5/control_effort',
            Float64,
            self.__control_effort_5_callback,
        )
        rospy.Subscriber(
            'joint_6/control_effort',
            Float64,
            self.__control_effort_6_callback,
        )
        rospy.Subscriber(
            'joint_7/control_effort',
            Float64,
            self.__control_effort_7_callback,
        )

    def __absolute_feedback_callback(self, msg):
        """
        
        """

        # Update from zero to the current arm position on initialization.
        if not self.is_initialized:
            self.__start_absolute_positions = msg.position
            self.is_initialized = True

            print('\nKinova joints control is ready.')

            return

        self.__current_absolute_positions = msg.position

        self.__relative_feedback()

    def __relative_feedback(self):
        """
        Recalculates joint feedback to be relative to the starting absolute
        position with start_absolute_positions being a relative origin (zero).
        
        """

        current_relative_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for joint_index in range(self.JOINTS_NUMBER):
            if joint_index not in self.CONTINOUS_JOINTS_INDICES:

                current_relative_positions[joint_index] = (
                    self.__current_absolute_positions[joint_index] -
                    self.__start_absolute_positions[joint_index]
                )

                continue

            # Recalculate joint relative feedback for continuous rotation joints
            # using geodesic distance.
            current_relative_positions[joint_index] = (
                self.geodesic_distance(
                    self.__current_absolute_positions[joint_index],
                    self.__start_absolute_positions[joint_index]
                )
            )

        # BUG: if the joint overshoots at 180 commanded relative position it
        # might get stuck in the infinite 360 loop (keep rotating).

        # Publish relative position feedbacks to PIDs.
        self.__state_1.publish(round(current_relative_positions[0], 4))
        self.__state_2.publish(round(current_relative_positions[1], 4))
        self.__state_3.publish(round(current_relative_positions[2], 4))
        self.__state_4.publish(round(current_relative_positions[3], 4))
        self.__state_5.publish(round(current_relative_positions[4], 4))
        self.__state_6.publish(round(current_relative_positions[5], 4))
        self.__state_7.publish(round(current_relative_positions[6], 4))

    def __absolute_setpoint_callback(self, msg):
        """
        
        """

        goal_absolute_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for joint_index in range(self.JOINTS_NUMBER):
            goal_absolute_positions[joint_index] = (
                msg.joint_angles[joint_index].value
            )

        self.__relative_setpoint(goal_absolute_positions)

    def __relative_setpoint(self, goal_absolute_positions):
        """
        Calculates geodesic (shortest) angular distance and sets the goal
        positions relative to current positions.

        """

        goal_relative_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Set origin positions.
        self.__start_absolute_positions = self.__current_absolute_positions

        for joint_index in range(self.JOINTS_NUMBER):
            if joint_index not in self.CONTINOUS_JOINTS_INDICES:
                goal_relative_positions[joint_index] = (
                    goal_absolute_positions[joint_index] -
                    self.__current_absolute_positions[joint_index]
                )

                continue

            # Recalculate joint relative setpoint for continuous rotation joints
            # using geodesic distance.
            goal_relative_positions[joint_index] = (
                self.geodesic_distance(
                    goal_absolute_positions[joint_index],
                    self.__current_absolute_positions[joint_index]
                )
            )

        # Publish relative position setpoints to PIDs.
        self.__setpoint_1.publish(round(goal_relative_positions[0], 4))
        self.__setpoint_2.publish(round(goal_relative_positions[1], 4))
        self.__setpoint_3.publish(round(goal_relative_positions[2], 4))
        self.__setpoint_4.publish(round(goal_relative_positions[3], 4))
        self.__setpoint_5.publish(round(goal_relative_positions[4], 4))
        self.__setpoint_6.publish(round(goal_relative_positions[5], 4))
        self.__setpoint_7.publish(round(goal_relative_positions[6], 4))

    # Update joint velocities.
    def __control_effort_1_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.is_initialized:
            self.__goal_velocities[0] = msg.data

    def __control_effort_2_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.is_initialized:
            self.__goal_velocities[1] = msg.data

    def __control_effort_3_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.is_initialized:
            self.__goal_velocities[2] = msg.data

    def __control_effort_4_callback(self, msg):

        # Block callback function until all components are initialized.
        self.__goal_velocities[3] = msg.data

    def __control_effort_5_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.is_initialized:
            self.__goal_velocities[4] = msg.data

    def __control_effort_6_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.is_initialized:
            self.__goal_velocities[5] = msg.data

    def __control_effort_7_callback(self, msg):

        # Block callback function until all components are initialized.
        if self.is_initialized:
            self.__goal_velocities[6] = msg.data

    def __pid_velocity_limit_handler(self, req):
        """
        Sets a velocity limit in fractions (0.1 to 1.0).
        
        """

        # Block callback function until all components are initialized.
        if self.is_initialized:

            if req.data > 1.0:
                self.velocity_fraction_limit = 1.0

            elif req.data < 0.1:
                self.velocity_fraction_limit = 0.1

            else:
                self.velocity_fraction_limit = req.data

            return True

        return False

    def geodesic_distance(self, position1, position2):
        """

        http://motion.pratt.duke.edu/RoboticSystems/3DRotations.html#Geodesic-distance-and-interpolation
        
        """

        geodesic_distance = 0.0

        # Resulting position is within (-180; 180].
        if (-1 * math.pi < position1 - position2 <= math.pi):
            geodesic_distance = (position1 - position2)

        # Resulting position is crossing -180/180 border from the negative sign
        # to the positive sign (Example: -20: -170 to 170).
        elif (position1 - position2 > math.pi):
            geodesic_distance = (position1 - position2 - 2 * math.pi)

        # Resulting position is crossing 180/-180 border from the positive sign
        # to the negative sign (Example: +20: 170 to -170).
        elif (position1 - position2 <= -1 * math.pi):
            geodesic_distance = (position1 - position2 + 2 * math.pi)

        return geodesic_distance

    def publish_joint_motion_finished(self):
        """
        Checks if all joints have an absolute velocity value lower than a
        threshold.
        
        """

        motion_finished = True

        for velocity in self.__goal_velocities:
            if abs(velocity) > self.motion_finished_threshold:
                motion_finished = False
                break

        self.__pid_motion_finished.publish(motion_finished)

    def publish_goal_velocities(self):
        """
        
        """

        # Block function until all components are initialized.
        if self.is_initialized:

            # Form a velocity message.
            velocity_message = Base_JointSpeeds()
            velocities = []

            for joint_index in range(self.JOINTS_NUMBER):
                joint_velocity = JointSpeed()
                joint_velocity.joint_identifier = joint_index

                # Limit the output velocity.
                if (
                    abs(self.__goal_velocities[joint_index]) >
                    self.MAX_SPEEDS[joint_index] * self.velocity_fraction_limit
                ):

                    if self.__goal_velocities[joint_index] >= 0:
                        self.__goal_velocities[joint_index] = (
                            self.MAX_SPEEDS[joint_index] *
                            self.velocity_fraction_limit
                        )

                    else:
                        self.__goal_velocities[joint_index] = (
                            -1 * self.MAX_SPEEDS[joint_index] *
                            self.velocity_fraction_limit
                        )

                joint_velocity.value = self.__goal_velocities[joint_index]
                joint_velocity.duration = 0  # Or 0.000333s
                velocities.append(joint_velocity)

            velocity_message.joint_speeds = velocities
            velocity_message.duration = 0

            # Publish a velocity message.
            self.__joint_velocity.publish(velocity_message)

    def __node_shutdown(self):
        """
        
        """

        print('\nNode is shutting down...')

        # Stop arm movement
        self.__stop_arm_srv()


def main():
    """
    
    """

    right_arm = KinovaJointsControl()

    while not rospy.is_shutdown():
        right_arm.publish_joint_motion_finished()
        right_arm.publish_goal_velocities()


if __name__ == '__main__':
    main()
