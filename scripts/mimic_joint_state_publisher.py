#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

'''
This node listens to the joint states of a source joint and publishes mimic joints based on the source joint's state.
For example, it can be used to mimic the state of a Robotiq 85 gripper's knuckle joint to its inner knuckle and finger tip joints.
Due to the nature of mimic joints, rviz and MoveIt, exposing the hardware state of the mimic joints via URDF in ros2_control tag is not possible.
This is a workaround for the gripper's joint state for mimic joints not being published by the URDF or MoveIt configuration.

Will need to decide if this is the best way to handle mimic joints in the future, or just make use of JointGroupPositionController instead of GripperActionController.

Note: This does not work. I have no idea why, but it does not publish the mimic joints correctly.
Attempting to fix through another approach (JointGroupPositionController).
'''
class MimicJointStatePublisher(Node):
    def __init__(self):
        super().__init__('mimic_joint_state_publisher')

        self.source_joint = 'robotiq_85_left_knuckle_joint'
        self.mimic_joint1 = 'robotiq_85_right_knuckle_joint'
        self.mimic_joint2 = 'robotiq_85_left_inner_knuckle_joint'
        self.mimic_joint3 = 'robotiq_85_right_inner_knuckle_joint'
        self.mimic_joint4 = 'robotiq_85_left_finger_tip_joint'
        self.mimic_joint5 = 'robotiq_85_right_finger_tip_joint'

        self.multiplier = -1 # for mimic_joint4 and mimic_joint5
        self.offset = 0 # for mimic_joint2 and mimic_joint3

        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.callback, 10)

    def callback(self, msg):
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.name = msg.name.copy()
        new_msg.position = list(msg.position)
        new_msg.velocity = list(msg.velocity)
        new_msg.effort = list(msg.effort)

        idx = msg.name.index(self.source_joint)

        # mimic_joint1
        mimic_pos1 = msg.position[idx]
        mimic_vel1 = msg.velocity[idx]
        mimic_effort1 = msg.effort[idx]
        new_msg.name.append(self.mimic_joint1)
        new_msg.position.append(mimic_pos1)
        new_msg.velocity.append(mimic_vel1)
        new_msg.effort.append(mimic_effort1)


        # mimic_joint2 and mimic_joint3
        mimic_pos2_3 = msg.position[idx] + self.offset
        mimic_vel2_3 = msg.velocity[idx]
        mimic_effort2_3 = msg.effort[idx]
        new_msg.name.append(self.mimic_joint2)
        new_msg.position.append(mimic_pos2_3)
        new_msg.velocity.append(mimic_vel2_3)
        new_msg.effort.append(mimic_effort2_3)
        new_msg.name.append(self.mimic_joint3)
        new_msg.position.append(mimic_pos2_3)
        new_msg.velocity.append(mimic_vel2_3)
        new_msg.effort.append(mimic_effort2_3)

        # mimic_joint4 and mimic_joint5
        mimic_pos4_5 = msg.position[idx] * self.multiplier
        mimic_vel4_5 = msg.velocity[idx] * self.multiplier
        mimic_effort4_5 = msg.effort[idx] * self.multiplier
        new_msg.name.append(self.mimic_joint4)
        new_msg.position.append(mimic_pos4_5)
        new_msg.velocity.append(mimic_vel4_5)
        new_msg.effort.append(mimic_effort4_5)
        new_msg.name.append(self.mimic_joint5)
        new_msg.position.append(mimic_pos4_5)
        new_msg.velocity.append(mimic_vel4_5)
        new_msg.effort.append(mimic_effort4_5)

        self.get_logger().info(f'Publishing mimic joint states: {new_msg.name[-5:]}')
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)

    mimic_joint_state_publisher = MimicJointStatePublisher()

    rclpy.spin(mimic_joint_state_publisher)

        

        