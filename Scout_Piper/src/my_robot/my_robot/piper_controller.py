import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class PiperController(Node):

    def __init__(self):
        super().__init__('piper_controller')

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for Piper controller...')

        self.action_client.wait_for_server()

        self.get_logger().info('Piper controller connected!')

        self.run_test()

    def move_arm(self, positions, duration):
        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration

        goal.trajectory.points.append(point)

        self.get_logger().info(
            f'Moving Piper to: {positions}'
        )

        future = self.action_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info('Movement complete!')

    def run_test(self):

        # Move to a clearly visible position
        self.move_arm(
            [1.5, 0.8, -1.0, 1.0, 0.8, 1.0],
            4
        )

        time.sleep(2)

        # Return to approximately the starting position
        self.move_arm(
            [0.0, 0.01, -0.01, 0.0, 0.0, 0.0],
            4
        )

        self.get_logger().info('Piper test complete!')


def main():
    rclpy.init()

    node = PiperController()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
