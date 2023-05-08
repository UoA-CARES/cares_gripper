import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import String
from gripper_msgs.action import Command

import pydantic
from argparse import ArgumentParser

from gripper_control.configurations import GripperConfig
from gripper_control.Gripper import Gripper

class GripperNode(Node):

    def __init__(self, gripper_config : GripperConfig):
        super().__init__('minimal_subscriber')
        self._action_server = ActionServer(
            self,
            Command,
            f"gripper-{gripper_config.gripper_id}",
            self.execute_callback)
        
        # self.gripper = Gripper(gripper_config)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # feedback_msg = Fibonacci.Feedback()
        # feedback_msg.partial_sequence = [0, 1]

        # for i in range(1, goal_handle.request.order):
        #     feedback_msg.partial_sequence.append(
        #         feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
        #     self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
        #     goal_handle.publish_feedback(feedback_msg)
        #     time.sleep(1)

        # goal_handle.succeed()

        # result = Fibonacci.Result()
        # result.sequence = feedback_msg.partial_sequence
        # return result
        result = Command.Result()
        result.status = 1
        return result
    
    def step(self):
        # self.gripper.step()
        pass

def parse_args():
    parser = ArgumentParser()
    # parser.add_argument("--gripper_config",  type=str)
    return parser.parse_args()

def main(args=None):
    rclpy.init(args=args)

    gripper_config = GripperConfig()
    gripper_node = GripperNode(gripper_config)

    while rclpy.ok():
        rclpy.spin_once(gripper_node, timeout_sec=0.1)
        gripper_node.step()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
