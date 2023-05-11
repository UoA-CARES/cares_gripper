import logging
logging.basicConfig(level=logging.INFO)

from time import time, sleep

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import String
from gripper_msgs.action import Command
from gripper_msgs.msg import State

from functools import wraps

import pydantic
from argparse import ArgumentParser

from gripper_control.configurations import GripperConfig
from gripper_control.Gripper import Gripper, GripperError

def exception_handler(error_message):
    def decorator(function):
        @wraps(function)
        def wrapper(self, *args, **kwargs):
            try:
                function(self, *args, **kwargs)
                return True
            except GripperError as error:
                logging.error(f"Gripper: {error_message}")
                return False
        return wrapper
    return decorator

#This is here for debugging purposes to remove once satisified
def timing(function):
    @wraps(function)
    def wrap(*args, **kw):
        t_start = time()
        result = function(*args, **kw)
        t_end = time()
        logging.info('func:%r args:[%r, %r] took: %2.4f ms' % \
          (function.__name__, args, kw, (t_end-t_start)*1000))
        return result
    return wrap

class GripperNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.declare_parameter('gripper_config')
        gripper_config = self.get_parameter('gripper_config').get_parameter_value().string_value
        
        # self.get_logger().info(f"Gripper Config: {gripper_config}")
        logging.info(f"Gripper Config: {gripper_config}")
        gripper_config = pydantic.parse_file_as(path=gripper_config,  type_=GripperConfig)

        self.gripper = Gripper(gripper_config)

        self.state_publisher = self.create_publisher(State, f"gripper_{gripper_config.gripper_id}/state", 1)

        self._action_server = ActionServer(
            self,
            Command,
            f"gripper_{gripper_config.gripper_id}",
            self.execute_callback)

    # @timing
    @exception_handler("Fault during step")
    def step(self):
        state = self.gripper.step()
        # logging.info(f"{state}")
        state_msg = State()
        state_msg.positions  = state["positions"]
        state_msg.velocities = state["velocities"]
        state_msg.loads      = state["loads"]
        self.state_publisher.publish(state_msg)

    @exception_handler("Fault during stop")
    def stop(self, goal_handle):
        # self.get_logger().info('Stopping')
        logging.info('Stopping')
        
        feedback_msg = Command.Feedback()
        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)
        
        self.gripper.stop_moving()

        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

    @exception_handler("Fault during home")
    def home(self, goal_handle):
        # self.get_logger().info('Homing')
        logging.info('Homing')
        feedback_msg = Command.Feedback()
        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)
        
        self.gripper.home()

        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

    @exception_handler("Fault during move position")
    def move_position(self, goal_handle):
        # self.get_logger().info('Moving Position')
        logging.info('Moving Position')
        
        feedback_msg = Command.Feedback()
        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

        timeout = goal_handle.request.timeout
        action  = goal_handle.request.action

        self.gripper.move(action, timeout=timeout)

        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

    @exception_handler("Fault during move velocity")
    def move_velocity(self, goal_handle):
        # self.get_logger().info('Moving Velocity')
        logging.info('Moving Velocity')
        
        feedback_msg = Command.Feedback()
        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

        timeout = goal_handle.request.timeout
        action  = goal_handle.request.action

        self.gripper.move_velocity(action, set_only=False)
        sleep(0.2)

        feedback_msg.status = Command.Result.OK        
        goal_handle.publish_feedback(feedback_msg)

    def execute_callback(self, goal_handle):
        # self.get_logger().info('Executing goal...')
        logging.info('Executing goal...')

        goal_handle.succeed()

        success = True
        command = goal_handle.request.command
        if command == Command.Goal.STOP:
            success = self.stop(goal_handle)
        elif command == Command.Goal.HOME:
            success = self.home(goal_handle)
        elif command == Command.Goal.MOVE_POSITION:
            success = self.move_position(goal_handle)
        elif command == Command.Goal.MOVE_VELOCITY:
            success = self.move_velocity(goal_handle)

        result = Command.Result()
        result.status = Command.Result.OK
        if not success:
            result.status = Command.Result.ERROR
            return result
        
        state = self.gripper.state()
        result.positions   = state["positions"]
        result.velocities  = state["velocities"]
        result.loads       = state["loads"]
        return result
    
def main(args=None):
    rclpy.init(args=args)

    gripper_node = GripperNode()

    while rclpy.ok():
        rclpy.spin_once(gripper_node, timeout_sec=0)
        gripper_node.step()

    gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
