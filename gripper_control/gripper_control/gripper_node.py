import logging
from time import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import String
from gripper_msgs.action import Command

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
                logging.error(f"Dynamixel#{error.servo.motor_id}: {error_message}")
                return False
        return wrapper
    return decorator

def timing(function):
    @wraps(function)
    def wrap(*args, **kw):
        t_start = time()
        result = function(*args, **kw)
        t_end = time()
        logging.error('func:%r args:[%r, %r] took: %2.4f ms' % \
          (function.__name__, args, kw, (t_end-t_start)*1000))
        return result
    return wrap

class GripperNode(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.declare_parameter('gripper_config')
        gripper_config = self.get_parameter('gripper_config').get_parameter_value().string_value
        
        self.get_logger().info(f"Gripper Config: {gripper_config}")
        gripper_config = pydantic.parse_file_as(path=gripper_config,  type_=GripperConfig)

        self.gripper = Gripper(gripper_config)

        self._action_server = ActionServer(
            self,
            Command,
            f"gripper_{0}",
            self.execute_callback)

    @timing
    @exception_handler("Fault during step")
    def step(self):
        self.gripper.step()

    @exception_handler("Fault during stop")
    def stop(self, goal_handle):
        self.get_logger().info('Stopping')
        
        feedback_msg = Command.Feedback()
        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)
        
        self.gripper.stop_moving()

        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

    @exception_handler("Fault during home")
    def home(self, goal_handle):
        self.get_logger().info('Homing')
        feedback_msg = Command.Feedback()
        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)
        
        self.gripper.home()

        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

    @exception_handler("Fault during move position")
    def move_position(self, goal_handle):
        self.get_logger().info('Moving Position')
        
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
        self.get_logger().info('Moving Velocity')
        
        feedback_msg = Command.Feedback()
        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

        timeout = goal_handle.request.timeout
        action  = goal_handle.request.action

        self.gripper.move_velocity(action, set_only=False)

        feedback_msg.status = Command.Result.OK
        goal_handle.publish_feedback(feedback_msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

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
    
def main(args=None):
    rclpy.init(args=args)

    gripper_node = GripperNode()

    while rclpy.ok():
        rclpy.spin_once(gripper_node, timeout_sec=0.1)
        gripper_node.step()

    gripper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
