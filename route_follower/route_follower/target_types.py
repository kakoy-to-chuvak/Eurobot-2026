from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import UInt8MultiArray, UInt16

import time


class TargetType:
    def __init__(self, node: Node) -> None: ...
    def get_progress(self) -> float: ...
    def is_complete(self) -> bool: ...
    def set_target(self, parameters: dict) -> bool: ...
    def cancel_target(self) -> bool: ...


class LiftTarget(TargetType):
    def __init__(self, node: Node) -> None:
        self.logger = node.get_logger()
        self.sub = node.create_subscription(UInt16, "/pwb/lift_current_height", self.get_current, 10)
        self.pub = node.create_publisher(UInt16, "/pwb/lift_target_height", 10)
        self.current_h = 0
        self.target_h = None
        self.start_h = 0

    def get_progress(self) -> float:
        if self.target_h is not None:
            if self.start_h != self.target_h:
                return (self.start_h - self.current_h) / (self.start_h - self.target_h)
            return 1.0
        return -1.0
    
    def is_complete(self) -> bool:
        if self.target_h is not None:
            return self.current_h == self.target_h
        return True
    
    def set_target(self, parameters: dict) -> bool:
        if "height" in parameters.keys() and isinstance(parameters['height'], int):
            self.target_h = parameters['height']
            msg = UInt16()
            msg.data = self.target_h
            self.pub.publish(msg)
            time.sleep(0.05)
            return True
        
        self.logger.warn("No parameter 'height' or wrong parameter type")
        return False
    
    def cancel_target(self) -> bool:
        msg = UInt16()
        msg.data = self.current_h
        self.target_h = None
        self.pub.publish(msg)
        time.sleep(0.05)
        return True

    def get_current(self, msg: UInt16):
        self.current_h = msg.data

class SleepTarget(TargetType):
    def __init__(self, node: Node) -> None:
        self.logger = node.get_logger()
        self.start = None
        self.sleep = 0.0

    def get_progress(self) -> float:
        if self.start != None:
            return ( time.perf_counter() - self.start ) / self.sleep 
        return -1.0
    
    def is_complete(self) -> bool:
        if self.start is not None:
            return time.perf_counter() - self.start >= self.sleep
        return True
    
    def set_target(self, parameters) -> bool:
        if 'sleep' in parameters.keys() and isinstance(parameters['sleep'], float):
            self.sleep = parameters['sleep']
            self.start = time.perf_counter()
            return True
        
        self.logger.warn("No parameter 'sleep' or wrong parameter type")
        return False
        
    def cancel_target(self):
        self.start = None
        self.sleep = 0.0
        return True
    
