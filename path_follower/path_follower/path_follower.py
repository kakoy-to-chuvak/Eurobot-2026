import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import math
import json
import time
from collections import deque


# Helpers
def q_yaw(yaw: float) -> Quaternion:
    """Quaternion из Yaw (рад)"""
    q = Quaternion()
    q.z = math.sin(yaw / 2)
    q.w = math.cos(yaw / 2)
    return q

class Parameters:
    def __init__(self, node: Node):
        self.__node__ = node

        self.file = self.create_parameter("file", "")



    def create_parameter(self, name: str, value):
        self.__node__.declare_parameter(name, value)
        return self.__node__.get_parameter(name).value
    

class ActionType:
    def __init__(self, parameters: Parameters) -> None: ...
    def set_target(self, parameters: dict) -> bool: ...
    def get_progress(self) -> float: ...
    def is_complete(self) -> bool: ...

class Palugin:
    def __init__(self, action_type: ActionType, parameters: Parameters):
        self.action_type = action_type
        
        self.task_queue = deque(maxlen=100)
        self.current_task = None

    def check_status(self) -> float:
        pass

    def add_task(self, parameters: dict) -> bool:
        self.task_queue.append(parameters)

    def clear_tasks(self):
        self.task_queue.clear()


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_folower')

        self.parameters = Parameters(self)

        self.tasks: list[deque] = []
        
    

    




def main(args=None):
    rclpy.init(args=args)

    node = PathFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown...")

    finally:
        try:
            node.destroy_node()
        except:
            pass



if __name__ == "__main__":
    main()