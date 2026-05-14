from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8MultiArray, UInt16
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos  import qos_profile_parameters

import time
import math


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
        self.pub = node.create_publisher(UInt16, "/pwb/lift_target_height", qos_profile_parameters)
        self.current = 0
        self.target = None
        self.start = 0

    def get_progress(self) -> float:
        if self.target is None:
            return -1.0
        if self.start == self.target:
            return 1.0
        return (self.start - self.current) / (self.start - self.target)
    
    def is_complete(self) -> bool:
        if self.target is None:
            return True
        
        return self.current == self.target
    
    def set_target(self, parameters: dict) -> bool:
        if not 'height' in parameters.keys():
            self.logger.warn("Missing parameter 'height'", name="LiftTarget")
            return False
        
        if not isinstance(parameters['height'], int):
            self.logger.warn("Wrong 'height' parameter type", name="LiftTarget")
            return False
        
        self.target = parameters['height']
        self.start = self.current
        msg = UInt16()
        msg.data = self.target
        self.pub.publish(msg)
        self.logger.info(f"Move lift to {self.target} mm", name="LiftTarget")
        time.sleep(0.05)
        return True
    
    def cancel_target(self) -> bool:
        msg = UInt16()
        msg.data = self.current
        self.target = None
        self.pub.publish(msg)
        self.logger.info(f"Canceling target", name="LiftTarget")
        time.sleep(0.05)
        return True

    def get_current(self, msg: UInt16):
        self.current = msg.data

class ServoTarget(TargetType):
    def __init__(self, node: Node) -> None:
        self.logger = node.get_logger()
        self.sub = node.create_subscription(UInt8MultiArray, "/pwb/servos_current_angles", self.get_current, 10)
        self.pub = node.create_publisher(UInt8MultiArray, "/pwb/servos_target_angles", qos_profile_parameters)
        self.current = [0, 0, 0, 0]
        self.target = None
        self.start = [0, 0, 0, 0]

    def get_progress(self) -> float:
        if self.target is None:
            return -1.0
        
        progresses = [ (start - cur) / (start - tgt) for start, cur, tgt in zip(self.start, self.current, self.target) if  start != tgt ]

        if len(progresses) == 0:
            return 1.0
        
        return sum(progresses) / len(progresses)
    
    def is_complete(self) -> bool:
        if self.target is None:
            return True

        for cur, tgt in zip(self.current, self.target):
            if cur != tgt:
                return False
        
        return True

    def set_target(self, parameters: dict) -> bool:
        if not 'angles' in parameters.keys():
            self.logger.warn("Missing parameter 'angles'", name="ServoTarget")
            return False

        angles = parameters['angles']
        if not isinstance(angles, list):
            self.logger.warn("Wrong parameter 'angles' type", name="ServoTarget")
            return False

        if not len(angles) == 4:
            self.logger.warn(f"Wrong servo count: {len(angles)}", name="ServoTarget")
            return False

        for i, ang in enumerate(angles):
            try:
                angles[i] = int(ang)
            except:
                self.logger.warn("Wrong angle type", name="ServoTarget")
                return False
        
        self.target = angles.copy()
        self.start = self.current.copy()
        msg = UInt8MultiArray()
        msg.data = angles
        self.pub.publish(msg)
        self.logger.info(f"Move servo to {angles}", name="ServoTarget")
        time.sleep(0.05)
        return True
    
    def cancel_target(self) -> bool:
        self.target = None
        msg = UInt8MultiArray()
        msg.data = self.current
        self.pub.publish(msg)
        self.logger.info(f"Canceling target", name="ServoTarget")
        time.sleep(0.05)
        return True


    def get_current(self, msg: UInt8MultiArray):
        self.current = msg.data.tolist()

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
            self.logger.info(f"Sleep {parameters['sleep']} seconds", name="SleepTarget")
            self.sleep = parameters['sleep']
            self.start = time.perf_counter()
            return True
        
        self.logger.warn("No parameter 'sleep' or wrong parameter type", name="SleepTarget")
        return False
        
    def cancel_target(self):
        self.logger.info(f"Canceling sleep target", name="SleepTarget")
        self.start = None
        self.sleep = 0.0
        return True
    
class NavTarget(TargetType):
    def __init__(self, node: Node) -> None:
        self.logger = node.get_logger()
        self.node = node
        self._goal_handle = None
        self._result_future = None
        self._current_goal = None
        self.completed = True
        self._distance_remaining = 0.0
        self._navigation_time = 0.0
        self._goal_status = None  # 0=unknown, 1=accepted, 2=canceled, 3=rejected, 4=succeeded, 5=failed
        self._start_dist = None
        
        self.logger.info("Creating action client for NavigateToPose...", name="NavTarget")
        self._action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.logger.info("Action client created", name="NavTarget")
    
    def _feedback_callback(self, feedback_msg):
        """Обработчик обратной связи от action сервера."""
        feedback = feedback_msg.feedback
        self._distance_remaining = feedback.distance_remaining
        self._navigation_time = feedback.navigation_time
    
    def get_progress(self) -> float:
        if self.completed:
            return 1.0
        
        if self._current_goal is None:
            return -1.0
        
        if self._start_dist is None or self._start_dist == 0.0:
            self._start_dist = self._distance_remaining
        
        if self._start_dist != 0.0:
            return 1.0 - self._distance_remaining / self._start_dist
        
        return 0.0
    
    def is_complete(self) -> bool:
        if self._current_goal is None:
            return True
        if self.completed:
            return True
        
        # Проверяем статус через _goal_status
        if self._goal_status == 4:  # SUCCEEDED
            self.logger.info("Navigation goal succeeded", name="NavTarget")
            self.completed = True
            return True
        elif self._goal_status == 5:  # FAILED
            self.logger.error("Navigation goal failed", name="NavTarget")
            self.completed = True
            return True
        elif self._goal_status == 2:  # CANCELED
            self.logger.warn("Navigation goal was canceled", name="NavTarget")
            self.completed = True
            return True
        
        # Проверяем, завершена ли задача через _result_future
        if self._result_future is not None and self._result_future.done():
            try:
                result = self._result_future.result()
                if result is not None:
                    status = result.status
                    if status == 4:  # STATUS_SUCCEEDED
                        self.logger.info("Navigation goal succeeded", name="NavTarget")
                    elif status == 5:  # STATUS_FAILED
                        self.logger.error("Navigation goal failed", name="NavTarget")
                    elif status == 2:  # STATUS_CANCELED
                        self.logger.warn("Navigation goal was canceled", name="NavTarget")
                    self.completed = True
                    return True
            except Exception as e:
                self.logger.error(f"Error getting result: {e}", name="NavTarget")
                self.completed = True
                return True
        
        return False
    
    def _goal_response_callback(self, future):
        """Колбэк после отправки цели."""
        try:
            self._goal_handle = future.result()
            if not self._goal_handle.accepted:
                self.logger.error("Goal was rejected by server", name="NavTarget")
                self._goal_status = 3  # REJECTED
                self.completed = True
                return
            
            self.logger.info("Goal accepted by server", name="NavTarget")
            self._goal_status = 1  # ACCEPTED
            
            # Запрашиваем результат с колбэком
            self._result_future = self._goal_handle.get_result_async()
            self._result_future.add_done_callback(self._result_callback)
            
        except Exception as e:
            self.logger.error(f"Failed to send goal: {e}", name="NavTarget")
            self.completed = True
    
    def _result_callback(self, future):
        """Колбэк получения результата."""
        try:
            result = future.result()
            if result is not None:
                status = result.status
                self._goal_status = status
                self.logger.info(f"Goal result status: {status}", name="NavTarget")
        except Exception as e:
            self.logger.error(f"Error in result callback: {e}", name="NavTarget")
    
    def set_target(self, parameters: dict) -> bool:
        if 'x' not in parameters or 'y' not in parameters:
            self.logger.warn("Missing 'x' or 'y' parameter for NavTarget", name="NavTarget")
            return False
        
        # Проверка типов
        try:
            x = float(parameters['x'])
            y = float(parameters['y'])
        except (ValueError, TypeError):
            self.logger.warn("'x' and 'y' must be numbers", name="NavTarget")
            return False
        
        # Ждём сервер
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error("NavigateToPose action server not available", name="NavTarget")
            return False
        
        # Создаём цель
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Установка ориентации (угол в радианах -> кватернион)
        theta = parameters.get('theta', 0.0)
        try:
            theta = float(theta)
        except (ValueError, TypeError):
            theta = 0.0
        
        goal_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_pose.pose.orientation.w = math.cos(theta / 2.0)
        
        self._current_goal = goal_pose
        self.completed = False
        self._distance_remaining = 0.0
        self._navigation_time = 0.0
        self._start_dist = None
        self._goal_status = 0  # UNKNOWN
        
        # Отправляем цель
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.logger.info(f"Setting navigation target: x={x}, y={y}, theta={theta} rad", name="NavTarget")
        
        # Отправляем цель асинхронно с колбэком
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def cancel_target(self) -> bool:
        if not self.completed and self._goal_handle is not None:
            self.logger.info("Canceling navigation goal", name="NavTarget")
            if hasattr(self._goal_handle, 'cancel_goal_async'):
                cancel_future = self._goal_handle.cancel_goal_async()
                self._start_dist = None
                # Не ждём, чтобы не блокировать
                cancel_future.add_done_callback(lambda f: self.logger.info("Cancel request sent"), name="NavTarget")
            self.completed = True
        return True