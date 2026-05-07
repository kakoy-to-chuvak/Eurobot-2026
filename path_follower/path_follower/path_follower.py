import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose, Wait_SendGoal_Response

import math
import json


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
    

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_folower')

        self.parameters = Parameters(self)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.action_client.wait_for_server()

        f = open(self.parameters.file, "r")
        self.path = json.loads(f.read())
        f.close()

        self.current_point = 0

        self.next_point()
    
    def send_goal(self, x, y, theta):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = q_yaw(theta)
        
        send_goal_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        # self.get_logger().info(
        #     f'Осталось: {fb.distance_remaining:.2f} м, '
        #     f'времени прошло: {fb.navigation_time:.2f} с'
        # )
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Цель отклонена')
            self.destroy_node()
            return
        
        self.get_logger().info('Цель принята')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = result.error_code
        if status == 0:
            self.get_logger().info('Навигация завершена успешно')
            self.next_point()
        else:
            self.get_logger().warn(f'Ошибка навигации: код {status}')
            self.destroy_node()

    def next_point(self):
        if self.current_point < len(self.path):
            point = self.path[self.current_point]
            self.send_goal(point['x'], point['y'], point['angle'])
            self.current_point += 1
        else:
            self.destroy_node()

    




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