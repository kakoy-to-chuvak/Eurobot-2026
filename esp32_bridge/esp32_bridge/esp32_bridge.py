import math
import time

try:
    import EspClientApi
except ImportError:
    # Если импорт не удался, возможно файл называется иначе или в другой папке
    try:
        from . import EspClientApi
    except ImportError:
        raise ImportError("Could not import EspClientApi module")

import rclpy
from rclpy.node import Node

# Messages
from std_msgs.msg import UInt8MultiArray, UInt16, String, Bool
from geometry_msgs.msg import Twist, Quaternion, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Services
from std_srvs.srv import Trigger 

# TF
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    TransformStamped,
)

from rclpy.qos  import qos_profile_parameters


    
    
# Helper functions    
def q_yaw(yaw: float) -> Quaternion:
    """Quaternion из Yaw (рад)"""
    q = Quaternion()
    q.z = math.sin(yaw / 2)
    q.w = math.cos(yaw / 2)
    return q

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
    """RPY (рад) → Quaternion"""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    q = Quaternion()
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    q.w = cr * cp * cy + sr * sp * sy
    return q


class Parameters:
    def __init__(self, node: Node):
        self.__node__ = node

        # Параметры сокетов
        self.host = self.create_parameter("host", "127.0.0.1")
        print(self.host)
        self.port = self.create_parameter("port", 8080)
        self.lidar_port = self.create_parameter("lidar_port", 8090)

        self.ping_frequency = self.create_parameter("ping_frequency", 10.0) # Hz (герцы)

        # Имена фреймов
        self.odom_frame  = self.create_parameter("odom_frame", "pwb_odom")
        self.base_frame  = self.create_parameter("base_frame", "base_link")
        self.lidar_frame = self.create_parameter("lidar_frame", "lidar")
        self.lift_frame  = self.create_parameter("lift_frame", "lift")
        self.servo_frames = self.create_parameter("servo_frames", [
            "servo0",
            "servo1",
            "servo2",
            "servo3",
        ])

        # Относительные координаты фреймов
        self.lidar_xyz = self.create_parameter("lidar_xyz", [0.0, 0.0, 0.40])
        self.base_lift_xyz = self.create_parameter("lift_xyz", [0.05, 0.0, 0.0])
        
        node.declare_parameter("servo0_pos", [0.01, -0.075, 0.0])
        node.declare_parameter("servo1_pos", [0.01, -0.025, 0.0])
        node.declare_parameter("servo2_pos", [0.01,  0.025, 0.0])
        node.declare_parameter("servo3_pos", [0.01,  0.075, 0.0])
        
        self.servos_pos = [ node.get_parameter(name + "_pos").value for name in self.servo_frames ]

        
        # Параметры лидара
        self.lidar_shift_deg = math.radians(self.create_parameter("lidar_shift_deg", -15.0)) # сдвиг облака CW
        self.lidar_mirror    = self.create_parameter("lidar_mirror", True)     # зеркально (True/False)
        self.lidar_range_min = self.create_parameter("lidar_range_min", 0.05)
        self.lidar_range_max = self.create_parameter("lidar_range_max", 4.0)

        # Другие параметры
        self.use_lift_tf = self.create_parameter("use_lift_tf", False)
        self.tf_publish_frequency =self.create_parameter("tf_publish_frequency", 10.0)
    
    def create_parameter(self, name: str, value):
        self.__node__.declare_parameter(name, value)
        return self.__node__.get_parameter(name).value

# Listener node
class Esp32_Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        self.parameters = Parameters(self)  
        
        # Объекты клиентов для подключения
        self.esp_client = EspClientApi.EspClient(self.parameters.host, self.parameters.port, logger=self.get_logger())
        self.esp_connected = False
        self.last_esp_ping = 0.0

        self.lidar_client = EspClientApi.LidarClient(self.parameters.host, self.parameters.lidar_port, logger=self.get_logger())
        self.lidar_connected = False
        self.last_lidar_ping = 0.0
        
        # Ping server
        if self.parameters.ping_frequency != 0:
            self.ping_timer    = self.create_timer(1 / self.parameters.ping_frequency, self.ping_esp)
            self.receive_timer = self.create_timer(0.02, self.receive_esp_data)

        if self.parameters.tf_publish_frequency != 0:
            self.tf_timer = self.create_timer(1 / self.parameters.tf_publish_frequency, self.publish_dynamic_tf)
        
        # Ping stats
        self.sended_msgs = 0
        self.received_msgs = 0
        
        # Состояние робота
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        self.xPos = 0.0
        self.yPos = 0.0
        self.theta = 0.0
        
        self.servo_state = (0, 0, 0, 0)
        self.lift_height = 0
        
        self.start = 0
        self.side = 'yellow'
        
        # Полжение скана лидара
        self.lidar_x = 0.0
        self.lidar_y = 0.0
        self.lidar_theta = 0.0     

        
        
        # ROS publishers
        self.odom_publisher = self.create_publisher(Odometry, "/pwb/odom", 10)
        self.lift_h_pub = self.create_publisher(UInt16, '/pwb/lift_current_height', 10)
        self.servo_pos_pub = self.create_publisher(UInt8MultiArray, '/pwb/servos_current_angles', 10)
        self.scan_pub = self.create_publisher(LaserScan, "/pwb/lidar_scan", 10)
        self.side_pub = self.create_publisher(String, "/pwb/side", qos_profile_parameters)
        self.start_pub = self.create_publisher(Bool, "/pwb/start", qos_profile_parameters)
        
        
        # ROS subscriptions
        self.cmd_vel_sub = self.create_subscription(Twist, '/pwb/cmd_vel', self.receive_motors_speed, 10)
        self.lift_h_sub = self.create_subscription(UInt16, '/pwb/lift_target_height', self.receive_lift_height, qos_profile_parameters)
        self.servo_pos_sub = self.create_subscription(UInt8MultiArray, '/pwb/servos_target_angles', self.receive_servo_state, qos_profile_parameters)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.receive_pose, qos_profile_parameters)
        
        # Services
        self.route_client = self.create_client(Trigger, "/start_route_execution")
        
        # TF
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        self.publish_odometry()
        
    def receive_pose(self, pose: PoseWithCovarianceStamped):
        self.xPos = pose.pose.pose.position.x
        self.yPos = pose.pose.pose.position.y
        self.theta = 2 * math.acos(pose.pose.pose.orientation.w)
        if pose.pose.pose.orientation.z < 0:
            self.theta = 2 * math.pi - self.theta
        self.esp_client.set_odometry(self.xPos, self.yPos, self.theta)
        
    def _publish_static_tf(self):
        # Publishing odom frame
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = self.parameters.odom_frame
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = rpy_to_quat(0.0,0.0,0.0)

        
        self.static_br.sendTransform(tf)
        self.get_logger().debug("Published static TF map → odom")
    
    def publish_dynamic_tf(self):
        time_stamp = self.get_clock().now().to_msg()

        # odometry tf
        tf = TransformStamped()
        tf.header.stamp    = time_stamp

        tf.header.frame_id = self.parameters.odom_frame
        tf.child_frame_id  = self.parameters.base_frame
        tf.transform.translation.x = float(self.xPos)
        tf.transform.translation.y = float(self.yPos)
        tf.transform.rotation = q_yaw(self.theta)
        self.tf_br.sendTransform(tf)

        # lidar tf
        xyz = self.parameters.lidar_xyz
        tf.header.frame_id = self.parameters.odom_frame
        tf.child_frame_id = self.parameters.lidar_frame
        tf.transform.translation.x = float(xyz[0] * math.cos(self.lidar_theta) - xyz[1] * math.sin(self.lidar_theta) + self.lidar_x)
        tf.transform.translation.y = float(xyz[0] * math.sin(self.lidar_theta) + xyz[1] * math.cos(self.lidar_theta) + self.lidar_y)
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation = q_yaw(self.parameters.lidar_shift_deg + self.lidar_theta)
        self.tf_br.sendTransform(tf)

        if self.parameters.use_lift_tf:
            # lift tf
            tf.header.frame_id = self.parameters.base_frame
            tf.child_frame_id = self.parameters.lift_frame
            tf.transform.translation.x = float(self.parameters.base_lift_xyz[0])
            tf.transform.translation.y = float(self.parameters.base_lift_xyz[1])
            tf.transform.translation.z = float(self.parameters.base_lift_xyz[2] + self.lift_height / 1000)
            tf.transform.rotation = Quaternion()
            self.tf_br.sendTransform(tf)

            # servo tfs
            tf.header.frame_id = self.parameters.lift_frame
        
            for frame, pos, ang in zip(self.parameters.servo_frames, self.parameters.servos_pos, self.servo_state):
                tf.child_frame_id = frame
                tf.transform.translation.x = float(pos[0])
                tf.transform.translation.y = float(pos[1])
                tf.transform.translation.z = float(pos[2])
                tf.transform.rotation = rpy_to_quat(0, -ang / 180 * math.pi, 0)
                self.tf_br.sendTransform(tf)
     
     
    def receive_motors_speed(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        
        if self.start:
            self.esp_client.set_motors_speed(msg.linear.x, msg.angular.z)
        
    def receive_lift_height(self, msg: UInt16):            
        self.esp_client.set_lift_height(msg.data)
        
    def receive_servo_state(self, msg: UInt8MultiArray):
        self.esp_client.set_servo_state(msg.data)        
        
    def publish_odometry(self):       
        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = self.parameters.odom_frame
        odom.child_frame_id  = self.parameters.base_frame
        odom.pose.pose.position.x  = float(self.xPos)
        odom.pose.pose.position.y  = float(self.yPos)
        odom.pose.pose.orientation = q_yaw(self.theta)
        odom.twist.twist.linear.x  = float(self.linear_x)
        odom.twist.twist.angular.z = float(self.angular_z)
        self.odom_publisher.publish(odom)

    def publish_servo(self):
        servo = UInt8MultiArray()
        servo.data = list(self.servo_state)
        self.servo_pos_pub.publish(servo)            
        
    def publish_lift_h(self):
        lift = UInt16()
        lift.data = int(self.lift_height)
        self.lift_h_pub.publish(lift)       
    
    def ping_esp(self):
        if not self.esp_connected:
            if time.perf_counter() - self.last_esp_ping < 5.0:
                return
            
            self.last_esp_ping = time.perf_counter()
            self.get_logger().info("Connecting to ESP")

            if self.esp_client.connect():
                self.esp_connected = True
                self.esp_client.get_all()  # Запрашиваем состояние после reconnect
                self.get_logger().info("ESP connected")
            else:
                self.get_logger().error(f"Connection failed")

        if not self.esp_client.is_connected():
            self.esp_connected = False
            self.esp_client.disconnect()
            self.get_logger().error("Esp disconnected")

        self.esp_client.get_all()
        self.last_esp_ping = time.perf_counter()        

    def publish_lidar(self, angles: list, ranges: list, intens: list):        
        if len(angles) < 10 or len(ranges) < 10 or len(intens) < 10:
            return None
        
        if self.parameters.lidar_mirror:
            angles.reverse()
            ranges.reverse()
            intens.reverse()
            angles = [-a for a in angles]
            
        # нормализуем, чтобы начало ≈ −π
        if angles[0] > 180.0:
            angles = [ang - 360.0 for ang in angles]
            
        radians = list(map(math.radians, angles))
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.parameters.lidar_frame
        scan.angle_min = float(radians[0])
        scan.angle_max = float(radians[-1])
        scan.angle_increment = (radians[-1] - radians[0]) / (len(radians) - 1)
        scan.range_min = self.parameters.lidar_range_min
        scan.range_max = self.parameters.lidar_range_max
        scan.ranges = ranges
        scan.intensities = intens
        self.scan_pub.publish(scan)
        
    def receive_esp_data(self):
        msg = self.esp_client.receive_msg()
        
        while msg:
            self.last_esp_ping = time.perf_counter()
            match msg['event']:
                case 'ANSWER_GET_MOTORS_SPEED':
                    self.linear_x = msg['linear']
                    self.angular_z = msg['angular']
                    self.publish_odometry()
                    
                case 'ANSWER_GET_LIFT_HEIGHT':
                    self.lift_height = msg['height']
                    self.publish_lift_h()
                    
                case 'ANSWER_GET_SERVO_STATE':
                    self.servo_state = msg['state']
                    self.publish_servo()
                    
                case 'ANSWER_GET_ODOMETRY':
                    self.xPos = msg['x']
                    self.yPos = msg['y']
                    self.theta = msg['theta']
                    self.publish_odometry()
                    
                case 'ANSWER_GET_ALL':
                    self.linear_x  = msg["motors_speed"]['linear']
                    self.angular_z = msg["motors_speed"]['angular']
                    
                    self.lift_height = msg["lift_height"]
                    
                    self.servo_state = msg['servo_state']
                    
                    self.xPos  = msg["odometry"]['x']
                    self.yPos  = msg["odometry"]['y']
                    self.theta = msg["odometry"]['theta']

                    # print(f"{time.perf_counter():.7} | {self.xPos} {self.yPos} {self.theta}")

                    # Publishing messages
                    self.publish_odometry()
                    self.publish_servo()
                    self.publish_lift_h()
                case 'SEND_SIDE':
                    self.side = msg['side']
                    side_msg = String()
                    side_msg.data = self.side
                    self.side_pub.publish(side_msg)
                    self.get_logger().info(f"Side: {self.side}")
                    
                case 'SEND_START':
                    self.start = msg['start']
                    start_msg = Bool()
                    start_msg.data = self.start
                    self.start_pub.publish(start_msg)
                    self.get_logger().info(f"Start: {self.start}")
                    if not self.start:
                        self.esp_client.set_motors_speed(0.0, 0.0)
                    else:
                        self.route_client.call_async(Trigger.Request())
                    
                case _:
                    self.get_logger().warn("Undefine event type:" + msg['event'])
   
            msg = self.esp_client.receive_msg()

        # Receive lidar data
        if self.lidar_connected:
            msg = self.lidar_client.receive_lidar()
            if msg is not None:
                self.last_lidar_ping = time.perf_counter()
                self.lidar_x = msg['x']
                self.lidar_y = msg['y']
                self.lidar_theta = msg['theta']
                
                self.publish_lidar(msg['angles'], msg['ranges'], msg['intens'])

            if time.perf_counter() - self.last_lidar_ping > 3.0:
                self.get_logger().warn("Lidar server disconnected")
                self.lidar_connected = False
                self.lidar_client.disconnect()
        else:
            if time.perf_counter() - self.last_lidar_ping > 5.0:
                self.get_logger().info("Try connect to lidar server")
                self.last_lidar_ping = time.perf_counter()
                if self.lidar_client.connect():
                    self.lidar_connected = True
                    self.get_logger().info("Lidar connected")
                else:
                    self.get_logger().error(f"Lidar connection faied")


                

def main(args=None):
    rclpy.init(args=args)

    node = Esp32_Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown...")

    finally:
        node.esp_client.disconnect()
        node.lidar_client.disconnect()
        node.destroy_node()

if __name__ == '__main__':
    main()
