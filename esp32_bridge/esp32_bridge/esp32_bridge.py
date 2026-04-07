import math


try:
    import EspClientApi
except:
    from esp32_bridge import EspClientApi

import rclpy
from rclpy.node import Node

# Messages
from std_msgs.msg import UInt8MultiArray, Float32
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# TF
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    TransformStamped,
)


    
    
    
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
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q




# Listener node
class Esp32_Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Socket parametrs
        self.declare_parameter("host", "127.0.0.1")
        self.host = self.get_parameter("host").value
        
        self.declare_parameter("port", 8080)
        self.port = self.get_parameter("port").value
        
        # Connecting to server
        self.client = EspClientApi.EspClient(self.host, self.port, self.get_logger().info)
        self.client.connect(self.client.password)
        
        # Ping server
        self.ping_timer = self.create_timer(0.1, self.ping_esp)
        self.receive_timer = self.create_timer(0.05, self.receive_esp_data)
        
        # Ping stats
        self.sended_msgs = 0
        self.received_msgs = 0
        
        
        # Robot parametrs
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        self.xPos = 0.0
        self.yPos = 0.0
        self.theta = 0.0
        
        self.servo_state = (0, 0, 0, 0)
        self.lift_height = 0.0

        # Frame names
        self.declare_parameter("odom_frame", "pwb_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("lidar_frame", "lidar")
        self.declare_parameter("lift_frame", "lift")
        self.declare_parameter("servo_frames", [
            "servo0",
            "servo1",
            "servo2",
            "servo3",
        ])
        
        self.servo_frames = self.get_parameter("servo_frames").value
        self.odom_frame  = self.get_parameter("odom_frame").value
        self.base_frame  = self.get_parameter("base_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value
        self.lift_frame  = self.get_parameter("lift_frame").value
        
        

        # Frame relative cords
        self.declare_parameter("lidar_xyz", [0.0, 0.0, 0.40])
        self.declare_parameter("lidar_rpy_deg", [0.0, 0.0, 0.0]) # orientation

        self.declare_parameter("lift_xyz",  [0.05, 0.0, 0.0])
        self.base_lift_xyz = self.get_parameter("lift_xyz").value
        
        self.declare_parameter("servo0_pos", [0.01, -0.075, 0.0])
        self.declare_parameter("servo1_pos", [0.01, -0.025, 0.0])
        self.declare_parameter("servo2_pos", [0.01,  0.025, 0.0])
        self.declare_parameter("servo3_pos", [0.01,  0.075, 0.0])
        
        
        self.servos_pos = [ self.get_parameter(name + "_pos").value for name in self.servo_frames ]
        
        if len(self.servo_frames) != len(self.servos_pos):
            self.get_logger().error("Different count of servo_frames and servo_positions")
            exit(-1)
        
        # Lidar params
        self.declare_parameter("lidar_shift_deg", 0.0) # сдвиг облака CW
        self.declare_parameter("lidar_mirror", True)     # зеркально (True/False)
        self.declare_parameter("lidar_range_min", 0.05)
        self.declare_parameter("lidar_range_max", 4.0)
        
        # ROS publishers
        self.odom_publisher = self.create_publisher(Odometry, "/pwb/odom", 10)
        self.lift_h_pub = self.create_publisher(Float32, '/pwb/lift_current_height', 10)
        self.servo_pos_pub = self.create_publisher(UInt8MultiArray, '/pwb/servos_current_angles', 10)
        self.scan_pub = self.create_publisher(LaserScan, "/pwb/lidar_scan", 10)
        
        
        # ROS subscriptions
        self.cmd_vel_sub = self.create_subscription(Twist, '/pwb/cmd_vel', self.receive_motors_speed, 10)
        self.lift_h_sub = self.create_subscription(Float32, '/pwb/lift_target_height', self.receive_lift_height, 10)
        self.servo_pos_sub = self.create_subscription(UInt8MultiArray, '/pwb/servos_target_angles', self.receive_servo_state, 10)
        
        
        # TF
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        
        
        
    def _publish_static_tf(self):
        # Publishing lidar frame  
        rpy_deg = self.get_parameter("lidar_rpy_deg").value  
        xyz = self.get_parameter("lidar_xyz").value

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.lidar_frame
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation = rpy_to_quat(rpy_deg[0], rpy_deg[1], rpy_deg[2])
        
        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF base_link → lidar")


        # Publishing odom frame
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        tf.transform.rotation = rpy_to_quat(0.0,0.0,0.0)

        
        self.static_br.sendTransform(tf)
        self.get_logger().info("Published static TF map → odom")
        
     
     
    def receive_motors_speed(self, msg):
        self.client.set_motors_speed(msg.linear.x, msg.angular.z)
        
    def receive_lift_height(self, msg):
        if msg.data < 0:
            msg.data = 0.0
            
        self.client.set_lift_height(msg.data)
        
        
    def receive_servo_state(self, msg):
        self.client.set_servo_state(msg.data[0], msg.data[1], msg.data[2], msg.data[3])
        
        
        
    def publish_odometry(self):
        time_stamp = self.get_clock().now().to_msg()
        
        # Publishing odometry
        odom = Odometry()
        odom.header.stamp = time_stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.xPos)
        odom.pose.pose.position.y = float(self.yPos)
        odom.pose.pose.orientation = q_yaw(self.theta)
        odom.twist.twist.linear.x = float(self.linear_x)
        odom.twist.twist.angular.z = float(self.angular_z)
        
        self.odom_publisher.publish(odom)
        
        # Publishing odometry tf
        tf = TransformStamped()
        tf.header.stamp = time_stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = float(self.xPos)
        tf.transform.translation.y = float(self.yPos)
        tf.transform.translation.z = 0.0
        tf.transform.rotation = odom.pose.pose.orientation
        
        self.tf_br.sendTransform(tf)

        # Publishing lift frame
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.lift_frame
        tf.transform.translation.x = float(self.base_lift_xyz[0])
        tf.transform.translation.y = float(self.base_lift_xyz[1])
        tf.transform.translation.z = float(self.base_lift_xyz[2] + self.lift_height / 1000)
        tf.transform.rotation = q_yaw(0)
        
        self.tf_br.sendTransform(tf)
        
        # Publishing servos
        tf.header.frame_id = self.lift_frame
        
        for frame, pos, ang in zip(self.servo_frames, self.servos_pos, self.servo_state):
            tf.child_frame_id = frame
            tf.transform.translation.x = float(pos[0])
            tf.transform.translation.y = float(pos[1])
            tf.transform.translation.z = float(pos[2])
            tf.transform.rotation = rpy_to_quat(0, -ang / 180 * math.pi, 0)
            self.tf_br.sendTransform(tf)

    def publish_servo(self):
        servo = UInt8MultiArray()
        servo.data = list(self.servo_state)
        
        self.servo_pos_pub.publish(servo)
        
    def publish_lift_h(self):
        lift = Float32()
        lift.data = float(self.lift_height)
        
        self.lift_h_pub.publish(lift)
        
    
    def ping_esp(self):
        try:
            self.client.get_all()
        except Exception as e:
            self.get_logger().error("Couldn`t send message:" + str(e))
            
    def publish_lidar(self, angles, ranges, intens):        
        if self.get_parameter("lidar_mirror").value:
            angles.reverse()
            ranges.reverse()
            intens.reverse()
            angles = [-a for a in angles]
            
        shift_deg = self.get_parameter("lidar_shift_deg").value
        if shift_deg:
            angles = [ang + shift_deg for ang in angles]
            
        # нормализуем, чтобы начало ≈ −π
        if angles[0] > 180.0:
            angles = [ang - 360.0 for ang in angles]
            
        radians = list(map(math.radians, angles))
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.lidar_frame
        scan.angle_min = float(radians[0])
        scan.angle_max = float(radians[-1])
        scan.angle_increment = (radians[-1] - radians[0]) / (len(radians) - 1)
        scan.range_min = self.get_parameter("lidar_range_min").value
        scan.range_max = self.get_parameter("lidar_range_max").value
        scan.ranges = ranges
        scan.intensities = intens
        self.scan_pub.publish(scan)
        
    def receive_esp_data(self):
        msg = self.client.receive_msg()
        
        while msg:
            match msg['event']:
                case 'ANSWER_GET_MOTORS_SPEED':
                    self.linear_x = msg['linear']
                    self.angular_z = msg['angular']

                    # Publishing messages
                    self.publish_odometry()
                    
                case 'ANSWER_GET_LIFT_HEIGHT':
                    self.lift_height = msg['height']

                    # Publishing messages
                    self.publish_lift_h()
                    
                case 'ANSWER_GET_SERVO_STATE':
                    self.servo_state = msg['state']

                    # Publishing messages
                    self.publish_servo()
                    
                case 'ANSWER_GET_ODOMETRY':
                    self.xPos = msg['x']
                    self.yPos = msg['y']
                    self.theta = msg['theta']

                    # Publishing messages
                    self.publish_odometry()
                    
                case 'ANSWER_GET_ALL':
                    self.linear_x  = msg["motors_speed"]['linear']
                    self.angular_z = msg["motors_speed"]['angular']
                    
                    self.lift_height = msg["lift_height"]
                    
                    self.servo_state = msg['servo_state']
                    
                    self.xPos  = msg["odometry"]['x']
                    self.yPos  = msg["odometry"]['y']
                    self.theta = msg["odometry"]['theta']

                    # Publishing messages
                    self.publish_odometry()
                    self.publish_servo()
                    self.publish_lift_h()
                    
                case 'SEND_LIDAR':
                    self.publish_lidar(msg["angles"], msg["ranges"], msg["intens"])
                    
                case _:
                    self.get_logger().warn("Undefine event type:", msg['event'])
   
            msg = self.client.receive_msg()
                

def main(args=None):
    rclpy.init(args=args)

    node = Esp32_Bridge()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
