import rclpy
import sys
import signal

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt8MultiArray, UInt16

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

#settings = termios.tcgetattr(sys.stdin)

msg = """
-----------------------------------------------------
Moving around:   |  Rotating Servo:  |  Moving lift
   u    i    o   |    1   2   3   4  |  -   =
   j    k    l   |    q   w   e   r  |
   m    ,    .   |                   |
All servos: 5 (10 deg), t (-10 deg)
anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear linear by 10%
e/c : increase/decrease only angular linear by 10%

SHIFT to increase influence
CTRL-C to quit
------------------------------------------------------
"""


moveBindings = {
    'i': ( 1,  0 ),
    'o': ( 1, -1 ),
    'j': ( 0,  1 ),
    'l': ( 0, -1 ),
    'u': ( 1,  1 ),
    ',': (-1,  0 ),
    '.': (-1,  1 ),
    'm': (-1, -1 ),
}

lift_binds = {
    "=":  10,
    "-": -10,
    "+":  50,
    "_": -50,
}

servo_binds = {
    "1": (0,  10), "q": (0, -10),
    "2": (1,  10), "w": (1, -10),
    "3": (2,  10), "e": (2, -10),
    "4": (3,  10), "r": (3, -10),

    "!": (0,  90), "Q": (0, -90),
    "@": (1,  90), "W": (1, -90),
    "#": (2,  90), "E": (2, -90),
    "$": (3,  90), "R": (3, -90),
}

all_servo_bind = {
    "5":  10, "t": -10,
    "%": 100, "T": -100,
}

speedBindings = {
    'a': (10/9, 10/9), "A": (100/81, 100/81),
    'z': (0.9,  0.9),  "Z": (0.81,   0.81),
    's': (10/9, 1),    "S": (100/81, 1),
    'x': (0.9,  1),    "X": (0.81,   1),
    'd': (1,    10/9), "D": (1,      100/81),
    'c': (1,    0.9),  "C": (1,      0.81),
}

MAX_LINEAR = 1.5   # м/с
MAX_ANGULAR = 3.0  # рад/с
MAX_LIFT = 1000    # мм
MAX_SERVO = 90
MIN_SERVO = 0


def getKey(settings):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def print_all(linear, angular, lift_h, servos, last_key=""):
    print("\033c", end="")
    print(msg)
    print(f"Last key: '{last_key}'")

    print(f"Linear: {linear:.3f} m/s\tAngular: {angular:.3f} rad/s")
    print(f"Lift height: {lift_h} mm\tServos: {servos}")


def publish_twist(publisher, stamp, linear, angular):
    twist = TwistStamped()
    twist.header.stamp = stamp
    twist.header.frame_id = "base_link"
    twist.twist.linear.x = float(linear)
    twist.twist.angular.z = float(angular)
    publisher.publish(twist)

def main(args=None):    
    if args is None:
        args = sys.argv
        
    settings = saveTerminalSettings()

    rclpy.init(args=args)
    node = rclpy.create_node('my_teleop')
    
    vel_pub   = node.create_publisher(TwistStamped,    '/pwb/cmd_vel', 10)
    lift_pub  = node.create_publisher(UInt16,          '/pwb/lift_target_height', 10)
    servo_pub = node.create_publisher(UInt8MultiArray, '/pwb/servos_target_angles', 10)

    # Настройки
    linear = 0.5
    angular = 1.0
    x_cmd = 0
    ang_cmd = 0
    
    # Состояние
    servos = [90, 90, 90, 90]
    lift_h = 0
    
    # Обработчик Ctrl+C
    def signal_handler(sig, frame):
        print("\nStopping robot...")
        publish_twist(vel_pub, node.get_clock().now().to_msg(), 0.0, 0.0)
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        print(msg)
        print_all(linear, angular, lift_h, servos)
        
        while rclpy.ok():
            key = getKey(settings)
            
            if key in moveBindings:
                x_cmd = moveBindings[key][0]
                ang_cmd = moveBindings[key][1]
                publish_twist(vel_pub, node.get_clock().now().to_msg(), x_cmd*linear, ang_cmd*angular)                
                print_all(linear, angular, lift_h, servos, key)

            elif key in speedBindings:
                linear = min(linear * speedBindings[key][0], MAX_LINEAR)
                angular = min(angular * speedBindings[key][1], MAX_ANGULAR)
                
                # Продолжаем текущее движение с новыми скоростями
                publish_twist(vel_pub, node.get_clock().now().to_msg(), x_cmd*linear, ang_cmd*angular)
                
                print_all(linear, angular, lift_h, servos, key)
            
            elif key in lift_binds:
                lift_h += lift_binds[key]
                lift_h = max(0, min(lift_h, MAX_LIFT))
                
                lift_msg = UInt16()
                lift_msg.data = int(lift_h)
                lift_pub.publish(lift_msg)
                
                print_all(linear, angular, lift_h, servos, key)

            elif key in servo_binds:
                idx, delta = servo_binds[key]
                servos[idx] += delta
                servos[idx] = max(MIN_SERVO, min(servos[idx], MAX_SERVO))
                
                servo_msg = UInt8MultiArray()
                servo_msg.data = servos
                servo_pub.publish(servo_msg)
                
                print_all(linear, angular, lift_h, servos, key)
                
            elif key in all_servo_bind:
                delta = all_servo_bind[key]
                for i in range(len(servos)):
                    servos[i] += delta
                    servos[i] = max(MIN_SERVO, min(servos[i], MAX_SERVO))
                
                servo_msg = UInt8MultiArray()
                servo_msg.data = servos
                servo_pub.publish(servo_msg)
                
                print_all(linear, angular, lift_h, servos, key)
                
            elif key == '\x03':  # Ctrl+C
                publish_twist(vel_pub, node.get_clock().now().to_msg(), 0.0, 0.0)
                break

    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        publish_twist(vel_pub, node.get_clock().now().to_msg(), 0.0, 0.0)
        restoreTerminalSettings(settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()