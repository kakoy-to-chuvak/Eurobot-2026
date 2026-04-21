import rclpy
import sys

from geometry_msgs.msg import Twist
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

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear linear by 10%
e/c : increase/decrease only angular linear by 10%

CTRL-C to quit
------------------------------------------------------
"""

moveBindings = {
		'i': (  1,  0 ),
		'o': (  1, -1 ),
		'j': (  0,  1 ),
		'l': (  0, -1 ),
		'u': (  1,  1 ),
		',': ( -1,  0 ),
		'.': ( -1,  1 ),
		'm': ( -1, -1 ),
}

lift_binds = {
        "=":  10,
        "-": -10,

        "+":  50,
        "_": -50,
}

servo_binds = {
    "1": ( 0,  10 ),
    "q": ( 0, -10 ),
    "2": ( 1,  10 ),
    "w": ( 1, -10 ),
    "3": ( 2,  10 ),
    "e": ( 2, -10 ),
    "4": ( 3,  10 ),
    "r": ( 3, -10 ),

    "!": ( 0,  90 ),
    "Q": ( 0, -90 ),
    "@": ( 1,  90 ),
    "W": ( 1, -90 ),
    "#": ( 2,  90 ),
    "E": ( 2, -90 ),
    "$": ( 3,  90 ),
    "R": ( 3, -90 ),
}

all_servo_bind = {
    "5": 10,
    "t": -10,

    "%": 100,
    "T": -100,
}

speedBindings={
		'a': ( 10/9 ,10/9 ),
		'z': ( 0.9  ,0.9  ),
		's': ( 10/9 ,1    ),
		'x': ( 0.9  ,1    ),
		'd': ( 1    ,10/9 ),
		'c': ( 1    ,0.9  ),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
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

def print_all(linear, angular, lift_h, servos):
    print("\033c")
    print(msg)
    print(f"linear: {linear:.3f}\tangular: {angular:.3f}\nlift h: {lift_h}\tservos: {servos}")

def main(args=None):    
    if args is None:
        args = sys.argv
        
    settings = saveTerminalSettings()

    rclpy.init()
    node = rclpy.create_node('my_teleop')
        
    vel_pub   = node.create_publisher(Twist,           '/pwb/cmd_vel', 10)
    lift_pub  = node.create_publisher(UInt16,          '/pwb/lift_target_height', 10)
    servo_pub = node.create_publisher(UInt8MultiArray, '/pwb/servos_target_angles', 10)

    linear = 0.5
    angular = 1.0
    x = 0
    y = 0
    th = 0

    servos = [ 90, 90, 90, 90 ]
    lift_h = 0
    


    try:
        print(msg)
        print_all(linear, angular, lift_h, servos)
        while(1):
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                
                twist = Twist()
                twist.linear.x = x*linear
                twist.angular.x = 0.0 
                twist.angular.y = 0.0 
                twist.angular.z = th*angular
                vel_pub.publish(twist)

            elif key in speedBindings.keys():
                linear = linear * speedBindings[key][0]
                angular = angular * speedBindings[key][1]

                print_all(linear, angular, lift_h, servos)

                twist = Twist()
                twist.linear.x = x*linear
                twist.angular.x = 0.0 
                twist.angular.y = 0.0 
                twist.angular.z = th*angular
                vel_pub.publish(twist)
            
            elif key in lift_binds:
                lift_h += lift_binds[key]
                if lift_h < 0:
                    lift_h = 0
                elif lift_h > 500:
                    lift_h = 500

                float_msg = UInt16()
                float_msg.data = int(lift_h)
                lift_pub.publish(float_msg)

                print_all(linear, angular, lift_h, servos)

            elif key in servo_binds.keys():
                servos[servo_binds[key][0]]+= servo_binds[key][1]
                if servos[servo_binds[key][0]] < 0:
                    servos[servo_binds[key][0]] = 0
                if servos[servo_binds[key][0]] > 90:
                    servos[servo_binds[key][0]] = 90
                
                servo_msg = UInt8MultiArray()
                servo_msg.data = servos
                servo_pub.publish(servo_msg)

                print_all(linear, angular, lift_h, servos)
            elif key in all_servo_bind.keys():
                for i in range(len(servos)):
                    servos[i] += all_servo_bind[key]
                    if servos[i] < 0:
                        servos[i] = 0
                    if servos[i] > 90:
                        servos[i] = 90
                print_all(linear, angular, lift_h, servos)

                servo_msg = UInt8MultiArray()
                servo_msg.data = servos
                servo_pub.publish(servo_msg)

            else:
                x = 0
                th = 0
                twist = Twist()
                twist.linear.x = x*linear
                twist.angular.x = 0.0 
                twist.angular.y = 0.0 
                twist.angular.z = th*angular
                vel_pub.publish(twist)
                if (key == '\x03'):
                    break

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        vel_pub.publish(twist)

        restoreTerminalSettings(settings)


