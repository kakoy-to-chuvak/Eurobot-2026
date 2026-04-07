import math
import rclpy
from rclpy.node import Node
import lidar_odometry.geometry as g

# Messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry



MIN_MATCH = 0.4









class LidarOdometry(Node):
    def __init__(self):
        super().__init__('lidar_odometry')

        self.declare_parameter("lidar_mirror", True)     # зеркально (True/False)

        self.lidar_mirror = self.get_parameter("lidar_mirror").value

        self.lidar_sub = self.create_subscription(LaserScan, "/pwb/lidar_scan", self.receive_scan, 10)



    def receive_scan(self, scan: LaserScan):
        points = []

        ranges = list(scan.ranges)
        angles = [ scan.angle_min + scan.angle_increment * i for i in range(len(ranges)) ]

        if self.lidar_mirror:
            angles.reverse()
            ranges.reverse()

        for rng, ang in zip(ranges, angles):
            if not scan.range_min <= rng <= scan.range_max:
                continue
            
            points.append(g.Vec2(rng*math.cos(ang), rng*math.sin(ang)))

        del scan, ranges, angles 

        self.handle_points(points)


    def handle_points(self, points: list[g.Vec2]):
        if len(points) < 5:
            return
        k = 0 # current line
        lines = [g.PointsLine(points[0], points[1])]
        
        i = 2 # current point
        while i < len(points):
            if lines[k].append(points[i]):
                i += 1
                continue
            
            if i + 1 == len(points):
                lines[0].add_to_start(points[i])
                break

            lines.append(g.PointsLine(points[i], points[i+1]))
            k += 1
            i += 2
        
        combined = lines[k].combine(lines[0])
        if combined:
            lines[0] = lines[k]
            lines.pop(k)

        del combined, k, i, points

        lines = list(map(lambda line: line.to_line(), lines))
        self.handle_lines(lines)

    def handle_lines(self, lines: list[g.Line]):
        print("Lines:", len(lines))

        objects = []
        new_line = True

        for line in lines:
            if new_line:
                objects.append(g.PointsObject(line))
                new_line = False
                continue

            if objects[-1].append(line):
                new_line = True
                continue
                
            objects.append(g.PointsObject(line))

        del new_line, lines

        objects = list(filter(lambda obj: obj.matches() > MIN_MATCH, objects))

        for obj in objects:
            print(obj)
    
    


def main(args=None):
    rclpy.init(args=args)

    node = LidarOdometry()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
