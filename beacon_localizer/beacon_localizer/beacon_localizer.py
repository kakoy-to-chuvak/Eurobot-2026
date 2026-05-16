import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import numpy as np
from itertools import combinations
import math

def q_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = float(np.sin(yaw / 2))
    q.w = float(np.cos(yaw / 2))
    return q

def procrustes(X, Y):
    """
    X, Y: (2, N) массивы точек.
    Возвращает R (2x2), t (2,), такие что Y ≈ R @ X + t.
    """
    x_mean = np.mean(X, axis=1, keepdims=True)
    y_mean = np.mean(Y, axis=1, keepdims=True)
    X_c = X - x_mean
    Y_c = Y - y_mean
    H = X_c @ Y_c.T
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = y_mean - R @ x_mean
    return R, t

class BeaconLocalizer(Node):
    def __init__(self):
        super().__init__('beacon_localizer')

        self.cluster_distance_threshold = 0.1   # расстояние для объединения точек в кластер
        self.max_theta_error = np.radians(30.0) # допустимый угол поворота
        self.max_s_error = 0.5                 # допустимое смещение

        self.beacon_size = 0.095
        self.beacon_r = self.beacon_size / 2.0

        # Маяки как numpy массив (2,2) – два столбца, каждый – (x,y)
        self.beacons_yellow = np.array([
            [-1.5 - self.beacon_r, -1.0 + self.beacon_r],
            [-1.5 - self.beacon_r,  1.0 - self.beacon_r]
        ]).T

        self.beacons_blue = np.array([
            [1.5 + self.beacon_r, -1.0 + self.beacon_r],
            [1.5 + self.beacon_r,  1.0 - self.beacon_r]
        ]).T

        self.beacons = self.beacons_blue

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.pose_history = []
        self.history_len = 5

        self.scan_sub = self.create_subscription(LaserScan, "/pwb/lidar_scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  "/pwb/odom",       self.odom_callback, 10)
        self.side_sub = self.create_subscription(String,    "/pwb/side",       self.side_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

    def odom_callback(self, odom: Odometry):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.theta = 2.0 * math.atan2(odom.pose.pose.orientation.z,
                                      odom.pose.pose.orientation.w)
        if self.theta < 0:
            self.theta += 2.0 * math.pi
        
    def side_callback(self, side: String):
        if side.data == 'yellow':
            self.beacons = self.beacons_yellow
        else:
            self.beacons = self.beacons_blue
        self.pose_history = []

    def scan_callback(self, scan: LaserScan):
        # 1. Переводим точки скана в глобальную систему (pwb_odom)
        points = []
        angle = scan.angle_min + self.theta
        for r in scan.ranges:
            if np.isfinite(r) and scan.range_min < r < scan.range_max:
                x = r * np.cos(angle) + self.x
                y = r * np.sin(angle) + self.y
                points.append((x, y))
            angle += scan.angle_increment
        if len(points) == 0:
            return
        points = np.array(points)

        # 2. Кластеризация по евклидову расстоянию
        clusters = []
        cur = [points[0]]
        for i in range(1, len(points)):
            if np.linalg.norm(points[i] - points[i-1]) < self.cluster_distance_threshold:
                cur.append(points[i])
            else:
                clusters.append(np.array(cur))
                cur = [points[i]]
        clusters.append(np.array(cur))

        # 3. Вычисляем центры кластеров и их радиусы (для веса)
        centers = []
        radii = []
        for cl in clusters:
            if len(cl) < 3:
                continue
            c = np.mean(cl, axis=0)
            r = np.max(np.linalg.norm(cl - c, axis=1))
            centers.append(c)
            radii.append(r)

        if len(centers) < 2:
            return

        # Расстояние между маяками
        beacon_dist = np.linalg.norm(self.beacons[:,0] - self.beacons[:,1])
        dist_tolerance = 0.12   # допустимое отклонение расстояния между кластерами

        best_score = float('inf')
        best_R, best_t = None, None

        # Перебираем все пары кластеров
        for (i, j) in combinations(range(len(centers)), 2):
            c1 = centers[i]
            c2 = centers[j]
            d = np.linalg.norm(c1 - c2)
            # Основной фильтр: расстояние между кластерами должно соответствовать расстоянию между маяками
            if abs(d - beacon_dist) > dist_tolerance:
                continue

            # Пробуем оба порядка соответствия маяков
            for perm in [(0,1), (1,0)]:
                X = np.array([c1, c2]).T          # (2,2)
                Y = self.beacons[:, perm]         # (2,2)
                R, t = procrustes(X, Y)

                # Проверяем, что поворот и перенос не выходят за допустимые пределы
                angle = math.atan2(R[1,0], R[0,0])
                if abs(angle) > self.max_theta_error:
                    continue
                if np.linalg.norm(t) > self.max_s_error:
                    continue

                # Вычисляем ошибку совмещения (сумма расстояний между преобразованными точками и маяками)
                transformed = R @ X + t
                error = np.linalg.norm(transformed - Y, axis=0).sum()

                # Вес на основе размера кластеров (чем ближе радиус к beacon_r, тем лучше)
                weight = 1.0 / (abs(radii[i] - self.beacon_r) + 0.01) * \
                         1.0 / (abs(radii[j] - self.beacon_r) + 0.01)

                score = error / weight
                if score < best_score:
                    best_score = score
                    best_R, best_t = R, t

        if best_R is None:
            return

        # 5. Применяем найденное преобразование к одометрической позе робота
        new_x = best_R[0,0] * self.x + best_R[0,1] * self.y + best_t[0]
        new_y = best_R[1,0] * self.x + best_R[1,1] * self.y + best_t[1]
        new_theta = self.theta + math.atan2(best_R[1,0], best_R[0,0])
        new_theta = new_theta % (2.0 * math.pi)

        # Сглаживание
        self.pose_history.append((new_x, new_y, new_theta))
        if len(self.pose_history) > self.history_len:
            self.pose_history.pop(0)
        sx = np.mean([p[0] for p in self.pose_history])
        sy = np.mean([p[1] for p in self.pose_history])
        st = np.mean([p[2] for p in self.pose_history])

        # 6. Публикация скорректированной позы
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'pwb_odom'
        pose.pose.pose.position.x = sx
        pose.pose.pose.position.y = sy
        pose.pose.pose.orientation = q_yaw(st)
        self.pose_pub.publish(pose)
        self.get_logger().info(f"Published pose: ({sx:.2f}, {sy:.2f}), theta={math.degrees(st):.1f}°")

def main(args=None):
    rclpy.init(args=args)
    node = BeaconLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutdown...")
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()