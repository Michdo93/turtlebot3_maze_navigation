#!/usr/bin/env python3
"""
Pledge-Algorithmus für TurtleBot3 Burger
ROS 2 Humble

Unterschied zu Wall-Following:
  Wall-Following versagt bei Labyrinthen mit "Inseln" (Hindernisse die nicht
  mit der Außenwand verbunden sind) – der Bot kreist endlos um die Insel.

  Pledge löst das mit einem Winkelzähler (pledge_angle):
  - Bot fährt geradeaus (Grundrichtung = 0°)
  - Trifft er auf ein Hindernis, folgt er der linken Wand
  - Er folgt der Wand NUR solange bis pledge_angle wieder 0 ist
  - Dann fährt er wieder geradeaus
  → Bricht aus Inselschleifen aus, kommt auch durch komplexe Labyrinthe

Starten:
    ros2 run <dein_package> pledge

Topics:
    /scan   (sensor_msgs/LaserScan)   - LiDAR Input
    /cmd_vel (geometry_msgs/Twist)    - Geschwindigkeit Output
    /odom   (nav_msgs/Odometry)       - Für Winkelberechnung
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


# ──────────────────────────────────────────────────────────────
# Tuning-Parameter
# ──────────────────────────────────────────────────────────────
DESIRED_WALL_DIST   = 0.25   # [m]   Wandabstand beim Folgen
FRONT_CLEAR_DIST    = 0.35   # [m]   Mindestabstand vorne
LINEAR_SPEED        = 0.12   # [m/s]
ANGULAR_SPEED       = 0.7    # [rad/s]
KP_WALL             = 1.8    # P-Regler Wandabstand
ANGLE_EPSILON       = 0.15   # [rad] Toleranz für "Winkel ≈ 0"

# LiDAR-Sektoren
FRONT_ANGLES        = list(range(350, 360)) + list(range(0, 10))
LEFT_ANGLES         = list(range(80, 100))
FRONT_LEFT_ANGLES   = list(range(40, 60))


def get_min_range(ranges, indices, max_range=3.5):
    vals = [ranges[i] for i in indices
            if i < len(ranges) and math.isfinite(ranges[i]) and ranges[i] > 0.0]
    return min(vals) if vals else max_range


def yaw_from_quaternion(q):
    """Quaternion → Yaw-Winkel in Radian."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Kürzeste Winkeldifferenz a - b, normiert auf [-π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class PledgeFollower(Node):

    def __init__(self):
        super().__init__('pledge_follower')
        self.get_logger().info('Pledge-Algorithmus Node gestartet')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # ── Pledge-Zustand ─────────────────────────────────────
        self.pledge_angle   = 0.0    # Akkumulierter Winkel seit letztem Freifahrt-Start
        self.current_yaw    = 0.0    # Aktueller Yaw aus Odometrie
        self.prev_yaw       = None   # Vorheriger Yaw (für Delta-Berechnung)
        self.following_wall = False  # True = Wandfolge-Modus, False = Geradeaus

        self.scan_data      = None   # Letzter LiDAR-Scan

    def odom_callback(self, msg: Odometry):
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.prev_yaw is not None and self.following_wall:
            # Winkel-Delta akkumulieren (nur im Wandfolge-Modus)
            delta = angle_diff(yaw, self.prev_yaw)
            self.pledge_angle += delta

            self.get_logger().debug(
                f'pledge_angle={math.degrees(self.pledge_angle):.1f}°  '
                f'yaw={math.degrees(yaw):.1f}°'
            )

        self.prev_yaw    = yaw
        self.current_yaw = yaw

    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg
        self.update()

    def update(self):
        if self.scan_data is None:
            return

        ranges = self.scan_data.ranges
        dist_front      = get_min_range(ranges, FRONT_ANGLES)
        dist_left       = get_min_range(ranges, LEFT_ANGLES)
        dist_front_left = get_min_range(ranges, FRONT_LEFT_ANGLES)

        twist = Twist()

        # ── Pledge-Logik ───────────────────────────────────────
        # Wechsel in Wandfolge-Modus wenn Hindernis vorne
        if not self.following_wall and dist_front < FRONT_CLEAR_DIST:
            self.following_wall = True
            self.pledge_angle   = 0.0   # Zähler auf 0 setzen
            self.get_logger().info('→ Wechsel in Wandfolge-Modus')

        # Wechsel zurück zu Geradeaus wenn pledge_angle wieder ~0
        if self.following_wall and abs(self.pledge_angle) < ANGLE_EPSILON:
            # Sicherstellen dass Weg vorne auch frei ist
            if dist_front > FRONT_CLEAR_DIST:
                self.following_wall = False
                self.get_logger().info('→ Wechsel in Geradeaus-Modus (pledge_angle ≈ 0)')

        # ── Bewegung ───────────────────────────────────────────
        if not self.following_wall:
            # Geradeaus – Grundrichtung beibehalten
            if dist_front < FRONT_CLEAR_DIST:
                # Noch nicht frei – warten / leicht drehen
                twist.linear.x  = 0.0
                twist.angular.z = -ANGULAR_SPEED   # rechts drehen bis frei
            else:
                twist.linear.x  = LINEAR_SPEED
                twist.angular.z = 0.0

        else:
            # Wandfolge (links) mit P-Regler
            if dist_front < FRONT_CLEAR_DIST:
                # Hindernis vorne → nach rechts drehen
                twist.linear.x  = 0.0
                twist.angular.z = -ANGULAR_SPEED

            elif dist_left > DESIRED_WALL_DIST * 2.0:
                # Keine Wand links → Ecke, nach links abbiegen
                twist.linear.x  = LINEAR_SPEED * 0.5
                twist.angular.z = ANGULAR_SPEED

            else:
                # Normal entlangfahren
                error = dist_left - DESIRED_WALL_DIST
                if dist_front_left < dist_left:
                    error -= 0.05   # Vorauskorrektur
                twist.linear.x  = LINEAR_SPEED
                twist.angular.z = KP_WALL * error

        self.get_logger().debug(
            f'Mode: {"WALL" if self.following_wall else "STRAIGHT":6s} | '
            f'pledge={math.degrees(self.pledge_angle):+6.1f}° | '
            f'Front:{dist_front:.2f}  Left:{dist_left:.2f}'
        )

        self.pub_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PledgeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())   # Stop bei Ctrl+C
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
