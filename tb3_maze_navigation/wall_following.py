#!/usr/bin/env python3
"""
Wall-Following Algorithmus für TurtleBot3 Burger
ROS 2 Humble

Strategie: Linke Wand folgen (left-hand rule)
- Ziel: Von außen in die Mitte eines Labyrinths
- Funktioniert NUR in einfach-zusammenhängenden Labyrinthen
  (d.h. alle Wände sind mit der Außenwand verbunden – keine Inseln!)

Starten:
    ros2 run <dein_package> wall_following

Topics:
    /scan       (sensor_msgs/LaserScan)   - LiDAR Input
    /cmd_vel    (geometry_msgs/Twist)     - Geschwindigkeit Output
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


# ──────────────────────────────────────────────────────────────
# Tuning-Parameter
# ──────────────────────────────────────────────────────────────
DESIRED_WALL_DIST   = 0.25   # [m]  Gewünschter Abstand zur linken Wand
FRONT_CLEAR_DIST    = 0.35   # [m]  Mindestabstand nach vorne (sonst drehen)
LINEAR_SPEED        = 0.12   # [m/s] Vorwärtsgeschwindigkeit
ANGULAR_SPEED       = 0.8    # [rad/s] Drehgeschwindigkeit
KP_WALL             = 1.8    # Proportionalfaktor für Wandabstand-Regelung

# LiDAR-Sektoren (Indizes in LaserScan.ranges, 360°-LiDAR des TB3)
# 0° = vorne, 90° = links, 270° = rechts (mathematisch positiv = CCW)
FRONT_ANGLES        = list(range(350, 360)) + list(range(0, 10))   # ±10° vorne
LEFT_ANGLES         = list(range(80, 100))                          # ~90° links
FRONT_LEFT_ANGLES   = list(range(40, 60))                           # ~45° vorne-links


def get_min_range(ranges, indices, max_range=3.5):
    """Minimale Distanz in einem Sektor, NaN/Inf werden ignoriert."""
    vals = [ranges[i] for i in indices
            if i < len(ranges) and math.isfinite(ranges[i]) and ranges[i] > 0.0]
    return min(vals) if vals else max_range


class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.get_logger().info('Wall-Following Node gestartet (linke Wand)')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.state = 'find_wall'   # Startzustand

    # ──────────────────────────────────────────────────────────
    # Zustände
    # ──────────────────────────────────────────────────────────
    # find_wall  : Keine Wand links – geradeaus fahren bis eine gefunden wird
    # follow_wall: Wand links vorhanden – entlangfahren und Abstand halten
    # turn_right : Wand vorne – nach rechts drehen
    # turn_left  : Ecke links – nach links drehen (Kurve nach links folgen)
    # ──────────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges

        dist_front      = get_min_range(ranges, FRONT_ANGLES)
        dist_left       = get_min_range(ranges, LEFT_ANGLES)
        dist_front_left = get_min_range(ranges, FRONT_LEFT_ANGLES)

        twist = Twist()

        # ── Zustandsübergänge ──────────────────────────────────
        if dist_front < FRONT_CLEAR_DIST:
            self.state = 'turn_right'

        elif dist_left > DESIRED_WALL_DIST * 2.0:
            # Keine Wand links mehr → Ecke, nach links abbiegen
            self.state = 'turn_left'

        elif dist_left < DESIRED_WALL_DIST * 0.5:
            # Zu nah an der Wand → etwas nach rechts korrigieren
            self.state = 'follow_wall'

        else:
            self.state = 'follow_wall'

        # ── Aktionen je Zustand ────────────────────────────────
        if self.state == 'find_wall':
            twist.linear.x  = LINEAR_SPEED
            twist.angular.z = 0.0

        elif self.state == 'turn_right':
            twist.linear.x  = 0.0
            twist.angular.z = -ANGULAR_SPEED   # Uhrzeigersinn (rechts)

        elif self.state == 'turn_left':
            twist.linear.x  = LINEAR_SPEED * 0.5
            twist.angular.z = ANGULAR_SPEED    # Gegen Uhrzeigersinn (links)

        elif self.state == 'follow_wall':
            # P-Regler: Fehler = aktueller Abstand – gewünschter Abstand
            error = dist_left - DESIRED_WALL_DIST

            # Zusätzliche Korrektur wenn front-left näher als left
            # (Wand läuft auf uns zu → präventiv nach rechts lenken)
            if dist_front_left < dist_left:
                error -= 0.05

            twist.linear.x  = LINEAR_SPEED
            twist.angular.z = KP_WALL * error   # positiv = links = weg von Wand

        self.get_logger().debug(
            f'State: {self.state:12s} | '
            f'Front: {dist_front:.2f}m  Left: {dist_left:.2f}m  FL: {dist_front_left:.2f}m'
        )

        self.pub_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
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
