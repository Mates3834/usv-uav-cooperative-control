import math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


def waypoint_controller(vehicle, waypoint, kp_heading=1.5,
                        max_yaw_rate=0.5, cruise_speed=2.0):
    dx = waypoint[0] - vehicle.x
    dy = waypoint[1] - vehicle.y
    desired = math.atan2(dy, dx)
    err = wrap(desired - vehicle.heading)
    yaw_rate = float(np.clip(kp_heading * err, -max_yaw_rate, max_yaw_rate))
    dist = math.hypot(dx, dy)
    speed_cmd = min(cruise_speed, 0.6 + 0.35 * dist)
    return yaw_rate, speed_cmd


def relative_recovery_controller(uav, recovery_point,
                                 max_yaw_rate=0.8, max_speed=4.5):
    dx = recovery_point[0] - uav.x
    dy = recovery_point[1] - uav.y
    desired = math.atan2(dy, dx)
    err = wrap(desired - uav.heading)
    yaw_rate = float(np.clip(1.8 * err, -max_yaw_rate, max_yaw_rate))
    dist = math.hypot(dx, dy)
    speed_cmd = float(np.clip(0.9 + 0.55 * dist, 0.8, max_speed))
    return yaw_rate, speed_cmd
