from dataclasses import dataclass
import math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi


@dataclass
class PlanarVehicle:
    x: float
    y: float
    speed: float
    heading: float

    def position(self):
        return np.array([self.x, self.y], dtype=float)


def propagate(vehicle, yaw_rate, speed_cmd, dt, max_yaw_rate, speed_limits):
    yaw_rate = float(np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate))
    speed_cmd = float(np.clip(speed_cmd, speed_limits[0], speed_limits[1]))

    # Simple first-order speed response.
    tau = 1.0
    speed = vehicle.speed + (speed_cmd - vehicle.speed) * dt / tau
    heading = wrap(vehicle.heading + yaw_rate * dt)

    x = vehicle.x + speed * math.cos(heading) * dt
    y = vehicle.y + speed * math.sin(heading) * dt
    return PlanarVehicle(x, y, speed, heading)
