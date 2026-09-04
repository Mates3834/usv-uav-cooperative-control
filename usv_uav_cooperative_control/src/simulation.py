import numpy as np
from .models import PlanarVehicle, propagate
from .controllers import waypoint_controller, relative_recovery_controller
from .communication import CommunicationLink
from .mission_manager import MissionState, MissionManager


def _advance_index(vehicle, path, idx, threshold):
    while idx < len(path)-1 and np.linalg.norm(vehicle.position()-path[idx]) < threshold:
        idx += 1
    return idx


def run(duration=220.0, dt=0.1, seed=8):
    usv_path = np.array([
        [0., 0.], [20., 4.], [40., 8.], [65., 10.],
        [90., 18.], [115., 22.], [140., 20.]
    ])
    uav_mission = np.array([
        [45., 30.], [65., 42.], [85., 36.], [100., 28.]
    ])

    usv = PlanarVehicle(0., 0., 0.0, 0.0)
    uav = PlanarVehicle(2., -2., 0.0, 0.0)

    usv_idx = 1
    uav_idx = 0
    manager = MissionManager()
    state = MissionState()
    link = CommunicationLink(seed=seed)

    hist = {
        "t": [], "usv": [], "uav": [], "mode": [],
        "separation": [], "comm_ok": [], "comm_prob": [],
        "recovery_error": []
    }

    uav_started = False

    for k in range(int(duration/dt)):
        t = k*dt

        # USV route tracking.
        usv_idx = _advance_index(usv, usv_path, usv_idx, 2.0)
        ur, us = waypoint_controller(
            usv, usv_path[usv_idx],
            max_yaw_rate=0.28, cruise_speed=1.7
        )
        usv = propagate(usv, ur, us, dt, 0.28, (0.0, 2.0))

        separation = float(np.linalg.norm(uav.position()-usv.position()))
        comm_ok, comm_prob = link.transmit(separation)

        recovery_point = usv.position().copy()
        recovery_error = float(np.linalg.norm(uav.position()-recovery_point))

        mission_done = (uav_idx >= len(uav_mission))
        state = manager.update(
            state, usv_idx, separation, comm_ok,
            mission_done, recovery_error
        )

        # UAV supervisory control.
        if state.mode == "TRANSIT":
            # UAV remains close to moving USV before mission release.
            wp = usv.position() + np.array([2.0, -2.0])
            yr, vs = relative_recovery_controller(uav, wp)

        elif state.mode == "AERIAL_MISSION":
            uav_started = True
            if uav_idx < len(uav_mission):
                if np.linalg.norm(uav.position()-uav_mission[uav_idx]) < 2.0:
                    uav_idx += 1
                if uav_idx < len(uav_mission):
                    yr, vs = waypoint_controller(
                        uav, uav_mission[uav_idx],
                        max_yaw_rate=0.8, cruise_speed=3.8
                    )
                else:
                    yr, vs = relative_recovery_controller(uav, recovery_point)
            else:
                yr, vs = relative_recovery_controller(uav, recovery_point)

        elif state.mode in ("RETURN_TO_USV", "RECOVERY_APPROACH"):
            yr, vs = relative_recovery_controller(uav, recovery_point)
            if state.mode == "RECOVERY_APPROACH":
                vs = min(vs, 1.6)

        else:  # COMPLETE
            yr, vs = 0.0, 0.0

        uav = propagate(uav, yr, vs, dt, 0.8, (0.0, 4.5))

        hist["t"].append(t)
        hist["usv"].append([usv.x, usv.y])
        hist["uav"].append([uav.x, uav.y])
        hist["mode"].append(state.mode)
        hist["separation"].append(separation)
        hist["comm_ok"].append(comm_ok)
        hist["comm_prob"].append(comm_prob)
        hist["recovery_error"].append(recovery_error)

        if state.mode == "COMPLETE" and uav_started:
            break

    for key in ("t", "usv", "uav", "separation", "comm_ok",
                "comm_prob", "recovery_error"):
        hist[key] = np.asarray(hist[key])

    usv_len = float(np.sum(np.linalg.norm(np.diff(hist["usv"],axis=0),axis=1))) if len(hist["usv"])>1 else 0.
    uav_len = float(np.sum(np.linalg.norm(np.diff(hist["uav"],axis=0),axis=1))) if len(hist["uav"])>1 else 0.
    dropout = float(1.0 - np.mean(hist["comm_ok"])) if len(hist["comm_ok"]) else 0.0
    completed_wp = min(uav_idx, len(uav_mission))
    mission_ratio = completed_wp / len(uav_mission)

    metrics = {
        "uav_mission_completion_ratio": float(mission_ratio),
        "communication_dropout_ratio": dropout,
        "return_commands": int(state.return_commands),
        "recovery_success": bool(state.recovery_success),
        "final_recovery_error": float(hist["recovery_error"][-1]),
        "usv_path_length": usv_len,
        "uav_path_length": uav_len,
        "max_separation": float(np.max(hist["separation"])),
        "simulation_time": float(hist["t"][-1] if len(hist["t"]) else 0.0)
    }
    return usv_path, uav_mission, hist, metrics
