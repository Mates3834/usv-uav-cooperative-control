from dataclasses import dataclass


@dataclass
class MissionState:
    mode: str = "TRANSIT"
    uav_waypoint_index: int = 0
    return_commands: int = 0
    recovery_success: bool = False
    comm_loss_count: int = 0


class MissionManager:
    def __init__(self, max_operating_range=60.0,
                 recovery_radius=2.5,
                 usv_release_index=2,
                 comm_loss_persistence=12):
        self.max_operating_range = max_operating_range
        self.recovery_radius = recovery_radius
        self.usv_release_index = usv_release_index
        self.comm_loss_persistence = comm_loss_persistence

    def update(self, state, usv_idx, separation,
               comm_ok, uav_mission_done, recovery_error):
        if comm_ok:
            state.comm_loss_count = 0
        else:
            state.comm_loss_count += 1

        if state.mode == "TRANSIT":
            if usv_idx >= self.usv_release_index:
                state.mode = "AERIAL_MISSION"

        elif state.mode == "AERIAL_MISSION":
            persistent_link_loss = state.comm_loss_count >= self.comm_loss_persistence

            if (separation > self.max_operating_range) or persistent_link_loss:
                state.mode = "RETURN_TO_USV"
                state.return_commands += 1
            elif uav_mission_done:
                state.mode = "RETURN_TO_USV"
                state.return_commands += 1

        elif state.mode == "RETURN_TO_USV":
            if recovery_error < 8.0:
                state.mode = "RECOVERY_APPROACH"

        elif state.mode == "RECOVERY_APPROACH":
            if recovery_error < self.recovery_radius:
                state.mode = "COMPLETE"
                state.recovery_success = True

        return state
