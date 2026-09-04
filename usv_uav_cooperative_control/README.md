# Cooperative USV–UAV Control and Mobile-Base Coordination

A research-oriented simulation framework for **cooperative control of an unmanned
aerial vehicle from an unmanned surface vessel acting as a mobile supervisory
and recovery platform**.

The project studies a generic maritime autonomy architecture in which:

- a USV follows a marine route,
- the USV acts as a mobile command / reference platform,
- a UAV performs an aerial waypoint mission,
- the UAV receives mission-state updates through a simplified communication link,
- the system monitors relative range and communication quality,
- the UAV can be commanded to return to the moving USV,
- a relative-motion controller guides the UAV toward a recovery point associated
  with the moving vessel.

The public implementation is generic and intended for research and educational use.
It does not contain weapon, targeting, payload-delivery, or operational mission logic.

## Architecture

```text
                Mission Supervisor
                       │
          ┌────────────┴────────────┐
          ↓                         ↓
      USV Route                 UAV Mission
          ↓                         ↓
    USV Controller             UAV Controller
          ↓                         ↓
      USV Model                  UAV Model
          │                         │
          └────────────┬────────────┘
                       ↓
             Relative-State Monitor
                       ↓
              Communication Model
                       ↓
       Return / Recovery Decision Logic
                       ↓
        Moving-USV Recovery Guidance
```

## Implemented modes

```text
TRANSIT
AERIAL_MISSION
RETURN_TO_USV
RECOVERY_APPROACH
COMPLETE
```

## Main metrics

- UAV mission completion ratio
- Relative USV–UAV separation
- Communication dropout ratio
- Number of return commands
- Recovery success
- Recovery position error
- USV path length
- UAV path length

## Run

```bash
pip install -r requirements.txt
python examples/run_demo.py
```

## Scope

This project uses generic 2D kinematics and simplified supervisory logic.
It does not implement PX4, ROS2, Gazebo, real radios, real deck landing,
aerodynamic flight dynamics, ship-motion compensation, or hardware experiments.
