# Cooperative USV–UAV Control and Mobile-Base Coordination

A research-oriented simulation framework for **cooperative autonomy between an Unmanned Surface Vehicle (USV) and an Unmanned Aerial Vehicle (UAV)**.

The project investigates a heterogeneous robotic architecture in which a USV acts as a **mobile supervisory, communication, and recovery platform** while a UAV performs an autonomous aerial waypoint mission.

The framework integrates:

- Autonomous USV route following
- Autonomous UAV waypoint navigation
- USV-based UAV mission supervision
- Relative USV–UAV state monitoring
- Range-dependent communication modelling
- Communication-aware mission management
- Return-to-USV logic
- Moving-platform recovery guidance
- Multi-mode mission management
- Mission and recovery performance evaluation

The implementation is generic and intended for autonomous-systems research and education.

---

# 1. Motivation

Cooperative operation between unmanned marine and aerial vehicles can extend the operational capabilities of heterogeneous autonomous systems.

A surface vehicle can provide a persistent mobile platform while an aerial vehicle provides greater mobility and access to areas beyond the immediate vicinity of the vessel.

A generic cooperative architecture can therefore be represented as:

```text
                USV
                 │
                 │ Mission Supervision
                 │
                 ↓
                UAV
                 │
                 │ Autonomous Mission
                 ↓
          Aerial Waypoints
                 │
                 ↓
         Return-to-USV Command
                 │
                 ↓
        Moving-Platform Recovery
```

Unlike navigation toward a stationary home location, recovery to a moving vessel requires the UAV reference to continuously change with the USV position.

This creates a coupled autonomy problem involving:

```text
USV Navigation
      +
UAV Navigation
      +
Communication
      +
Mission Management
      +
Relative Guidance
```

---

# 2. Project Objectives

The project is designed to investigate:

1. Autonomous navigation of a USV along a predefined marine route
2. Autonomous UAV waypoint navigation
3. Supervisory control of the UAV mission from a moving USV
4. Relative USV–UAV state monitoring
5. Communication-aware mission decisions
6. Autonomous return-to-USV behaviour
7. Recovery guidance toward a moving platform
8. Mission-state transitions
9. Cooperative-system performance metrics

The emphasis is on **heterogeneous multi-vehicle autonomy and supervisory coordination** rather than high-fidelity vehicle dynamics.

---

# 3. Overall Architecture

The implemented architecture is:

```text
                     Mission Definition
                            ↓
                   Mission Supervisor
                            │
              ┌─────────────┴─────────────┐
              ↓                           ↓
        USV Navigation               UAV Mission
              ↓                           ↓
       USV Controller               UAV Controller
              ↓                           ↓
          USV Model                    UAV Model
              │                           │
              └─────────────┬─────────────┘
                            ↓
                  Relative-State Monitor
                            ↓
                   Communication Model
                            ↓
                  Mission Decision Logic
                            ↓
              Continue / Return / Recover
```

The UAV mission therefore operates within a higher-level supervisory framework associated with the moving USV.

---

# 4. USV Model

The USV is represented using a planar kinematic state:

```text
x_USV
y_USV
V_USV
ψ_USV
```

where:

```text
x, y = Cartesian position
V = forward speed
ψ = heading
```

The planar motion is described by:

```text
x_dot = V cos(ψ)

y_dot = V sin(ψ)

ψ_dot = r
```

where:

```text
r = commanded yaw rate
```

The current implementation intentionally uses a lightweight kinematic model.

---

# 5. UAV Model

The UAV is also represented by a generic planar kinematic model:

```text
x_UAV
y_UAV
V_UAV
ψ_UAV
```

with:

```text
x_dot = V cos(ψ)

y_dot = V sin(ψ)

ψ_dot = ω
```

The UAV has different:

- Speed limits
- Maximum yaw-rate limits
- Mission references

from the USV.

This allows the aerial vehicle to operate independently while remaining part of the cooperative architecture.

---

# 6. USV Route Following

The USV follows a predefined sequence of route waypoints:

```text
P_USV =
[
p1,
p2,
...
pN
]
```

The current target waypoint is selected according to the vessel's position.

When:

```text
||p_USV - p_waypoint|| < d_threshold
```

the next route waypoint becomes active.

---

# 7. USV Heading Controller

For a reference waypoint:

```text
p_ref =
[x_ref, y_ref]
```

the desired heading is:

```text
ψ_d =
atan2(
y_ref - y,
x_ref - x
)
```

The heading error is:

```text
e_ψ =
wrap(
ψ_d - ψ
)
```

The yaw-rate command is:

```text
r_cmd =
sat(
Kψ e_ψ
)
```

This provides lightweight closed-loop route following.

---

# 8. UAV Mission

After the cooperative system reaches the appropriate mission state, the UAV is assigned a sequence of aerial waypoints:

```text
P_UAV =
[
q1,
q2,
q3,
...
qM
]
```

The UAV autonomously tracks these waypoints using its own heading and speed controller.

The mission is completed when all configured UAV waypoints have been reached.

---

# 9. Mobile Supervisory Platform

The USV is treated as a mobile supervisory platform.

Conceptually:

```text
USV
 │
 ├── Maintains marine route
 ├── Monitors UAV separation
 ├── Maintains mission state
 ├── Represents communication origin
 ├── Issues return logic
 └── Provides moving recovery reference
```

The UAV is therefore not simulated as an entirely independent robot.

Its mission is coordinated relative to the state of the surface platform.

---

# 10. Relative USV–UAV State

The relative position is:

```text
p_rel =
p_UAV
-
p_USV
```

The separation between vehicles is:

```text
d_rel =
||p_UAV - p_USV||
```

This quantity is continuously monitored.

Relative distance is used by:

- Communication modelling
- Mission supervision
- Return logic
- Recovery evaluation

---

# 11. Communication Model

The project includes a simplified range-dependent communication model.

The probability of receiving a communication update depends on:

```text
USV–UAV Separation
```

Conceptually:

```text
Short Separation
      ↓
High Communication Probability

Increasing Separation
      ↓
Reduced Communication Probability
```

The model intentionally abstracts away physical radio/network implementation.

---

# 12. Communication Success Probability

For short ranges, communication reliability is high.

As separation approaches and exceeds the nominal communication range, the probability of successful communication decreases.

Conceptually:

```text
P_comm =
f(d_USV-UAV)
```

where:

```text
d_USV-UAV =
||p_UAV - p_USV||
```

The model is stochastic, allowing different communication outcomes between simulation runs.

---

# 13. Communication Dropout

At every simulation step, communication can be represented as:

```text
COMM_OK
```

or:

```text
COMM_DROPOUT
```

A single lost update does not necessarily cause immediate mission termination.

Instead, the supervisory logic can use persistent communication-loss behaviour before initiating a return command.

This avoids reacting excessively to isolated stochastic packet losses.

---

# 14. Mission State Machine

The cooperative mission is organized using five states:

```text
TRANSIT
    ↓
AERIAL_MISSION
    ↓
RETURN_TO_USV
    ↓
RECOVERY_APPROACH
    ↓
COMPLETE
```

This provides an explicit supervisory structure for coordinating both vehicles.

---

# 15. TRANSIT Mode

During:

```text
TRANSIT
```

the USV follows its marine route.

The UAV remains close to the moving USV.

Conceptually:

```text
USV → Marine Route

UAV → Follow Mobile USV Reference
```

Once the USV reaches the configured mission-release region, the mission manager transitions to:

```text
AERIAL_MISSION
```

---

# 16. AERIAL_MISSION Mode

During:

```text
AERIAL_MISSION
```

the UAV follows its aerial waypoint sequence.

The USV continues moving along its own route.

Therefore:

```text
USV
 ↓
Marine Navigation

UAV
 ↓
Independent Aerial Waypoint Mission
```

while the supervisory layer continues monitoring the relative state.

---

# 17. Return Conditions

The UAV can transition from:

```text
AERIAL_MISSION
```

to:

```text
RETURN_TO_USV
```

when one of the configured conditions is satisfied.

Examples implemented in the framework include:

```text
Aerial Mission Completed
```

or:

```text
USV–UAV Separation Exceeds Operating Limit
```

or persistent:

```text
Communication Loss
```

The return logic therefore combines mission progress with communication-aware supervision.

---

# 18. RETURN_TO_USV Mode

When a return command is generated, the UAV no longer tracks the aerial mission waypoints.

Instead, the moving USV becomes the new reference.

```text
Aerial Mission
      ↓
Return Command
      ↓
Current USV Position
      ↓
Relative Recovery Guidance
      ↓
UAV
```

This is fundamentally different from returning to a fixed home coordinate because the target itself is moving.

---

# 19. Moving Recovery Reference

The recovery point is continuously updated according to the current USV position:

```text
p_recovery(t)
=
p_USV(t)
```

Therefore:

```text
p_recovery(t+Δt)
≠
p_recovery(t)
```

as the vessel moves.

The UAV controller must continuously update its desired heading.

---

# 20. Relative Recovery Guidance

The relative recovery error is:

```text
e_r =
p_recovery
-
p_UAV
```

and the recovery distance is:

```text
d_recovery =
||e_r||
```

The desired UAV heading becomes:

```text
ψ_d =
atan2(
e_r,y,
e_r,x
)
```

with heading error:

```text
e_ψ =
wrap(
ψ_d - ψ_UAV
)
```

The yaw-rate command is:

```text
ω_cmd =
sat(
Kψ e_ψ
)
```

---

# 21. Adaptive Recovery Speed

The UAV speed command is also adjusted according to recovery distance.

Conceptually:

```text
Large Recovery Error
        ↓
Higher UAV Speed

Small Recovery Error
        ↓
Reduced UAV Speed
```

This allows rapid return while reducing speed near the moving platform.

---

# 22. RECOVERY_APPROACH Mode

When the UAV reaches the vicinity of the USV, the mission manager switches to:

```text
RECOVERY_APPROACH
```

During this phase:

- The moving USV remains the recovery reference
- UAV speed is reduced
- Relative position error continues to be minimized

This represents the final approach region.

---

# 23. COMPLETE Mode

Recovery is considered successful when:

```text
d_recovery
<
d_recovery_threshold
```

The state then becomes:

```text
COMPLETE
```

and:

```text
Recovery Success = True
```

This represents rendezvous with the moving surface platform.

It should not be interpreted as a simulated physical deck touchdown.

---

# 24. Cooperative Closed Loop

The complete architecture becomes:

```text
                 USV Route
                    ↓
             USV Controller
                    ↓
                  USV
                    │
                    │ Position
                    ↓
             Relative Monitor
                    ↑
                    │ Position
                   UAV
                    ↑
             UAV Controller
                    ↑
          Mission / Recovery Ref.
                    ↑
             Mission Manager
                    ↑
          Communication Monitor
```

This creates a coupled heterogeneous autonomous-system simulation.

---

# 25. Performance Metrics

The framework evaluates several quantities.

## UAV Mission Completion Ratio

```text
Mission Completion =
Completed UAV Waypoints
/
Total UAV Waypoints
```

---

## Communication Dropout Ratio

```text
Dropout Ratio =
Lost Communication Updates
/
Total Communication Updates
```

---

## Maximum Separation

```text
d_max =
max(
||p_UAV - p_USV||
)
```

---

## Return Commands

The number of UAV return commands generated by the mission manager is recorded.

---

## Recovery Success

```text
Recovery Success =
True / False
```

---

## Final Recovery Error

```text
e_final =
||p_UAV(T) - p_USV(T)||
```

---

## Vehicle Path Lengths

The total path travelled by each vehicle is calculated.

```text
L_USV =
Σ ||p_USV,k+1 - p_USV,k||
```

and:

```text
L_UAV =
Σ ||p_UAV,k+1 - p_UAV,k||
```

---

# 26. Current Generic Sanity Check

A generic software sanity run of the current implementation produced approximately:

| Metric | Result |
|---|---:|
| UAV Mission Completion | 100% |
| Recovery Success | True |
| Communication Dropout Ratio | 2.9% |
| Maximum USV–UAV Separation | 37.14 |
| Return Commands | 1 |
| Final Recovery Error | 2.26 |
| USV Path Length | 79.62 |
| UAV Path Length | 143.19 |
| Simulation Time | 47.9 s |

These values are generated by the included synthetic demonstration.

They are provided as a software sanity check and should not be interpreted as real-world vehicle performance.

---

# 27. Repository Structure

```text
usv_uav_cooperative_control/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── docs/
│   └── scope.md
│
├── src/
│   ├── __init__.py
│   ├── models.py
│   ├── controllers.py
│   ├── communication.py
│   ├── mission_manager.py
│   └── simulation.py
│
├── examples/
│   └── run_demo.py
│
└── results/
    └── sanity_metrics.json
```

---

# 28. Module Description

| Module | Purpose |
|---|---|
| `models.py` | Generic planar USV and UAV kinematic models |
| `controllers.py` | Waypoint and relative-recovery controllers |
| `communication.py` | Range-dependent stochastic communication model |
| `mission_manager.py` | Cooperative mission state machine |
| `simulation.py` | Integrated USV–UAV simulation |
| `run_demo.py` | Demonstration and visualization |

---

# 29. Installation

Clone the repository:

```bash
git clone <repository-url>
cd usv-uav-cooperative-control
```

Install dependencies:

```bash
pip install -r requirements.txt
```

Dependencies:

```text
NumPy
Matplotlib
```

---

# 30. Running the Simulation

Run:

```bash
python examples/run_demo.py
```

The demonstration executes:

```text
USV Transit
     ↓
UAV Mission Release
     ↓
Aerial Waypoint Mission
     ↓
Communication Monitoring
     ↓
Return Command
     ↓
Moving-USV Recovery
```

---

# 31. Visualization

The example generates three primary visualizations.

## Cooperative Trajectories

Displays:

```text
USV Reference Route
USV Actual Trajectory
UAV Trajectory
UAV Mission Waypoints
```

## Relative State

Displays:

```text
USV–UAV Separation
Recovery-Point Error
```

## Communication Quality

Displays the time evolution of:

```text
Communication Success Probability
```

---

# 32. Recommended GitHub Results

Useful figures for the `results/` directory include:

```text
results/
├── usv_uav_trajectories.png
├── mission_modes.png
├── relative_separation.png
├── communication_quality.png
├── return_to_usv.png
└── recovery_error.png
```

Only outputs generated from actual simulation runs should be published.

---

# 33. Research Areas

The project is related to:

- Autonomous Surface Vehicles
- Autonomous UAVs
- Marine Robotics
- Heterogeneous Multi-Robot Systems
- Cooperative Autonomy
- Guidance and Control
- Mission Management
- Mobile-Base Robotics
- Communication-Aware Autonomy
- Autonomous Recovery
- Multi-Agent Systems

---

# 34. Technologies

- Python
- NumPy
- Matplotlib
- Autonomous Navigation
- Waypoint Guidance
- Relative Guidance
- State-Machine Design
- Multi-Vehicle Simulation
- Communication Modelling
- Performance Evaluation

---

# 35. Current Scope

The current implementation includes:

```text
Planar USV Kinematics
Planar UAV Kinematics
USV Route Following
UAV Waypoint Navigation
Mobile Supervisory Architecture
Relative-State Monitoring
Range-Dependent Communication Model
Communication Dropouts
Mission State Machine
Return-to-USV Logic
Moving-USV Recovery Guidance
Mission Metrics
```

---

# 36. Current Limitations

The current implementation does not include:

- 3-DoF USV hydrodynamics
- 6-DoF UAV flight dynamics
- Wind, wave, and current disturbances
- Ship roll, pitch, and heave
- Deck-motion prediction
- Precision deck landing
- Visual landing
- Fiducial-marker detection
- Camera-based relative localization
- Kalman-filter-based relative navigation
- Real communication protocols
- Network latency modelling
- ROS2
- PX4
- ArduPilot
- Gazebo / Ignition
- Hardware-in-the-loop testing
- Real UAV flight tests
- Real USV sea trials

Therefore, the current project should be interpreted as a **cooperative USV–UAV mission-control and relative-recovery simulation**, rather than a complete autonomous shipboard UAV landing system.

---

# 37. Future Extensions

## Relative State Estimation

A future version could introduce:

```text
USV GNSS / INS
       +
UAV GNSS / INS
       +
Visual Relative Measurement
       ↓
       EKF
       ↓
Relative Pose Estimate
```

---

## Visual Recovery

The final recovery stage could be extended with:

```text
Camera
   ↓
Deck / Marker Detection
   ↓
Relative Pose Estimation
   ↓
Visual Servoing
   ↓
Precision Recovery
```

---

## Vessel Motion Compensation

The USV model could be extended to include:

```text
Surge
Sway
Yaw
Roll
Pitch
Heave
```

allowing recovery guidance to account for vessel motion.

---

## Environmental Disturbances

The simulation could incorporate:

```text
Wind
+
Current
+
Wave Disturbances
```

for robustness evaluation.

---

## Communication Delay

The communication model could be extended from stochastic dropout to:

```text
Packet Loss
+
Latency
+
Variable Update Rate
+
Delayed State Information
```

This would allow investigation of communication-aware cooperative control.

---

## Predictive Recovery Guidance

Instead of tracking the current USV position:

```text
p_USV(t)
```

the UAV could track a predicted future recovery point:

```text
p_USV(t + T_prediction)
```

using estimated vessel velocity and heading.

This would turn the recovery problem into a moving-target rendezvous problem.

---

## MPC-Based Cooperative Control

A future MPC architecture could optimize:

```text
Tracking Error
+
Relative Separation
+
Control Effort
+
Communication Constraint
+
Recovery Error
```

subject to vehicle and mission constraints.

---

## ROS2 Integration

The system could later be separated into ROS2 nodes:

```text
/usv_state
/uav_state
/mission_manager
/communication_monitor
/uav_controller
/usv_controller
/recovery_manager
```

allowing the cooperative architecture to be tested as a distributed robotic system.

---

# 38. Potential Extended Architecture

A higher-fidelity future system could become:

```text
                       USV
                        │
            ┌───────────┼───────────┐
            ↓           ↓           ↓
          GNSS         IMU      Marine Sensors
            └───────────┼───────────┘
                        ↓
                 USV State Estimate
                        ↓
                 Mission Supervisor
                        ↓
                 Communication Link
                        ↓
                       UAV
            ┌───────────┼───────────┐
            ↓           ↓           ↓
          GNSS         IMU        Camera
            └───────────┼───────────┘
                        ↓
                 UAV State Estimate
                        ↓
                 Mission Controller
                        ↓
                 Return / Recovery
                        ↓
                Relative Navigation
                        ↓
                Precision Approach
```

This would provide a natural progression from the current lightweight simulation toward a more complete heterogeneous autonomous-system architecture.

---

# 39. Public Implementation Notice

This repository contains a **generic and sanitized research implementation**.

All:

- Vehicle parameters
- Routes
- Waypoints
- Communication ranges
- Mission thresholds
- Recovery parameters
- Simulation scenarios

are synthetic and generic.

The public implementation intentionally excludes:

- Real operational vehicle parameters
- Restricted communication configurations
- Proprietary control systems
- Real platform coordinates
- Real mission data
- Confidential sensor information
- Platform-specific recovery procedures

The project contains no weapon, targeting, payload-delivery, or operational engagement logic.

---

# 40. Status

**Research-oriented cooperative-autonomy simulation framework / active development**

The project currently demonstrates:

```text
USV Navigation
       ↓
Mobile Mission Supervision
       ↓
UAV Autonomous Mission
       ↓
Communication Monitoring
       ↓
Return Decision
       ↓
Relative Guidance
       ↓
Moving-USV Recovery
       ↓
Performance Evaluation
```

The primary research focus is on **heterogeneous autonomous systems, cooperative USV–UAV control, communication-aware mission management, and mobile-platform recovery**.

---

# Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- Marine Robotics
- Unmanned Surface Vehicles
- UAV Autonomy
- Cooperative Robotics
- Guidance, Navigation and Control
- State Estimation
- Sensor Fusion
- Model Predictive Control
- Reinforcement Learning
- Multi-Agent Systems
