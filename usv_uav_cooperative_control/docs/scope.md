# Scope and limitations

This repository is a generic research simulation of cooperative USV–UAV autonomy.

Implemented:
- 2D kinematic USV and UAV models,
- USV route following,
- UAV waypoint mission,
- mobile-USV supervisory architecture,
- range-dependent communication-dropout model,
- return-to-USV decision logic,
- relative recovery guidance,
- mission/recovery metrics.

Not implemented:
- real command-and-control links,
- real RF/network protocols,
- ROS2,
- PX4/ArduPilot,
- Gazebo/Ignition,
- 3D/6-DoF UAV dynamics,
- 3-DoF vessel hydrodynamics,
- deck-motion compensation,
- visual landing,
- precision landing,
- real shipboard launch/recovery hardware,
- sea trials or flight tests.

The implementation contains no weapon, targeting, payload-delivery, or operational
mission logic.
