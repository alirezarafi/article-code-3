# article-code-3
fopid-trajectory-tracking
# README - Fractional-Order PID (FOPID) Controller for Path Planning and Obstacle Avoidance

This code implements a mobile robot navigation framework that combines the **Dragonfly Optimization Algorithm** for global path planning with a **fractional-order PID (FOPID)** controller for speed and heading tracking, plus a fuzzy obstacle-avoidance controller.

### Requirements:
- MATLAB 2021 or newer
- Robotics System Toolbox
- Fuzzy Logic Toolbox
- Fractional-order / FOMCON toolbox (for `fotf`, `oustapp`, etc.)
- Optimization Toolbox

### Usage:
1. Ensure that the environment map file `Obstaculo2.mat` is in the project folder.
2. To run the simulation, execute the following commands in MATLAB:
```matlab
runFOPIDmodel1()
runFOPIDmodel2()
```

### Overview:
- The **Dragonfly Algorithm** (`DragonflyPathPlanning_DF`) is used for global path planning on an occupancy-map-based environment.
- The resulting spline path is optionally refined using a **local path optimization** routine based on a distance transform map (`localPathOptimization`).
- A **FOPID controller** regulates the robot's heading (ψ) and forward speed along the planned path.
- Fractional orders (λ, μ) for integral/derivative actions are approximated using Oustaloup filters and discretized via Tustin (`c2d`).
- The desired speed is **scheduled based on distance to the final goal** (higher speed far from the goal, lower speed near the goal).
- A **fuzzy obstacle-avoidance controller** (`a107.m`) is activated when the robot approaches obstacles closer than a hysteresis threshold, using front-left, front, and front-right range measurements.
- Multiple **moving obstacles** are modeled (vertical, horizontal, and circular motion patterns) and dynamically updated in the runtime occupancy map.
- The robot is stopped when a moving obstacle becomes too close and resumes when the distance exceeds a safe margin (stop/resume hysteresis).
- The simulation computes and prints **error metrics** for:
  - heading tracking (MSE, IAE, ISE, MAE, RMSE),
  - speed tracking (MSE, IAE, ISE, MAE, RMSE),
  - path length error (planned vs. actual traveled distance).
- Several plots are generated, including:
  - Final path (Dragonfly spline, checkpoints, robot trajectory),
  - Dragonfly cost convergence,
  - Left/right wheel voltages,
  - Linear speeds (forward, planar magnitude) and yaw rate,
  - Wheel torques and wheel angular speeds,
  - Scalar robot speed profile,
  - Robot heading (yaw), X and Y position vs. time.
- All key time-series data and paths are packed into a `res` structure and saved to a MAT file (e.g., `run_FOPID2m2.mat`) for further analysis and comparison with other controllers (PID+Fuzzy, Type-3 FLS, etc.).

### Files:
- `runFOPIDmodel1.m`, `runFOPIDmodel2.m`: Main scripts/functions for running the FOPID-based path tracking and obstacle-avoidance simulation.
- `Obstaculo2.mat`: Environment/occupancy map file used to build the `binaryOccupancyMap`.
- `a107.m`: Fuzzy logic controller for local obstacle avoidance based on range sensor readings (front/FL/FR).
- `DynamicalModel.m`: Implementation of the robot dynamic model (states include wheel speeds, torques, body velocities, pose, etc.).
- `DrawRobot.m`: Visualization helper function for plotting the robot pose and footprint on the map.
- `DragonflyPathPlanning_DF` (local function in the same file): Dragonfly-based path planning routine operating on the inflated occupancy map.
- `localPathOptimization`, `localCost`, `estimateGradient`, `getDistanceToObstacle` (local functions): Auxiliary routines for local path refinement using a distance transform.
