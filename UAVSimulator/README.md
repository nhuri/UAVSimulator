# UAV Path Planning Simulation

This project simulates the 2D motion of Unmanned Aerial Vehicles (UAVs) from a starting position to a specified target location, subject to constraints like constant velocity and minimum turning radius.

## Overview

The UAVs are initialized with position, heading (azimuth), velocity, and minimum turning radius. At specified times during the simulation, each UAV receives a target position. The UAV then computes a smooth path to reach the target.

### Path Calculation

The computed path consists of three segments:

1. Initial arc movement along a circle of radius `r_min` that is determined based on the UAV’s starting position and heading.
2. Straight-line segment tangent to both the initial circle and the target circle.
3. Final arc movement around a circle centered at the target point, assuming clockwise movement.

At every simulation step, the UAV moves forward along the planned path, adjusting its position and heading accordingly.

> **Note:** The algorithm dynamically computes the direction of turning (clockwise or counterclockwise) based on the UAV’s current heading and the position of the target.

> **Important:** The code currently includes some duplicated calculations that are unnecessarily recomputed every simulation step — such as turning circle centers, tangent line parameters, and rotation directions. These could be calculated once per maneuver to improve performance and code clarity. Refactoring these aspects can help optimize the system for future development.

## Project Structure

- `main.cpp`: Entry point for the simulation.
- `Simulation.h/cpp`: Handles loading parameters, running the main simulation loop, and applying commands to UAVs.
- `UAV.h/cpp`: Manages UAV state, path computation, and movement logic.
- `Utils.h/cpp`: Contains helper functions, such as geometric calculations and angle normalization.
- `Command.h`: Defines the structure of commands used to direct UAVs during simulation.
- `input/SimParams.ini`: Defines simulation parameters (initial position, velocity, turning radius, etc.).
- `input/SimCmds.txt`: List of commands for UAVs, including time and target coordinates.
- `output/`: Folder where each UAV’s movement log is saved as a separate `.txt` file.

## Input Format

**SimParams.ini**

```
Dt=0.01
N_uav=1
R=100
X0=0
Y0=0
Z0=0
V0=30
Az=45
TimeLim=30
```

**SimCmds.txt**

```
5 0 300 400
15 0 600 100
```

Each line represents:

```
<time> <UAV_ID> <targetX> <targetY>
```

## Output

The simulation writes the position and heading (azimuth) of each UAV to a file named `UAV<ID>.txt` in the `output/` folder. Each line contains:

```
<time> <x> <y> <azimuth>
```

Example:

```
0.00 0.00 0.00 45.00
0.01 0.21 0.21 45.00
...
```

## Path Planning Logic (Clarified)

The system computes a flight path from the UAV’s current position to the target point by:

- Calculating the initial turning circle based on the UAV's current heading and minimum turning radius.
- Creating a second circle centered at the target point, assuming clockwise direction.
- Finding a straight-line segment that is tangent to both circles.
- The UAV follows this path: arc → straight → arc.

This avoids explicitly referencing the Dubins path and provides an intuitive geometric explanation of the behavior.

## Known Limitations and Improvement Opportunities

- **Redundant Computations:** Many geometric values are re-calculated at every time step, though they only need to be computed once per maneuver.
- **Code Duplication:** Logic for computing directions, angles, and circle centers appears in multiple locations and can be centralized.
- **Modularity:** Refactoring and separating responsibilities more cleanly could enhance the scalability and testability of the system.

## Possible Future Extensions

- 3D flight support using the Z0 parameter
- Collision avoidance between multiple UAVs
- Real-time visualization using tools like Python/Matplotlib
- GUI interface for simulation control
- Path smoothing using parametric curves (e.g., Bézier curves)

## How to Run

1. Compile the project using a C++ compiler (e.g., g++):

```
g++ main.cpp UAV.cpp Simulation.cpp utils.cpp -o uav_simulation
```

2. Create the following folder structure:

```
input/
  └── SimParams.ini
  └── SimCmds.txt
output/
```

3. Run the simulation:

```
./uav_simulation
```

4. Check the `output/` folder for simulation logs.
