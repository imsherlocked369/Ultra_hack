.

## Overview

This code does intelligent exploration algorithm for autonomous aerial vehicles  to:
- Systematically search a defined area using a grid-based exploration strategy
- Detect candidates (false positives and target buildings) using simulated perception
- Confirm target buildings through stabilized multi-frame detection
- Optimize flight paths to minimize travel distance while maximizing search coverage
- Track mission metrics including distance traveled, time, and detection statistics

## Features

- **Next-Best-View (NBV) Planning**: Intelligently selects viewpoints that maximize unexplored areas while minimizing travel distance
- **Grid-Based Exploration**: Divides search area into cells to track visited and visible regions
- **Rejection Zone Management**: Maintains exclusion zones around false candidates to avoid repeated inspections
- **MAVLink Integration**: Communicates with drone flight controllers via MAVLink protocol
- **Performance Metrics**: Logs detailed mission statistics to CSV for analysis
- **Simulated Perception**: Tests algorithm without physical hardware using configurable object detection

## System Architecture

### Core Components

1. **FlightClient**: Interfaces with drone hardware via MAVLink
   - Mode control (GUIDED, LAND, RTL)
   - Arming and takeoff operations
   - Local position tracking and navigation
   - Landing and Return-to-Launch (RTL) commands

2. **ExplorationGrid**: Manages search space state
   - Tracks visited and visible grid cells
   - Maintains rejection zones around false candidates
   - Provides cell-based spatial queries

3. **NBVPlanner**: Computes optimal viewpoints
   - Samples candidate viewpoints across search radius
   - Selects best yaw orientation for maximum information gain
   - Scores viewpoints based on coverage, distance, and rejection penalties

4. **SimulatedPerception**: Simulates object detection and confirmation
   - Detects candidates within camera range and field-of-view
   - Confirms target building identity through multi-frame validation

### Configuration Parameters

```python
TAKEOFF_ALTITUDE = 5.0                    # Takeoff altitude (meters)
SEARCH_RADIUS_M = 60                      # Search area radius (meters)
GRID_CELL_SIZE_M = 5                      # Grid cell size (meters)
CAMERA_RANGE_M = 25                       # Camera detection range (meters)
CAMERA_FOV_DEG = 70                       # Camera field of view (degrees)
REJECTION_RADIUS_M = 8                    # Rejection zone radius (meters)
MISSION_TIME_LIMIT_SEC = 900              # Maximum mission duration (15 minutes)
VIEWPOINT_STEP_M = 8                      # Spacing between sampled viewpoints (meters)
ALTITUDE_M = 5.0                          # Operating altitude (meters)
ARRIVAL_RADIUS_M = 2.5                    # Target arrival tolerance (meters)
MAX_MOVE_WAIT_SEC = 60                    # Maximum time to reach waypoint (seconds)
```

## Simulated Objects

The system can search for three types of objects:

```python
SIM_OBJECTS = [
    {"id": "F1", "xy": (15, 10), "type": "false_cuboid"},      # False positive 1
    {"id": "F2", "xy": (-20, 15), "type": "false_cuboid"},     # False positive 2
    {"id": "B1", "xy": (35, -20), "type": "target_building"},  # Target building
]
```

## Mission Flow

1. **Initialize**: Connect to drone via MAVLink, set GUIDED mode, arm motors
2. **Takeoff**: Climb to configured altitude
3. **Explore Loop**:
   - Get current position from drone
   - Select next best viewpoint using NBV planner
   - Navigate to viewpoint
   - Mark visited cells and update visible regions
   - Scan for candidate detections
   - If candidate detected:
     - Approach for closer inspection (if beyond safe distance)
     - Confirm candidate identity through multi-frame validation
     - If target building: Land and end mission
     - If false positive: Add to rejection zones and continue
4. **Terminate**: Land (if target found) or Return-to-Launch (RTL)
5. **Report**: Write mission metrics to CSV file

## Output

### Mission Metrics (explore_area_metrics.csv)

The system generates a CSV file with the following metrics:

| Metric | Description |
|--------|-------------|
| mission_time_sec | Total mission duration |
| total_distance_m | Total distance traveled |
| viewpoints_visited | Number of exploration waypoints |
| false_candidates_inspected | Count of false positives investigated |
| repeated_visits_to_rejected_candidates | Inefficiency metric (avoided re-visits) |
| rejected_objects | Total false positives confirmed |
| target_building_found | Mission success flag |
| confirmed_target_id | ID of confirmed target building |

## Usage

### Prerequisites

- Python 3.7+
- pymavlink library: `pip install pymavlink`
- MAVLink-compatible drone with connection string (e.g., UDP at 14551)

### Running the System

```bash
python explore_area_test.py
```

The system will:
1. Connect to the drone on `udpin:0.0.0.0:14551`
2. Execute autonomous exploration
3. Generate `explore_area_metrics.csv` with results

### Simulation Mode

The current implementation uses `SimulatedPerception` with predefined objects. For testing without hardware:
- Update `MAVLINK_CONNECTION` to point to a simulator (e.g., SITL)
- Modify `SIM_OBJECTS` to test different scenarios

## Algorithm Details

### Viewpoint Scoring

Each candidate viewpoint is scored using:

```
score = 3.0 * visible_unknown - 2.0 * travel_distance - 5.0 * rejected_penalty
```

Where:
- **visible_unknown**: Number of unexplored cells visible from this viewpoint
- **travel_distance**: Distance from current position to viewpoint
- **rejected_penalty**: 1 if in rejection zone, 0 otherwise

The planner selects the viewpoint with the highest score.

### Camera Coverage

The camera detects objects when all conditions are met:
- Object is within `CAMERA_RANGE_M` meters
- Object is within `CAMERA_FOV_DEG / 2` degrees of drone yaw direction
- Object has not been rejected

### Confirmation Logic

Target buildings require 3 consecutive stable frames for confirmation before mission termination.

## Data Structures

### Viewpoint
```python
@dataclass
class Viewpoint:
    x: float            # X coordinate
    y: float            # Y coordinate
    yaw_deg: float      # Yaw orientation (degrees)
    score: float        # Selection score
```

### Metrics
```python
@dataclass
class Metrics:
    start_time: float
    total_distance: float
    viewpoints_visited: int
    false_candidates_inspected: int
    repeated_visits_to_rejected_candidates: int
    rejected_objects: int
    target_building_found: bool
    confirmed_target_id: Optional[str]
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "Heartbeat not received" | Check MAVLink connection string and drone power |
| Mission exceeds time limit | Reduce `SEARCH_RADIUS_M` or increase `VIEWPOINT_STEP_M` |
| Repeated visits to rejected zones | Algorithm is working correctly; monitor `repeated_visits_to_rejected_candidates` |
| No candidate detected | Verify `SIM_OBJECTS` locations and camera parameters |

## Performance Optimization

- **Larger grid cells**: Increases exploration speed but reduces precision
- **Fewer viewpoint samples**: Faster planning but may miss optimal paths
- **Higher rejection radius**: More conservative, may skip areas
- **Camera range**: Balance between coverage and accuracy

## Future Enhancements

- Real-time object classification with neural networks
- Dynamic mission replanning based on environmental changes
- Multi-drone coordination for cooperative search
- Probability-based uncertainty mapping
- Integration with real camera feed

## License

[Add appropriate license information]

## Contact

For questions or contributions, please contact the development team.
