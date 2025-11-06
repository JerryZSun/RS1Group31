# Obstacle Avoidance Implementation - Fixed

## Overview
The drone navigation system now includes intelligent obstacle avoidance with proper state management and two protocols:
1. **Lateral Avoidance** (fly-around): Navigate around obstacles via clear side paths
2. **Vertical Avoidance** (fly-over): Climb over obstacles when no lateral path exists

## Critical Fixes Applied

### Problem 1: Navigation Deadlock ✅ FIXED
**Issue**: Once obstacle detected, navigation entered `OBSTACLE_DETECTED` state and never exited because:
- Obstacle still present
- `scan_callback` only checks obstacles in `MOVING` state
- Created infinite loop

**Solution**:
- Navigation sends **direction** (yaw angle), not waypoint
- When mission sends avoidance waypoint, navigation exits `OBSTACLE_DETECTED` 
- Temporarily ignores obstacles for 3.5m after avoidance (prevents immediate re-detection)
- Separate emergency stop threshold (1m) always active

### Problem 2: Immediate Re-triggering ✅ FIXED
**Issue**: After avoidance, drone would immediately detect same obstacle again

**Solution**:
- `ignore_obstacles_` flag enabled after receiving avoidance waypoint
- Tracks distance traveled via odometry
- Re-enables detection after 3.5m traveled
- Provides buffer to clear obstacle zone

### Problem 3: Verbose Logging ✅ FIXED
**Issue**: Periodic status updates every 2-3 seconds cluttered console

**Solution**:
- Navigation: Still has 2-second status (useful for monitoring)
- Mission: **Event-based only** - logs only on state changes, waypoint changes, avoidance triggers

## System Architecture

### Navigation Node (`navigation.cpp`)

**Responsibilities:**
- Detect obstacles using LIDAR
- Calculate optimal avoidance direction (not waypoint)
- Send direction to mission for waypoint calculation
- Exit obstacle state when receiving avoidance waypoint
- Emergency stop for close obstacles (<1m)

**Key Topics Published:**
- `/navigation/avoidance_direction` (Vector3) - Contains: `x=yaw (rad), y=distance (m), z=yaw_deviation (rad)`
- `/navigation/no_clear_path` (Bool) - Triggers fly-over protocol
- `/navigation/waypoint_reached` (Bool) - Waypoint completion

**Obstacle Detection Parameters:**
```cpp
normal_obstacle_threshold_    = 2.0m   // Normal detection
emergency_stop_threshold_     = 1.0m   // Emergency stop
clear_path_distance_          = 6.0m   // Required clear distance
obstacle_margin_              = 1.0m   // Margin from obstacles
avoidance_ignore_distance_    = 3.5m   // Temp ignore after avoidance
```

**Console Output:**
```
# On waypoint received
Waypoint received: (6.00, 5.00, 0.60)

# Every 2 seconds during navigation
Status: MOVING | Pos:(5.24,4.18,0.60) | Target:(6.00,5.00,0.60) | Dist:1.11m | Alt_err:0.00m | Yaw_err:-4.5°

# During temporary obstacle ignore
Status: MOVING | ... [Ignoring obstacles: 2.3m remaining]

# On obstacle detection
OBSTACLE DETECTED at (5.24, 4.18, 0.60) - Analyzing avoidance options...
Avoidance direction: yaw=65.3°, dist=6.0m, deviation=20.1°

# On receiving avoidance waypoint (exits OBSTACLE_DETECTED)
Avoidance waypoint received: (10.57, 6.92, 0.60) - Resuming navigation
Obstacle detection temporarily disabled for 3.5m

# After traveling 3.5m
Traveled 3.51m - Re-enabling obstacle detection
```

### Mission Node (`mission.cpp`)

**Responsibilities:**
- Manage mission waypoint sequence
- Calculate avoidance waypoints from direction
- Coordinate avoidance protocols (lateral/vertical)
- Track avoidance attempt counter
- Generate fly-over waypoints

**States:**
- `NORMAL_MISSION` - Following predefined waypoints
- `AVOIDING_OBSTACLE` - Executing lateral avoidance
- `FLYING_OVER` - Executing vertical avoidance (2 stages)

**Console Output** (Event-based only):
```
# Mission start
Mission node initialized with 16 waypoints
Navigation node connected - Starting mission
═══════════════════════════════════════════════════════
➤ Mission waypoint 1/16: (0.00, 0.00, 0.10)

# Waypoint completion
Navigation confirmed waypoint reached
─────────────────────────────────────────────────
Progress: Mission waypoint 1/16 completed
➤ Mission waypoint 2/16: (0.00, 0.00, 7.00)

# Obstacle avoidance
Obstacle avoidance request 1/2 - Using lateral avoidance
Calculated avoidance waypoint: (10.57, 6.92, 0.60) at yaw=65.3° (dev=20.1°)
State: NORMAL → AVOIDING | Storing mission waypoint 11
Avoidance waypoint sent to navigation

# Avoidance complete
Navigation confirmed waypoint reached
Avoidance waypoint reached - Resuming mission
State: AVOIDING → NORMAL | Resuming mission waypoint 11
➤ Mission waypoint 11/16: (6.00, 5.00, 0.60)

# Fly-over trigger
No clear lateral path available - Initiating FLY-OVER protocol
State: NORMAL → FLYING_OVER | Storing mission waypoint 11
Fly-over Stage 1/2: Climb to (5.24, 4.18, 7.00)

# Mission complete
═══════════════════════════════════════════════════════
🎉 MISSION COMPLETE: All 16 waypoints reached!
═══════════════════════════════════════════════════════
```

## Avoidance Protocols

### Protocol 1: Lateral Avoidance (Fly-Around)

**Trigger:** Obstacle detected AND clear path found within ±90° arc

**Flow:**
```
1. Navigation detects obstacle (MOVING state, aligned with target, >5 detections in ±20° cone)
   ↓
2. Navigation analyzes LIDAR ±90° arc
   - Finds directions with ≥6m clear distance
   - Filters out directions near obstacles (<1m margin)
   - Selects direction with minimum yaw deviation from target
   ↓
3. Navigation publishes avoidance_direction (yaw, distance, deviation)
   - Enters OBSTACLE_DETECTED state (stops moving)
   ↓
4. Mission receives direction
   - Calculates waypoint: current_pos + distance * (cos(yaw), sin(yaw))
   - Stores current mission waypoint
   - Enters AVOIDING state
   - Publishes avoidance waypoint
   ↓
5. Navigation receives avoidance waypoint
   - Exits OBSTACLE_DETECTED state
   - Enables ignore_obstacles_ flag
   - Enters TURNING state (aligns with avoidance waypoint)
   - Starts tracking distance traveled
   ↓
6. Navigation navigates to avoidance waypoint
   - Obstacles ignored for first 3.5m
   - After 3.5m, re-enables obstacle detection
   ↓
7. Avoidance waypoint reached
   - Mission publishes stored mission waypoint
   - Mission enters NORMAL state
   ↓
8. If still blocked: Repeat (max 2 lateral attempts, then fly-over)
```

### Protocol 2: Vertical Avoidance (Fly-Over)

**Trigger:** 
- No clear lateral path (no direction with ≥6m clear within ±90°)
- OR max lateral attempts reached (2)

**Process:**
```
Stage 0 (Climb):
- Waypoint: current_pos + 5m altitude (min 7m, max 10m)
- Wait for waypoint_reached
- Obstacle detection temporarily disabled

Stage 1 (Move Above):
- Waypoint: target_waypoint (x,y) at current_altitude
- Maintains elevated altitude
- Wait for waypoint_reached

Stage 2 (Descend):
- Waypoint: target_waypoint at original altitude
- Descends to continue mission
- Resets avoidance attempt counter
```

## Testing

### Build
```bash
cd ~/trail_patrol_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### Launch
```bash
# Obstacle world
ros2 launch trail_patrol_drone trail_patrol_drone_obstacle.launch.py slam:=true rviz:=true
```

### Run Nodes
```bash
# Terminal 1: Navigation
source ~/trail_patrol_ws/install/setup.bash
ros2 run trail_patrol_drone navigation

# Terminal 2: Mission
source ~/trail_patrol_ws/install/setup.bash
ros2 run trail_patrol_drone mission
```

### Expected Behavior at Obstacle

**Before (Waypoint 11 → 12, obstacle at 7,6.25,0.6):**
```
[Navigation] Status: MOVING | Pos:(5.24,4.18,0.60) | Target:(6.00,5.00,0.60) | Dist:1.11m
[Navigation] OBSTACLE DETECTED at (5.24, 4.18, 0.60) - Analyzing avoidance options...
[Navigation] Avoidance direction: yaw=65.3°, dist=6.0m, deviation=20.1°
[Mission] Obstacle avoidance request 1/2 - Using lateral avoidance
[Mission] Calculated avoidance waypoint: (10.57, 6.92, 0.60) at yaw=65.3°
[Mission] State: NORMAL → AVOIDING | Storing mission waypoint 11
[Navigation] Avoidance waypoint received: (10.57, 6.92, 0.60) - Resuming navigation
[Navigation] Obstacle detection temporarily disabled for 3.5m
[Navigation] Heading aligned, transitioning to MOVING
[Navigation] Status: MOVING | ... [Ignoring obstacles: 2.8m remaining]
[Navigation] Traveled 3.51m - Re-enabling obstacle detection
[Navigation] Waypoint reached at (10.57, 6.92, 0.60)
[Mission] Avoidance waypoint reached - Resuming mission
[Mission] State: AVOIDING → NORMAL | Resuming mission waypoint 11
[Navigation] Waypoint received: (6.00, 5.00, 0.60)
```

**If path still blocked (2nd attempt):**
- Same flow, mission increments avoidance_attempt_count_ to 2
- After 2nd avoidance completes, logs warning about next obstacle triggering fly-over

**If no clear path (fly-over):**
```
[Navigation] No clear lateral path (need 6.0m clear) - Requesting fly-over
[Mission] No clear lateral path available - Initiating FLY-OVER protocol
[Mission] State: NORMAL → FLYING_OVER | Storing mission waypoint 11
[Mission] Fly-over Stage 1/2: Climb to (5.24, 4.18, 7.00)
[Navigation] Waypoint received: (5.24, 4.18, 7.00)
... [climbs] ...
[Mission] Fly-over Stage 1 complete - Moving to above target
[Mission] Fly-over Stage 2/2: Move above target to (6.00, 5.00, 7.00)
... [moves above obstacle] ...
[Mission] Fly-over Stage 2 complete - Descending to mission altitude
[Mission] State: FLYING_OVER → NORMAL | Resuming mission waypoint 11
[Navigation] Waypoint received: (6.00, 5.00, 0.60)
```

## Monitoring Topics

```bash
# Watch avoidance direction
ros2 topic echo /navigation/avoidance_direction

# Watch mission waypoints
ros2 topic echo /mission/current_waypoint

# Monitor status
ros2 topic echo /navigation/waypoint_reached
ros2 topic echo /navigation/no_clear_path
```

## Key Improvements

1. **No More Deadlock**: Navigation exits OBSTACLE_DETECTED when receiving avoidance waypoint
2. **No Re-triggering**: Temporary obstacle ignore for 3.5m after avoidance
3. **Clean Architecture**: Navigation does sensing/control, Mission does planning
4. **Event-Based Logging**: Mission only logs on changes (much cleaner console)
5. **Emergency Stop**: Always active regardless of ignore flag (<1m threshold)
6. **Proper State Transitions**: Clear state machine with explicit transitions logged

## Troubleshooting

**Drone still frozen:**
- Check navigation received avoidance waypoint: `ros2 topic echo /mission/current_waypoint`
- Verify ignore flag enabled: Look for "Obstacle detection temporarily disabled" message
- Check distance tracking: Should see "Ignoring obstacles: Xm remaining" in status

**Obstacle detection too sensitive:**
- Adjust `normal_obstacle_threshold_` (currently 2m)
- Adjust detection count requirement (currently ≥5 points)

**Avoidance too aggressive:**
- Increase `clear_path_distance_` (currently 6m)
- Decrease `avoidance_ignore_distance_` (currently 3.5m)

**Emergency stops happening:**
- Check for obstacles <1m
- Adjust `emergency_stop_threshold_` if needed