# Trajectory Visualization Guide for Simulation

This guide explains how to test and visualize the trajectory-trench intersection feature in simulation using AdvantageScope.

## Overview

The FlywheelCommand now publishes comprehensive trajectory data to NetworkTables that can be visualized in AdvantageScope. You'll be able to see:
- Robot's current position (from odometry)
- Projected future position (0.8 seconds ahead)
- Trajectory line (10 points showing the path)
- All 4 trench boundaries with tolerance
- Which trenches the trajectory intersects

---

## Running Simulation

### 1. Start Robot Simulation

```bash
./gradlew simulateJava
```

Or use your IDE's simulation button.

### 2. Enable Keyboard Control

The simulation GUI should allow you to drive the robot with keyboard controls. Typical controls:
- **WASD** or **Arrow Keys**: Drive the robot
- **Q/E**: Rotate

Make sure the FlywheelCommand is running (it should be the default command for the flywheel subsystem or triggered somehow).

### 3. Connect AdvantageScope

- Open AdvantageScope
- Connect to **localhost** (or 127.0.0.1)
- The NetworkTables data will start streaming

---

## AdvantageScope Visualization Setup

### Data Available in NetworkTables

All trajectory data is published under the `Trajectory/` prefix:

| Key | Type | Description |
|-----|------|-------------|
| `Trajectory/current_x` | Number | Current robot x-position (meters) |
| `Trajectory/current_y` | Number | Current robot y-position (meters) |
| `Trajectory/projected_x` | Number | Projected x-position after 0.8s |
| `Trajectory/projected_y` | Number | Projected y-position after 0.8s |
| `Trajectory/velocity_x` | Number | Robot x-velocity (m/s) |
| `Trajectory/velocity_y` | Number | Robot y-velocity (m/s) |
| `Trajectory/velocity_magnitude` | Number | Total velocity magnitude |
| `Trajectory/trajectory_line` | String | Pose array (10 points) |
| `Trajectory/projected_pose` | String | Final projected pose |
| `Trajectory/intersects_trench` | Boolean | TRUE if any intersection |
| `Trajectory/intersects_blue_left` | Boolean | Blue left trench intersection |
| `Trajectory/intersects_blue_right` | Boolean | Blue right trench intersection |
| `Trajectory/intersects_red_left` | Boolean | Red left trench intersection |
| `Trajectory/intersects_red_right` | Boolean | Red right trench intersection |
| `Trajectory/trench_blue_left` | Number[] | [xMin, yMin, xMax, yMax] |
| `Trajectory/trench_blue_right` | Number[] | [xMin, yMin, xMax, yMax] |
| `Trajectory/trench_red_left` | Number[] | [xMin, yMin, xMax, yMax] |
| `Trajectory/trench_red_right` | Number[] | [xMin, yMin, xMax, yMax] |

---

## Creating Visualizations in AdvantageScope

### Method 1: 2D Field View (Recommended)

1. **Add a 2D Field widget**
   - Click "Add Widget" → "2D Field"

2. **Add Robot Pose**
   - Find your odometry pose topic (usually `/SmartDashboard/Field/Robot` or similar)
   - Drag it to the 2D Field
   - Set color to blue

3. **Add Current Position Point**
   - Create a custom pose from `Trajectory/current_x` and `Trajectory/current_y`
   - Display as a small circle/marker
   - Color: Green

4. **Add Projected Position Point**
   - Create a custom pose from `Trajectory/projected_x` and `Trajectory/projected_y`
   - Display as a larger circle or different shape
   - Color: Orange or Red (depending on `Trajectory/intersects_trench`)

5. **Add Trajectory Line**
   - Use `Trajectory/trajectory_line` (if AdvantageScope supports string pose arrays)
   - Or manually plot using the individual points
   - Color: Yellow or Cyan
   - Line style: Dashed

6. **Add Trench Boundaries** (Optional)
   - Use the `Trajectory/trench_*` number arrays to draw rectangles
   - Create 4 separate rectangle overlays
   - Color: Red with transparency
   - Fill: Semi-transparent red

### Method 2: Line Graph View

For debugging the algorithm logic:

1. **Create a Line Graph widget**

2. **Add velocity components:**
   - `Trajectory/velocity_x` (Blue)
   - `Trajectory/velocity_y` (Green)
   - `Trajectory/velocity_magnitude` (Red)

3. **Add position tracking:**
   - `Trajectory/current_x`
   - `Trajectory/current_y`
   - `Trajectory/projected_x`
   - `Trajectory/projected_y`

### Method 3: Boolean Indicators

Create a table or boolean indicator panel:

- `Trajectory/intersects_trench` (Main indicator)
- `Trajectory/intersects_blue_left`
- `Trajectory/intersects_blue_right`
- `Trajectory/intersects_red_left`
- `Trajectory/intersects_red_right`

This will light up RED when the trajectory intersects any trench.

---

## Testing Scenarios

### Test 1: Stationary Inside Trench
1. Teleop the robot into a trench region
2. Stop moving
3. **Expected:** `intersects_trench` should be TRUE (already inside)

### Test 2: Approaching Trench Head-On
1. Position robot outside a trench
2. Drive directly toward the trench
3. **Expected:** As you get closer, `intersects_trench` should turn TRUE when within 0.8s of crossing

### Test 3: Parallel Motion (No Intersection)
1. Position robot parallel to a trench edge
2. Drive parallel (not crossing)
3. **Expected:** `intersects_trench` should remain FALSE

### Test 4: Diagonal Crossing
1. Approach a trench at an angle
2. Watch the projected position marker
3. **Expected:** Should detect intersection when diagonal path crosses trench

### Test 5: Fast vs Slow Movement
1. Drive slowly toward trench → Should detect later (closer to trench)
2. Drive quickly toward trench → Should detect earlier (further from trench)
3. **Reason:** 0.8s at high speed covers more distance

---

## Interpreting the Visualization

### Colors (Suggested Scheme)

- **Green Dot**: Current robot position
- **Orange/Red Dot**: Projected position after 0.8s
- **Yellow Dashed Line**: Trajectory path
- **Red Rectangles**: Trench regions (with tolerance)
- **Blue**: Normal operation (no intersection)
- **Red**: Danger zone (intersection detected)

### What to Look For

1. **Trajectory Line**: Should point in the direction of motion, length proportional to velocity
2. **Projected Position**: Should be at the end of the trajectory line
3. **Intersection Detection**: When the line or projected point enters a red rectangle, the boolean should flip TRUE
4. **Flywheel Behavior**: Check that flywheel stops when `intersects_trench` is TRUE

---

## Debugging Tips

### If Visualization Isn't Showing Up

1. **Check NetworkTables Connection**
   - Verify AdvantageScope is connected to robot/sim
   - Look for `/SmartDashboard/Trajectory/` topics

2. **Verify FlywheelCommand is Running**
   ```bash
   # Check SmartDashboard for any Trajectory/* entries
   ```

3. **Check Console for Errors**
   - Look for exceptions in robot code
   - Verify imports are correct

### If Detection Seems Wrong

1. **Log More Data**
   - Add print statements in `trajectoryIntersectsTrench()`
   - Print the calculated `t` values for each boundary

2. **Verify Trench Coordinates**
   - Check that `Trajectory/trench_*` arrays match expected field dimensions
   - Field is 16.54m × 8.07m

3. **Test Edge Cases**
   - Stationary robot (velocity = 0)
   - Very slow motion
   - Backward motion (negative velocity)

---

## Advanced: Custom AdvantageScope Layout

Save this as a `.json` layout file for quick loading:

```json
{
  "tabs": [
    {
      "name": "Trajectory Test",
      "widgets": [
        {
          "type": "2D Field",
          "poses": [
            "/SmartDashboard/Field/Robot",
            "Custom:Trajectory/projected_pose"
          ]
        },
        {
          "type": "Boolean Array",
          "keys": [
            "/SmartDashboard/Trajectory/intersects_trench",
            "/SmartDashboard/Trajectory/intersects_blue_left",
            "/SmartDashboard/Trajectory/intersects_blue_right",
            "/SmartDashboard/Trajectory/intersects_red_left",
            "/SmartDashboard/Trajectory/intersects_red_right"
          ]
        },
        {
          "type": "Line Graph",
          "keys": [
            "/SmartDashboard/Trajectory/velocity_x",
            "/SmartDashboard/Trajectory/velocity_y",
            "/SmartDashboard/Trajectory/velocity_magnitude"
          ]
        }
      ]
    }
  ]
}
```

---

## Field Dimensions Reference

```
Field Size: 16.54m × 8.07m
Coordinate Origin: Bottom-left corner

Trenches (with 0.5m tolerance):
┌────────────────────────────────────┐
│                                    │
│  [BL]        [BR]  [RL]      [RR]  │
│  Blue        Blue   Red       Red  │
│  Left        Right  Left     Right │
│                                    │
└────────────────────────────────────┘

BL: x=[0.567, 3.869], y=[1.429, 4.534]
BR: x=[4.126, 7.791], y=[0.628, 3.938]
RL: x=[8.749,12.414], y=[4.131, 7.441]
RR: x=[12.671,15.973], y=[3.534, 6.639]
```

---

## Example Test Session

```bash
# Terminal 1: Start simulation
./gradlew simulateJava

# Terminal 2: Open AdvantageScope
advantagescope

# In AdvantageScope:
# 1. Connect to localhost
# 2. Open 2D Field view
# 3. Add Trajectory/current_x, current_y as a point
# 4. Add Trajectory/projected_x, projected_y as a point
# 5. Watch Trajectory/intersects_trench boolean

# Drive the robot around with keyboard
# - Drive toward trenches
# - Watch projected position
# - Verify detection triggers at correct time
```

---

## Questions or Issues?

The trajectory intersection algorithm uses boundary crossing analysis:
- For each trench boundary (4 edges per trench)
- Solve: when does robot cross this boundary? (time `t`)
- Check: is `0 ≤ t ≤ 0.8s`?
- Check: is perpendicular coordinate within trench bounds?

See `TRAJECTORY_TECHNICAL_REPORT.md` for detailed mathematical explanation.
