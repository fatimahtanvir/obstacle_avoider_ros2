# VFH Algorithm — Technical Breakdown

## Overview

The **Vector Field Histogram (VFH)** algorithm was introduced by Borenstein and Koren in 1991 and remains one of the most widely used reactive obstacle avoidance algorithms in mobile robotics. This implementation uses a simplified single-stage VFH on a 2D LIDAR scan.

---

## How it works — step by step

### Step 1 — Raw sensor data
The LIDAR publishes a `sensor_msgs/LaserScan` message containing:
- 360 range readings (one per degree)
- Each reading = distance to nearest obstacle in that direction
- Range: 0.12m to 8.0m
- Invalid readings (walls too close or too far) set to `range_max`

### Step 2 — Polar obstacle density histogram
We divide the 360° field of view into **36 bins of 10° each**.

For each laser reading `r` at angle `a`:
```
bin_index = floor((a + π) / bin_width) mod 36
if r < warning_distance:
    hist[bin_index] = max(hist[bin_index], (warning_dist - r) / warning_dist)
```

This gives each bin a danger value from 0.0 (completely clear) to 1.0 (obstacle right there).

### Step 3 — Threshold to find open valleys
Apply a threshold (0.3) to the histogram:
```
open_bins = hist < 0.3
```
Consecutive open bins form "valleys" — safe directions to travel.

### Step 4 — Pick the best valley
Starting from the forward direction (bin 18 = 0°), search outward in both directions for the nearest open bin:
```
for offset in 1..18:
    check bin (forward + offset) % 36
    check bin (forward - offset) % 36
```
Pick whichever open bin is closest to forward.

### Step 5 — Compute steering command
Convert the best bin index back to an angle error and publish a `Twist` command:
```
angle_error = ((best_bin - forward_bin) / n_bins) * 2π
angular_z = clip(-angle_error * 1.5, -max_speed, +max_speed)
linear_x  = linear_speed * 0.6  (reduced while turning)
```

---

## State machine

The state machine sits on top of VFH and handles edge cases:

```
         ┌─────────────────┐
    ┌───▶│   EXPLORING     │◀───┐
    │    │  Drive forward  │    │
    │    └────────┬────────┘    │
    │             │ obstacle    │
    │             ▼ detected    │
    │    ┌─────────────────┐    │ clear
    │    │    AVOIDING     │────┘
    │    │  Steer to gap   │
    │    └────────┬────────┘
    │             │ stuck 3x
    │             ▼
    │    ┌─────────────────┐
    └────│   RECOVERING    │
         │ Reverse + spin  │
         └─────────────────┘
```

**EXPLORING** — No obstacles within warning distance. Drive straight at full speed.

**AVOIDING** — Obstacle within warning distance. Reduce speed, steer toward best open valley. Prefer the side with more clearance (compare min_left vs min_right).

**RECOVERING** — Robot has been in AVOIDING state 3+ consecutive times without escaping. Reverse slightly and spin aggressively for `recovery_time` seconds, then return to EXPLORING.

---

## Why VFH over simpler methods?

| Method | Problem |
|--------|---------|
| Simple threshold (stop if front blocked) | Gets stuck in corners, oscillates |
| Bug algorithm | Slow, follows walls inefficiently |
| **VFH** | Smooth steering, handles complex environments, computationally cheap |

---

## Limitations and improvements

- **Single layer** — this is a 2D implementation. A 3D point cloud (e.g. depth camera) would catch overhanging obstacles.
- **No memory** — VFH is purely reactive. Adding a costmap layer would let it remember where it has been.
- **Local minima** — the robot can get trapped in U-shaped environments. A global planner (Nav2) solves this.
- **Fixed threshold** — the 0.3 danger threshold could be adaptive based on robot speed.

---

## References

- Borenstein, J., & Koren, Y. (1991). *The vector field histogram — fast obstacle avoidance for mobile robots*. IEEE Transactions on Robotics and Automation.
- Ulrich, I., & Borenstein, J. (1998). *VFH+: Reliable obstacle avoidance for fast mobile robots*. ICRA.
