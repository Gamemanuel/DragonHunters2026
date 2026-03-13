# Limelight 3A Setup Guide – Team 10380 (2026 REBUILT Season)

This guide covers the complete configuration of a Limelight 3A for hub AprilTag
detection on the 2026 REBUILT FRC field.  Follow every step before running the
flywheel subsystem on a real robot.

---

## 1. Hardware Requirements

| Item | Notes |
|------|-------|
| Limelight 3A | Mount facing the hub |
| Power (8–12 V) | Wire directly from PDP/PDH (recommended: fused at 5 A) |
| USB-C cable | Connect Limelight USB-C port → roboRIO 2.0 USB-A port (primary connection) |
| Ethernet cable | Optional: connect to radio/switch for web UI access during setup |

### USB vs Ethernet connection

The Limelight 3A supports two connection modes for FRC:

- **USB-C → roboRIO** (Team 10380 configuration): The Limelight creates a USB gadget network interface on the roboRIO.  The Limelight is available at `172.22.11.2` and publishes NetworkTables data automatically.  No robot-side network configuration is needed — `ntcore` discovers the Limelight over the USB interface at startup.
- **Ethernet → radio/switch**: Requires static IP assignment (see Section 2).  Preferred when you want to access the Limelight web UI wirelessly during a match.

Both modes publish identical NetworkTable keys (`tv`, `tid`, `botpose_targetspace`, etc.) so **no robot code changes** are needed to switch between them.

### Recommended Mounting Position

- **Height**: ~0.5 m above the floor (adjust to maximize hub tag visibility).
- **Tilt (pitch)**: Angled slightly upward so the hub tags are centred in the
  frame at the expected shooting distance range.
- **Horizontal centre**: Mount on the robot's shooting axis so the camera is as
  close to centre as possible for the simplest geometry.

Record the mount height and pitch angle – you will need them during calibration.

---

## 2. Network Configuration (Ethernet mode only)

> **Skip this section if using the USB-C → roboRIO connection.**  The USB
> interface assigns `172.22.11.2` to the Limelight automatically — no static
> IP configuration is required.

If connecting the Limelight via Ethernet (e.g. for wireless web UI access),
team 10380's robot network uses the `10.103.80.x` subnet.

### Steps

1. Connect a laptop directly to the Limelight via Ethernet (or through the robot radio).
2. Open a browser and navigate to `http://172.22.11.2:5801` when connected via
   USB, or the current Limelight DHCP address when connected via Ethernet.
3. Go to **Settings → Networking**.
4. Set:
   - **IP assignment**: Static
   - **IP address**: `10.103.80.11`
   - **Netmask**: `255.255.255.0`
   - **Gateway**: `10.103.80.1`
5. Click **Save** and reboot the Limelight.

> **Note**: The NetworkTable name defaults to `limelight`.  If you mount a
> second Limelight on the robot, rename each unit in **Settings → NetworkTables**
> and update `LIMELIGHT_TABLE_NAME` in `constants.py` accordingly.

---

## 3. AprilTag Pipeline Configuration

### 3.1 Create a new pipeline

1. Open the Limelight web interface.
2. Go to **Pipelines** and click **+ New Pipeline**.
3. Select **AprilTag** as the pipeline type.
4. Name it `hub` (or similar).
5. Set this as the **active pipeline** (pipeline index 0).

### 3.2 AprilTag family and field layout

1. In the pipeline settings, set:
   - **Tag family**: `36h11` (standard for FRC 2024 / 2025 / 2026)
   - **Tag layout**: Upload or select the 2026 REBUILT field JSON.
     - The official layout file is distributed with WPILib and available at
       `<wpilib install>/share/apriltag/2026_REBUILT.json` after installing the
       2026 WPILib update.
2. Enable **Multi-tag pose estimation** so that `botpose_targetspace` fuses all
   visible hub tags for maximum accuracy.

### 3.3 Hub AprilTag IDs (verify against official game manual)

> ⚠️ **Important**: Confirm these IDs against the official 2026 REBUILT field
> layout before competition.  The values in `constants.py` default to the IDs
> shown below and must match the field JSON.

| Alliance | Tag IDs |
|----------|---------|
| Red hub  | 3, 4   |
| Blue hub | 7, 8   |

In `constants.py`:

```python
HUB_APRILTAG_IDS_RED:  list = [3, 4]
HUB_APRILTAG_IDS_BLUE: list = [7, 8]
```

---

## 4. NetworkTable Integration (Python WPILib)

The `FlywheelSubsystem` reads the following NetworkTable entries every robot
cycle.  No additional code is required – the subsystem handles all reads.

| Entry | Type | Description |
|-------|------|-------------|
| `tv` | double | 1 = target visible, 0 = no target |
| `tid` | double | ID of the primary tracked AprilTag |
| `botpose_targetspace` | double[] | `[tx, ty, tz, rx, ry, rz]` – robot position relative to the tag in meters / degrees |

Default table name: **`limelight`** (configured in `constants.py` as
`LIMELIGHT_TABLE_NAME`).

### Verifying NT data in Python

```python
import ntcore
nt = ntcore.NetworkTableInstance.getDefault()
table = nt.getTable("limelight")

tv  = table.getEntry("tv").getDouble(0)
tid = table.getEntry("tid").getDouble(-1)
pose = table.getEntry("botpose_targetspace").getDoubleArray([0.0] * 6)

print(f"Target visible: {tv}, Tag ID: {int(tid)}")
print(f"tx={pose[0]:.3f}  ty={pose[1]:.3f}  tz={pose[2]:.3f} m")
```

Run this snippet from the Driver Station console or in Robot Test mode to confirm
that the Limelight is publishing data before enabling the flywheel.

---

## 5. Calibration: Distance-to-Velocity Tuning

The flywheel subsystem uses a linear model:

```
velocity_rps = FLYWHEEL_VELOCITY_SLOPE * distance_m + FLYWHEEL_VELOCITY_INTERCEPT
```

### Calibration procedure

1. **Place the robot at a known distance** from the hub (e.g. 1 m, 2 m, 3 m, 4 m, 5 m).
2. **Enable the flywheel** (press the toggle button / X on the tool controller).
3. **Fire a game piece** and note whether it hits the hub target.
4. **Adjust `FLYWHEEL_VELOCITY_SLOPE` and `FLYWHEEL_VELOCITY_INTERCEPT`** in
   `constants.py` until shots are on target across the expected distance range.
5. Once linear tuning is satisfactory, uncomment the **quadratic** or **lookup
   table** block in `FlywheelSubsystem._calculate_velocity()` for higher accuracy
   at the extremes of the range.

### Useful SmartDashboard keys during calibration

| Key | Value |
|-----|-------|
| `Flywheel/Enabled` | Whether the system is active |
| `Flywheel/TargetVelocity_RPS` | Calculated target speed |
| `Flywheel/ActualVelocity_RPS` | Measured motor speed |
| `Flywheel/AtSpeed` | True when within ±2 RPS of target |

---

## 6. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `tv` is always 0 | Limelight can't see the tag | Adjust mount angle / check lighting |
| Wrong tag ID detected | Another field tag in view | Narrow the field of view in pipeline settings |
| `botpose_targetspace` all zeros | Multi-tag not enabled | Enable multi-tag pose estimation |
| Flywheel speed never changes | Wrong table name | Check `LIMELIGHT_TABLE_NAME` matches the web UI |
| `AtSpeed` never true | PID gains need tuning | Adjust `FLYWHEEL_KP`, `FLYWHEEL_KV`, `FLYWHEEL_KS` |
| NT entries missing (USB mode) | USB cable not connected or roboRIO not recognising device | Reconnect USB-C cable; check roboRIO USB port; reboot both |

---

## 7. References

- [Limelight 3A docs](https://docs.limelightvision.io)
- [WPILib AprilTag field layouts](https://github.com/wpilibsuite/allwpilib/tree/main/apriltag)
- [Phoenix 6 VelocityVoltage API](https://api.ctr-electronics.com/phoenix6/release/python/)
- 2026 REBUILT official game manual (FIRST website)
