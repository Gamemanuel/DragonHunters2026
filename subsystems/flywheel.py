"""
Flywheel subsystem for the 2026 REBUILT FRC season.

Uses a Limelight 3A to detect hub AprilTags and dynamically set the flywheel
velocity based on the calculated distance to the hub.  Both red and blue
alliances are supported via FMS / DriverStation alliance data.

Motors
------
- Two TalonFX motors in a differential flywheel configuration (Phoenix 6).
- One REV SparkMax conveyor motor whose speed is controlled by the toggle.

Limelight integration
---------------------
Data is read from the Limelight NetworkTable each robot cycle.  The primary
distance estimate comes from ``botpose_targetspace`` (6-element array: tx, ty,
tz in meters followed by roll/pitch/yaw in degrees).  When multiple hub tags
are simultaneously visible the Limelight solver already fuses all of them to
produce a single best-estimate robot pose, so no additional averaging is needed
in this code.

Velocity calculation
--------------------
A simple linear model is used in production.  Quadratic and piecewise-linear
alternatives are provided in commented-out blocks for future tuning.
"""

from __future__ import annotations

import math
from typing import List, Optional

import commands2
import ntcore
import rev
import wpilib
from phoenix6 import CANBus, configs, controls, hardware
from wpilib import DriverStation, SmartDashboard

import constants


class FlywheelSubsystem(commands2.Subsystem):
    """
    Subsystem that owns the two flywheel TalonFX motors and coordinates the
    conveyor SparkMax.  Call :meth:`toggle` (or bind it to a button) to
    enable/disable the entire shooting system in one action.
    """

    def __init__(self, conveyor_motor: rev.SparkMax) -> None:
        super().__init__()

        # ------------------------------------------------------------------
        # Flywheel motors – TalonFX (Phoenix 6)
        # ------------------------------------------------------------------
        self._flywheel_one = hardware.TalonFX(constants.FLYWHEEL_MOTOR_ONE_ID, CANBus("rio"))
        self._flywheel_two = hardware.TalonFX(constants.FLYWHEEL_MOTOR_TWO_ID, CANBus("rio"))

        # Motor output inversion – motor one spins counter-clockwise positive
        cfg = configs.TalonFXConfiguration()
        cfg.motor_output.inverted = (
            configs.config_groups.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self._flywheel_one.configurator.apply(cfg)

        # PID / feed-forward gains (Slot 0) for velocity control
        slot0 = configs.Slot0Configs()
        slot0.k_s = constants.FLYWHEEL_KS
        slot0.k_v = constants.FLYWHEEL_KV
        slot0.k_p = constants.FLYWHEEL_KP
        slot0.k_i = constants.FLYWHEEL_KI
        slot0.k_d = constants.FLYWHEEL_KD
        self._flywheel_one.configurator.apply(slot0)
        self._flywheel_two.configurator.apply(slot0)

        # Configure motor two as a strict follower of motor one, opposing its
        # direction so both rollers propel the game piece the same way.
        self._flywheel_two.set_control(
            controls.StrictFollower(constants.FLYWHEEL_MOTOR_ONE_ID, oppose_master_direction=True)
        )

        # Reusable velocity-voltage control request (avoids per-cycle allocation)
        self._velocity_request = controls.VelocityVoltage(0).with_slot(0)

        # ------------------------------------------------------------------
        # Conveyor motor – REV SparkMax
        # (The caller creates and owns the motor; we only drive its speed.)
        # ------------------------------------------------------------------
        self._conveyor = conveyor_motor

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._enabled: bool = False
        self._target_velocity_rps: float = 0.0

        # ------------------------------------------------------------------
        # Limelight NetworkTable
        # ------------------------------------------------------------------
        nt = ntcore.NetworkTableInstance.getDefault()
        self._limelight = nt.getTable(constants.LIMELIGHT_TABLE_NAME)

    # ----------------------------------------------------------------------
    # Subsystem periodic – called every 20 ms
    # ----------------------------------------------------------------------

    def periodic(self) -> None:
        if self._enabled:
            distance = self._get_distance_to_hub()
            if distance is not None:
                self._target_velocity_rps = self._calculate_velocity(distance)
            self._set_velocity(self._target_velocity_rps)
            self._conveyor.set(constants.CONVEYOR_SPEED)
        else:
            self._target_velocity_rps = 0.0

        # Dashboard telemetry
        SmartDashboard.putBoolean("Flywheel/Enabled", self._enabled)
        SmartDashboard.putNumber(
            "Flywheel/TargetVelocity_RPS", self._target_velocity_rps
        )
        SmartDashboard.putNumber(
            "Flywheel/ActualVelocity_RPS",
            self._flywheel_one.get_velocity().value,
        )
        SmartDashboard.putBoolean("Flywheel/AtSpeed", self.is_at_speed())

    # ----------------------------------------------------------------------
    # Public control API
    # ----------------------------------------------------------------------

    def enable(self) -> None:
        """Activate the flywheel and conveyor system."""
        self._enabled = True

    def disable(self) -> None:
        """Stop the flywheel and conveyor system."""
        self._enabled = False
        # Setting NeutralOut on the master is enough; the strict follower will
        # mirror it automatically.
        self._flywheel_one.set_control(controls.NeutralOut())
        self._conveyor.set(0)

    def toggle(self) -> None:
        """Toggle the flywheel / conveyor system on or off."""
        if self._enabled:
            self.disable()
        else:
            self.enable()

    def is_enabled(self) -> bool:
        """Return whether the flywheel system is currently active."""
        return self._enabled

    def is_at_speed(self) -> bool:
        """Return ``True`` when the flywheel is within tolerance of its target."""
        if self._target_velocity_rps == 0:
            return False
        actual = self._flywheel_one.get_velocity().value
        return abs(actual - self._target_velocity_rps) <= constants.FLYWHEEL_VELOCITY_TOLERANCE_RPS

    # ----------------------------------------------------------------------
    # Direct motor accessors (used by named auto commands in RobotContainer)
    # ----------------------------------------------------------------------

    @property
    def flywheel_one(self) -> hardware.TalonFX:
        return self._flywheel_one

    @property
    def flywheel_two(self) -> hardware.TalonFX:
        return self._flywheel_two

    # ----------------------------------------------------------------------
    # Limelight / distance helpers
    # ----------------------------------------------------------------------

    def _get_distance_to_hub(self) -> Optional[float]:
        """
        Read AprilTag data from the Limelight and return the horizontal
        distance to the hub in **meters**.  Returns ``None`` when no hub tag
        for the current alliance is visible.

        **Triangulation note**: ``botpose_targetspace`` is a 6-element array
        ``[tx, ty, tz, rx, ry, rz]`` where tx/ty/tz are the robot's
        translation relative to the *primary* tracked tag in meters.  When
        more than one hub tag is simultaneously visible the Limelight's
        multi-tag solver already fuses all detections into a single
        best-estimate pose, so no additional averaging is required here.
        In Limelight's target-space frame tx is the lateral (left/right)
        offset, ty is vertical, and tz is the forward depth.
        Horizontal distance = ``sqrt(tx² + tz²)`` (lateral + depth,
        ignoring the vertical ty offset).
        """
        # Is there a valid target?
        tv = self._limelight.getEntry("tv").getDouble(0)
        if tv < 1:
            return None

        # Confirm the tracked tag is a hub tag for the active alliance
        tag_id = int(self._limelight.getEntry("tid").getDouble(-1))
        if tag_id not in self._get_hub_tag_ids():
            return None

        # Read robot pose relative to the target tag
        pose = self._limelight.getEntry("botpose_targetspace").getDoubleArray(
            [0.0] * 6
        )
        if len(pose) < 3:
            return None

        tx, tz = pose[0], pose[2]
        horizontal_distance = math.hypot(tx, tz)

        if horizontal_distance <= 0:
            return None

        return horizontal_distance

    def _get_hub_tag_ids(self) -> List[int]:
        """Return hub AprilTag IDs for the alliance reported by FMS / DS."""
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kRed:
            return constants.HUB_APRILTAG_IDS_RED
        # Default to blue (also covers kInvalid / not yet known)
        return constants.HUB_APRILTAG_IDS_BLUE

    # ----------------------------------------------------------------------
    # Velocity calculation
    # ----------------------------------------------------------------------

    def _calculate_velocity(self, distance_m: float) -> float:
        """
        Convert a distance (meters) to a flywheel target velocity (RPS).

        **Active model – simple linear**::

            velocity = SLOPE * distance + INTERCEPT

        Adjust ``FLYWHEEL_VELOCITY_SLOPE`` and ``FLYWHEEL_VELOCITY_INTERCEPT``
        in ``constants.py`` during practice-field testing.

        The result is clamped to ``[FLYWHEEL_MIN_VELOCITY_RPS,
        FLYWHEEL_MAX_VELOCITY_RPS]``.
        """
        # --- Simple linear calculation (active) ---
        velocity = (
            constants.FLYWHEEL_VELOCITY_SLOPE * distance_m
            + constants.FLYWHEEL_VELOCITY_INTERCEPT
        )

        # --- Quadratic model (inactive – uncomment and tune for higher accuracy) ---
        # velocity = (
        #     constants.FLYWHEEL_QUAD_A * distance_m ** 2
        #     + constants.FLYWHEEL_QUAD_B * distance_m
        #     + constants.FLYWHEEL_QUAD_C
        # )

        # --- Piecewise linear lookup table (inactive – uncomment when data is available) ---
        # Populate distances_m and velocities_rps with measured calibration data,
        # then use linear interpolation between the nearest two points.
        # distances_m    = [1.0, 2.0, 3.0, 4.0,  5.0,  6.0]   # meters (measured)
        # velocities_rps = [20.0, 28.0, 38.0, 50.0, 64.0, 80.0]  # RPS (measured)
        # for i in range(len(distances_m) - 1):
        #     if distances_m[i] <= distance_m <= distances_m[i + 1]:
        #         t = (distance_m - distances_m[i]) / (distances_m[i + 1] - distances_m[i])
        #         velocity = velocities_rps[i] + t * (velocities_rps[i + 1] - velocities_rps[i])
        #         break

        return max(
            constants.FLYWHEEL_MIN_VELOCITY_RPS,
            min(constants.FLYWHEEL_MAX_VELOCITY_RPS, velocity),
        )

    # ----------------------------------------------------------------------
    # Motor control
    # ----------------------------------------------------------------------

    def _set_velocity(self, velocity_rps: float) -> None:
        """
        Drive the master flywheel motor to *velocity_rps* using closed-loop
        VelocityVoltage control (Phoenix 6 Slot 0 gains).  Motor two follows
        automatically via the StrictFollower request configured in ``__init__``.
        """
        self._velocity_request = self._velocity_request.with_velocity(velocity_rps)
        self._flywheel_one.set_control(self._velocity_request)
