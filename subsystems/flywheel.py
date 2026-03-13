"""
Flywheel subsystem for the 2026 REBUILT FRC season.

Uses PhotonVision (photonlibpy) to detect hub AprilTags via a USB-connected
camera and dynamically set the flywheel velocity based on the calculated
distance to the hub.  Both red and blue alliances are supported via FMS /
DriverStation alliance data.

PhotonVision supports any USB camera attached to a coprocessor (Raspberry Pi,
Orange Pi, etc.) that is connected to the robot network, including cameras
originally designed for FTC.  The coprocessor runs the PhotonVision software
and publishes results to NetworkTables – the robot code reads from NT via the
``photonlibpy`` client library without needing to know the camera connection
type (USB or Ethernet).

Motors
------
- Two TalonFX motors in a differential flywheel configuration (Phoenix 6).
- One REV SparkMax conveyor motor whose speed is controlled by the toggle.

Vision integration
------------------
``PhotonCamera`` (photonlibpy) is queried each robot cycle.  Each
``PhotonTrackedTarget`` exposes ``getFiducialId()`` and
``getBestCameraToTarget()`` (a ``Transform3d``).  Horizontal distance is
computed as ``hypot(translation.X(), translation.Y())``.  When multiple hub
tags are visible simultaneously the target with the smallest
``getPoseAmbiguity()`` score is used.

Velocity calculation
--------------------
A simple linear model is used in production.  Quadratic and piecewise-linear
alternatives are provided in commented-out blocks for future tuning.
"""

from __future__ import annotations

import math
from typing import List, Optional

import commands2
from photonlibpy import PhotonCamera
from phoenix6 import CANBus, configs, controls, hardware
import rev
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
        # Vision camera (PhotonVision – supports USB cameras on a coprocessor)
        # ------------------------------------------------------------------
        self._camera = PhotonCamera(constants.PHOTON_CAMERA_NAME)

        # --- Alternative: raw Limelight NetworkTables (uncomment if preferred) ---
        # import ntcore
        # nt = ntcore.NetworkTableInstance.getDefault()
        # self._limelight = nt.getTable(constants.LIMELIGHT_TABLE_NAME)

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
    # Vision / distance helpers
    # ----------------------------------------------------------------------

    def _get_distance_to_hub(self) -> Optional[float]:
        """
        Query PhotonVision for the best hub AprilTag and return the horizontal
        distance to it in **meters**.  Returns ``None`` when no hub tag for the
        current alliance is visible.

        **Triangulation**: when multiple hub tags are visible at the same time,
        the tag with the lowest pose ambiguity (most reliable pose estimate) is
        selected.  ``getBestCameraToTarget()`` returns the best-fit
        ``Transform3d`` for that tag.  Horizontal distance is
        ``hypot(translation.X(), translation.Y())``.

        **Coordinate system**: PhotonVision / WPIMath uses a
        right-hand, Z-up frame.  ``X`` is forward from the camera,
        ``Y`` is left.  ``hypot(X, Y)`` therefore gives the horizontal
        range to the tag centre, independent of the camera tilt.
        """
        result = self._camera.getLatestResult()
        if not result.hasTargets():
            return None

        hub_ids = self._get_hub_tag_ids()
        best_target = None
        best_ambiguity = float("inf")

        for target in result.getTargets():
            if target.getFiducialId() in hub_ids:
                ambiguity = target.getPoseAmbiguity()
                # Skip targets with invalid/missing ambiguity values
                if ambiguity is None or ambiguity < 0:
                    continue
                if ambiguity < best_ambiguity:
                    best_ambiguity = ambiguity
                    best_target = target

        if best_target is None:
            return None

        translation = best_target.getBestCameraToTarget().translation()
        horizontal_distance = math.hypot(translation.X(), translation.Y())

        if horizontal_distance <= 0:
            return None

        return horizontal_distance

        # --- Alternative: raw Limelight NetworkTables distance calculation ---
        # tv = self._limelight.getEntry("tv").getDouble(0)
        # if tv < 1:
        #     return None
        # tag_id = int(self._limelight.getEntry("tid").getDouble(-1))
        # if tag_id not in hub_ids:
        #     return None
        # pose = self._limelight.getEntry("botpose_targetspace").getDoubleArray([0.0] * 6)
        # if len(pose) < 3:
        #     return None
        # tx, tz = pose[0], pose[2]
        # horizontal_distance = math.hypot(tx, tz)
        # if horizontal_distance <= 0:
        #     return None
        # return horizontal_distance

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

