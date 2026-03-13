import rev
from typing import TypeAlias
from pathplannerlib.config import RobotConfig
import wpilib
import commands2
import typing
from constants import *
from phoenix6 import CANBus, controls, hardware
from robotcontainer import RobotContainer
from XboxController import XboxController
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation
from commands2.button import CommandXboxController, Trigger


Direction: TypeAlias = bool


REVMotorType   = rev.SparkBase.MotorType
REVResetMode   = rev.ResetMode
REVPersistMode = rev.PersistMode
REVIdleMode    = rev.SparkBaseConfig.IdleMode
config = RobotConfig.fromGUISettings()

BRUSHED      : REVMotorType   = REVMotorType.kBrushed
BRUSHLESS    : REVMotorType   = REVMotorType.kBrushless

SAFE_RESET   : REVResetMode   = REVResetMode.kResetSafeParameters
NO_SAFE_RESET: REVResetMode   = REVResetMode.kNoResetSafeParameters

PERSIST      : REVPersistMode = REVPersistMode.kPersistParameters
NO_PERSIST   : REVPersistMode = REVPersistMode.kNoPersistParameters

COAST        : REVIdleMode    = REVIdleMode.kCoast
BRAKE        : REVIdleMode    = REVIdleMode.kBrake

FORWARD      : Direction      = False
REVERSE      : Direction      = True

# ---------------------------------------------------------------------------
# Flywheel subsystem – motor CAN IDs
# ---------------------------------------------------------------------------
FLYWHEEL_MOTOR_ONE_ID: int = 16
FLYWHEEL_MOTOR_TWO_ID: int = 17
CONVEYOR_MOTOR_ID: int = 22  # SparkMax conveyor controlled by the flywheel toggle

# ---------------------------------------------------------------------------
# Flywheel PID / feed-forward gains (TalonFX Slot 0)
# ---------------------------------------------------------------------------
FLYWHEEL_KS: float = 0.10   # Static friction compensation (V)
FLYWHEEL_KV: float = 0.12   # Velocity feed-forward (V per RPS)
FLYWHEEL_KP: float = 0.11   # Proportional gain (V per RPS error)
FLYWHEEL_KI: float = 0.0    # Integral gain
FLYWHEEL_KD: float = 0.0    # Derivative gain  # TUNE during testing

# ---------------------------------------------------------------------------
# Flywheel velocity limits (rotations per second)
# ---------------------------------------------------------------------------
FLYWHEEL_MIN_VELOCITY_RPS: float = 15.0   # Minimum useful shot speed
FLYWHEEL_MAX_VELOCITY_RPS: float = 80.0   # Motor / mechanism hard limit
FLYWHEEL_VELOCITY_TOLERANCE_RPS: float = 2.0  # ±tolerance for "at speed" check

# ---------------------------------------------------------------------------
# Distance-to-velocity calibration
#   Simple linear model:  velocity_rps = SLOPE * distance_m + INTERCEPT
#   Tune SLOPE and INTERCEPT on the practice field before competition.
# ---------------------------------------------------------------------------
FLYWHEEL_VELOCITY_SLOPE: float = 5.0       # RPS per metre
FLYWHEEL_VELOCITY_INTERCEPT: float = 20.0  # Base RPS at 0 m (tune during testing)

# Quadratic coefficients (inactive – see FlywheelSubsystem._calculate_velocity)
FLYWHEEL_QUAD_A: float = 0.5   # RPS / m²
FLYWHEEL_QUAD_B: float = 4.0   # RPS / m
FLYWHEEL_QUAD_C: float = 18.0  # RPS offset

# ---------------------------------------------------------------------------
# Conveyor speed when the flywheel toggle is active
# ---------------------------------------------------------------------------
CONVEYOR_SPEED: float = 0.5

# ---------------------------------------------------------------------------
# Limelight 3A – NetworkTable configuration
#   Default table name is "limelight".  If multiple Limelights are connected,
#   rename each one via the Limelight web interface and update this constant.
#   Team 10380 default static IP: 10.103.80.11
# ---------------------------------------------------------------------------
LIMELIGHT_TABLE_NAME: str = "limelight"

# ---------------------------------------------------------------------------
# 2026 REBUILT – hub AprilTag IDs
#   Verify these IDs against the official 2026 REBUILT game manual and the
#   AprilTag layout JSON shipped with WPILib before your first match.
#   Red alliance hub tags
HUB_APRILTAG_IDS_RED: list = [3, 4]
#   Blue alliance hub tags
HUB_APRILTAG_IDS_BLUE: list = [7, 8]
# ---------------------------------------------------------------------------
