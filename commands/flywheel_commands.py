"""
Commands for controlling the flywheel subsystem.

ToggleFlywheelCommand  – toggle the flywheel + conveyor on/off (bind to a button).
EnableFlywheelCommand  – explicitly enable the system.
DisableFlywheelCommand – explicitly disable the system.

Velocity updates happen automatically inside FlywheelSubsystem.periodic(),
so no separate "update velocity" command is required.
"""

from __future__ import annotations

import commands2

from subsystems.flywheel import FlywheelSubsystem


class ToggleFlywheelCommand(commands2.InstantCommand):
    """
    Toggle the flywheel and conveyor system on or off with a single button
    press.  Because the velocity is updated every cycle in
    :meth:`FlywheelSubsystem.periodic`, the shooter is always targeting the
    correct speed while enabled.
    """

    def __init__(self, flywheel: FlywheelSubsystem) -> None:
        super().__init__(flywheel.toggle, flywheel)


class EnableFlywheelCommand(commands2.InstantCommand):
    """Explicitly enable the flywheel and conveyor system."""

    def __init__(self, flywheel: FlywheelSubsystem) -> None:
        super().__init__(flywheel.enable, flywheel)


class DisableFlywheelCommand(commands2.InstantCommand):
    """Explicitly disable the flywheel and conveyor system."""

    def __init__(self, flywheel: FlywheelSubsystem) -> None:
        super().__init__(flywheel.disable, flywheel)
