from commands2 import Command
from constants import Constants
from wpimath.controller import ProfiledPIDControllerRadians, PIDController
from subsystems.Swerve import Swerve
from commands.TeleopSwerve import TeleopSwerve
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.trajectory import TrapezoidProfile
import math
from typing import Callable

class TurnInPlace(TeleopSwerve):
    def __init__(self, s_Swerve, desiredRotationSup: Callable[[], Rotation2d], translationSup, strafeSup, rotationSup, robotCentricSup):
        super().__init__(s_Swerve, translationSup, strafeSup, rotationSup, robotCentricSup)
        self.turnPID = PIDController(1.0, 0.0, 0.0)
        self.turnPID.enableContinuousInput(-math.pi, math.pi)
        self.desiredRotationSupplier = desiredRotationSup
        self.angle = desiredRotationSup().radians()
        self.currentRotation = rotationSup

    def initialize(self):
        super().initialize()
        self.start_angle = self.s_Swerve.getHeading().radians()
        self.turnPID.reset()
        self.turnPID.setTolerance(math.radians(1))
        self.turnPID.setSetpoint(self.angle)

    def getRotationValue(self):
        rotationStick = self.currentRotation()
        if abs(rotationStick) > 0.0:
            return rotationStick*Constants.Swerve.maxAngularVelocity
        elif self.turnPID.atSetpoint():
            return 0.0
        else:
            self.angularvelMRadiansPerSecond = -self.turnPID.calculate(self.s_Swerve.getHeading().radians(), self.desiredRotationSupplier().radians())
            return self.angularvelMRadiansPerSecond

    def isFinished(self) -> bool:
        return self.turnPID.atSetpoint()