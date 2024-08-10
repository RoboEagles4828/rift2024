from commands2 import Command
from constants import Constants
from wpimath.controller import ProfiledPIDControllerRadians, PIDController
from subsystems.Swerve import Swerve
from commands.TeleopSwerve import TeleopSwerve
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.trajectory import TrapezoidProfile
import math
from typing import Callable

class TurnToTag(TeleopSwerve):
    def __init__(self, s_Swerve, desiredRotationSup: Callable[[], Rotation2d], translationSup, strafeSup, rotationSup, robotCentricSup):
        super().__init__(s_Swerve, translationSup, strafeSup, rotationSup, robotCentricSup)
        # This is added to the calculated output as a static feedforward.
        # This value is wrong and needs to be determined by setting he PID
        # gains to 0 and seeing what value for this makes the robot just
        # start turing. We probably want a value just below that. The just
        # starts turning value may be okay too.
        self.kBarelyNotTurnFeedforward = 0.5 #degrees per second
        # With the feedforward right, the gains should be able to be much
        # smaller. We may not even need the I and D. This should result
        # in much more control. These changes are based on some CD reading
        # and example code checking, such as here (https://www.chiefdelphi.com/t/heading-pid-tips/150244)
        # and here (https://github.com/6391-Ursuline-Bearbotics/2020_UARobotics_Infinite_Recharge/blob/master/src/main/java/frc/robot/commands/TurnToAngle.java).
        # ALL ZERO FOR FEEDFORWARD TUNING.
        self.turnPID = PIDController(7.0, 0.0, 0.0)
        # INITIAL VALUES FOR FEEDFORWARD TUNING.
        # self.turnPID = PIDController(1.0, 0.0, 0.0)
        # PUT THIS BACK IF I GAIN ATTEMPTED.
        # self.turnPID.setIZone(math.radians(10.0))
        self.turnPID.enableContinuousInput(-math.pi, math.pi)
        self.desiredRotationSupplier = desiredRotationSup
        self.currentRotation = rotationSup


    def initialize(self):
        super().initialize()
        self.turnPID.reset()
        self.turnPID.setTolerance(math.radians(0.5))
        self.angle = self.desiredRotationSupplier().radians()

    def getRotationValue(self):
        rotationStick = self.currentRotation()
        if abs(rotationStick) > 0.0:
            return rotationStick*Constants.Swerve.maxAngularVelocity
        else:
            self.angularvelMRadiansPerSecond = -self.turnPID.calculate(self.s_Swerve.getHeading().radians(), self.angle)
            # if abs(self.turnPID.getPositionError()) >= 7.5:
            self.angularvelMRadiansPerSecond = math.copysign(math.fabs(self.angularvelMRadiansPerSecond) + self.kBarelyNotTurnFeedforward, self.angularvelMRadiansPerSecond)
            # NOTE: It is possible that you will want to adjust or eliminate the feed forward.
            # This would be late in tuning if there is a small oscillation that will not go away.
            # Then, the line above that adds the feed forward would change to be a conditional
            # something like this with "someValue" needing to be determined but is probably small
            # like 2 or 3 degrees (with radian conversion as needed).
            # if (math.fabs(self.turnPID.getPositionError()) > someValue):
            #     self.angularvelMRadiansPerSecond = math.copysign(math.fabs(self.angularvelMRadiansPerSecond) + self.kBarelyNotTurnFeedforward, self.angularvelMRadiansPerSecond)
            # else:
            #     self.angularvelMRadiansPerSecond = math.copysign(math.fabs(self.angularvelMRadiansPerSecond) + (self.kBarelyNotTurnFeedforward / 2.0), self.angularvelMRadiansPerSecond)
            return self.angularvelMRadiansPerSecond

    def isFinished(self) -> bool:
        return self.turnPID.atSetpoint()
    
    def end(self, interrupted):
        self.s_Swerve.stop()


        

