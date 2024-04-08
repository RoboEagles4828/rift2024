from constants import Constants
from subsystems.Swerve import Swerve

from wpimath.geometry import Translation2d
from commands2 import Command

from typing import Callable
import math

from wpimath import applyDeadband
from wpimath.controller import PIDController


class TeleopSwerve(Command):    
    s_Swerve: Swerve  
    translationSup: Callable[[], float]
    strafeSup: Callable[[], float]
    rotationSup: Callable[[], float]
    robotCentricSup: Callable[[], bool]
    slowSup: Callable[[], list[float]]

    def __init__(self, s_Swerve, translationSup, strafeSup, rotationSup, robotCentricSup, slowSup=lambda: 0.0):
        self.s_Swerve: Swerve = s_Swerve
        self.addRequirements(s_Swerve)

        self.translationSup = translationSup
        self.strafeSup = strafeSup
        self.rotationSup = rotationSup
        self.robotCentricSup = robotCentricSup
        self.slowSup = slowSup

        self.lastHeading = self.s_Swerve.getHeading().radians()

        self.headingPID = PIDController(4.0, 0.0, 0.0)
        self.headingPID.enableContinuousInput(-math.pi, math.pi)
        self.headingPID.reset()
        self.headingPID.setTolerance(math.radians(1))

    def initialize(self):
        super().initialize()
        self.lastHeading = self.s_Swerve.getHeading().radians()

    def execute(self):
        # Get Values, Deadband
        # translationVal = math.copysign(self.translationSup()**2, self.translationSup())
        # strafeVal = math.copysign(self.strafeSup()**2, self.strafeSup())
        translationVal = self.translationSup()
        strafeVal = self.strafeSup()
        rotationVal = self.getRotationValue()

        # Apply slowmode
        slow = self.slowSup()

        # TODO: REMOVE THIS IN PRODUCTION. THIS IS TO SAVE THE ROBOT DURING TESTING.
        if slow < 0:
            print("SLOWMODE ERROR: SLOW OFFSET IS NEGATIVE\nCheck that your controller axis mapping is correct and goes between [0, 1]!")
            slow = 0

        translationVal -= translationVal*slow*Constants.Swerve.slowMoveModifier
        strafeVal -= strafeVal*slow*Constants.Swerve.slowMoveModifier
        rotationVal -= rotationVal*slow*Constants.Swerve.slowTurnModifier

        # Drive
        self.s_Swerve.drive(
            Translation2d(translationVal, strafeVal).__mul__(Constants.Swerve.maxSpeed), 
            rotationVal, 
            not self.robotCentricSup(), 
            True
        )

    def getRotationValue(self):
        return self.rotationSup() * Constants.Swerve.maxAngularVelocity

        # rotation = 0.0

        #heading correction
        # if abs(self.rotationSup()) > 0.0:
        #     # heading correction is disabled, record last heading
        #     self.lastHeading = self.s_Swerve.getHeading().radians()
        #     rotation = self.rotationSup() * Constants.Swerve.maxAngularVelocity
        # elif abs(self.translationSup()) > 0.0 or abs(self.strafeSup()) > 0.0:
        #     # heading correction is enabled, calculate correction
        #     rotation = -self.headingPID.calculate(self.s_Swerve.getHeading().radians(), self.lastHeading)
            

        # return rotation