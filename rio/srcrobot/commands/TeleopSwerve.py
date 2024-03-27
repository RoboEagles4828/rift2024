from constants import Constants
from subsystems.Swerve import Swerve

from wpimath.geometry import Translation2d
from commands2 import Command

from typing import Callable
import math

from wpimath import applyDeadband


class TeleopSwerve(Command):    
    s_Swerve: Swerve  
    translationSup: Callable[[], float]
    strafeSup: Callable[[], float]
    rotationSup: Callable[[], float]
    robotCentricSup: Callable[[], bool]
    slowSup: Callable[[], list[float]]

    def __init__(self, s_Swerve, translationSup, strafeSup, rotationSup, robotCentricSup, slowSup=lambda: 0.0):
        self.s_Swerve = s_Swerve
        self.addRequirements(s_Swerve)

        self.translationSup = translationSup
        self.strafeSup = strafeSup
        self.rotationSup = rotationSup
        self.robotCentricSup = robotCentricSup
        self.slowSup = slowSup


    def execute(self):
        # Get Values, Deadband
        translationVal = math.copysign(self.translationSup()**2, self.translationSup())
        strafeVal = math.copysign(self.strafeSup()**2, self.strafeSup())
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
        return (math.copysign(self.rotationSup()**2, self.rotationSup())) * Constants.Swerve.maxAngularVelocity