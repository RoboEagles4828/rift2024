from constants import Constants
from subsystems.Swerve import Swerve

from wpimath.geometry import Translation2d
from commands2 import Command

from typing import Callable

from wpimath import applyDeadband


class TeleopSwerve(Command):    
    s_Swerve: Swerve  
    translationSup: Callable[[], float]
    strafeSup: Callable[[], float]
    rotationSup: Callable[[], float]
    robotCentricSup: Callable[[], bool]
    slowSup: Callable[[], list[float]]

    def __init__(self, s_Swerve, translationSup, strafeSup, rotationSup, robotCentricSup, slowSup):
        self.s_Swerve = s_Swerve
        self.addRequirements(s_Swerve)

        self.translationSup = translationSup
        self.strafeSup = strafeSup
        self.rotationSup = rotationSup
        self.robotCentricSup = robotCentricSup
        self.slowSup = slowSup


    def execute(self):
        # Get Values, Deadband
        translationVal = applyDeadband(self.translationSup(), Constants.stickDeadband)**3
        strafeVal = applyDeadband(self.strafeSup(), Constants.stickDeadband)**3
        rotationVal = self.getRotationValue()

        # Apply slowmode
        translationVal *= self.slowSup[0]
        strafeVal *= self.slowSup[0]
        rotationVal *= self.slowSup[1]

        # Drive
        self.s_Swerve.drive(
            Translation2d(translationVal, strafeVal).__mul__(Constants.Swerve.maxSpeed), 
            rotationVal, 
            not self.robotCentricSup(), 
            True
        )

    def getRotationValue(self):
        return (applyDeadband(self.rotationSup(), Constants.stickDeadband)**3) * Constants.Swerve.maxAngularVelocity