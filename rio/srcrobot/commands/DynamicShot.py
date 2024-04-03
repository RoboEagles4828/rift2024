from subsystems.Vision import Vision
from subsystems.Swerve import Swerve
from subsystems.Arm import Arm
from constants import Constants
import lib.mathlib.units as Units
import math
from wpimath.geometry import Rotation2d, Translation2d, Transform2d
from wpilib import DriverStation, RobotBase
from robotState import RobotState

class DynamicShot():
    def __init__(self, swerve: Swerve, vision: Vision, arm: Arm):
        self.swerve = swerve
        self.arm = arm
        self.vision = vision
        self.speakerTargetHeightMeters = Units.inchesToMeters(80.5)
        self.robotState = RobotState()
    
    def getArmAngle(self):
        # distanceFromSpeaker = self.vision.getDistanceVectorToSpeaker(self.swerve.getPose()).norm()
        # robotVelocity = self.swerve.getTranslationVelocity().rotateBy(self.swerve.getHeading())
        # return math.degrees(math.atan(
        #     (self.speakerTargetHeightMeters - Units.inchesToMeters(Constants.Swerve.robotHeight)) / (distanceFromSpeaker + robotVelocity.X()*0.02)
        # )) - Constants.ShooterConstants.kMechanicalAngle

        return Constants.NextShot.DYNAMIC.calculateInterpolatedArmAngle(Units.metersToFeet(self.vision.getDistanceVectorToSpeaker(self.swerve.getPose()).norm()) - (36.37 / 12.0) - (Constants.Swerve.robotLength / 2.0 / 12.0))
    
    def getTrigArmAngle(self):
        distanceFromSpeaker = self.vision.getDistanceVectorToSpeaker(self.swerve.getPose()).norm() - (Units.inchesToMeters(Constants.Swerve.robotLength / 2.0)) + Units.inchesToMeters(math.cos(math.radians(self.arm.getDegrees())) * Constants.Swerve.armLength)
        robotVelocity = self.swerve.getTranslationVelocity().rotateBy(self.swerve.getHeading())
        robotHeight = (math.sin(math.radians(self.arm.getDegrees())) * Constants.Swerve.armLength) + 16.5

        denom = distanceFromSpeaker + robotVelocity.X()*0.02

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            denom = distanceFromSpeaker - robotVelocity.X()*0.02

        return math.degrees(math.atan(
            denom / (self.speakerTargetHeightMeters - Units.inchesToMeters(robotHeight))
        )) - Constants.ShooterConstants.kMechanicalAngle
    
    def getInterpolatedArmAngle(self):
        robotVelocity = self.swerve.getTranslationVelocity().rotateBy(self.swerve.getHeading())
        nextPose = self.swerve.getPose().__add__(Transform2d(robotVelocity.__mul__(-0.02), Rotation2d()))
        distance = Units.metersToFeet(self.vision.getDistanceVectorToSpeaker(nextPose).norm()) - (36.37 / 12.0) - (Constants.Swerve.robotLength / 2.0 / 12.0)
        return Constants.NextShot.DYNAMIC.calculateInterpolatedArmAngle(distance)

    def getRobotAngle(self):
        distanceVector = self.vision.getDistanceVectorToSpeaker(self.swerve.getPose())
        robotVelocity = self.swerve.getTranslationVelocity().rotateBy(self.swerve.getHeading())
        angle = 90.0 - math.degrees(math.atan(
            (distanceVector.X() - robotVelocity.X()*0.02) / (distanceVector.Y() - robotVelocity.Y()*0.02)
        ))

        if angle >= 90:
            angle = angle - 180

        if RobotBase.isSimulation() and DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            return Rotation2d.fromDegrees(angle).rotateBy(Rotation2d.fromDegrees(180.0))
        return Rotation2d.fromDegrees(angle)
    
    def getRotationTarget(self):
        if self.robotState.m_gameState.getNextShot() == Constants.NextShot.DYNAMIC:
            return self.getRobotAngle()
        else:
            return None
    
