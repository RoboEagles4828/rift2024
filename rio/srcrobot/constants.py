from phoenix6.signals import InvertedValue
from phoenix6.signals import NeutralModeValue
from phoenix6.signals import SensorDirectionValue
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
import lib.mathlib.units as Units
from lib.util.COTSTalonFXSwerveConstants import COTSTalonFXSwerveConstants
from lib.util.SwerveModuleConstants import SwerveModuleConstants
import math
from enum import Enum
from pytreemap import TreeMap

from lib.util.InterpolatingTreeMap import InterpolatingTreeMap
from wpimath.units import rotationsToRadians

from pathplannerlib.auto import HolonomicPathFollowerConfig
from pathplannerlib.controller import PIDConstants
from pathplannerlib.config import ReplanningConfig

class Constants:
    stickDeadband = 0.1

    class Swerve:
        navxID = 0

        chosenModule = COTSTalonFXSwerveConstants.MK4i.Falcon500(COTSTalonFXSwerveConstants.MK4i.driveRatios.L2)

        # Drivetrain Constants
        trackWidth = Units.inchesToMeters(20.75)
        wheelBase = Units.inchesToMeters(20.75)
        rotationBase = Units.inchesToMeters(31.125 - 5.25)

        robotWidth = 26.0
        robotLength = 31.125

        frontOffset = rotationBase - wheelBase

        wheelCircumference = chosenModule.wheelCircumference

        frontLeftLocation = Translation2d(-((wheelBase / 2.0) - frontOffset), -trackWidth / 2.0)
        frontRightLocation = Translation2d(-((wheelBase / 2.0) - frontOffset), trackWidth / 2.0)
        backLeftLocation = Translation2d(wheelBase / 2.0, -trackWidth / 2.0)
        backRightLocation = Translation2d(wheelBase / 2.0, trackWidth / 2.0)

        # frontLeftLocation = Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
        # frontRightLocation = Translation2d(-wheelBase / 2.0, trackWidth / 2.0)
        # backLeftLocation = Translation2d(wheelBase / 2.0, -trackWidth / 2.0)
        # backRightLocation = Translation2d(wheelBase / 2.0, trackWidth / 2.0)


        robotCenterLocation = Translation2d(0.0, 0.0)

        swerveKinematics = SwerveDrive4Kinematics(
            frontLeftLocation,
            frontRightLocation,
            backLeftLocation,
            backRightLocation
        )

        # Module Gear Ratios
        driveGearRatio = chosenModule.driveGearRatio
        angleGearRatio = chosenModule.angleGearRatio

        # Motor Inverts
        angleMotorInvert = chosenModule.angleMotorInvert
        driveMotorInvert = chosenModule.driveMotorInvert

        # Angle Encoder Invert
        cancoderInvert = chosenModule.cancoderInvert

        # Swerve Current Limiting
        angleCurrentLimit = 25
        angleCurrentThreshold = 40
        angleCurrentThresholdTime = 0.1
        angleEnableCurrentLimit = True

        driveCurrentLimit = 35
        driveCurrentThreshold = 60
        driveCurrentThresholdTime = 0.1
        driveEnableCurrentLimit = True

        openLoopRamp = 0.0
        closedLoopRamp = 0.0

        # Angle Motor PID Values
        angleKP = chosenModule.angleKP
        angleKI = chosenModule.angleKI
        angleKD = chosenModule.angleKD

        # Drive Motor PID Values
        driveKP = 2.5
        driveKI = 0.0
        driveKD = 0.0
        driveKF = 0.0

        driveKS = 0.2
        driveKV = 0.28
        driveKA = 0.0

        # Swerve Profiling Values
        # Meters per Second
        maxSpeed = 5.0
        maxAutoModuleSpeed = 4.5
        # Radians per Second
        maxAngularVelocity = 2.5 * math.pi

        # Neutral Modes
        angleNeutralMode = NeutralModeValue.COAST
        driveNeutralMode = NeutralModeValue.BRAKE

        holonomicPathConfig = HolonomicPathFollowerConfig(
            PIDConstants(5.0, 0.0, 0.0),
            PIDConstants(4.0, 0.0, 0.0),
            maxAutoModuleSpeed,
            #distance from center to the furthest module
            Units.inchesToMeters(16),
            ReplanningConfig(),
        )

        # Slowdown speed
        ## The speed is multiplied by this value when the trigger is fully held down
        slowMoveModifier = 0.5
        slowTurnModifier = 0.5

        # Module Specific Constants
        # Front Left Module - Module 0
        class Mod0:
            driveMotorID = 2
            angleMotorID = 1
            canCoderID = 3
            angleOffset = Rotation2d(rotationsToRadians(0.145020))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Front Right Module - Module 1
        class Mod1:
            driveMotorID = 19
            angleMotorID = 18
            canCoderID = 20
            angleOffset = Rotation2d(rotationsToRadians(1.267334))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        
        # Back Left Module - Module 2
        class Mod2:
            driveMotorID = 9
            angleMotorID = 8
            canCoderID = 7
            angleOffset = Rotation2d(rotationsToRadians(0.648926))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Back Right Module - Module 3
        class Mod3:
            driveMotorID = 12
            angleMotorID = 11
            canCoderID = 10
            angleOffset = Rotation2d(rotationsToRadians(1.575195))
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

    class AutoConstants:
        kMaxSpeedMetersPerSecond = 3
        kMaxModuleSpeed = 4.5
        kMaxAccelerationMetersPerSecondSquared = 3
        kMaxAngularSpeedRadiansPerSecond = math.pi
        kMaxAngularSpeedRadiansPerSecondSquared = math.pi

        kPXController = 4.0
        kPYController = 4.0
        kPThetaController = 1.5
    
        kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
            kMaxAngularSpeedRadiansPerSecond, 
            kMaxAngularSpeedRadiansPerSecondSquared
        )
    class ShooterConstants:
        kSubwooferPivotAngle = 0.0
        kPodiumPivotAngle = 45.0
        kAmpPivotAngle = 90.0
        kSubwooferShootSpeed = 25.0
        kPodiumShootSpeed = 20.0
        kAmpShootSpeed = 5.0

    class IntakeConstants:
        kIntakeMotorID = 0
        kIntakeSpeed = 5.0
    
    class IndexerConstants:
        kIndexerMotorID = 14
        kIndexerMaxSpeedMS = 20.0
        kIndexerIntakeSpeedMS = 0.5
        kBeamBreakerID = 0

    class ClimberConstants:
        kLeftMotorID = 14
        kRightMotorID = 13
        kLeftCANID = 4
        kRightCANID = 15
        maxClimbHeight = 0
        kClimberSpeed = 0.85 # percent output
    
    class ArmConstants:
        kKnownArmAngles = InterpolatingTreeMap()

        kKnownArmAngles.put(0.0, 5.0)
        kKnownArmAngles.put(1.0, 6.5)
        kKnownArmAngles.put(2.0, 10.0)
        kKnownArmAngles.put(3.0, 12.0)
        kKnownArmAngles.put(4.0, 17.0)
        kKnownArmAngles.put(5.0, 20.0)
        kKnownArmAngles.put(6.0, 23.0)


    # An enumeration of known shot locations and data critical to executing the
    # shot. TODO decide on shooter velocity units and tune angles.
    class NextShot(Enum):
      AMP = (0, -90.0, 90.0, 80.0, 8.0, 5, 6)
      SPEAKER_AMP = (1, -60.0, -60.0, 5.0, 25.0, 4, 7)
      SPEAKER_CENTER = (2, 0.0, 0.0, 5.0, 25.0, 4, 7)
      SPEAKER_SOURCE = (3, 60.0, 60.0, 5.0, 25.0, 4, 7)
      PODIUM = (4, -30.0, 30.0, 26.5, 35.0, 4, 7)
      CENTER_AUTO = (5, -30.0, 30.0, 31.0, 35.0, 4, 7)
      DYNAMIC = (6, 0.0, 0.0, 0.0, 30.0, 4, 7)



      def __init__(self, value, blueSideBotHeading, redSideBotHeading, armAngle, shooterVelocity, red_tagID, blue_tagID):
        self._value_ = value
        self.m_blueSideBotHeading = blueSideBotHeading
        self.m_redSideBotHeading = redSideBotHeading
        self.m_armAngle = armAngle
        self.m_shooterVelocity = shooterVelocity
        self.m_redTagID = red_tagID
        self.m_blueTagID = blue_tagID

      def calculateArmAngle(self, distance: float) -> float:
        # a = -0.0694444
        # b = 0.755952
        # c = 0.968254
        # d = 5.0
        # armRegressionEquation = lambda x: a * (x**3) + b * (x**2) + c * x + d

        a = 0.130952
        b = 2.35714
        c = 4.58333
        armRegressionEquation = lambda x: a * (x**2) + b * x + c

        if distance <= 0.5:
            armAngle = armRegressionEquation(0)
        else:
            armAngle = armRegressionEquation(distance) + 2.0

        return armAngle
      
      def calculateInterpolatedArmAngle(self, distance: float) -> float:
          armAngle = Constants.ArmConstants.kKnownArmAngles.get(distance)
          return armAngle
      
      def calculateShooterVelocity(self, distance: float) -> float:
            if distance <= 0.5:
                shooterVelocity = 25.0
            else:
                shooterVelocity = 35.0
            
            return shooterVelocity
    
