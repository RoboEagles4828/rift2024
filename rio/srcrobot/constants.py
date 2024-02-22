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
        trackWidth = Units.inchesToMeters(26.0)
        wheelBase = Units.inchesToMeters(26.0)
        wheelCircumference = chosenModule.wheelCircumference

        frontLeftLocation = Translation2d(Units.inchesToMeters(-10.375), Units.inchesToMeters(7.8125))
        frontRightLocation = Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(7.8125))
        backLeftLocation = Translation2d(Units.inchesToMeters(-10.375), Units.inchesToMeters(-12.9375))
        backRightLocation = Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(-12.9375))

        robotCenterLocation = Translation2d(0.0, 0.0)

        # Swerve Kinematics 
        swerveKinematics = SwerveDrive4Kinematics(
            frontRightLocation,
            backRightLocation,
            frontLeftLocation,
            backLeftLocation
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
            PIDConstants(10.0, 0.0, 0.0),
            PIDConstants(10.0, 0.0, 0.0),
            maxAutoModuleSpeed,
            #distance from center to the furthest module
            robotCenterLocation.distance(backLeftLocation),
            ReplanningConfig(),
        )

        # Module Specific Constants
        # Front Left Module - Module 0
        class Mod0:
            driveMotorID = 2
            angleMotorID = 1
            canCoderID = 3
            angleOffset = Rotation2d(rotationsToRadians(2.63623)+math.pi)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Front Right Module - Module 1
        class Mod1:
            driveMotorID = 19
            angleMotorID = 18
            canCoderID = 20
            angleOffset = Rotation2d(rotationsToRadians(0.766846)+math.pi)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)
        
        # Back Left Module - Module 2
        class Mod2:
            driveMotorID = 9
            angleMotorID = 8
            canCoderID = 7
            angleOffset = Rotation2d(rotationsToRadians(1.149902)+math.pi)
            constants = SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset)

        # Back Right Module - Module 3
        class Mod3:
            driveMotorID = 12
            angleMotorID = 11
            canCoderID = 10
            angleOffset = Rotation2d(rotationsToRadians(1.075195)+math.pi)
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
        kSubwooferShootSpeed = 3.0
        kPodiumShootSpeed = 20.0
        kAmpShootSpeed = 2.0

    class IntakeConstants:
        kIntakeMotorID = 0
        kIntakeSpeed = 5.0
    
    class IndexerConstants:
        kIndexerMotorID = 14
        kIndexerSpeed = 10.0

    # An enumeration of known shot locations and data critical to executing the
    # shot. TODO decide on shooter velocity units and tune angles.
    class NextShot(Enum):
      AMP = (0, -90.0, 90.0, 90.0, 100.0)
      SPEAKER_AMP = (1, 45.0, -45.0, 0.0, 1000.0)
      SPEAKER_CENTER = (2, 0.0, 0.0, 0.0, 1000.0)
      SPEAKER_PODIUM = (3, -45.0, 45.0, 0.0, 1000.0)
      PODIUM = (4, -30.0, 30.0, 45.0, 2000.00)

      def __init__(self, value, blueSideBotHeading, redSideBotHeading, armAngle, shooterVelocity):
        self._value_ = value
        self.m_blueSideBotHeading = blueSideBotHeading
        self.m_redSideBotHeading = redSideBotHeading
        self.m_armAngle = armAngle
        self.m_shooterVelocity = shooterVelocity
