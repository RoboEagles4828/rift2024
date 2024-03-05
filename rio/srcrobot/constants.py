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
        trackWidth = Units.inchesToMeters(20.75)
        wheelBase = Units.inchesToMeters(20.75)
        rotationBase = Units.inchesToMeters(31.125 - 5.25)
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
            PIDConstants(0.0, 0.0, 0.0),
            maxAutoModuleSpeed,
            #distance from center to the furthest module
            Units.inchesToMeters(16),
            ReplanningConfig(),
        )

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
        kSubwooferShootSpeed = 20.0
        kPodiumShootSpeed = 20.0
        kAmpShootSpeed = 5.0

    class IntakeConstants:
        kIntakeMotorID = 0
        kIntakeSpeed = 5.0
    
    class IndexerConstants:
        kIndexerMotorID = 14
        kIndexerMaxSpeedMS = 10.0
        kIndexerIntakeSpeedMS = 2.0
        kBeamBreakerID = 0

    class ClimberConstants:
        kLeftMotorID = 14
        kRightMotorID = 13
        kLeftCANID = 4
        kRightCANID = 15
        maxClimbHeight = 0
        kClimberSpeed = 0.75 # percent output

    # An enumeration of known shot locations and data critical to executing the
    # shot. TODO decide on shooter velocity units and tune angles.
    class NextShot(Enum):
      AMP = (0, -90.0, 90.0, 90.0, 5.0)
      SPEAKER_AMP = (1, 60.0, -60.0, 0.0, 25.0)
      SPEAKER_CENTER = (2, 0.0, 0.0, 0.0, 25.0)
      SPEAKER_SOURCE = (3, -60.0, 60.0, 0.0, 25.0)
      PODIUM = (4, -30.0, 30.0, 25.0, 45.0)

      def __init__(self, value, blueSideBotHeading, redSideBotHeading, armAngle, shooterVelocity):
        self._value_ = value
        self.m_blueSideBotHeading = blueSideBotHeading
        self.m_redSideBotHeading = redSideBotHeading
        self.m_armAngle = armAngle
        self.m_shooterVelocity = shooterVelocity

      def calculate(self, distance):
        shooterRegressionEquation = lambda x: (0.178571*(x**2)) + (0.107143*x) + 24.9286
        armRegressionEquation = lambda x: (0.175*(x**2)) + (1.605*x) + 8.88

        shooterSpeed = shooterRegressionEquation(distance)
        armAngle = armRegressionEquation(distance)

        return (shooterSpeed, armAngle)
    
