from SwerveModule import SwerveModule
from constants import Constants

from subsystems.Vision import Vision

from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import SwerveModulePosition

from navx import AHRS

from wpimath.geometry import Pose2d
from wpimath.geometry import Rotation2d
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveModuleState
from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets
from commands2.subsystem import Subsystem

from wpilib.sysid import SysIdRoutineLog
from wpimath.units import volts

from pathplannerlib.auto import AutoBuilder, PathConstraints

from wpilib import DriverStation
from wpiutil import Sendable, SendableBuilder

class Swerve(Subsystem):
    swerveOdometry: SwerveDrive4PoseEstimator
    mSwerveMods: list[SwerveModule, SwerveModule, SwerveModule, SwerveModule]
    gyro: AHRS

    def __init__(self):
        self.gyro = AHRS.create_spi()
        self.gyro.calibrate()
        self.gyro.zeroYaw()

        self.mSwerveMods = [
            SwerveModule(0, Constants.Swerve.Mod0.constants),
            SwerveModule(1, Constants.Swerve.Mod1.constants),
            SwerveModule(2, Constants.Swerve.Mod2.constants),
            SwerveModule(3, Constants.Swerve.Mod3.constants)
        ]

        # self.swerveOdometry = SwerveDrive4Odometry(Constants.Swerve.swerveKinematics, self.getGyroYaw(), self.getModulePositions())
        self.swerveOdometry = SwerveDrive4PoseEstimator(Constants.Swerve.swerveKinematics, self.getGyroYaw(), self.getModulePositions(), Pose2d(0, 0, Rotation2d()))
        self.vision : Vision = Vision.getInstance()

        AutoBuilder.configureHolonomic(
            self.getPose,
            self.setPose,
            self.getRobotRelativeSpeeds,
            self.driveRobotRelative,
            Constants.Swerve.holonomicPathConfig,
            self.shouldFlipPath,
            self
        )

    def drive(self, translation: Translation2d, rotation, fieldRelative, isOpenLoop):
        discreteSpeeds = ChassisSpeeds.discretize(translation.X(), translation.Y(), rotation, 0.02)
        swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                discreteSpeeds.vx, 
                discreteSpeeds.vy, 
                discreteSpeeds.omega, 
                self.getHeading()
            )
        ) if fieldRelative else Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds(translation.X(), translation.Y(), rotation)
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed)

        self.mSwerveMods[0].setDesiredState(swerveModuleStates[0], isOpenLoop)
        self.mSwerveMods[1].setDesiredState(swerveModuleStates[1], isOpenLoop)
        self.mSwerveMods[2].setDesiredState(swerveModuleStates[2], isOpenLoop)
        self.mSwerveMods[3].setDesiredState(swerveModuleStates[3], isOpenLoop)
    
    def driveRobotRelative(self, speeds: ChassisSpeeds):
        self.drive(Translation2d(speeds.vx, speeds.vy), speeds.omega, False, False)

    def shouldFlipPath(self):
        return False
        # return DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    def setModuleStates(self, desiredStates):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed)
        
        self.mSwerveMods[0].setDesiredState(desiredStates[0], False)
        self.mSwerveMods[1].setDesiredState(desiredStates[1], False)
        self.mSwerveMods[2].setDesiredState(desiredStates[2], False)
        self.mSwerveMods[3].setDesiredState(desiredStates[3], False)

    def getModuleStates(self):
        states = list()
        states.append(self.mSwerveMods[0].getState())
        states.append(self.mSwerveMods[1].getState())
        states.append(self.mSwerveMods[2].getState())
        states.append(self.mSwerveMods[3].getState())
        return states

    def getModulePositions(self):
        positions = list()
        positions.append(self.mSwerveMods[0].getPosition())
        positions.append(self.mSwerveMods[1].getPosition())
        positions.append(self.mSwerveMods[2].getPosition())
        positions.append(self.mSwerveMods[3].getPosition())
        return positions

    def getModules(self):
        return self.mSwerveMods

    def getPose(self):
        return self.swerveOdometry.getEstimatedPosition()

    def setPose(self, pose):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), tuple(self.getModulePositions()), pose)

    def getHeading(self):
        return self.getPose().rotation()

    def setHeading(self, heading):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), tuple(self.getModulePositions()), Pose2d(self.getPose().translation(), heading))

    def zeroHeading(self):
        self.swerveOdometry.resetPosition(self.getGyroYaw(), tuple(self.getModulePositions()), Pose2d(self.getPose().translation(), Rotation2d()))

    def zeroYaw(self):
        self.gyro.zeroYaw()

    def getGyroYaw(self):
        return Rotation2d.fromDegrees(self.gyro.getYaw()).__mul__(-1)

    def resetModulesToAbsolute(self):
        self.mSwerveMods[0].resetToAbsolute()
        self.mSwerveMods[1].resetToAbsolute()
        self.mSwerveMods[2].resetToAbsolute()
        self.mSwerveMods[3].resetToAbsolute()

    def resetModuleZero(self):
        self.mSwerveMods[0].setDesiredStateNoOptimize(SwerveModuleState(0, Rotation2d(0)), False)
        self.mSwerveMods[1].setDesiredStateNoOptimize(SwerveModuleState(0, Rotation2d(0)), False)
        self.mSwerveMods[2].setDesiredStateNoOptimize(SwerveModuleState(0, Rotation2d(0)), False)
        self.mSwerveMods[3].setDesiredStateNoOptimize(SwerveModuleState(0, Rotation2d(0)), False)

    def getRobotRelativeSpeeds(self):
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(tuple(self.getModuleStates()))

    def driveMotorsVoltage(self, volts):
        self.mSwerveMods[0].driveMotorVoltage(volts)
        self.mSwerveMods[1].driveMotorVoltage(volts)
        self.mSwerveMods[2].driveMotorVoltage(volts)
        self.mSwerveMods[3].driveMotorVoltage(volts)

    def logDriveMotors(self, routineLog: SysIdRoutineLog):
        for mod in self.mSwerveMods:
            moduleName = "Module " + str(mod.moduleNumber)
            routineLog.motor(moduleName)\
                .voltage(mod.mDriveMotor.get_motor_voltage().value_as_double)\
                .position(mod.getPosition().distance)\
                .velocity(mod.getState().speed)
            
    def pathFindToPose(self, pose: Pose2d, constraints: PathConstraints, goalEndVel: float):
        return AutoBuilder.pathfindToPose(pose, constraints, goalEndVel)
            
    def stop(self):
        self.drive(Translation2d(), 0, False, True)

    def periodic(self):
        self.swerveOdometry.update(self.getGyroYaw(), tuple(self.getModulePositions()))
        optestimatedPose = self.vision.getEstimatedGlobalPose()

        if optestimatedPose is not None:
            estimatedPose = optestimatedPose
            self.swerveOdometry.addVisionMeasurement(Pose2d(estimatedPose.estimatedPose.toPose2d().X(), estimatedPose.estimatedPose.toPose2d().Y(), self.getHeading()), estimatedPose.timestampSeconds)
