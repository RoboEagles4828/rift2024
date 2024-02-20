from SwerveModule import SwerveModule
from constants import Constants

from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.kinematics import SwerveDrive4Odometry
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

from pathplannerlib.auto import AutoBuilder

from wpilib import DriverStation

class Swerve(Subsystem):
    swerveOdometry: SwerveDrive4Odometry
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

        self.swerveOdometry = SwerveDrive4Odometry(Constants.Swerve.swerveKinematics, -self.getGyroYaw(), self.getModulePositions())

        AutoBuilder.configureHolonomic(
            self.getPose,
            self.setPose,
            self.getRobotRelativeSpeeds,
            self.driveRobotRelative,
            Constants.Swerve.holonomicPathConfig,
            self.shouldFlipPath,
            self
        )

        for mod in self.mSwerveMods:
            Shuffleboard.getTab("Diagnostics").addDouble("Mod " + str(mod.moduleNumber) + " CANcoder", mod.getCANcoder().degrees)\
                .withPosition(0, mod.moduleNumber)\
                .withSize(2, 1)\
                .withWidget(BuiltInWidgets.kDial)
            Shuffleboard.getTab("Diagnostics").addDouble("Mod " + str(mod.moduleNumber) + " Angle", mod.getPosition().angle.degrees)\
                .withPosition(2, mod.moduleNumber)\
                .withSize(2, 1)\
                .withWidget(BuiltInWidgets.kDial)
            Shuffleboard.getTab("Diagnostics").addDouble("Mod " + str(mod.moduleNumber) + " Velocity", lambda: mod.getState().speed)\
                .withPosition(4, mod.moduleNumber)\
                .withSize(2, 1)\
                .withWidget(BuiltInWidgets.kNumberBar)
            Shuffleboard.update()

    def drive(self, translation: Translation2d, rotation, fieldRelative, isOpenLoop):
        discreteSpeeds = ChassisSpeeds.discretize(translation.X(), translation.Y(), rotation, 0.02)
        swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                discreteSpeeds.vx, 
                discreteSpeeds.vy, 
                discreteSpeeds.omega, 
                self.getGyroYaw()
            )
        ) if fieldRelative else Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds(translation.X(), translation.Y(), rotation)
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed)

        for mod in self.mSwerveMods:
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop)    
    
    def driveRobotRelative(self, speeds: ChassisSpeeds):
        self.drive(Translation2d(speeds.vx, speeds.vy), speeds.omega, False, False)

    def shouldFlipPath(self):
        return False
        #return DriverStation.getAlliance() == DriverStation.Alliance.kBlue

    def setModuleStates(self, desiredStates):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed)
        
        for mod in self.mSwerveMods:
            mod.setDesiredState(desiredStates[mod.moduleNumber], False)

    def getModuleStates(self):
        states = [SwerveModuleState]*4
        for mod in self.mSwerveMods:
            states[mod.moduleNumber] = mod.getState()
        return states

    def getModulePositions(self):
        positions = [SwerveModulePosition]*4
        for mod in self.mSwerveMods:
            positions[mod.moduleNumber] = mod.getPosition()
        return positions

    def getModules(self):
        return self.mSwerveMods

    def getPose(self):
        return self.swerveOdometry.getPose()

    def setPose(self, pose):
        self.swerveOdometry.resetPosition(-self.getGyroYaw(), tuple(self.getModulePositions()), pose)

    def getHeading(self):
        return self.getPose().rotation()

    def setHeading(self, heading):
        self.swerveOdometry.resetPosition(-self.getGyroYaw(), tuple(self.getModulePositions()), Pose2d(self.getPose().translation(), heading))

    def zeroHeading(self):
        self.swerveOdometry.resetPosition(-self.getGyroYaw(), tuple(self.getModulePositions()), Pose2d(self.getPose().translation(), Rotation2d()))

    def zeroYaw(self):
        self.gyro.zeroYaw()

    def getGyroYaw(self):
        return Rotation2d.fromDegrees(self.gyro.getYaw()).__mul__(-1)

    def resetModulesToAbsolute(self):
        for mod in self.mSwerveMods:
            mod.resetToAbsolute()

    def resetModuleZero(self):
        for mod in self.mSwerveMods:
            mod.setDesiredStateNoOptimize(SwerveModuleState(0, Rotation2d(0)), False)

    def getRobotRelativeSpeeds(self):
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(tuple(self.getModuleStates()))

    def driveMotorsVoltage(self, volts):
        for mod in self.mSwerveMods:
            mod.driveMotorVoltage(volts)

    def logDriveMotors(self, routineLog: SysIdRoutineLog):
        for mod in self.mSwerveMods:
            moduleName = "Module " + str(mod.moduleNumber)
            routineLog.motor(moduleName)\
                .voltage(mod.mDriveMotor.get_motor_voltage().value_as_double)\
                .position(mod.getPosition().distance)\
                .velocity(mod.getState().speed)
            
    def stop(self):
        self.drive(Translation2d(), 0, False, True)

    def periodic(self):
        self.swerveOdometry.update(-self.getGyroYaw(), tuple(self.getModulePositions()))