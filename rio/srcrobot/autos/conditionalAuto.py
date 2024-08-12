from constants import Constants
from subsystems.Swerve import Swerve
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDControllerRadians, HolonomicDriveController
from wpimath.geometry import Pose2d;
from wpimath.geometry import Rotation2d;
from wpimath.geometry import Translation2d;
from wpimath.trajectory import Trajectory;
from wpimath.trajectory import TrajectoryConfig;
from wpimath.trajectory import TrajectoryGenerator;
from commands2 import InstantCommand
from commands2 import SequentialCommandGroup
from commands2 import SwerveControllerCommand
from subsystems.Swerve import Swerve
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.Arm import Arm
from subsystems.Climber import Climber
from subsystems.Shooter import Shooter
from subsystems.Vision import Vision
from subsystems.Led import LED
import math as Math
import time

class exampleAuto:

    # Subsystems
    s_Swerve : Swerve = Swerve()
    s_Arm : Arm = Arm()
    s_Intake : Intake = Intake()
    s_Indexer : Indexer = Indexer()
    s_Shooter : Shooter = Shooter()
    s_Vision : Vision = Vision.getInstance()

    #s_Indexer.getBeamBreakState
    
    def dynamicShot():
        time.sleep(0.2)
        autoDynamicShot()
        time.sleep(0.2)
    
    def endAuton():
        pass

    def __init__(self, s_Swerve: Swerve):
        config = \
            TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
        
        config.setKinematics(Constants.Swerve.swerveKinematics)

        self.s_Swerve = s_Swerve
        self.centerToA2 = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(1.35, 5.5, Rotation2d.fromDegrees(0)),
                Pose2d(3.35, 5.5, Rotation2d.fromDegrees(0)),
                config
            )
        self.A2ToA1 = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(3.35, 5.5, Rotation2d.fromDegrees(0)),
                [Translation2d(2.03, 6.25)],
                Pose2d(3, 7.11, Rotation2d.fromDegrees(49.18)),
                config
            )
        self.A1ToM1 = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(3, 7.11, Rotation2d.fromDegrees(49.18)),
                Pose2d(8.5, 7.44, Rotation2d.fromDegrees(0)),
                config
            )
        self.M1ToShoot = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(8.5, 7.44, Rotation2d.fromDegrees(0)),
                Pose2d(3.2, 6.39, Rotation2d.fromDegrees(28.6)),
                config
            )
        self.M1ToM2 = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(3.2, 6.39, Rotation2d.fromDegrees(0)),
                Pose2d(8.21, 5.8, Rotation2d.fromDegrees(-90)),
                config
            )
        self.ShootToM2 = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(3.2, 6.39, Rotation2d.fromDegrees(28.6)),
                [Translation2d(6, 6.72)],
                Pose2d(8.21, 5.8, Rotation2d.fromDegrees(-23.2)),
                config
            )
        self.M2ToShoot = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(8.21, 5.8, Rotation2d.fromDegrees(-23.2)),
                [Translation2d(6, 6.72)],
                Pose2d(3.2, 6.39, Rotation2d.fromDegrees(28.6)),
                config
            )
        self.M2ToM3 = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(8.21, 5.8, Rotation2d.fromDegrees(-90)),
                Pose2d(8.21, 4.1, Rotation2d.fromDegrees(-90)),
                config
            )
        self.ShootToM3 = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(3.2, 6.39, Rotation2d.fromDegrees(28.6)),
                [Translation2d(6, 6.72)],
                Pose2d(8.21, 4.1, Rotation2d.fromDegrees(23)),
                config
            )
        self.M3ToShoot = \
            TrajectoryGenerator.generateTrajectory(
                Pose2d(8.21, 4.1, Rotation2d.fromDegrees(23)),
                [Translation2d(6, 6.72)],
                Pose2d(3.2, 6.39, Rotation2d.fromDegrees(28.6)),
                config
            )

        thetaController = \
            ProfiledPIDControllerRadians(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints)
        thetaController.enableContinuousInput(-Math.pi, Math.pi)

        self.holonomicController = HolonomicDriveController(
            PIDController(Constants.AutoConstants.kPXController, 0, 0),
            PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController
        )

        self.swerveControllerCommand = \
            SwerveControllerCommand(
                self.centerToA2,
                s_Swerve.getPose,
                Constants.Swerve.swerveKinematics,
                self.holonomicController,
                s_Swerve.setModuleStates,
                (s_Swerve,)
            )

    def getCommand(self):
        return SequentialCommandGroup(
            exampleAuto.dynamicShot(),
            InstantCommand(lambda: self.s_Swerve.setPose(self.centerToA2.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            exampleAuto.dynamicShot(),
            InstantCommand(lambda: self.s_Swerve.setPose(self.A2ToA1.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            exampleAuto.dynamicShot(),
            InstantCommand(lambda: self.s_Swerve.setPose(self.A1ToM1.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            InstantCommand(lambda: self.s_Swerve.setPose(self.M1ToM2.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if not exampleAuto.s_Intake.getIntakeBeamBreakState else SequentialCommandGroup(InstantCommand(lambda: self.s_Swerve.setPose(self.M1ToShoot.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand), exampleAuto.dynamicShot(), InstantCommand(lambda: self.s_Swerve.setPose(self.ShootToM2.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand)),
            InstantCommand(lambda: self.s_Swerve.setPose(self.M2ToM3.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if not exampleAuto.s_Intake.getIntakeBeamBreakState else SequentialCommandGroup(InstantCommand(lambda: self.s_Swerve.setPose(self.M2ToShoot.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand), exampleAuto.dynamicShot(), InstantCommand(lambda: self.s_Swerve.setPose(self.ShootToM3.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand)),
            exampleAuto.endAuton() if not exampleAuto.s_Intake.getIntakeBeamBreakState else SequentialCommandGroup(InstantCommand(lambda: self.s_Swerve.setPose(self.M3ToShoot.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand), exampleAuto.dynamicShot()),
        )
