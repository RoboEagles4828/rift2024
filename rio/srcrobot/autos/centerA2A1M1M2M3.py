#*DONOTREMOVE[{"x":1.3600420533333335,"y":5.621111154362416,"theta":0,"type":"Shot","id":0,"status":null,"delay":0},{"x":2.903873573333333,"y":5.602741510067114,"theta":0,"type":"Shot","id":1,"status":null,"delay":0},{"x":1.9297894,"y":6.172200483221476,"theta":0,"type":"Translation","id":2,"status":null,"delay":0},{"x":2.7384630533333336,"y":6.9437255436241605,"theta":-45,"type":"Shot","id":3,"status":null,"delay":0},{"x":8.16025232,"y":7.402966651006711,"theta":0,"type":"Regular","id":4,"status":null,"delay":0},{"x":2.6833262133333333,"y":6.172200483221476,"theta":-10,"type":"Shot","id":5,"status":null,"delay":0},{"x":5.936399773333333,"y":6.668180879194631,"theta":0,"type":"Translation","id":6,"status":null,"delay":0},{"x":8.16025232,"y":5.8599165302013425,"theta":0,"type":"Regular","id":7,"status":null,"delay":0},{"x":5.862883986666667,"y":6.686550523489933,"theta":0,"type":"Translation","id":8,"status":null,"delay":0},{"x":2.664947266666667,"y":6.190570127516779,"theta":-15,"type":"Shot","id":9,"status":null,"delay":0},{"x":4.888799813333334,"y":4.335236053691275,"theta":45,"type":"Regular","id":10,"status":null,"delay":0},{"x":8.141873373333334,"y":4.188278899328858,"theta":0,"type":"Regular","id":11,"status":null,"delay":0}]DONOTREMOVE*
from constants import Constants
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

class centerA2A1M1M2M3:

    # Subsystems
    s_Swerve : Swerve = Swerve()
    s_Arm : Arm = Arm()
    s_Intake : Intake = Intake()
    s_Indexer : Indexer = Indexer()
    s_Shooter : Shooter = Shooter()
    s_Vision : Vision = Vision.getInstance()

    def dynamicShot():
        time.sleep(0.2)
        autoDynamicShot()
        time.sleep(0.2)

    def endAuton():
        pass

    def doNothing():
        pass

    def __init__(self, s_Swerve: Swerve):
        config =             TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )

        config.setKinematics(Constants.Swerve.swerveKinematics)

        self.s_Swerve = s_Swerve
        self.Trajectory0Blue =             TrajectoryGenerator.generateTrajectory( #1.544m (0.406sec)
                Pose2d(1.3600420533333335, 5.621111154362416, Rotation2d.fromDegrees(0)),
                Pose2d(2.903873573333333, 5.602741510067114, Rotation2d.fromDegrees(0)),
                config
            )
        self.Trajectory1Blue =             TrajectoryGenerator.generateTrajectory( #2.246m (0.591sec)
                Pose2d(2.903873573333333, 5.602741510067114, Rotation2d.fromDegrees(0)),
                [Translation2d(1.9297894, 6.172200483221476)],
                Pose2d(2.7384630533333336, 6.9437255436241605, Rotation2d.fromDegrees(-45)),
                config
            )
        self.Trajectory2Blue =             TrajectoryGenerator.generateTrajectory( #5.441m (1.432sec)
                Pose2d(2.7384630533333336, 6.9437255436241605, Rotation2d.fromDegrees(-45)),
                Pose2d(8.16025232, 7.402966651006711, Rotation2d.fromDegrees(0)),
                config
            )
        self.Trajectory3Blue =             TrajectoryGenerator.generateTrajectory( #5.614m (1.477sec)
                Pose2d(8.16025232, 7.402966651006711, Rotation2d.fromDegrees(0)),
                Pose2d(2.6833262133333333, 6.172200483221476, Rotation2d.fromDegrees(-10)),
                config
            )
        self.Trajectory4Blue =             TrajectoryGenerator.generateTrajectory( #5.657m (1.489sec)
                Pose2d(2.6833262133333333, 6.172200483221476, Rotation2d.fromDegrees(-10)),
                [Translation2d(5.936399773333333, 6.668180879194631)],
                Pose2d(8.16025232, 5.8599165302013425, Rotation2d.fromDegrees(0)),
                config
            )
        self.Trajectory5Blue =             TrajectoryGenerator.generateTrajectory( #5.678m (1.494sec)
                Pose2d(8.16025232, 5.8599165302013425, Rotation2d.fromDegrees(0)),
                [Translation2d(5.862883986666667, 6.686550523489933)],
                Pose2d(2.664947266666667, 6.190570127516779, Rotation2d.fromDegrees(-15)),
                config
            )
        self.Trajectory6Blue =             TrajectoryGenerator.generateTrajectory( #2.896m (0.762sec)
                Pose2d(2.664947266666667, 6.190570127516779, Rotation2d.fromDegrees(-15)),
                Pose2d(4.888799813333334, 4.335236053691275, Rotation2d.fromDegrees(45)),
                config
            )
        self.Trajectory7Blue =             TrajectoryGenerator.generateTrajectory( #3.256m (0.857sec)
                Pose2d(4.888799813333334, 4.335236053691275, Rotation2d.fromDegrees(45)),
                Pose2d(8.141873373333334, 4.188278899328858, Rotation2d.fromDegrees(0)),
                config
            )
        self.Trajectory0Red =               TrajectoryGenerator.generateTrajectory( #1.544m (0.406sec)
                  Pose2d(15.181009946666666, 5.621111154362416, Rotation2d.fromDegrees(180)),
                  Pose2d(13.637178426666667, 5.602741510067114, Rotation2d.fromDegrees(180)),
                  config
              )
        self.Trajectory1Red =               TrajectoryGenerator.generateTrajectory( #2.246m (0.591sec)
                  Pose2d(13.637178426666667, 5.602741510067114, Rotation2d.fromDegrees(180)),
                  [Translation2d(14.6112626, 6.172200483221476)],
                  Pose2d(13.802588946666667, 6.9437255436241605, Rotation2d.fromDegrees(225)),
                  config
              )
        self.Trajectory2Red =               TrajectoryGenerator.generateTrajectory( #5.441m (1.432sec)
                  Pose2d(13.802588946666667, 6.9437255436241605, Rotation2d.fromDegrees(225)),
                  Pose2d(8.38079968, 7.402966651006711, Rotation2d.fromDegrees(180)),
                  config
              )
        self.Trajectory3Red =               TrajectoryGenerator.generateTrajectory( #5.614m (1.477sec)
                  Pose2d(8.38079968, 7.402966651006711, Rotation2d.fromDegrees(180)),
                  Pose2d(13.857725786666666, 6.172200483221476, Rotation2d.fromDegrees(190)),
                  config
              )
        self.Trajectory4Red =               TrajectoryGenerator.generateTrajectory( #5.657m (1.489sec)
                  Pose2d(13.857725786666666, 6.172200483221476, Rotation2d.fromDegrees(190)),
                  [Translation2d(10.604652226666667, 6.668180879194631)],
                  Pose2d(8.38079968, 5.8599165302013425, Rotation2d.fromDegrees(180)),
                  config
              )
        self.Trajectory5Red =               TrajectoryGenerator.generateTrajectory( #5.678m (1.494sec)
                  Pose2d(8.38079968, 5.8599165302013425, Rotation2d.fromDegrees(180)),
                  [Translation2d(10.678168013333334, 6.686550523489933)],
                  Pose2d(13.876104733333333, 6.190570127516779, Rotation2d.fromDegrees(195)),
                  config
              )
        self.Trajectory6Red =               TrajectoryGenerator.generateTrajectory( #2.896m (0.762sec)
                  Pose2d(13.876104733333333, 6.190570127516779, Rotation2d.fromDegrees(195)),
                  Pose2d(11.652252186666667, 4.335236053691275, Rotation2d.fromDegrees(135)),
                  config
              )
        self.Trajectory7Red =               TrajectoryGenerator.generateTrajectory( #3.256m (0.857sec)
                  Pose2d(11.652252186666667, 4.335236053691275, Rotation2d.fromDegrees(135)),
                  Pose2d(8.399178626666666, 4.188278899328858, Rotation2d.fromDegrees(180)),
                  config
              )
        
        thetaController =             ProfiledPIDControllerRadians(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints)
        thetaController.enableContinuousInput(-Math.pi, Math.pi)

        self.holonomicController = HolonomicDriveController(
            PIDController(Constants.AutoConstants.kPXController, 0, 0),
            PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController
        )

        self.swerveControllerCommand =             SwerveControllerCommand(
                self.centerToA2,
                s_Swerve.getPose,
                Constants.Swerve.swerveKinematics,
                self.holonomicController,
                s_Swerve.setModuleStates,
                (s_Swerve,)
            )

    def getCommand(self):
        return SequentialCommandGroup(
            self.dynamicShot() if self.s_Indexer.getBeamBreakState else (SequentialCommandGroup(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0), self.dynamicShot()) if self.s_Intake.getIntakeBeamBreakState else self.doNothing()),
            self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0),
            delay(0),
            InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory0Red.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if self.s_Swerve.shouldFlipPath else InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory0Blue.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            self.dynamicShot() if self.s_Indexer.getBeamBreakState else (SequentialCommandGroup(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0), self.dynamicShot()) if self.s_Intake.getIntakeBeamBreakState else self.doNothing()),
            self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0),
            delay(0),
            InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory1Red.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if self.s_Swerve.shouldFlipPath else InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory1Blue.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            self.dynamicShot() if self.s_Indexer.getBeamBreakState else (SequentialCommandGroup(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0), self.dynamicShot()) if self.s_Intake.getIntakeBeamBreakState else self.doNothing()),
            self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0),
            delay(0),
            InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory2Red.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if self.s_Swerve.shouldFlipPath else InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory2Blue.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0),
            delay(0),
            InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory3Red.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if self.s_Swerve.shouldFlipPath else InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory3Blue.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            self.dynamicShot() if self.s_Indexer.getBeamBreakState else (SequentialCommandGroup(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0), self.dynamicShot()) if self.s_Intake.getIntakeBeamBreakState else self.doNothing()),
            self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0),
            delay(0),
            InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory4Red.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if self.s_Swerve.shouldFlipPath else InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory4Blue.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0),
            delay(0),
            InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory5Red.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if self.s_Swerve.shouldFlipPath else InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory5Blue.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            self.dynamicShot() if self.s_Indexer.getBeamBreakState else (SequentialCommandGroup(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0), self.dynamicShot()) if self.s_Intake.getIntakeBeamBreakState else self.doNothing()),
            self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0),
            delay(undefined),
            InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory6Red.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if self.s_Swerve.shouldFlipPath else InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory6Blue.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withTimeout(5.0),
            delay(undefined),
            InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory7Red.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand) if self.s_Swerve.shouldFlipPath else InstantCommand(lambda: self.s_Swerve.setPose(self.Trajectory7Blue.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand),
            self.endAuton())
