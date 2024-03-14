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

import math as Math

class exampleAuto:
    def __init__(self, s_Swerve: Swerve):
        config = \
            TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared
            )
        
        config.setKinematics(Constants.Swerve.swerveKinematics)

        self.s_Swerve = s_Swerve

        # An example trajectory to follow.  All units in meters.
        self.exampleTrajectory = \
            TrajectoryGenerator.generateTrajectory(
                # Start at the origin facing the +X direction
                Pose2d(0, 2, Rotation2d(0)),
                # Pass through these two interior waypoints, making an 's' curve path
                [Translation2d(1, 3), Translation2d(2, 1)],
                # End 3 meters straight ahead of where we started, facing forward
                Pose2d(3, 2, Rotation2d(0)),
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
                self.exampleTrajectory,
                s_Swerve.getPose,
                Constants.Swerve.swerveKinematics,
                self.holonomicController,
                s_Swerve.setModuleStates,
                (s_Swerve,)
            )

    def getCommand(self):
        return InstantCommand(lambda: self.s_Swerve.setPose(self.exampleTrajectory.initialPose()), (self.s_Swerve,)).andThen(self.swerveControllerCommand)