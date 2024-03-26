from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand
from pathplannerlib.auto import PathPlannerAuto
from subsystems.Swerve import Swerve
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
from wpimath.trajectory import Trajectory, TrajectoryGenerator, TrajectoryConfig

from pathplannerlib.path import *

class PathPlannerAutoRunner:
    def __init__(self, pathplannerauto, swerve):
        self.pathplannerauto: str = pathplannerauto
        self.swerve: Swerve = swerve
        self.auto = PathPlannerAuto(self.pathplannerauto)

    def getCommand(self):
        return SequentialCommandGroup(
            InstantCommand(lambda: self.swerve.setPose(self.auto.getStartingPoseFromAutoFile(self.pathplannerauto)), self.swerve),
            self.auto,
            InstantCommand(lambda: self.swerve.stop(), self.swerve)
        ).withName(self.pathplannerauto)