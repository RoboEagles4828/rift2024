from commands2 import SequentialCommandGroup, InstantCommand
from pathplannerlib.auto import PathPlannerAuto
from subsystems.Swerve import Swerve

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
        )