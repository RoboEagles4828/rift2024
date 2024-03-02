from commands2 import Command, ParallelCommandGroup, InstantCommand, SequentialCommandGroup
from constants import Constants
from robotState import RobotState
from commands.TurnInPlace import TurnInPlace
from subsystems.Arm import Arm
from subsystems.indexer import Indexer
from subsystems.Shooter import Shooter
from subsystems.Swerve import Swerve
from wpilib import DriverStation

class QueueCommand(SequentialCommandGroup):
    def __init__(self, shot: Constants.NextShot):
        super().__init__()
        self.shot = shot
        self.robotState = RobotState()

        self.robotState.m_gameState.setNextShot(self.shot)

        self.setName(f"Queue {self.shot.name}")

        self.addCommands(
            InstantCommand(lambda: self.robotState.m_gameState.setNextShot(self.shot)),
        )

        


