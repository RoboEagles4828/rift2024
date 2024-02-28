from commands2 import SequentialCommandGroup, ConditionalCommand
from subsystems.indexer import Indexer
from robotState import RobotState

class ExecuteCommand(SequentialCommandGroup):
    def __init__(self, indexer: Indexer, robotState: RobotState):
        super().__init__()
        self.indexer = indexer
        self.robotState = robotState

        self.addRequirements(self.indexer)
        self.setName(f"Execute {self.robotState.m_gameState.getNextShot().name}")

        self.addCommands(
            ConditionalCommand(
                self.indexer.indexerShoot(),
                self.indexer.stopIndexer(),
                lambda: self.robotState.isRobotReady()
            )
        )
