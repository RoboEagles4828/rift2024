from commands2 import Command, FunctionalCommand, Subsystem
from wpilib import Spark

from gameState import GameState
from robotState import RobotState


# This subsystem will continued to be developed as the season progresses
class LED(Subsystem):
    gameState = GameState()
    robotState = RobotState()

    def __init__(self):
        self.kLedPort = 0
        self.blinkin = Spark(self.kLedPort)

        self.RED = 0.61
        self.PURPLE = 0.91
        self.GREEN = 0.77
        self.GOLD = 0.67

        self.last = None

    # The robot is empty, that is, it has no note.
    def empty(self):
        self.last = self.RED
        self.refreshLast()

    # The robot may have just gotten two notes!!!
    def suspectTwoNotes(self):
        self.last = self.GOLD
        self.refreshLast()

    # A note has been detected in the indexer.
    def noteDetected(self):
        self.last = self.PURPLE
        self.refreshLast()

    # The shooter and arm are ready for the selected shot.
    def readytoShoot(self):
        self.last = self.GREEN
        self.refreshLast()

    def getStateCommand(self) -> Command:
        """
        Return the default command for the LED subsystem. It initializes to empty
        and executes evaluating the game and robot states to set the LEDs for the
        rest of the match.
        """
        return FunctionalCommand(
            self.empty, self.setNextLED, lambda _: None, lambda: False, self
        )

    def setNextLED(self):
        if not self.gameState.hasNote():
            self.empty()
        # elif self.gameState.mayHaveTooManyNotes():
        #     self.suspectTwoNotes()
        elif self.robotState.isShooterReady():
            self.readytoShoot()
        else:
            self.noteDetected()

    def refreshLast(self):
        self.blinkin.set(self.last)
