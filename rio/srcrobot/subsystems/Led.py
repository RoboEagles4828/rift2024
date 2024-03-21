from commands2 import Subsystem, Command, FunctionalCommand
import wpilib
from wpilib import PneumaticsControlModule, Solenoid
from constants import Constants

from gameState import GameState
from robotState import RobotState

from wpilib import PneumaticsControlModule, Solenoid
from constants import Constants

# This subsystem will continued to be developed as the season progresses
class LED(Subsystem):
    gameState = GameState()
    robotState = RobotState()

    def __init__(self):
        self.kLedPort = 22
        self.pcm = PneumaticsControlModule(self.kLedPort)
        self.pcm.disableCompressor() # only using for leds, so don't even use the compressor

        self.redChan = Solenoid(0, PneumaticsControlModule, 0)
        self.greenChan = Solenoid(0, PneumaticsControlModule, 1)
        self.blueChan = Solenoid(0, PneumaticsControlModule, 2)

        self.BLACK = 0
        self.OFF = 0
        self.RED = 1
        self.YELLOW = 2
        self.GREEN = 3
        self.TEAL = 4
        self.BLUE = 5
        self.PURPLE = 6
        self.WHITE = 7

    def set(self, color): # BLACK - 0, RED - 1, YELLOW - 2, GREEN - 3, TEAL - 4, BLUE - 5, PURPLE - 6, WHITE - 7
        if color == 0: # BLACK / OFF
            self.redChan.set(False)
            self.greenChan.set(False)
            self.blueChan.set(False)
        elif color == 1: # RED
            self.redChan.set(True)
            self.greenChan.set(False)
            self.blueChan.set(False)
        elif color == 2: # YELLOW
            self.redChan.set(True)
            self.greenChan.set(True)
            self.blueChan.set(False)
        elif color == 3: # GREEN
            self.redChan.set(False)
            self.greenChan.set(True)
            self.blueChan.set(False)
        elif color == 4: # TEAL
            self.redChan.set(False)
            self.greenChan.set(True)
            self.blueChan.set(True)
        elif color == 5: # BLUE
            self.redChan.set(False)
            self.greenChan.set(False)
            self.blueChan.set(True)
        elif color == 6: # PURPLE
            self.redChan.set(True)
            self.greenChan.set(False)
            self.blueChan.set(True)
        elif color == 7: # WHITE / ON
            self.redChan.set(True)
            self.greenChan.set(True)
            self.blueChan.set(True)


        self.last = None

    # The robot is empty, that is, it has no note.
    def empty(self):
        self.last = self.RED
        self.refreshLast()

    # The robot may have just gotten two notes!!!
    def suspectTwoNotes(self):
        self.last = self.YELLOW
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
        self.set(self.last)
