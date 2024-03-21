from commands2 import Subsystem
import wpilib
from wpilib import PneumaticsControlModule, Solenoid
from constants import Constants

# This subsystem will continued to be developed as the season progresses

class LED(Subsystem):

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


# Red will be used when robot is on
    def idle(self):
        return self.run(lambda: self.blinkin.set(self.RED))
# Purple Pattern will be used when a note is detected in the indexer
    def noteDetected(self):
        return self.run(lambda: self.blinkin.set(self.PURPLE))
# Green Pattern will be used when the flywheels begin to run
    def readytoShoot(self):
        return self.run(lambda: self.blinkin.set(self.GREEN))

# Green Pattern will be used when the flywheels begin to run    
    def autonomous(self):
        return self.run(lambda: self.blinkin.set(self.GOLD))