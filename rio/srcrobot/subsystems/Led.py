from commands2 import Subsystem
import wpilib
from wpilib import Spark
from constants import Constants

# This subsystem will continued to be developed as the season progresses

class LED(Subsystem):

    def __init__(self):
        self.kLedPort = 0
        self.blinkin = Spark(self.kLedPort)

        self.RED = 0.61
        self.PURPLE = 0.91
        self.GREEN = 0.77
        self.GOLD = 0.67

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