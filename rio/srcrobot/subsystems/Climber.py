from commands2.subsystem import Subsystem
from constants import Constants
from phoenix5 import TalonFX, ControlMode

class Climber(Subsystem):
    def __init__(self):
        self.left = TalonFX(Constants.ClimberConstants.kLeftMotorID)
        self.right = TalonFX(Constants.ClimberConstants.kRightMotorID)

    def __setClimbers(self, newHeight):
        self.left.set(ControlMode.MotionMagic, newHeight)
        self.right.set(ControlMode.MotionMagic, newHeight)

    def climberUp(self):
        return self.run(lambda: self.__setClimbers(Constants.ClimberConstants.maxClimbHeight)).withName("ClimberUp")
    
    def climberDown(self):
        return self.run(lambda: self.__setClimbers(0)).withName("ClimberDown")