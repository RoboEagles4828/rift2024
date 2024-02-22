from commands2.subsystem import Subsystem
from constants import Constants
import phoenix5

class climber(Subsystem):
    def __init__(self):
        self.left = phoenix5.TalonFX(Constants.ClimberConstants.kLeftMotorID)
        self.right = phoenix5.TalonFX(Constants.ClimberConstants.kRightMotorID)
        self.maxClimbHeight = 0 #in ticks

    def climberUp(self):
        self.left.set(phoenix5.ControlMode.MotionMagic, self.maxClimbHeight)
        self.right.set(phoenix5.ControlMode.MotionMagic, self.maxClimbHeight)

    def climberDown(self):
        self.left.set(phoenix5.ControlMode.MotionMagic, 0)
        self.right.set(phoenix5.ControlMode.MotionMagic, 0)