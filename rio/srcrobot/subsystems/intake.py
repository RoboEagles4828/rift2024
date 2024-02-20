from constants import Constants
from commands2.subsystem import Subsystem
from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode
import math

class Intake(Subsystem):

    def __init__(self):
        self.intakeMotor = TalonSRX(Constants.IntakeConstants.kIntakeMotorID)
        self.intakeMotor.configFactoryDefault()

    def setIntakeSpeed(self, speed: float):
        self.intakeMotor.set(TalonSRXControlMode.Velocity, (speed/(math.pi*0.031))*2048.0)

    def stopIntake(self):
        self.intakeMotor.set(TalonSRXControlMode.Velocity, 0.0)