from constants import Constants
from commands2.subsystem import Subsystem
from phoenix5 import TalonFX, TalonFXConfiguration, TalonFXControlMode
import math

class Intake(Subsystem):

    def __init__(self):
        self.intakeMotor = TalonFX(Constants.IntakeConstants.kIntakeMotorID)
        self.intakeMotor.configFactoryDefault()

    def setIntakeSpeed(self, speed: float):
        self.intakeMotor.set(TalonFXControlMode.Velocity, (speed/(math.pi*0.031))*2048.0)

    def stopIntake(self):
        self.intakeMotor.set(TalonFXControlMode.Velocity, 0.0)