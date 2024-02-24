from constants import Constants
from commands2.subsystem import Subsystem
from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode
from commands2 import InstantCommand
import math

class Intake(Subsystem):

    def __init__(self):
        self.intakeMotor = TalonSRX(Constants.IntakeConstants.kIntakeMotorID)
        self.intakeMotor.configFactoryDefault()

    def intake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, 1.0))
    
    def outtake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, -1.0))

    def stopIntake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.Current, 0.0))