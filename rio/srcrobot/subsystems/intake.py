from constants import Constants
from commands2.subsystem import Subsystem
from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode, SupplyCurrentLimitConfiguration
from commands2 import InstantCommand
import math

class Intake(Subsystem):

    def __init__(self):
        self.intakeMotor = TalonSRX(Constants.IntakeConstants.kIntakeMotorID)
        self.intakeMotor.configFactoryDefault()

        current_limit = 10
        current_threshold = 20
        current_threshold_time = 0.1
        supply_configs = SupplyCurrentLimitConfiguration(True, current_limit, current_threshold, current_threshold_time)

        self.intakeMotor.configSupplyCurrentLimit(supply_configs)
        self.intakeMotor.configContinuousCurrentLimit(10)
        self.intakeMotor.enableCurrentLimit(True)

    def intake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, 1.0)).withName("Intake")
    
    def intakeOnce(self):
        return self.runOnce(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, 1.0)).withName("IntakeOnce")
    
    def outtake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, -1.0)).withName("Outtake")
    
    def instantStop(self):
        return InstantCommand(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)).withName("InstantStop")

    def stopIntake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)).withName("StopIntake")