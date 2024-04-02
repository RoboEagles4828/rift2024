from constants import Constants
from commands2.subsystem import Subsystem
from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode, SupplyCurrentLimitConfiguration
from commands2 import InstantCommand
from wpilib import DigitalInput
from wpimath.filter import Debouncer
import math

class Intake(Subsystem):

    def __init__(self):
        self.intakeMotor = TalonSRX(Constants.IntakeConstants.kIntakeMotorID)
        self.intakeMotor.configFactoryDefault()

        current_limit = 40
        current_threshold = 60
        current_threshold_time = 3.0
        supply_configs = SupplyCurrentLimitConfiguration(True, current_limit, current_threshold, current_threshold_time)

        self.intakeMotor.configSupplyCurrentLimit(supply_configs)

        self.intakeSpeed = 1.0

        self.intakeBeam = DigitalInput(1)
        # self.intakeMotor.configContinuousCurrentLimit(30)
        # self.intakeMotor.enableCurrentLimit(True)

        self.debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)

    def getIntakeBeamBreakState(self):
        return self.debouncer.calculate(not bool(self.intakeBeam.get()))

    def intake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, self.intakeSpeed)).withName("Intake")
    
    def intakeOnce(self):
        return self.runOnce(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, self.intakeSpeed)).withName("IntakeOnce")
    
    def outtake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, -self.intakeSpeed)).withName("Outtake")
    
    def instantStop(self):
        return InstantCommand(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)).withName("InstantStop")

    def stopIntake(self):
        return self.run(lambda: self.intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0)).withName("StopIntake")