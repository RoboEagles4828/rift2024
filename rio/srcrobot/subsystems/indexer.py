from constants import Constants
from commands2.subsystem import Subsystem
from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode, TalonSRXFeedbackDevice
from wpilib import DigitalInput
from commands2 import InstantCommand
import math

class Indexer(Subsystem):
    def __init__(self):
        self.indexerMotor = TalonSRX(Constants.IndexerConstants.kIndexerMotorID)
        self.indexerMotor.configFactoryDefault()

        self.beamBreak = DigitalInput(Constants.IndexerConstants.kBeamBreakerID)

        self.indexerMotor.setInverted(True)
        self.indexerMotor.config_kP(0, 2.0)

    def getBeamBreakState(self):
        return -self.beamBreak.get()

    def indexerIntake(self):
        return InstantCommand(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, -(Constants.IndexerConstants.kIndexerSpeed/(math.pi*0.031))*2048.0))

    def indexerShoot(self):
        return InstantCommand(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, -(Constants.IndexerConstants.kIndexerSpeed/(math.pi*0.031))*2048.0))

    def stopIndexer(self):
        return InstantCommand(lambda: self.indexerMotor.set(TalonSRXControlMode.PercentOutput, 0.0))
