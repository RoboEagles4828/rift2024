from constants import Constants
from commands2.subsystem import Subsystem
from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode
import math

class Indexer(Subsystem):
    def __init__(self):
        self.indexerMotor = TalonSRX(Constants.IndexerConstants.kIndexerMotorID)
        self.indexerMotor.configFactoryDefault()

    def indexerIntake(self):
        self.indexerMotor.set(TalonSRXControlMode.Velocity, (Constants.IndexerConstants.kIndexerSpeed/(math.pi*0.031))*2048.0)

    def indexerShoot(self):
        self.indexerMotor.set(TalonSRXControlMode.Velocity, -(Constants.IndexerConstants.kIndexerSpeed/(math.pi*0.031))*2048.0)

    def stopIndexer(self):
        self.indexerMotor.set(TalonSRXControlMode.Velocity, 0.0)
