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

        self.indexerDiameter = 0.031
        self.indexerEncoderCPR = 2048.0
        self.indexerIntakeVelocity = -(Constants.IndexerConstants.kIndexerIntakeSpeedMS/(math.pi*self.indexerDiameter))*self.indexerEncoderCPR
        self.indexerShootVelocity = -(Constants.IndexerConstants.kIndexerMaxSpeedMS/(math.pi*self.indexerDiameter))*self.indexerEncoderCPR
        
    def getBeamBreakState(self):
        return not self.beamBreak.get()

    def indexerIntake(self):
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, self.indexerIntakeVelocity))

    def indexerShoot(self):
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, self.indexerShootVelocity))
    
    def indexerOuttake(self):
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, -self.indexerIntakeVelocity))

    def stopIndexer(self):
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.PercentOutput, 0.0))
