from constants import Constants
from commands2.subsystem import Subsystem
from commands2.cmd import waitSeconds
from phoenix5 import TalonSRX, TalonSRXConfiguration, TalonSRXControlMode, TalonSRXFeedbackDevice, NeutralMode
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
        self.indexerMotor.setNeutralMode(NeutralMode.Brake)

        self.indexerDiameter = 0.031
        self.indexerEncoderCPR = 2048.0
        self.indexerIntakeVelocity = -(Constants.IndexerConstants.kIndexerIntakeSpeedMS/(math.pi*self.indexerDiameter))*self.indexerEncoderCPR
        self.indexerShootVelocity = -(Constants.IndexerConstants.kIndexerMaxSpeedMS/(math.pi*self.indexerDiameter))*self.indexerEncoderCPR
        
    def getBeamBreakState(self):
        return not bool(self.beamBreak.get())

    def indexerIntake(self):
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, self.indexerIntakeVelocity)).withName("Intake")
    
    def indexerIntakeOnce(self):
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, self.indexerIntakeVelocity)).until(self.getBeamBreakState)\
            .andThen(self.indexerOuttake().withTimeout(0.15))\
            .finallyDo(lambda interrupted: self.stopIndexer()).withName("IntakeOnce")

    def indexerShoot(self): 
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, self.indexerShootVelocity)).until(lambda: not self.getBeamBreakState()).withTimeout(1.0).andThen(waitSeconds(0.2)).finallyDo(self.stopIndexer()).withName("Shoot")
    
    def indexerOuttake(self):
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, -self.indexerIntakeVelocity)).withName("Outtake")
    
    def setIndexerVelocity(self, velocity):
        return self.run(lambda: self.indexerMotor.set(TalonSRXControlMode.Velocity, -(velocity/(math.pi*self.indexerDiameter))*self.indexerEncoderCPR)).withName("SetIndexerVelocity")
    
    def levelIndexer(self):
        return self.indexerIntake().withTimeout(0.1)\
        .andThen(self.indexerOuttake().withTimeout(0.1))\
        .andThen(self.indexerIntake().until(self.getBeamBreakState))\
        .andThen(self.indexerOuttake().withTimeout(0.1))\
        .andThen(self.stopIndexer()).withName("LevelIndexer")

    def instantStop(self):
        return InstantCommand(lambda: self.indexerMotor.set(TalonSRXControlMode.PercentOutput, 0.0)).withName("InstantStop")

    def stopIndexer(self):
        return self.runOnce(lambda: self.indexerMotor.set(TalonSRXControlMode.PercentOutput, 0.0)).withName("StopIndexer")
