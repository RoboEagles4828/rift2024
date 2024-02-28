from commands2 import Command, ParallelCommandGroup
from constants import Constants
from robotState import RobotState
from commands.TurnInPlace import TurnInPlace
from subsystems.Arm import Arm
from subsystems.indexer import Indexer
from subsystems.Shooter import Shooter
from subsystems.Swerve import Swerve
from wpilib import DriverStation

class QueueCommand(ParallelCommandGroup):
    def __init__(self, shot: Constants.NextShot, arm: Arm, shooter: Shooter, swerve: Swerve, robotState: RobotState, suppliers: list):
        super().__init__()
        self.shot = shot
        self.arm = arm
        self.shooter = shooter
        self.swerve = swerve
        self.robotState = robotState

        self.robotState.m_gameState.setNextShot(self.shot)

        self.arm_angle = self.robotState.m_gameState.getNextShot().m_armAngle
        self.shooter_speed = self.robotState.m_gameState.getNextShot().m_shooterVelocity
        self.robot_heading = self.robotState.m_gameState.getNextShotRobotAngle()

        self.addRequirements(self.arm, self.indexer, self.shooter, self.swerve)
        self.setName(f"Queue {self.shot.name}")

        self.addCommands(
            self.arm.servoArmToTarget(self.arm_angle),
            self.shooter.setShooterVelocity(self.shooter_speed),
            TurnInPlace(self.swerve, lambda: self.robot_heading, suppliers[0], suppliers[1], suppliers[2], suppliers[3], suppliers[4]).withTimeout(2.0)
        )


