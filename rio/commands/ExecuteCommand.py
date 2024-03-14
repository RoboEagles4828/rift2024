from commands2 import ParallelCommandGroup, RunCommand, InstantCommand
from commands.TurnInPlace import TurnInPlace
from subsystems.Arm import Arm
from subsystems.Shooter import Shooter
from subsystems.Swerve import Swerve

from wpimath.geometry import Rotation2d

from robotState import RobotState

class ExecuteCommand(ParallelCommandGroup):
    def __init__(self, arm : Arm, shooter : Shooter, swerve : Swerve, translationSupplier, strafeSupplier, rotationSupplier, robotCentricSupplier):
        super().__init__()
        self.robotState = RobotState()
        self.arm = arm
        self.shooter = shooter
        self.swerve = swerve

        self.arm_angle = self.robotState.m_gameState.getNextShot().m_armAngle
        self.shooter_velocity = self.robotState.m_gameState.getNextShot().m_shooterVelocity
        self.robot_angle = self.robotState.m_gameState.getNextShotRobotAngle()

        self.setName(f"Execute {self.robotState.m_gameState.getNextShot().name}")

        self.addCommands(
            self.arm.servoArmToTarget(self.arm_angle).withTimeout(2.0),
            self.shooter.shootVelocity(self.shooter_velocity),
            TurnInPlace(self.swerve, lambda: Rotation2d.fromDegrees(self.robot_angle), translationSupplier, strafeSupplier, rotationSupplier, robotCentricSupplier).repeatedly()
        )

    def initialize(self):
        super().initialize()
        self.robotState = RobotState()
        self.arm_angle = self.robotState.m_gameState.getNextShot().m_armAngle
        self.shooter_velocity = self.robotState.m_gameState.getNextShot().m_shooterVelocity
        self.robot_angle = self.robotState.m_gameState.getNextShotRobotAngle()

        self.setName(f"Execute {self.robotState.m_gameState.getNextShot().name}")
