from commands2.subsystem import Subsystem
import wpimath.filter
import wpimath
import wpilib
import phoenix6 
from wpimath.geometry import Rotation2d, Pose3d, Pose2d, Rotation3d
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls import VelocityVoltage, VoltageOut, StrictFollower, DutyCycleOut
from phoenix6.signals import InvertedValue
from lib.mathlib.conversions import Conversions
import math
from constants import Constants
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from copy import deepcopy
from commands2 import InstantCommand
class Shooter(Subsystem):
    def __init__(self):
        self.kBottomShooterCANID = 6 
        self.kTopShooterCANID = 13
        
        self.bottomShooter = TalonFX(self.kBottomShooterCANID)
        self.topShooter = TalonFX(self.kTopShooterCANID)

        self.topShooterConfig = TalonFXConfiguration()

        self.topShooterConfig.slot0.k_p = 100.0
        self.topShooterConfig.slot0.k_i = 0.0
        self.topShooterConfig.slot0.k_d = 0.0

        self.bottomShooterConfig = deepcopy(self.topShooterConfig)

        self.topShooterConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.bottomShooterConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.topShooter.configurator.apply(self.topShooterConfig)
        self.bottomShooter.configurator.apply(self.bottomShooterConfig)

        self.VelocityControl = VelocityVoltage(0)
        self.VoltageControl = VoltageOut(0)
        self.percentOutput = DutyCycleOut(0)
        
    def shoot(self):
        # return InstantCommand(lambda: self.bottomShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi)))).alongWith(
        #     InstantCommand(lambda:self.topShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi))))
        # )
        return InstantCommand(lambda: self.bottomShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi)))).alongWith(
            InstantCommand(lambda:self.topShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi))))
        )

    def shootReverse(self):
        return InstantCommand(lambda: self.bottomShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(-Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi)))).alongWith(
            InstantCommand(lambda:self.topShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(-Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi))))
        )

    def stop(self):
        return InstantCommand(lambda: self.bottomShooter.set_control(self.VoltageControl.with_output(0))).alongWith(
            InstantCommand(lambda:self.topShooter.set_control(self.VoltageControl.with_output(0)))
        )