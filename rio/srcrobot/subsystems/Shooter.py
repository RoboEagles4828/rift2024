from commands2.subsystem import Subsystem
from commands2.subsystem import Command
import wpimath.filter
import wpimath
import wpilib
import phoenix6 
from wpimath.geometry import Rotation2d, Pose3d, Pose2d, Rotation3d
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls import VelocityVoltage, VoltageOut, StrictFollower
from phoenix6.signals import InvertedValue
from lib.mathlib.conversions import Conversions
import math
from constants import Constants
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from copy import deepcopy
class Shooter(Subsystem):
    def __init__(self):
        self.kBottomShooterCANID = 6 
        self.kTopShooterCANID = 13 
        
        self.bottomShooter = TalonFX(self.kBottomShooterCANID)
        self.topShooter = TalonFX(self.kBottomShooterCANID)

        self.topShooterConfig = TalonFXConfiguration()

        self.topShooterConfig.slot0.k_p = 0.1
        self.topShooterConfig.slot0.k_i = 0.0
        self.topShooterConfig.slot0.k_d = 0.0
        self.topShooterConfig.slot0.k_v = 0.1
        self.topShooterConfig.slot0.k_a = 0.0
        self.topShooterConfig.slot0.k_s = 0.0

        self.bottomShooterConfig = deepcopy(self.topShooterConfig)

        self.topShooterConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.bottomShooterConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.topShooter.configurator.apply(self.topShooterConfig)
        self.bottomShooter.configurator.apply(self.bottomShooterConfig)

        self.bottomShooter.set_control(StrictFollower(self.topShooter.device_id))

        self.VelocityControl = VelocityVoltage(0)
        self.VoltageControl = VoltageOut(0)
        
    def shoot(self):
        # self.bottomShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi)))
        self.topShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi)))

    def shootReverse(self):
        # self.bottomShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(-Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi)))
        self.topShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(-Constants.ShooterConstants.kPodiumShootSpeed,  0.101 * math.pi)))

    def getSpeed(self) -> float:
        # TODO implemenent this!!!
        return 5.0

    def stop(self):
        self.topShooter.set_control(self.VoltageControl.with_output(0))