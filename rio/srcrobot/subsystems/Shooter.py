from commands2.subsystem import Subsystem
from commands2.subsystem import Command
import wpimath.filter
import wpimath
import wpilib
import phoenix6 
from wpimath.geometry import Rotation2d, Pose3d, Pose2d, Rotation3d
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls import VelocityVoltage, VoltageOut
from lib.mathlib.conversions import Conversions
import math
from constants import Constants
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
class Shooter(Subsystem):
    def __init__(self):
        self.kBottomShooterCANID = 6 
        self.kTopShooterCANID = 13 
        
        self.bottomShooter = TalonFX(self.kBottomShooterCANID)
        self.topShooter = TalonFX(self.kBottomShooterCANID)

        self.VelocityControl = VelocityVoltage(0)
        self.VoltageControl = VoltageOut(0)
        
    def shoot(self):
        self.bottomShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(Constants.ShooterConstants.kShootSpeed,  0.101 * math.pi)))
        self.topShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(-Constants.ShooterConstants.kShootSpeed,  0.101 * math.pi)))

    def shootReverse(self):
        self.bottomShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(-Constants.ShooterConstants.kShootSpeed,  0.101 * math.pi)))
        self.topShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(Constants.ShooterConstants.kShootSpeed,  0.101 * math.pi)))


    def stop(self):
        self.bottomShooter.set_control(self.VoltageControl.with_output(0))
        self.topShooter.set_control(self.VoltageControl.with_output(0))