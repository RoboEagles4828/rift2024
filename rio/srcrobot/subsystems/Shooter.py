from commands2.subsystem import Subsystem
from commands2.subsystem import Command
import wpimath.filter
import wpimath
import wpilib
import phoenix6 
from wpimath.geometry import Rotation2d, Pose3d, Pose2d, Rotation3d
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls import VelocityVoltage
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

        self.topVelocityControl = VelocityVoltage(0)
        self.bottomVelocityControl = VelocityVoltage(0)
        
    def shoot(self):
        self.topVelocityControl.velocity = Constants.ShooterConstants.shootSpeed
        self.bottomVelocityControl.velocity = -Constants.ShooterConstants.shootSpeed