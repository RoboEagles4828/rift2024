from commands2.subsystem import Subsystem
import wpimath.filter
import wpimath
import wpilib
import phoenix6 
from wpimath.geometry import Rotation2d, Pose3d, Pose2d, Rotation3d
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls import VelocityVoltage, VoltageOut, StrictFollower, DutyCycleOut, PositionVoltage
from phoenix6.signals import InvertedValue, NeutralModeValue
from lib.mathlib.conversions import Conversions
import math
from constants import Constants
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from copy import deepcopy
from commands2 import InstantCommand

class Climber(Subsystem):
    def __init__(self):
        self.kLeftClimberCANID = 4
        self.kRightClimberCANID = 15

        self.leftClimber = TalonFX(self.kLeftClimberCANID)
        self.rightClimber = TalonFX(self.kRightClimberCANID)
        self.gearRatio = 1.0/14.0
        self.wheelCircumference = 0.01905*math.pi
        self.leftClimberConfig = TalonFXConfiguration()
        self.rightClimberConfig = TalonFXConfiguration()

        self.leftClimberConfig.slot0.k_p = 1.0
        self.leftClimberConfig.slot0.k_i = 0.0
        self.leftClimberConfig.slot0.k_d = 0.0

        self.leftClimberConfig.motor_output.neutral_mode = NeutralModeValue.COAST

        self.leftClimberConfig.current_limits.supply_current_limit_enable = True
        self.leftClimberConfig.current_limits.supply_current_limit = 10
        self.leftClimberConfig.current_limits.supply_current_threshold = 25
        self.leftClimberConfig.current_limits.stator_current_limit_enable = True
        self.leftClimberConfig.current_limits.stator_current_limit = 10

        self.rightClimberConfig = deepcopy(self.rightClimberConfig)

        self.rightClimberConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.leftClimberConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.leftClimber.configurator.apply(self.leftClimberConfig)
        self.rightClimber.configurator.apply(self.rightClimberConfig)

        self.velocityControl = VelocityVoltage(0)
        self.VoltageControl = VoltageOut(0)

        self.leftClimbervelocitySupplier = self.leftClimber.get_velocity().as_supplier()
        self.rightClimbervelocitySupplier = self.rightClimber.get_velocity().as_supplier()

    def setClimbers(self, velocity):
        self.leftClimber.set_control(self.velocityControl.with_velocity(Conversions.MPSToRPS(circumference=self.wheelCircumference, wheelMPS=velocity)*self.gearRatio))
        self.rightClimber.set_control(self.velocityControl.with_velocity(Conversions.MPSToRPS(velocity, self.wheelCircumference)))