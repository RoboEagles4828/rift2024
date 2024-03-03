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
        self.gearRatio = 1.0/14.2
        self.wheelCircumference = 0.01905*math.pi
        self.leftClimberConfig = TalonFXConfiguration()
        self.rightClimberConfig = TalonFXConfiguration()

        self.leftClimberConfig.slot0.k_p = 10.0
        self.leftClimberConfig.slot0.k_i = 0.0
        self.leftClimberConfig.slot0.k_d = 0.0

        self.leftClimberConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE

        self.leftClimberConfig.current_limits.supply_current_limit_enable = True
        self.leftClimberConfig.current_limits.supply_current_limit = 40
        self.leftClimberConfig.current_limits.supply_current_threshold = 40
        self.leftClimberConfig.current_limits.stator_current_limit_enable = True
        self.leftClimberConfig.current_limits.stator_current_limit = 40

        self.rightClimberConfig = deepcopy(self.leftClimberConfig)

        self.rightClimberConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.leftClimberConfig.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        self.rightClimberConfig.software_limit_switch.forward_soft_limit_enable = True
        self.rightClimberConfig.software_limit_switch.forward_soft_limit_threshold = 1000.0 # TODO: Change this value

        self.leftClimberConfig.software_limit_switch.forward_soft_limit_enable = True
        self.leftClimberConfig.software_limit_switch.forward_soft_limit_threshold = 1000.0

        self.leftClimber.configurator.apply(self.leftClimberConfig)
        self.rightClimber.configurator.apply(self.rightClimberConfig)

        self.dutyCycleControl = DutyCycleOut(0)
        self.velocityControl = VelocityVoltage(0)
        self.VoltageControl = VoltageOut(0)

        self.leftClimbervelocitySupplier = self.leftClimber.get_velocity().as_supplier()
        self.rightClimbervelocitySupplier = self.rightClimber.get_velocity().as_supplier()
        # Conversions.MPSToRPS(circumference=self.wheelCircumference, wheelMPS=velocity)*self.gearRatio)

    def setClimbers(self, percentOutput):
        self.leftClimber.set_control(self.dutyCycleControl.with_output(percentOutput))
        self.rightClimber.set_control(self.dutyCycleControl.with_output(percentOutput))
        # print(Conversions.MPSToRPS(velocity, self.wheelCircumference)*self.gearRatio)

    def setClimbersLambda(self, climberAxis):
        return self.run(lambda: self.setClimbers(climberAxis())).withName("Manual Climbers")

    def runClimbersUp(self):
        return self.run(lambda: self.setClimbers(-Constants.ClimberConstants.kClimberSpeed)).withName("Climbers Up")
    
    def stopClimbers(self):
        return self.run(lambda: self.setClimbers(0.0)).withName("Stop Climbers")
    
    def runClimbersDown(self):
        return self.run(lambda: self.setClimbers(Constants.ClimberConstants.kClimberSpeed)).withName("Climbers Down")