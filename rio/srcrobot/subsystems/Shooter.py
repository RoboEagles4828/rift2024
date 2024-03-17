from commands2.subsystem import Subsystem
import wpimath.filter
import wpimath
import wpilib
import phoenix6 
from wpimath.geometry import Rotation2d, Pose3d, Pose2d, Rotation3d
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.controls import VelocityVoltage, VoltageOut, StrictFollower, DutyCycleOut
from phoenix6.signals import InvertedValue, NeutralModeValue
from lib.mathlib.conversions import Conversions
import math
from constants import Constants
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from copy import deepcopy
from commands2 import InstantCommand, Command
class Shooter(Subsystem):
    def __init__(self):
        self.kBottomShooterCANID = 6 
        self.kTopShooterCANID = 13
        
        self.bottomShooter = TalonFX(self.kBottomShooterCANID)
        self.topShooter = TalonFX(self.kTopShooterCANID)

        self.gearRatio = 1.0

        self.wheelCircumference = 0.101*math.pi

        self.topShooterConfig = TalonFXConfiguration()

        self.topShooterConfig.slot0.k_v = (1/(Conversions.MPSToRPS(Constants.ShooterConstants.kPodiumShootSpeed, self.wheelCircumference)*self.gearRatio))
        self.topShooterConfig.slot0.k_p = 1.5
        self.topShooterConfig.slot0.k_i = 0.0
        self.topShooterConfig.slot0.k_d = 0.0

        self.topShooterConfig.motor_output.neutral_mode = NeutralModeValue.COAST

        self.topShooterConfig.current_limits.supply_current_limit_enable = True
        self.topShooterConfig.current_limits.supply_current_limit = 30
        self.topShooterConfig.current_limits.supply_current_threshold = 50
        self.topShooterConfig.current_limits.supply_time_threshold = Constants.Swerve.driveCurrentThresholdTime

        self.topShooterConfig.current_limits.stator_current_limit_enable = True
        self.topShooterConfig.current_limits.stator_current_limit = 30

        self.bottomShooterConfig = deepcopy(self.topShooterConfig)

        self.topShooterConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.bottomShooterConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        self.topShooter.configurator.apply(self.topShooterConfig)
        self.bottomShooter.configurator.apply(self.bottomShooterConfig)

        self.VelocityControl = VelocityVoltage(0)
        self.VoltageControl = VoltageOut(0)
        self.percentOutput = DutyCycleOut(0)

        self.currentShotVelocity = 0.0

        self.topShooterVelocitySupplier = self.topShooter.get_velocity().as_supplier()
        self.bottomShooterVelocitySupplier = self.bottomShooter.get_velocity().as_supplier()

    def setShooterVelocity(self, velocity):
        self.topShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(velocity,  self.wheelCircumference)*self.gearRatio))
        self.bottomShooter.set_control(self.VelocityControl.with_velocity(Conversions.MPSToRPS(velocity,  self.wheelCircumference)*self.gearRatio))

        self.currentShotVelocity = Conversions.MPSToRPS(velocity,  self.wheelCircumference)*self.gearRatio

    def shootVelocity(self, velocity) -> Command:
        return self.run(lambda: self.setShooterVelocity(velocity)).withName("ShootVelocity")
    
    def shootVelocityWithSupplier(self, velSup):
        return self.run(lambda: self.setShooterVelocity(velSup())).withName("ShootVelocity")

    def neutralizeShooter(self):
        # self.topShooterConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        # self.bottomShooterConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        # self.topShooter.configurator.apply(self.topShooterConfig)
        # self.bottomShooter.configurator.apply(self.bottomShooterConfig)

        self.topShooter.set_control(self.VoltageControl.with_output(0))
        self.bottomShooter.set_control(self.VoltageControl.with_output(0))

    def idle(self):
        return self.run(lambda: self.setShooterVelocity(Constants.ShooterConstants.kAmpShootSpeed)).withName("IdleShooter")
    
    def shoot(self):
        return self.run(lambda: self.setShooterVelocity(Constants.ShooterConstants.kSubwooferShootSpeed)).withName("Shoot")
    
    def amp(self):
        return self.run(lambda: self.setShooterVelocity(Constants.ShooterConstants.kAmpShootSpeed)).withName("Amp")

    def shootReverse(self):
        return self.run(lambda: self.setShooterVelocity(-Constants.ShooterConstants.kPodiumShootSpeed)).withName("ShootReverse")
    
    def isShooterReady(self, isAuto=False):
        if not isAuto:
            topShooterReady = abs(self.topShooterVelocitySupplier() - self.currentShotVelocity) < 5
            bottomShooterReady = abs(self.bottomShooterVelocitySupplier() - self.currentShotVelocity) < 5
        else:
            topShooterReady = abs(self.topShooterVelocitySupplier() - Conversions.MPSToRPS(Constants.ShooterConstants.kSubwooferShootSpeed,  self.wheelCircumference)*self.gearRatio) < 5
            bottomShooterReady = abs(self.bottomShooterVelocitySupplier() - self.currentShotVelocity) < 5

        return topShooterReady and bottomShooterReady
    
    def isShooterAtSubwooferSpeed(self):
        if abs(self.topShooterVelocitySupplier() - Conversions.MPSToRPS(Constants.ShooterConstants.kSubwooferShootSpeed,  self.wheelCircumference)*self.gearRatio) < 5:
            return True
        return False
    
    def brake(self):
        # self.topShooterConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # self.topShooter.configurator.apply(self.topShooterConfig)
        self.topShooter.set_control(self.VoltageControl.with_output(0).with_override_brake_dur_neutral(True))

        # self.bottomShooterConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # self.bottomShooter.configurator.apply(self.bottomShooterConfig)
        self.bottomShooter.set_control(self.VoltageControl.with_output(0).with_override_brake_dur_neutral(True))

    def stop(self):
        return self.run(lambda: self.neutralizeShooter()).withName("StopShooter")
    
    def getTargetVelocity(self):
        return Conversions.RPSToMPS(self.currentShotVelocity, self.wheelCircumference)/self.gearRatio
    
    def getVelocity(self):
        return Conversions.RPSToMPS(self.topShooterVelocitySupplier(), self.wheelCircumference)/self.gearRatio