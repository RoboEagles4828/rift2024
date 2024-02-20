from phoenix6.configs import CANcoderConfiguration
from phoenix6.configs import TalonFXConfiguration

from phoenix6.signals.spn_enums import AbsoluteSensorRangeValue
from phoenix6.signals import InvertedValue

from constants import Constants

from copy import deepcopy

from wpimath.units import radiansToRotations

class CTREConfigs:
    swerveAngleFXConfig = TalonFXConfiguration()
    swerveDriveFXConfig = TalonFXConfiguration()
    swerveCANcoderConfig = CANcoderConfiguration()

    # swerveCANcoderConfigList = [CANcoderConfiguration(), CANcoderConfiguration(), CANcoderConfiguration(), CANcoderConfiguration()]

    def __init__(self):
        # Swerve CANCoder Configuration

        # for config in self.swerveCANcoderConfigList:
        self.swerveCANcoderConfig.magnet_sensor.sensor_direction = Constants.Swerve.cancoderInvert

        #Swerve Angle Motor Configurations
        # Motor Inverts and Neutral Mode
        self.swerveAngleFXConfig.motor_output.inverted = Constants.Swerve.angleMotorInvert
        self.swerveAngleFXConfig.motor_output.neutral_mode = Constants.Swerve.angleNeutralMode

        # Gear Ratio and Wrapping Config 
        self.swerveAngleFXConfig.feedback.sensor_to_mechanism_ratio = Constants.Swerve.angleGearRatio
        self.swerveAngleFXConfig.closed_loop_general.continuous_wrap = True
        
        # Current Limiting 
        self.swerveAngleFXConfig.current_limits.supply_current_limit_enable = Constants.Swerve.angleEnableCurrentLimit
        self.swerveAngleFXConfig.current_limits.supply_current_limit = Constants.Swerve.angleCurrentLimit
        self.swerveAngleFXConfig.current_limits.supply_current_threshold = Constants.Swerve.angleCurrentThreshold
        self.swerveAngleFXConfig.current_limits.supply_time_threshold = Constants.Swerve.angleCurrentThresholdTime

        # PID Config 
        self.swerveAngleFXConfig.slot0.k_p = Constants.Swerve.angleKP
        self.swerveAngleFXConfig.slot0.k_i = Constants.Swerve.angleKI
        self.swerveAngleFXConfig.slot0.k_d = Constants.Swerve.angleKD

        #* Swerve Drive Motor Configuration 
        # Motor Inverts and Neutral Mode 
        self.swerveDriveFXConfig.motor_output.inverted = Constants.Swerve.driveMotorInvert
        self.swerveDriveFXConfig.motor_output.neutral_mode = Constants.Swerve.driveNeutralMode

        # Gear Ratio Config 
        self.swerveDriveFXConfig.feedback.sensor_to_mechanism_ratio = Constants.Swerve.driveGearRatio

        # Current Limiting 
        self.swerveDriveFXConfig.current_limits.supply_current_limit_enable = Constants.Swerve.driveEnableCurrentLimit
        self.swerveDriveFXConfig.current_limits.supply_current_limit = Constants.Swerve.driveCurrentLimit
        self.swerveDriveFXConfig.current_limits.supply_current_threshold = Constants.Swerve.driveCurrentThreshold
        self.swerveDriveFXConfig.current_limits.supply_time_threshold = Constants.Swerve.driveCurrentThresholdTime

        # PID Config 
        self.swerveDriveFXConfig.slot0.k_p = Constants.Swerve.driveKP
        self.swerveDriveFXConfig.slot0.k_i = Constants.Swerve.driveKI
        self.swerveDriveFXConfig.slot0.k_d = Constants.Swerve.driveKD

        # Open and Closed Loop Ramping 
        self.swerveDriveFXConfig.open_loop_ramps.duty_cycle_open_loop_ramp_period = Constants.Swerve.openLoopRamp
        self.swerveDriveFXConfig.open_loop_ramps.voltage_open_loop_ramp_period = Constants.Swerve.openLoopRamp

        self.swerveDriveFXConfig.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = Constants.Swerve.closedLoopRamp
        self.swerveDriveFXConfig.closed_loop_ramps.voltage_closed_loop_ramp_period = Constants.Swerve.closedLoopRamp

        self.swerveDriveFXConfigFR = deepcopy(self.swerveDriveFXConfig)
        self.swerveDriveFXConfigFR.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE