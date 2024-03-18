
#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib.simulation
import wpimath.system.plant
from wpimath import units as Units

from wpilib import RobotController

import navx

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from pyfrc.physics.units import units
from CTREConfigs import Constants

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.signals import InvertedValue
from phoenix6.sim import TalonFXSimState, CANcoderSimState
from SwerveModule import SwerveModule

import math

from robot import Robot

import typing

if typing.TYPE_CHECKING:
    from robot import Robot

# Calculations
axle_radius = 0.05
axle_mass = 0.23739
wheel_radius = 0.0508
wheel_length = 0.0381
wheel_mass = 0.2313

center_axle_moi = 0.5 * pow(axle_radius, 2) * axle_mass
center_side_wheel_moi = (0.25 * pow(wheel_radius, 2) * wheel_mass) + ((1/12) * pow(wheel_length, 2) * wheel_mass)
center_wheel_moi = 0.5 * pow(wheel_radius, 2) * wheel_mass


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: Robot):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller

        self.driveStick = wpilib.simulation.XboxControllerSim(0)
        self.operatorStick = wpilib.simulation.XboxControllerSim(1)

        self.roborio = wpilib.simulation.RoboRioSim()
        self.battery = wpilib.simulation.BatterySim()
        self.roborio.setVInVoltage(self.battery.calculate([0.0]))

        self.frontLeftModuleSim = SwerveModuleSim(robot.m_robotContainer.s_Swerve.mSwerveMods[0])
        self.frontRightModuleSim = SwerveModuleSim(robot.m_robotContainer.s_Swerve.mSwerveMods[1])
        self.backLeftModuleSim = SwerveModuleSim(robot.m_robotContainer.s_Swerve.mSwerveMods[2])
        self.backRightModuleSim = SwerveModuleSim(robot.m_robotContainer.s_Swerve.mSwerveMods[3])

        self.navxSim = NavXSim(robot.m_robotContainer.s_Swerve.gyro)

    def update_sim(self, now: float, tm_diff: float) -> None:
        self.frontLeftModuleSim.update(tm_diff)
        self.frontRightModuleSim.update(tm_diff)
        self.backLeftModuleSim.update(tm_diff)
        self.backRightModuleSim.update(tm_diff)

        # update based on simulated robot velocity
        self.navxSim.update(tm_diff, )

        self.roborio.setVInVoltage(self.battery.calculate([0.0]))

class SwerveModuleSim:
    def __init__(self, module: SwerveModule):
        driveMOI = center_wheel_moi
        angleMOI = center_axle_moi + center_side_wheel_moi
        
        self.driveSensorPhase = True if Constants.Swerve.driveMotorInvert == InvertedValue.COUNTER_CLOCKWISE_POSITIVE else False
        self.angleSensorPhase = True if Constants.Swerve.angleMotorInvert == InvertedValue.COUNTER_CLOCKWISE_POSITIVE else False

        self.drive = TalonFXSim(module.mDriveMotor, driveMOI, Constants.Swerve.driveGearRatio, Constants.Swerve.driveMotorInvert)
        self.angle = TalonFXSim(module.mAngleMotor, angleMOI, Constants.Swerve.angleGearRatio, False)

        self.encoder = CancoderSim(module.angleEncoder, module.angleOffset.degrees(), self.angleSensorPhase)

    def update(self, tm_diff: float) -> None:
        self.drive.update(tm_diff)
        self.angle.update(tm_diff)
        self.encoder.update(tm_diff, self.angle.getVelocityRadians())
        
class TalonFXSim():
    def __init__(self, talon: TalonFX, moi: float, gearRatio: float, sensorPhase: bool):
        self.talon = talon
        self.moi = moi
        self.gearRatio = gearRatio
        self.sensorPhase = -1 if sensorPhase else 1
        self.gearbox = wpimath.system.plant.DCMotor.falcon500(1)
        self.motor = wpilib.simulation.DCMotorSim(self.gearbox, self.gearRatio, self.moi, [0.0, 0.0])
        self.fwdLimitEnabled = False
        self.fwdLimit = 0.0
        self.velocity = 0.0
        self.position = 0.0
        self.revLimitEnabled = False
        self.revLimit = 0.0

    def update(self, period: float):
        self.talonSim = self.talon.sim_state

        voltage = self.talonSim.motor_voltage * self.sensorPhase
        self.motor.setInputVoltage(voltage)
        self.motor.update(period)
        newPosition = self.motor.getAngularPosition()
        self.deltaPosition = newPosition - self.position
        self.position = newPosition
        self.velocity = self.motor.getAngularVelocity()

        if self.fwdLimitEnabled and self.position >= self.fwdLimit:
            self.talonSim.set_forward_limit(True)
            self.position = self.fwdLimit
        else:
            self.talonSim.set_forward_limit(False)

        if self.revLimitEnabled and self.position <= self.revLimit:
            self.talonSim.set_reverse_limit(True)
            self.position = self.revLimit
        else:
            self.talonSim.set_reverse_limit(False)

        positionRotations = (self.position * self.gearRatio) / (2*math.pi)
        velocityRotations = (self.velocity * self.gearRatio) / (2*math.pi) / 10.0

        self.talonSim.set_raw_rotor_position(positionRotations)
        self.talonSim.set_rotor_velocity(velocityRotations)

        self.talonSim.set_supply_voltage(RobotController.getBatteryVoltage())

    def getVelocityRadians(self) -> float:
        return self.velocity

class CancoderSim():
    def __init__(self, cancoder: CANcoder, offsetDegrees: float = 0.0, sensorPhase: bool = False) -> None:
        self.cancoder = cancoder
        self.cancoderSim = self.cancoder.sim_state
        self.cancoderSim.set_raw_position(0.0)
        self.offset = Units.degreesToRotations(offsetDegrees)
        self.sensorPhase = -1 if sensorPhase else 1
        self.velocity = 0.0
        self.position = 0.0

    def update(self, period: float, velocityRadians: float) -> None:
        self.position = velocityRadians * period * self.sensorPhase
        self.velocity = velocityRadians * self.sensorPhase

        self.cancoderSim = self.cancoder.sim_state
        self.cancoderSim.add_position(Units.radiansToRotations(self.position))
        self.cancoderSim.set_velocity(Units.radiansToRotations(self.velocity))

class NavXSim():
    def __init__(self, navx: navx.AHRS) -> None:
        self.navx = navx
        self.gyro = wpilib.AnalogGyro(1)
        self.navxSim = wpilib.simulation.AnalogGyroSim(self.gyro)
        self.navxSim.setInitialized(True)
        self.navxSim.setAngle(0.0)
        self.navxSim.setRate(0.0)

    def update(self, period: float) -> None:
        self.navxSim.setAngle(self.gyro.getAngle())
        self.navxSim.setRate(self.gyro.getRate())

