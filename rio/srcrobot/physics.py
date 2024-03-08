
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

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from pyfrc.physics.units import units
from CTREConfigs import Constants

from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.sim import TalonFXSimState, CANcoderSimState

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

    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot"):
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

        self.frontLeftModuleSim = SwerveModuleSim(robot.robot_container.s_Swerve.mSwerveMods[0])
        self.frontRightModuleSim = SwerveModuleSim(robot.robot_container.s_Swerve.mSwerveMods[1])
        self.backLeftModuleSim = SwerveModuleSim(robot.robot_container.s_Swerve.mSwerveMods[2])
        self.backRightModuleSim = SwerveModuleSim(robot.robot_container.s_Swerve.mSwerveMods[3])

    def update_sim(self, now: float, tm_diff: float) -> None:
        self.frontLeftModuleSim.update(tm_diff)
        self.frontRightModuleSim.update(tm_diff)
        self.backLeftModuleSim.update(tm_diff)
        self.backRightModuleSim.update(tm_diff)

        self.roborio.setVInVoltage(self.battery.calculate([0.0]))

class SwerveModuleSim:
    drive: TalonFXSimState
    angle: TalonFXSimState
    encoder: CANcoderSimState

    def __init__(self, module: "SwerveModule"):
        driveMOI = center_wheel_moi
        angleMOI = center_axle_moi + center_side_wheel_moi

        self.drive = TalonFXSim(module.mDriveMotor, driveMOI, Constants.Swerve.driveGearRatio, False)
        self.angle = TalonFXSim(module.mAngleMotor, angleMOI, Constants.Swerve.angleGearRatio, False)
        self.encoder = module.ca

    def update(self, tm_diff: float) -> None:
        self.drive.update(tm_diff)
        self.angle.update(tm_diff)
        self.encoder.update(tm_diff, self.angle.getVelocityRadians())


