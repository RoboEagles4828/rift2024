from wpilib.interfaces import GenericHID
from wpilib import Joystick
from wpilib import XboxController
from commands2.button import CommandXboxController
from commands2 import Command, Subsystem
from commands2 import InstantCommand, ConditionalCommand
from commands2.button import JoystickButton
from CTREConfigs import CTREConfigs
from commands2 import CommandScheduler

from wpimath.geometry import Rotation2d
from constants import Constants

from autos.exampleAuto import exampleAuto
from commands.TeleopSwerve import TeleopSwerve
from subsystems.Swerve import Swerve
from subsystems.intake import Intake
from subsystems.Arm import Arm
from commands.TurnInPlace import TurnInPlace

from commands.SysId import DriveSysId

from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets, BuiltInLayouts
from wpilib import SendableChooser

from autos.PathPlannerAutoRunner import PathPlannerAutoRunner
from pathplannerlib.auto import NamedCommands

class RobotContainer:
    ctreConfigs = CTREConfigs()
    # Drive Controls
    translationAxis = XboxController.Axis.kLeftY
    strafeAxis = XboxController.Axis.kLeftX
    rotationAxis = XboxController.Axis.kRightX

    driver = CommandXboxController(0)
    operator = CommandXboxController(1)

    sysId = JoystickButton(driver, XboxController.Button.kY)

    robotCentric_value = True

    # Subsystems
    s_Swerve : Swerve = Swerve()
    s_Arm : Arm = Arm()
    s_Intake : Intake = Intake()

    #SysId
    driveSysId = DriveSysId(s_Swerve)


    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):
        translation = lambda: 0.0
        strafe = lambda: 0.0
        rotation = lambda: 0.0
        robotcentric = lambda: False

        NamedCommands.registerCommand("FaceForward", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(180)), translation, strafe, rotation, robotcentric))
        NamedCommands.registerCommand("FaceBackward", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(0)), translation, strafe, rotation, robotcentric))
        NamedCommands.registerCommand("FaceLeft", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(90)), translation, strafe, rotation, robotcentric))
        NamedCommands.registerCommand("FaceRight", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(-90)), translation, strafe, rotation, robotcentric))
        # Driver Controls
        self.auton = Shuffleboard.getTab("Auton")
        self.teleop = Shuffleboard.getTab("Teleop")
        self.zeroGyro = self.driver.back()
        self.robotCentric = self.driver.start()
        self.faceForward = self.driver.y()
        self.faceBack = self.driver.a()
        self.faceRight = self.driver.b()
        self.faceLeft = self.driver.x()
        self.resetToAbsoluteButton = self.driver.rightBumper()
        self.intakeOn = self.driver.povRight()
        self.intakeOff = self.driver.povLeft()
        # Operator Controls
        self.manualArm = self.operator.leftBumper() 
        self.armHome = self.operator.rightTrigger()
        self.shooterOff = self.operator.rightBumper()
        self.reverse = self.operator.leftTrigger()
        self.queSubFront = self.operator.a()
        self.quePodium = self.operator.y()
        self.queSubRight = self.operator.b()
        self.queSubLeft = self.operator.x()
        self.queAmp = self.operator.povUp()
        self.queClimbFront = self.operator.povDown()
        self.queClimbRight = self.operator.povRight()
        self.queClimbLeft = self.operator.povLeft()
        self.configureButtonBindings()

        self.auton_selector = SendableChooser()
        self.auton_selector.setDefaultOption("Test Auto", PathPlannerAutoRunner("TestAuto", self.s_Swerve).getCommand())
        self.auton_selector.addOption("Example Auto", exampleAuto(self.s_Swerve).getCommand())

        self.auton.add("Auton Selector", self.auton_selector)\
            .withWidget(BuiltInWidgets.kComboBoxChooser)\
            .withSize(2, 1)\
            .withPosition(0, 0)
        
        self.teleop.addBoolean("Field Centric", lambda: not self.robotCentric_value)\
            .withPosition(9, 0)\
            .withSize(1, 1)\
            .withWidget(BuiltInWidgets.kBooleanBox)
        self.teleop.addBoolean("Zero Gyro", lambda: self.zeroGyro.getAsBoolean())\
            .withPosition(10, 0)\
            .withSize(1, 1)\
            .withWidget(BuiltInWidgets.kBooleanBox)
        # self.teleop.addBoolean("SysId", lambda: self.sysId.getAsBoolean())\
        #     .withPosition(11, 0)\
        #     .withSize(1, 1)\
        #     .withWidget(BuiltInWidgets.kBooleanBox)
        self.teleop.add("Gyro", self.s_Swerve.gyro)\
            .withPosition(0, 0)\
            .withSize(2, 2)\
            .withWidget(BuiltInWidgets.kGyro)
        self.teleop.add("Swerve Subsystem", self.s_Swerve)\
            .withPosition(0, 2)\
            .withSize(3, 3)
        
        Shuffleboard.update()

    """
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    """
    def configureButtonBindings(self):
        translation = lambda: -self.driver.getRawAxis(self.translationAxis) 
        strafe = lambda: -self.driver.getRawAxis(self.strafeAxis)
        rotation = lambda: self.driver.getRawAxis(self.rotationAxis)
        robotcentric = lambda: self.robotCentric_value

        self.s_Swerve.setDefaultCommand(
            TeleopSwerve(
                self.s_Swerve, 
                translation,
                strafe,
                rotation,
                robotcentric
            )
        )

        # Arm Buttons
        self.s_Arm.setDefaultCommand(self.s_Arm.seekArmZero())
        self.manualArm.whileTrue(self.s_Arm.moveArm(lambda: self.operator.getLeftY()))
        self.queAmp.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kAmpPivotAngle))
        self.quePodium.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kPodiumPivotAngle))
        self.queSubFront.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kSubwooferPivotAngle))
        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroYaw()))
        self.robotCentric.onFalse(InstantCommand(lambda: self.toggleFieldOriented()))

        self.faceForward.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(180)), translation, strafe, rotation, robotcentric))
        self.faceBack.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(0)), translation, strafe, rotation, robotcentric))
        self.faceLeft.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(90)), translation, strafe, rotation, robotcentric))
        self.faceRight.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(-90)), translation, strafe, rotation, robotcentric))
        

        #Intake Buttons
        self.intakeOn.onTrue(self.s_Intake.setIntakeSpeed(Constants.IntakeConstants.kIntakeSpeed))
        self.intakeOff.onTrue(self.s_Intake.stopIntake())



    def toggleFieldOriented(self):
        self.robotCentric_value = not self.robotCentric_value


    """
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
    """
    def getAutonomousCommand(self) -> Command:
        auto = self.auton_selector.getSelected()
        return auto