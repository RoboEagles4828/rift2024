from wpilib.interfaces import GenericHID
from wpilib import Joystick
from wpilib import XboxController
from commands2.button import CommandXboxController, Trigger
from commands2 import Command, Subsystem
from commands2 import InstantCommand, ConditionalCommand, WaitCommand, PrintCommand
from commands2.button import JoystickButton
from CTREConfigs import CTREConfigs
from commands2 import CommandScheduler

from wpimath.geometry import Rotation2d
from constants import Constants

from autos.exampleAuto import exampleAuto
from commands.TeleopSwerve import TeleopSwerve
from subsystems.Swerve import Swerve
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.Arm import Arm
from subsystems.Shooter import Shooter
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
    s_Indexer : Indexer = Indexer()
    s_Shooter : Shooter = Shooter()

    #SysId
    driveSysId = DriveSysId(s_Swerve)


    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):

        self.currentArmAngle = self.s_Arm.getDegrees()

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

        # self.armAngleSlider = self.teleop.add("ARM POS", self.currentArmAngle)\
        #     .withWidget(BuiltInWidgets.kNumberSlider)\
        #     .withProperties({"min": -360, "max": 360})\
        #     .getEntry()

        self.zeroGyro = self.driver.back()
        self.robotCentric = self.driver.start()
        self.faceForward = self.driver.y()
        self.faceBack = self.driver.a()
        self.faceRight = self.driver.b()
        self.faceLeft = self.driver.x()
        self.shoot = self.driver.rightTrigger() #just for testing will be removed later
        self.intake = self.driver.rightBumper()
        self.intakeReverse = self.driver.leftBumper()
        # Operator Controls
        self.manualArm = self.operator.leftBumper() 
        self.armHome = self.operator.rightBumper()
        self.shooterRev = self.operator.rightTrigger()
        self.shooterAmpRev = self.operator.leftTrigger()
        # self.queSubFront = self.operator.a()
        # self.quePodium = self.operator.y()
        # self.queSubRight = self.operator.b()
        # self.queSubLeft = self.operator.x()
        # self.queAmp = self.operator.povUp()
        # self.queClimbFront = self.operator.povDown()
        # self.queClimbRight = self.operator.povRight()
        # self.queClimbLeft = self.operator.povLeft()
        self.configureButtonBindings()

        self.s_Intake.setDefaultCommand(self.s_Intake.stopIntake())
        self.s_Indexer.setDefaultCommand(self.s_Indexer.stopIndexer())
        self.s_Shooter.setDefaultCommand(self.s_Shooter.stop())

        NamedCommands.registerCommand("RevShooter", self.s_Shooter.shoot())
        NamedCommands.registerCommand("ShootSubwoofer", self.s_Shooter.shoot().withTimeout(2.0).andThen(self.s_Indexer.indexerShoot().withTimeout(1.0)))
        NamedCommands.registerCommand("runIntake", self.s_Shooter.stop().withTimeout(0.5).andThen(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake().withTimeout(2.0)).withTimeout(2.0)))
        NamedCommands.registerCommand("print0", InstantCommand(lambda: print("0")))
        NamedCommands.registerCommand("print1", InstantCommand(lambda: print("1")))
        NamedCommands.registerCommand("print2", InstantCommand(lambda: print("2")))
        NamedCommands.registerCommand("print3", InstantCommand(lambda: print("3")))
        NamedCommands.registerCommand("print4", InstantCommand(lambda: print("4")))

        self.auton_selector = SendableChooser()
        self.auton_selector.setDefaultOption("Straight Auto", PathPlannerAutoRunner("StraightAuto", self.s_Swerve).getCommand())
        self.auton_selector.addOption("RightSubwooferTaxiAuto", PathPlannerAutoRunner("RightSubwooferTaxiAuto", self.s_Swerve).getCommand())
        self.auton_selector.addOption("CenterSubwoofer2PieceAuto", PathPlannerAutoRunner("CenterSubwoofer2Piece", self.s_Swerve).getCommand())

        self.auton.add("Auton Selector", self.auton_selector)\
            .withWidget(BuiltInWidgets.kComboBoxChooser)\
            .withSize(2, 1)\
            .withPosition(0, 0)

        self.teleop.addBoolean("Field Centric", lambda: not self.robotCentric_value)\
            .withPosition(7, 0)\
            .withSize(1, 1)\
            .withWidget(BuiltInWidgets.kBooleanBox)
        self.teleop.addBoolean("Zero Gyro", lambda: self.zeroGyro.getAsBoolean())\
            .withPosition(8, 0)\
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
        self.teleop.addDouble("HEADING", lambda: self.s_Swerve.getHeading().degrees())
        self.teleop.add("Swerve Subsystem", self.s_Swerve)\
            .withPosition(0, 2)\
            .withSize(3, 3)
        self.teleop.addDouble("FLYWHEEL TARGET", lambda: self.s_Shooter.getTargetVelocity())
        self.teleop.addDouble("FLYWHEEL CURRENT", self.s_Shooter.getVelocity)
        self.teleop.addDouble("ARM ANGLE", self.s_Arm.getDegrees)
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
        # self.queAmp.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kAmpPivotAngle))
        # self.quePodium.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kPodiumPivotAngle))
        # self.queSubFront.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kSubwooferPivotAngle))

        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroHeading()))
        self.robotCentric.onTrue(InstantCommand(lambda: self.toggleFieldOriented()))

        self.faceForward.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(180)), translation, strafe, rotation, robotcentric))
        self.faceBack.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(0)), translation, strafe, rotation, robotcentric))
        self.faceLeft.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(90)), translation, strafe, rotation, robotcentric))
        self.faceRight.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(-90)), translation, strafe, rotation, robotcentric))

        #Intake Buttons
        self.intake.whileTrue(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()))
        self.intakeReverse.whileTrue(self.s_Intake.outtake().alongWith(self.s_Indexer.indexerOuttake()))

        #Shooter Buttons
        self.shooterRev.whileTrue(self.s_Shooter.shoot())
        self.shooterAmpRev.whileTrue(self.s_Shooter.amp())
        self.shoot.and_(self.s_Shooter.isShooterReady).whileTrue(self.s_Indexer.indexerShoot())
        
        self.beamBreakTrigger = Trigger(self.s_Indexer.getBeamBreakState)
        self.beamBreakTrigger.onTrue(WaitCommand(0.02).andThen(self.s_Intake.stopIntake().alongWith(self.s_Indexer.levelIndexer())))
        # self.alwaysTrue = Trigger(lambda: True)
        # self.alwaysTrue.whileTrue(self.s_Arm.servoArmToTarget(self.armAngleSlider.getDouble()))

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