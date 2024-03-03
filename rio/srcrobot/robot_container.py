from wpilib.interfaces import GenericHID
from wpilib import Joystick
from wpilib import XboxController
from commands2.button import CommandXboxController, Trigger
from commands2 import Command, Subsystem
from commands2 import InstantCommand, ConditionalCommand, WaitCommand, PrintCommand, RunCommand
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
from subsystems.Climber import Climber
from subsystems.Shooter import Shooter
from subsystems.Led import LED
# from subsystems.Climber import Climber
from commands.TurnInPlace import TurnInPlace

from commands.SysId import DriveSysId

from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets, BuiltInLayouts
from wpilib import SendableChooser, SmartDashboard, DriverStation

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

    robotCentric_value = False

    # Subsystems
    s_Swerve : Swerve = Swerve()
    s_Arm : Arm = Arm()
    s_Intake : Intake = Intake()
    s_Indexer : Indexer = Indexer()
    s_Shooter : Shooter = Shooter()
    s_Climber : Climber = Climber()
    s_LED : LED = LED()

    #SysId
    driveSysId = DriveSysId(s_Swerve)


    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):

        self.currentArmAngle = self.s_Arm.getDegrees()

        translation = lambda: 0.0
        strafe = lambda: 0.0
        rotation = lambda: 0.0
        robotcentric = lambda: False

        self.auto = False

        # NamedCommands.registerCommand("FaceForward", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(180)), translation, strafe, rotation, robotcentric))
        # NamedCommands.registerCommand("FaceBackward", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(0)), translation, strafe, rotation, robotcentric))
        # NamedCommands.registerCommand("FaceLeft", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(90)), translation, strafe, rotation, robotcentric))
        # NamedCommands.registerCommand("FaceRight", TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(-90)), translation, strafe, rotation, robotcentric))
        # Driver Controls

        self.zeroGyro = self.driver.back()
        self.robotCentric = self.driver.start()
        self.faceForward = self.operator.y() # Does arm Stuff
        self.faceBack = self.operator.a() #Does armm Stuff
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
        self.climbUp = self.operator.povLeft()
        self.climbDown = self.operator.povRight()
        # self.climbUpM = self.operator.leftStick()
        self.configureButtonBindings()

        NamedCommands.registerCommand("RevShooter", self.s_Shooter.shoot().withTimeout(2.0).withName("AutoRevShooter"))
        NamedCommands.registerCommand("Shoot", self.s_Indexer.indexerShoot().withTimeout(1.0).withName("AutoShoot"))
        NamedCommands.registerCommand("ShooterOff", InstantCommand(self.s_Shooter.brake, self.s_Shooter).withName("AutoShooterBrake"))
        NamedCommands.registerCommand("IndexerIntake", self.s_Indexer.indexerIntakeOnce().withName("AutoIndexerIntake"))
        NamedCommands.registerCommand("IndexerOff", self.s_Indexer.stopIndexer().withName("AutoIndexerOff"))
        NamedCommands.registerCommand("IntakeOn", self.s_Intake.intakeOnce().withName("AutoIntakeOn"))
        NamedCommands.registerCommand("IntakeOff", self.s_Intake.stopIntake().withName("AutoIntakeOff"))
        # NamedCommands.registerCommand("Command ")

        self.auton_selector = SendableChooser()
        self.auton_selector.setDefaultOption("Straight Auto", PathPlannerAutoRunner("StraightAuto", self.s_Swerve).getCommand())
        self.auton_selector.addOption("RightSubwooferTaxiAuto", PathPlannerAutoRunner("RightSubwooferTaxiAuto", self.s_Swerve).getCommand())
        self.auton_selector.addOption("CenterSubwoofer2PieceAuto", PathPlannerAutoRunner("CenterSubwoofer2Piece", self.s_Swerve).getCommand())

        SmartDashboard.putData("Auton Selector", self.auton_selector)

        SmartDashboard.putBoolean("Field Centric", self.getFieldOriented())
        SmartDashboard.putBoolean("Zero Gyro", self.zeroGyro.getAsBoolean())
        SmartDashboard.putData("Swerve Subsystem", self.s_Swerve)
        SmartDashboard.putData("Intake Sub", self.s_Intake)
        SmartDashboard.putData("Indexer Sub", self.s_Indexer)
        SmartDashboard.putData("Shooter Sub", self.s_Shooter)
        SmartDashboard.putNumber("FLYWHEEL TARGET", self.s_Shooter.getTargetVelocity())

        Shuffleboard.getTab("Teleop").addDouble("FLYWHEEL", self.s_Shooter.getVelocity)
        # SmartDashboard.putNumber("FLYWHEEL CURRENT", self.s_Shooter.getVelocity())
        SmartDashboard.putNumber("ARM ANGLE", self.s_Arm.getDegrees())

        SmartDashboard.putBoolean("BEAM BREAK", self.s_Indexer.getBeamBreakState())
        SmartDashboard.updateValues()

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

        climberAxis = lambda: self.operator.getRawAxis(self.translationAxis)

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
        self.armHome.onTrue(self.s_Arm.seekArmZero().andThen(InstantCommand(lambda: self.s_Arm.hardSetEncoderToZero())))
        # self.queAmp.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kAmpPivotAngle))
        # self.quePodium.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kPodiumPivotAngle))
        # self.queSubFront.onTrue(self.s_Arm.servoArmToTarget(Constants.ShooterConstants.kSubwooferPivotAngle))

        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroHeading()))
        self.robotCentric.onTrue(InstantCommand(lambda: self.toggleFieldOriented()))

        # self.faceForward.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(180)), translation, strafe, rotation, robotcentric))
        # self.faceBack.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(0)), translation, strafe, rotation, robotcentric))
        self.faceBack.onTrue(self.s_Arm.servoArmToTarget(90))
        self.faceForward.onTrue(self.s_Arm.servoArmToTarget(20))
        # self.faceRight.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(-90)), translation, strafe, rotation, robotcentric))

        #Intake Buttons
        self.s_Indexer.setDefaultCommand(self.s_Indexer.stopIndexer())
        self.s_Intake.setDefaultCommand(self.s_Intake.stopIntake())
        self.intake.whileTrue(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()))
        self.intakeReverse.whileTrue(self.s_Intake.outtake().alongWith(self.s_Indexer.indexerOuttake()))

        #Shooter Buttons
        self.s_Shooter.setDefaultCommand(self.s_Shooter.stop())
        self.shooterRev.whileTrue(self.s_Shooter.shoot())
        self.shooterAmpRev.whileTrue(self.s_Shooter.amp())
        self.shoot.and_(self.s_Shooter.isShooterReady).whileTrue(self.s_Indexer.indexerShoot())
        
        self.beamBreakTrigger = Trigger(self.s_Indexer.getBeamBreakState)
        # self.beamBreakTrigger.onTrue(InstantCommand(self.driver.getHID().setRumble(XboxController.RumbleType.kBothRumble, 0.5)).andThen(WaitCommand(0.5)).andThen(InstantCommand(self.driver.getHID().setRumble(XboxController.RumbleType.kBothRumble, 0.0))))
        self.beamBreakTrigger.and_(self.intake.getAsBoolean).onTrue(WaitCommand(0.02).andThen(self.s_Intake.stopIntake().alongWith(self.s_Indexer.levelIndexer())))
        # self.alwaysTrue = Trigger(lambda: True)
        # self.alwaysTrue.whileTrue(self.s_Arm.servoArmToTarget(self.armAngleSlider.getDouble()))

        #LED Controls
        self.s_LED.setDefaultCommand(self.s_LED.idle())
        self.beamBreakTrigger.whileTrue(self.s_LED.noteDetected())
        self.shooterReady = Trigger(self.s_Shooter.isShooterReady)
        self.shooterReady.whileTrue(self.s_LED.readytoShoot())
        # self.autonTrigger = Trigger(lambda: DriverStation.isAutonomous()) 
        # self.autonTrigger.whileTrue(self.s_LED.autonomous())

        # Climber Buttons
        self.s_Climber.setDefaultCommand(self.s_Climber.setClimbersLambda(climberAxis))
        self.climbUp.whileTrue(self.s_Climber.runClimbersUp())
        self.climbDown.whileTrue(self.s_Climber.runClimbersDown())

    def toggleFieldOriented(self):
        self.robotCentric_value = not self.robotCentric_value

    def getFieldOriented(self):
        return not self.robotCentric_value


    """
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
    """
    def getAutonomousCommand(self) -> Command:
        auto = self.auton_selector.getSelected()
        return auto