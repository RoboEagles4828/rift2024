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
from commands.QueueCommand import QueueCommand
from commands.ExecuteCommand import ExecuteCommand
from subsystems.Swerve import Swerve
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.Arm import Arm
from subsystems.Shooter import Shooter
from commands.TurnInPlace import TurnInPlace

from commands.SysId import DriveSysId

from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets, BuiltInLayouts
from wpilib import SendableChooser, SmartDashboard

from autos.PathPlannerAutoRunner import PathPlannerAutoRunner
from pathplannerlib.auto import NamedCommands

from robotState import RobotState



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

    m_robotState : RobotState = RobotState()
    m_robotState.initialize(
        s_Swerve.getHeading,
        s_Arm.getDegrees,
        s_Shooter.getVelocity
    )


    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):

        self.currentArmAngle = self.s_Arm.getDegrees()

        translation = lambda: 0.0
        strafe = lambda: 0.0
        rotation = lambda: 0.0
        robotcentric = lambda: False

        self.auto = False

        # Driver Controls
        self.zeroGyro = self.driver.back()
        self.robotCentric = self.driver.start()

        self.slowModeMove = self.driver.leftTrigger()
        self.slowModeTurn = self.driver.rightTrigger()
        self.execute = self.driver.rightBumper()

        # self.armTest = self.driver.a()
        # self.shoot = self.driver.rightTrigger() #just for testing will be removed later
        # self.intake = self.driver.rightBumper()
        # self.intakeReverse = self.driver.leftBumper()

        # Operator Controls
        self.armHome = self.operator.rightTrigger()
        self.flywheel = self.operator.rightBumper()
        self.systemReverse = self.operator.leftBumper()
        self.intake = self.operator.leftBumper()

        #Que Controls
        self.queSubFront = self.operator.a()
        self.quePodium = self.operator.y()
        self.queSubRight = self.operator.b()
        self.queSubLeft = self.operator.x()
        self.queAmp = self.operator.povUp()
        # self.queClimbFront = self.operator.povDown()
        # self.queClimbRight = self.operator.povRight()
        # self.queClimbLeft = self.operator.povLeft()
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

        SmartDashboard.putBoolean("Field Centric", not self.robotCentric_value)
        SmartDashboard.putBoolean("Zero Gyro", self.zeroGyro.getAsBoolean())
        SmartDashboard.putData("Swerve Subsystem", self.s_Swerve)
        SmartDashboard.putData("Intake Sub", self.s_Intake)
        SmartDashboard.putData("Indexer Sub", self.s_Indexer)
        SmartDashboard.putData("Shooter Sub", self.s_Shooter)
        SmartDashboard.putNumber("FLYWHEEL TARGET", self.s_Shooter.getTargetVelocity())
        SmartDashboard.putNumber("FLYWHEEL CURRENT", self.s_Shooter.getVelocity())
        SmartDashboard.putNumber("ARM ANGLE", self.s_Arm.getDegrees())
        SmartDashboard.putBoolean("State", self.m_robotState.isReadyToIntake())

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

        SmartDashboard.putNumber("Translation", translation())
        SmartDashboard.putNumber("Strafe", strafe())
        SmartDashboard.putNumber("Rotation", rotation())

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
        # self.manualArm.whileTrue(self.s_Arm.moveArm(lambda: self.operator.getLeftY()))
        self.armHome.onTrue(self.s_Arm.seekArmZero().andThen(InstantCommand(lambda: self.s_Arm.hardSetEncoderToZero())))

        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroHeading()))
        self.robotCentric.onTrue(InstantCommand(lambda: self.toggleFieldOriented()))

        # self.faceForward.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(180)), translation, strafe, rotation, robotcentric).withTimeout(2.0))
        # self.faceBack.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(0)), translation, strafe, rotation, robotcentric))
        # self.armTest.whileTrue(self.s_Arm.servoArmToTarget(45))
        # self.faceLeft.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(90)), translation, strafe, rotation, robotcentric).withTimeout(2.0))
        # self.faceRight.onTrue(TurnInPlace(self.s_Swerve, lambda: (Rotation2d.fromDegrees(-90)), translation, strafe, rotation, robotcentric).withTimeout(2.0))

        #Intake Buttons
        self.s_Indexer.setDefaultCommand(self.s_Indexer.stopIndexer())
        self.s_Intake.setDefaultCommand(self.s_Intake.stopIntake())
        self.intake.whileTrue(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()))
        self.systemReverse.whileTrue(self.s_Intake.outtake().alongWith(self.s_Indexer.indexerOuttake()))

        #Shooter Buttons
        self.s_Shooter.setDefaultCommand(self.s_Shooter.stop())
        self.flywheel.whileTrue(self.s_Shooter.shoot())
        
        self.beamBreakTrigger = Trigger(self.s_Indexer.getBeamBreakState)
        self.beamBreakTrigger.and_(self.intake.getAsBoolean).onTrue(WaitCommand(0.02).andThen(self.s_Intake.stopIntake().alongWith(self.s_Indexer.levelIndexer().withTimeout(1.0))))

        self.beamBreakTrigger.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setHasNote(True)))

        #Que Buttons
        self.queSubFront.onTrue(QueueCommand(
            Constants.NextShot.SPEAKER_CENTER,
            self.s_Arm,
            self.s_Shooter,
            self.s_Swerve,
            self.m_robotState,
            [translation, strafe, rotation, robotcentric]
        ))
        self.quePodium.onTrue(QueueCommand(
            Constants.NextShot.PODIUM,
            self.s_Arm,
            self.s_Shooter,
            self.s_Swerve,
            self.m_robotState,
            [translation, strafe, rotation, robotcentric]
        ))
        self.queSubRight.onTrue(QueueCommand(
            Constants.NextShot.SPEAKER_AMP,
            self.s_Arm,
            self.s_Shooter,
            self.s_Swerve,
            self.m_robotState,
            [translation, strafe, rotation, robotcentric]
        ))
        self.queSubLeft.onTrue(QueueCommand(
            Constants.NextShot.SPEAKER_PODIUM,
            self.s_Arm,
            self.s_Shooter,
            self.s_Swerve,
            self.m_robotState,
            [translation, strafe, rotation, robotcentric]
        ))
        self.queAmp.onTrue(QueueCommand(
            Constants.NextShot.AMP,
            self.s_Arm,
            self.s_Shooter,
            self.s_Swerve,
            self.m_robotState,
            [translation, strafe, rotation, robotcentric]
        ))

        self.execute.onTrue(ExecuteCommand(self.intake, self.m_robotState))



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