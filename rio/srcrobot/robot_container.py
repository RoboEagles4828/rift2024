from wpilib.interfaces import GenericHID
from wpilib import Joystick
from wpilib import XboxController
from commands2.button import CommandXboxController, Trigger
from commands2 import Command, Subsystem
from commands2 import InstantCommand, ConditionalCommand, WaitCommand, PrintCommand, RunCommand
from commands2.button import JoystickButton
import commands2.cmd as cmd
from CTREConfigs import CTREConfigs
from commands2 import CommandScheduler

from wpimath.geometry import Rotation2d
from constants import Constants

from autos.exampleAuto import exampleAuto
from commands.TeleopSwerve import TeleopSwerve
from commands.ExecuteCommand import ExecuteCommand
from subsystems.Swerve import Swerve
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.Arm import Arm
from subsystems.Climber import Climber
from subsystems.Shooter import Shooter
# from subsystems.Climber import Climber
from commands.TurnInPlace import TurnInPlace

from commands.SysId import DriveSysId

from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets, BuiltInLayouts
from wpilib import SendableChooser

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

    robotCentric_value = False

    # Subsystems
    s_Swerve : Swerve = Swerve()
    s_Arm : Arm = Arm()
    s_Intake : Intake = Intake()
    s_Indexer : Indexer = Indexer()
    s_Shooter : Shooter = Shooter()
    s_Climber : Climber = Climber()

    # The container for the robot. Contains subsystems, OI devices, and commands.
    def __init__(self):
        self.m_robotState : RobotState = RobotState()
        self.m_robotState.initialize(
            lambda: self.s_Swerve.getHeading().degrees(),
            self.s_Arm.getDegrees,
            self.s_Shooter.getVelocity
        )

        self.currentArmAngle = self.s_Arm.getDegrees()

        # Driver Controls
        self.zeroGyro = self.driver.back()
        self.robotCentric = self.driver.start()

        self.slowModeMove = self.driver.leftTrigger()
        self.slowModeTurn = self.driver.rightTrigger()
        self.execute = self.driver.rightBumper()

        self.shoot = self.driver.leftBumper()

        # Operator Controls
        self.autoHome = self.operator.rightTrigger()
        self.flywheel = self.operator.rightBumper()
        self.systemReverse = self.operator.leftBumper()
        self.intake = self.operator.leftBumper()

        #Que Controls
        self.queSubFront = self.operator.a()
        self.quePodium = self.operator.y()
        self.queSubRight = self.operator.b()
        self.queSubLeft = self.operator.x()
        self.queAmp = self.operator.povUp()
        self.climbUp = self.operator.povLeft()
        self.climbDown = self.operator.povRight()
        self.configureButtonBindings()

        NamedCommands.registerCommand("RevShooter", self.s_Shooter.shoot().withTimeout(2.0).withName("AutoRevShooter"))
        NamedCommands.registerCommand("Shoot", self.s_Indexer.indexerShoot().withTimeout(1.0).withName("AutoShoot"))
        NamedCommands.registerCommand("ShooterOff", InstantCommand(self.s_Shooter.brake, self.s_Shooter).withName("AutoShooterBrake"))
        NamedCommands.registerCommand("IndexerIntake", self.s_Indexer.indexerIntakeOnce().withName("AutoIndexerIntake"))
        NamedCommands.registerCommand("IndexerOff", self.s_Indexer.instantStop().withName("AutoIndexerOff"))
        NamedCommands.registerCommand("IntakeOn", self.s_Intake.intakeOnce().withName("AutoIntakeOn"))
        NamedCommands.registerCommand("IntakeOff", self.s_Intake.instantStop().withName("AutoIntakeOff"))

        self.auton_selector = SendableChooser()
        self.auton_selector.setDefaultOption("Straight Auto", PathPlannerAutoRunner("StraightAuto", self.s_Swerve).getCommand())
        self.auton_selector.addOption("RightSubwooferTaxiAuto", PathPlannerAutoRunner("RightSubwooferTaxiAuto", self.s_Swerve).getCommand())
        self.auton_selector.addOption("CenterSubwoofer2PieceAuto", PathPlannerAutoRunner("CenterSubwoofer2Piece", self.s_Swerve).getCommand())

        Shuffleboard.getTab("Autonomous").add("Auton Selector", self.auton_selector)
        Shuffleboard.getTab("Teleoperated").add("Swerve Subsystem", self.s_Swerve)
        Shuffleboard.getTab("Teleoperated").add("Intake Sub", self.s_Intake)
        Shuffleboard.getTab("Teleoperated").add("Indexer Sub", self.s_Indexer)
        Shuffleboard.getTab("Teleoperated").add("Shooter Sub", self.s_Shooter)
        Shuffleboard.getTab("Teleoperated").add("Arm Sub", self.s_Arm)
        Shuffleboard.getTab("Teleoperated").add("Climber Sub", self.s_Climber)
        Shuffleboard.getTab("Teleoperated").addString("QUEUED SHOT", self.getQueuedShot)

        Shuffleboard.getTab("Teleoperated").addBoolean("Field Oriented", self.getFieldOriented)
        Shuffleboard.getTab("Teleoperated").addBoolean("Zero Gyro", self.zeroGyro.getAsBoolean)

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

        robotStateSupplier = lambda: self.m_robotState

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
        self.s_Arm.setDefaultCommand(self.s_Arm.stop())

        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroHeading()))
        self.robotCentric.onTrue(InstantCommand(lambda: self.toggleFieldOriented()))

        #Intake Buttons
        self.s_Indexer.setDefaultCommand(self.s_Indexer.stopIndexer())
        self.s_Intake.setDefaultCommand(self.s_Intake.stopIntake())
        self.intake.whileTrue(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()))
        self.systemReverse.whileTrue(self.s_Intake.outtake().alongWith(self.s_Indexer.indexerOuttake()))
        
        # self.beamBreakTrigger = Trigger(self.s_Indexer.getBeamBreakState)
        self.beamBreakTrigger = Trigger(lambda: True)
        self.beamBreakTrigger.and_(self.intake.getAsBoolean).onTrue(self.s_Intake.instantStop().alongWith(self.s_Indexer.levelIndexer().withTimeout(1.0)))

        self.beamBreakTrigger.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setHasNote(True)).andThen(self.rumbleDriver())).onFalse(InstantCommand(lambda: self.m_robotState.m_gameState.setHasNote(False)))

        #Que Buttons
        self.queSubFront.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.SPEAKER_CENTER
        )))
        self.quePodium.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.PODIUM
        )))
        self.queSubRight.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.SPEAKER_AMP
        )))
        self.queSubLeft.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.SPEAKER_PODIUM
        )))
        self.queAmp.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.AMP
        )))

        self.execute.onTrue(ExecuteCommand(
            self.s_Arm,
            self.s_Shooter,
            self.s_Swerve,
            translation,
            strafe,
            rotation,
            robotcentric
        ))

        # Climber Buttons
        self.s_Climber.setDefaultCommand(self.s_Climber.runClimbersDown())
        self.climbUp.whileTrue(self.s_Climber.runClimbersUp())
        self.climbDown.whileTrue(self.s_Climber.runClimbersDown())
    
        #Shooter Buttons
        self.s_Shooter.setDefaultCommand(self.s_Shooter.stop())
        self.flywheel.whileTrue(self.s_Shooter.shootVelocity(self.m_robotState.m_gameState.getNextShot().m_shooterVelocity))
        self.shoot.onTrue(cmd.deadline(self.s_Indexer.indexerShoot(), self.s_Intake.intake()).andThen(WaitCommand(1.0).andThen(self.s_Arm.seekArmZero())))

        self.autoHome.onTrue(self.s_Arm.seekArmZero())

    def toggleFieldOriented(self):
        self.robotCentric_value = not self.robotCentric_value

    def getFieldOriented(self):
        return not self.robotCentric_value
    
    def getQueuedShot(self):
        return self.m_robotState.m_gameState.getNextShot().name
        
    def rumbleDriver(self):
        return InstantCommand(
            lambda: self.driver.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 1.0)
        ).andThen(WaitCommand(0.5)).andThen(InstantCommand(lambda: self.driver.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 0.0))).withName("Rumble")


    """
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
    """
    def getAutonomousCommand(self) -> Command:
        auto = self.auton_selector.getSelected()
        return auto