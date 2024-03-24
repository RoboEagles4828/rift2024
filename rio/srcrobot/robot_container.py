from wpilib.interfaces import GenericHID
from wpilib import Joystick
from wpilib import XboxController
from commands2.button import CommandXboxController, Trigger
from commands2 import Command, ParallelDeadlineGroup, Subsystem
from commands2 import InstantCommand, ConditionalCommand, WaitCommand, PrintCommand, RunCommand, SequentialCommandGroup, ParallelCommandGroup, WaitUntilCommand, StartEndCommand
from commands2.button import JoystickButton
import commands2.cmd as cmd
from CTREConfigs import CTREConfigs
from commands2 import CommandScheduler
import math

from wpimath.geometry import *
import wpimath.units as Units
from constants import Constants

from autos.exampleAuto import exampleAuto
from commands.TeleopSwerve import TeleopSwerve
from commands.PathFindToTag import PathFindToTag

from commands.ExecuteCommand import ExecuteCommand
from subsystems.Swerve import Swerve
from subsystems.intake import Intake
from subsystems.indexer import Indexer
from subsystems.Arm import Arm
from subsystems.Climber import Climber
from subsystems.Shooter import Shooter
from subsystems.Vision import Vision
from subsystems.Led import LED
# from subsystems.Climber import Climber
from commands.TurnInPlace import TurnInPlace
from commands.TurnToTag import TurnToTag

from commands.SysId import DriveSysId

from wpilib.shuffleboard import Shuffleboard, BuiltInWidgets, BuiltInLayouts
from wpilib import SendableChooser, RobotBase

from wpimath import applyDeadband

from autos.PathPlannerAutoRunner import PathPlannerAutoRunner
from pathplannerlib.auto import NamedCommands, PathConstraints

from robotState import RobotState


class RobotContainer:
    ctreConfigs = CTREConfigs()
    # Drive Controls
    translationAxis = XboxController.Axis.kLeftY
    strafeAxis = XboxController.Axis.kLeftX
    rotationAxis = XboxController.Axis.kRightX
    slowAxis = XboxController.Axis.kRightTrigger # This causes issues on certain controllers, where kRightTrigger is for some reason mapped to [5] instead of [3]

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
    s_Vision : Vision = Vision.getInstance()
    # s_Climber : Climber = Climber()

    #SysId
    driveSysId = DriveSysId(s_Swerve)

    s_Climber : Climber = Climber()
    s_LED : LED = LED()

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
        self.execute = self.driver.leftBumper()
        self.shoot = self.driver.rightBumper()
        self.intake = self.driver.leftTrigger()
        self.goToTag = self.driver.a()
        self.fastTurn = self.driver.povUp()
        # Slowmode is defined with the other Axis objects

        # Operator Controls
        self.autoHome = self.operator.rightTrigger()
        # self.flywheel = self.operator.rightBumper()
        self.systemReverse = self.operator.leftBumper()
        self.opExec = self.operator.leftTrigger()
        self.opShoot = self.operator.rightBumper()
        self.emergencyArmUp = self.operator.povDown()

        # Que Controls
        self.queSubFront = self.operator.a()
        self.quePodium = self.operator.y()
        self.queSubRight = self.operator.b()
        self.queSubLeft = self.operator.x()
        self.queAmp = self.operator.povUp()
        self.climbUp = self.operator.povRight()
        self.climbDown = self.operator.povLeft()
        self.manualClimb  = self.operator.back()
        self.configureButtonBindings()

        NamedCommands.registerCommand("RevShooter", self.s_Shooter.shoot().withTimeout(1.0).withName("AutoRevShooter"))
        NamedCommands.registerCommand("Shoot", self.s_Indexer.indexerShoot().withTimeout(1.0).withName("AutoShoot"))
        NamedCommands.registerCommand("ShooterOff", InstantCommand(self.s_Shooter.brake, self.s_Shooter).withName("AutoShooterBrake"))
        NamedCommands.registerCommand("IndexerIntake", self.s_Indexer.indexerIntakeOnce().withName("AutoIndexerIntake"))
        NamedCommands.registerCommand("ArmUp", self.s_Arm.servoArmToTarget(5.0).withTimeout(0.5))
        NamedCommands.registerCommand("ArmDown", self.s_Arm.seekArmZero().withTimeout(1.0))
        NamedCommands.registerCommand("IndexerOff", self.s_Indexer.instantStop().withName("AutoIndexerOff"))
        NamedCommands.registerCommand("IntakeOn", self.s_Intake.intakeOnce().withName("AutoIntakeOn"))
        NamedCommands.registerCommand("IntakeOff", self.s_Intake.instantStop().withName("AutoIntakeOff"))
        NamedCommands.registerCommand("IndexerOut", self.s_Indexer.indexerOuttake().withTimeout(4.0))

        NamedCommands.registerCommand("IntakeUntilBeamBreak", SequentialCommandGroup(
            self.s_Indexer.indexerIntakeOnce(),
            WaitUntilCommand(self.s_Indexer.getBeamBreakState).withTimeout(4.0).finallyDo(lambda interrupted: self.s_Indexer.stopMotor()),
            self.s_Indexer.instantStop()
        ).withTimeout(5.0))

        # Compiled Commands
        NamedCommands.registerCommand("Full Shooter Rev", self.s_Shooter.shoot())
        NamedCommands.registerCommand("Full Intake", self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerOuttake().withTimeout(0.0005)).withName("AutoIntake").withTimeout(5.0))
        NamedCommands.registerCommand("Full Shoot", self.s_Arm.servoArmToTarget(4.0).withTimeout(0.5).andThen(WaitCommand(0.7)).andThen(self.s_Indexer.indexerShoot().withTimeout(0.5).alongWith(self.s_Intake.intake().withTimeout(0.5)).withTimeout(1.0)).andThen(self.s_Arm.seekArmZero()))
        NamedCommands.registerCommand("All Off", self.s_Intake.instantStop().alongWith(self.s_Indexer.instantStop(), self.s_Shooter.brake()).withName("AutoAllOff"))

        NamedCommands.registerCommand("Combo Shot", self.autoModeShot(Constants.NextShot.SPEAKER_CENTER))

        NamedCommands.registerCommand("Combo Podium Shot", self.autoModeShot(Constants.NextShot.CENTER_AUTO))

        self.auton_selector = SendableChooser()
        self.auton_selector.addOption("SourceSubwoofer2PieceMidline", PathPlannerAutoRunner("LeftSubwoofer2MidlinePiece", self.s_Swerve).getCommand())
        self.auton_selector.addOption("AmpSubwoofer3PieceMidline", PathPlannerAutoRunner("RightSubwoofer2MidlinePiece", self.s_Swerve).getCommand())
        self.auton_selector.addOption("Straight Auto No Shoot", PathPlannerAutoRunner("StraightAutoNoShoot", self.s_Swerve).getCommand())
        self.auton_selector.addOption("AmpSubwooferTaxiAuto", PathPlannerAutoRunner("RightSubwooferTaxiAuto", self.s_Swerve).getCommand())
        self.auton_selector.setDefaultOption("CenterSubwoofer2PieceAuto", PathPlannerAutoRunner("CenterSubwoofer2Piece", self.s_Swerve).getCommand())
        self.auton_selector.addOption("CenterSubwoofer3PieceAuto", PathPlannerAutoRunner("CenterSubwoofer3Piece", self.s_Swerve).getCommand())
        self.auton_selector.addOption("CenterSubwoofer4PieceAuto", PathPlannerAutoRunner("CenterSubwoofer4Piece", self.s_Swerve).getCommand())
        self.auton_selector.addOption("Do Nothing", InstantCommand())
        self.auton_selector.addOption("AmpSubwoofer2Piece", PathPlannerAutoRunner("RightSubwooferTurn", self.s_Swerve).getCommand())
        self.auton_selector.addOption("ShootOnlyAuto", self.autoModeShot(Constants.NextShot.SPEAKER_CENTER))


        Shuffleboard.getTab("Autonomous").add("Auton Selector", self.auton_selector)
        Shuffleboard.getTab("Teleoperated").addString("QUEUED SHOT", self.getQueuedShot)

        Shuffleboard.getTab("Teleoperated").addBoolean("Field Oriented", self.getFieldOriented)
        Shuffleboard.getTab("Teleoperated").addBoolean("Zero Gyro", self.zeroGyro.getAsBoolean)
        Shuffleboard.getTab("Teleoperated").addBoolean("Beam Break", self.s_Indexer.getBeamBreakState)
        Shuffleboard.getTab("Teleoperated").addDouble("Shooter Speed", self.s_Shooter.getVelocity)
        Shuffleboard.getTab("Teleoperated").addBoolean("Arm + Shooter Ready", lambda: self.m_robotState.isArmAndShooterReady())
        Shuffleboard.getTab("Teleoperated").addDouble("Swerve Heading", lambda: self.s_Swerve.getHeading().degrees())

        Shuffleboard.getTab("Teleoperated").addDouble("Swerve Pose X", lambda: self.s_Swerve.getPose().X())
        Shuffleboard.getTab("Teleoperated").addDouble("Swerve Pose Y", lambda: self.s_Swerve.getPose().Y())
        Shuffleboard.getTab("Teleoperated").addDouble("Swerve Pose Theta", lambda: self.s_Swerve.getPose().rotation().degrees())

    def autoModeShot(self, autoShot: Constants.NextShot) -> Command:
        """
        Used during automode commands to shoot any Constants.NextShot.
        """
        shotTimeoutSec = (autoShot.m_armAngle / 45.0) + 1.0
        return SequentialCommandGroup(
            InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(autoShot)),
            ParallelDeadlineGroup(
                WaitUntilCommand(lambda: self.m_robotState.isArmAndShooterReady())
                .withTimeout(shotTimeoutSec)
                .andThen(WaitCommand(0.1))
                .andThen(self.s_Indexer.indexerShoot()),
                InstantCommand(
                    lambda: self.s_Shooter.setShooterVelocity(
                        autoShot.m_shooterVelocity
                    ),
                    self.s_Shooter,
                ),
                self.s_Arm.servoArmToTargetGravity(autoShot.m_armAngle)
            ),
            self.s_Indexer.instantStop(),
            self.s_Arm.seekArmZero().withTimeout(1.0),
        )

    """
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    """
    def configureButtonBindings(self):
        translation = lambda: -applyDeadband(self.driver.getRawAxis(self.translationAxis), 0.1)
        strafe = lambda: -applyDeadband(self.driver.getRawAxis(self.strafeAxis), 0.1)
        rotation = lambda: applyDeadband(self.driver.getRawAxis(self.rotationAxis), 0.1)
        robotcentric = lambda: applyDeadband(self.robotCentric_value, 0.1)
        slow = lambda: applyDeadband(self.driver.getRawAxis(self.slowAxis), 0.1)
        climberaxis = lambda: applyDeadband(self.operator.getRawAxis(XboxController.Axis.kLeftY), 0.1)

        self.s_Swerve.setDefaultCommand(
            TeleopSwerve(
                self.s_Swerve, 
                translation,
                strafe,
                rotation,
                robotcentric,
                slow
            )
        )

        # Arm Buttons
        self.s_Arm.setDefaultCommand(self.s_Arm.stop())

        # Driver Buttons
        self.zeroGyro.onTrue(InstantCommand(lambda: self.s_Swerve.zeroHeading()))
        self.robotCentric.onTrue(InstantCommand(lambda: self.toggleFieldOriented()))

        # Intake Buttons
        self.s_Indexer.setDefaultCommand(self.s_Indexer.stopIndexer())
        self.s_Intake.setDefaultCommand(self.s_Intake.stopIntake())
        self.intake.whileTrue(self.s_Intake.intake().alongWith(self.s_Indexer.indexerIntake()).until(self.s_Indexer.getBeamBreakState).andThen(self.s_Indexer.instantStop()).andThen(self.s_Indexer.indexerIntake().withTimeout(0.0005)))
        self.systemReverse.whileTrue(self.s_Intake.outtake().alongWith(self.s_Indexer.indexerOuttake(), self.s_Shooter.shootReverse()))

        self.beamBreakTrigger = Trigger(self.s_Indexer.getBeamBreakState)
        # self.beamBreakTrigger.and_(self.intake.getAsBoolean).onTrue(self.s_Intake.instantStop().alongWith(self.s_Indexer.instantStop().withTimeout(1.0)))

        self.beamBreakTrigger.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setHasNote(True)).alongWith(self.rumbleAll())).onFalse(InstantCommand(lambda: self.m_robotState.m_gameState.setHasNote(False)))

        # Que Buttons
        self.queSubFront.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.SPEAKER_CENTER
        )).andThen(self.s_Arm.seekArmZero()))
        self.quePodium.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.PODIUM
        )).andThen(self.s_Arm.seekArmZero()))
        self.queSubRight.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.SPEAKER_AMP
        )).andThen(self.s_Arm.seekArmZero()))
        self.queSubLeft.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.SPEAKER_SOURCE
        )).andThen(self.s_Arm.seekArmZero()))
        self.queAmp.onTrue(InstantCommand(lambda: self.m_robotState.m_gameState.setNextShot(
            Constants.NextShot.AMP
        )).andThen(self.s_Arm.seekArmZero()))

        self.execute.or_(self.opExec.getAsBoolean).onTrue(
            self.s_Arm.servoArmToTargetWithSupplier(
                lambda: self.m_robotState.m_gameState.getNextShot()
            ).alongWith(
                self.s_Shooter.shootVelocityWithSupplier(
                    lambda: self.m_robotState.m_gameState.getNextShot().m_shooterVelocity
                ),
                TurnInPlace(
                    self.s_Swerve,
                    lambda: Rotation2d.fromDegrees(
                        self.m_robotState.m_gameState.getNextShotRobotAngle()
                    ),
                    translation,
                    strafe,
                    rotation,
                    robotcentric
                ).repeatedly()
            )
        )

        self.fastTurn.whileTrue(InstantCommand(lambda: self.setFastTurn(True))).whileFalse(InstantCommand(lambda: self.setFastTurn(False)))

        # LED Controls
        self.s_LED.setDefaultCommand(self.s_LED.getStateCommand())

        # Climber Buttons
        self.s_Climber.setDefaultCommand(self.s_Climber.stopClimbers())
        self.climbUp.whileTrue(self.s_Climber.runClimbersUp())
        self.climbDown.whileTrue(self.s_Climber.runClimbersDown())
        self.manualClimb.whileTrue(self.s_Climber.setClimbersLambda(climberaxis))

        # Shooter Buttons
        self.s_Shooter.setDefaultCommand(self.s_Shooter.stop())
        # self.flywheel.whileTrue(self.s_Shooter.shootVelocity(self.m_robotState.m_gameState.getNextShot().m_shooterVelocity))
        self.shoot.or_(self.opShoot.getAsBoolean).whileTrue(cmd.parallel(self.s_Indexer.indexerTeleopShot(), self.s_Intake.intake(), self.s_Shooter.shootVelocityWithSupplier(lambda: self.m_robotState.m_gameState.getNextShot().m_shooterVelocity)))

        self.autoHome.onTrue(self.s_Arm.seekArmZero())

        self.emergencyArmUp.onTrue(
            ConditionalCommand(
                self.s_Arm.seekArmZero(),
                self.s_Arm.servoArmToTargetGravity(90.0),
                lambda: self.s_Arm.getDegrees() > 45.0
            )
        )

        #Vision Buttons

    def toggleFieldOriented(self):
        self.robotCentric_value = not self.robotCentric_value

    def getFieldOriented(self):
        return not self.robotCentric_value

    def getQueuedShot(self):
        return self.m_robotState.m_gameState.getNextShot().name
    
    def rumbleAll(self):
        return ParallelCommandGroup(
            self.rumbleDriver(),
            self.rumbleOperator()
        )

    def rumbleDriver(self):
        return InstantCommand(
            lambda: self.driver.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 1.0)
        ).andThen(WaitCommand(0.5)).andThen(InstantCommand(lambda: self.driver.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 0.0))).withName("Rumble")
    
    def rumbleOperator(self):
        return InstantCommand(
            lambda: self.operator.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 1.0)
        ).andThen(WaitCommand(0.5)).andThen(InstantCommand(lambda: self.operator.getHID().setRumble(XboxController.RumbleType.kLeftRumble, 0.0))).withName("Rumble")

    """
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
    """
    def getAutonomousCommand(self) -> Command:
        auto = self.auton_selector.getSelected()
        return auto
    
    def setFastTurn(self, value: bool):
        if value:
            Constants.Swerve.maxAngularVelocity = 2.5 * math.pi * 2.0
        else:
            Constants.Swerve.maxAngularVelocity = 2.5 * math.pi
