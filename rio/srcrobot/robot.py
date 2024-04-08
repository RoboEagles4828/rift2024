from wpilib import TimedRobot
from commands2 import Command
from commands2 import CommandScheduler
from CTREConfigs import CTREConfigs
from constants import Constants
from gameState import GameState
from robot_container import RobotContainer
from wpimath.geometry import Rotation2d
import wpilib

from wpilib.shuffleboard import Shuffleboard, ShuffleboardTab
from wpilib import SmartDashboard, DriverStation

class Robot(TimedRobot):
  m_autonomousCommand: Command = None

  m_robotContainer: RobotContainer

  auton_tab: ShuffleboardTab
  teleop_tab: ShuffleboardTab

  def robotInit(self):
    # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    # autonomous chooser on the dashboard.
    # wpilib.CameraServer.launch()
    self.m_robotContainer = RobotContainer()
    self.m_robotContainer.m_robotState.m_gameState.setHasNote(False)
    CommandScheduler.getInstance().setPeriod(0.02)

  def robotPeriodic(self):
    # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    # commands, running already-scheduled commands, removing finished or interrupted commands,
    # and running subsystem periodic() methods.  This must be called from the robot's periodic
    # block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run()

  def autonomousInit(self):
    # self.m_robotContainer.s_Shooter.setDefaultCommand(self.m_robotContainer.s_Shooter.idle())

    m_autonomousCommand: Command = self.m_robotContainer.getAutonomousCommand()

    # schedule the autonomous command (example)
    if m_autonomousCommand != None:
      m_autonomousCommand.schedule()

  def teleopInit(self):
    # This makes sure that the autonomous stops running when
    # teleop starts running. If you want the autonomous to
    # continue until interrupted by another command, remove
    # this line or comment it out.
    # flip heading
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
      self.m_robotContainer.s_Swerve.setHeading(self.m_robotContainer.s_Swerve.getHeading().rotateBy(Rotation2d.fromDegrees(180.0)))

    GameState().setNextShot(Constants.NextShot.SPEAKER_CENTER)
    self.m_robotContainer.s_Shooter.setDefaultCommand(self.m_robotContainer.s_Shooter.stop())

    if self.m_autonomousCommand is not None:
      self.m_autonomousCommand.cancel()

  def testInit(self):
    CommandScheduler.getInstance().cancelAll()