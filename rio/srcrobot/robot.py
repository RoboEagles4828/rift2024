from wpilib import TimedRobot
from commands2 import Command
from commands2 import CommandScheduler
from CTREConfigs import CTREConfigs
from robot_container import RobotContainer
import wpilib

from wpilib.shuffleboard import Shuffleboard, ShuffleboardTab
from wpilib import SmartDashboard

class Robot(TimedRobot):
  m_autonomousCommand: Command = None

  m_robotContainer: RobotContainer

  auton_tab: ShuffleboardTab
  teleop_tab: ShuffleboardTab

  def robotInit(self):
    # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    # autonomous chooser on the dashboard.
    wpilib.CameraServer.launch()
    self.m_robotContainer = RobotContainer()

  def robotPeriodic(self):
    # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    # commands, running already-scheduled commands, removing finished or interrupted commands,
    # and running subsystem periodic() methods.  This must be called from the robot's periodic
    # block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run()

  def autonomousInit(self):
    m_autonomousCommand: Command = self.m_robotContainer.getAutonomousCommand()

    # schedule the autonomous command (example)
    if m_autonomousCommand != None:
      m_autonomousCommand.schedule()

  def teleopInit(self):
    # This makes sure that the autonomous stops running when
    # teleop starts running. If you want the autonomous to
    # continue until interrupted by another command, remove
    # this line or comment it out.

    if self.m_autonomousCommand is not None:
      self.m_autonomousCommand.cancel()

  def testInit(self):
    CommandScheduler.getInstance().cancelAll()