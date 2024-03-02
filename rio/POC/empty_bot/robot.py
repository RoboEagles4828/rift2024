from wpilib import TimedRobot
from commands2.button import CommandXboxController
from commands2 import CommandScheduler, Command, InstantCommand

from wpilib.shuffleboard import Shuffleboard, ShuffleboardTab
from wpimath.units import seconds

class Robot(TimedRobot):

  def robotInit(self):
    import jurigged;
    jurigged.watch()
    # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    # autonomous chooser on the dashboard.

    self.controller = CommandXboxController(0)
    self.buttonBindings()

  def buttonBindings(self):
      self.controller.a().onTrue(InstantCommand(self.buttonCallbackA))
      self.controller.b().onTrue(InstantCommand(lambda: print("B button pressed")))

  def buttonCallbackA(self):
    print("A button pressed")

  def robotPeriodic(self):
    # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    # commands, running already-scheduled commands, removing finished or interrupted commands,
    # and running subsystem periodic() methods.  This must be called from the robot's periodic
    # block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run()

  def disabledInit(self):
    pass

  def disabledPeriodic(self):
    pass

  def autonomousInit(self):
    pass

  def autonomousPeriodic(self):
    pass

  def teleopInit(self):
    # This makes sure that the autonomous stops running when
    # teleop starts running. If you want the autonomous to
    # continue until interrupted by another command, remove
    # this line or comment it out.
    pass

  def teleopPeriodic(self):
    pass

  def testInit(self):
    CommandScheduler.getInstance().cancelAll()

  def testPeriodic(self):
    pass