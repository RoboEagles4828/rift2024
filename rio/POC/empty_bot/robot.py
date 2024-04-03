import wpilib
from robot_container import RobotContainer
from commands2 import CommandScheduler

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Setup jurigged
        try:
            import jurigged;
            jurigged.watch("/home/lvuser/py/*.py") # This is on the robot
        except ImportError:
            print("Jurigged not found, did you install through `rio install`?")
        except:
            print("Unknown error when initializing jurigged")
        self.container = RobotContainer()

    def robotPeriodic(self):
        pass

    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        CommandScheduler.getInstance().run()
        # self.drive.arcadeDrive(self.getRightSpeed(), self.getLeftSpeed(), squareInputs=True)