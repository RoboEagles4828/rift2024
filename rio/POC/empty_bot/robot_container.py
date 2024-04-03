import wpilib
from subsystems.ArcadeDrive import ArcadeDrive
from commands.TeleopArcade import TeleopArcade
from commands2.button import CommandXboxController
from commands2 import RepeatCommand, InstantCommand

class RobotContainer:

    s_Arcade : ArcadeDrive = ArcadeDrive()

    def __init__(self):
        self.xbox = CommandXboxController(0)
        self.spin = self.xbox.a()

        self.setupCommands()

    
    def setupCommands(self):
        # self.s_Arcade.setDefaultCommand(
        #     TeleopArcade(
        #         self.s_Arcade,
        #         self.getLeftSpeed,
        #         self.getRightSpeed
        #     )
        # )

        self.spin.whileTrue(RepeatCommand(InstantCommand(self.goForward)))

    def goForward(self):
        self.s_Arcade.drive(-1.5, 0)
        
    def getRightSpeed(self):
        return self.xbox.getRightX()
    
    def getLeftSpeed(self):
        return -self.xbox.getLeftY()