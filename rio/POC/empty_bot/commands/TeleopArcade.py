from commands2 import Command

from typing import Callable

from subsystems.ArcadeDrive import ArcadeDrive

from wpimath import applyDeadband


class TeleopArcade(Command):    

    s_Drive : ArcadeDrive

    def __init__(self, drive : ArcadeDrive, moveSup : Callable, turnSup : Callable):
        self.s_Drive : ArcadeDrive = drive
        self.moveSup = moveSup
        self.turnSup = turnSup
        self.addRequirements(drive)

    def execute(self):
        move = self.moveSup()
        turn = self.turnSup()

        self.s_Drive.drive(move, turn)
