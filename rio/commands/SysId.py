from commands2.sysid import SysIdRoutine
from commands2 import SequentialCommandGroup, InstantCommand, WaitCommand, Command
from subsystems.Swerve import Swerve

class DriveSysId():

    routine: SysIdRoutine

    def __init__(self, s_Swerve: Swerve):
        self.routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                s_Swerve.driveMotorsVoltage,
                s_Swerve.logDriveMotors,
                s_Swerve,
                name="SwerveDrive"
            )
        )

        self.swerve = s_Swerve

        self.quasiStaticForward = self.routine.quasistatic(SysIdRoutine.Direction.kForward)
        self.quasiStaticReverse = self.routine.quasistatic(SysIdRoutine.Direction.kReverse)
        self.dynamicForward = self.routine.dynamic(SysIdRoutine.Direction.kForward)
        self.dynamicReverse = self.routine.dynamic(SysIdRoutine.Direction.kReverse)

        self.resetAngleMotors = InstantCommand(lambda: (s_Swerve.resetModulesToAbsolute()), s_Swerve)

        self.generalCommand = SequentialCommandGroup(
            self.resetAngleMotors,
            InstantCommand(lambda: (s_Swerve.stop()), s_Swerve),
        )

        self.quasiForwardCommand =  SequentialCommandGroup(
            self.quasiStaticForward,
            InstantCommand(lambda: (s_Swerve.stop()), s_Swerve),
            WaitCommand(2),
        )

        self.quasiReverseCommand = SequentialCommandGroup(
            self.quasiStaticReverse,
            InstantCommand(lambda: (s_Swerve.stop()), s_Swerve),
            WaitCommand(2),
        )

        self.dynamicForwardCommand =  SequentialCommandGroup(
            self.dynamicForward,
            InstantCommand(lambda: (s_Swerve.stop()), s_Swerve),
            WaitCommand(2),
        )

        self.dynamicReverseCommand = SequentialCommandGroup(
            self.dynamicReverse,
            InstantCommand(lambda: (s_Swerve.stop()), s_Swerve),
            WaitCommand(2),
        )

    def getQuasiForwardCommand(self):
        return self.quasiForwardCommand

    def getQuasiReverseCommand(self):
        return self.quasiReverseCommand
    
    def getDynamicForwardCommand(self):
        return self.dynamicForwardCommand
    
    def getDynamicReverseCommand(self):
        return self.dynamicReverseCommand