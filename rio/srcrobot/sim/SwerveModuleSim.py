from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpilib import Timer


class SwerveModuleSim():
    def __init__(self) -> None:
        self.timer = Timer()
        self.timer.start()
        self.lastTime = self.timer.get()
        self.state = SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0))
        self.fakeSpeed = 0.0
        self.fakePos = 0.0
        self.dt = 0.0

    def updateStateAndPosition(self, desiredState : SwerveModuleState) -> None:
        self.dt = self.timer.get() - self.lastTime
        self.lastTime = self.timer.get()

        self.state = desiredState
        self.fakeSpeed = desiredState.speed
        self.fakePos += self.fakeSpeed * self.dt

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.fakePos, self.state.angle)
    
    def getState(self) -> SwerveModuleState:
        return self.state