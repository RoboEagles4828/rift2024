from wpimath.geometry import Rotation2d, Pose2d, Rotation3d, Translation3d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModuleState
from wpilib import Timer
from wpilib import Field2d

class SwerveIMUSim():
    def __init__(self) -> None:
        self.timer = Timer()
        self.timer.start()
        self.lastTime = self.timer.get()
        self.angle = 0.0

    def getYaw(self) -> Rotation2d:
        return Rotation2d(self.angle).degrees()
    
    def getPitch(self) -> Rotation2d:
        return Rotation2d()

    def getRoll(self) -> Rotation2d:
        return Rotation2d()
    
    def updateOdometry(self, kinematics: SwerveDrive4Kinematics, states: list[SwerveModuleState], modulePoses: list[Pose2d], field: Field2d) -> None:
        self.angle += kinematics.toChassisSpeeds(states).omega * (self.timer.get() - self.lastTime)
        self.lastTime = self.timer.get()
        field.getObject("XModules").setPoses(modulePoses)

    def setAngle(self, angle : float):
        self.angle = angle

    def zeroYaw(self):
        self.setAngle(0.0)