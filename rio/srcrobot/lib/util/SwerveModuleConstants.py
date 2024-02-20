from wpimath.geometry import Rotation2d

class SwerveModuleConstants:
    driveMotorID: int
    angleMotorID: int
    cancoderID: int
    angleOffset: Rotation2d

    def __init__(self, driveMotorID: int, angleMotorID: int, canCoderID: int, angleOffset: Rotation2d):
        self.driveMotorID = driveMotorID
        self.angleMotorID = angleMotorID
        self.cancoderID = canCoderID
        self.angleOffset = angleOffset