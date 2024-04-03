from wpilib.drive import DifferentialDrive
import phoenix5
from commands2.subsystem import Subsystem

class ArcadeDrive(Subsystem):
    d_Drive : DifferentialDrive

    def __init__(self):
        self.leftLeader = phoenix5.WPI_TalonSRX(20)
        self.leftFollower = phoenix5.WPI_TalonSRX(14)
        self.rightLeader = phoenix5.WPI_TalonSRX(13)
        self.rightFollower = phoenix5.WPI_TalonSRX(15)

        self.leftFollower.follow(self.leftLeader)
        self.rightFollower.follow(self.rightLeader)

        self.d_Drive = DifferentialDrive(self.leftLeader, self.rightLeader)
        super().__init__()
    
    def drive(self, xSpeed, zRotation):
        self.d_Drive.arcadeDrive(xSpeed, zRotation, squareInputs=True)