from commands2 import Command, DeferredCommand, InstantCommand, SequentialCommandGroup

from subsystems.Vision import Vision
from subsystems.Swerve import Swerve

from wpilib import SmartDashboard

from photonlibpy.photonTrackedTarget import PhotonTrackedTarget

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints

from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d
import wpimath.units as Units

class PathFindToTag(SequentialCommandGroup):
    def __init__(self, swerve : Swerve, vision : Vision, TAG_ID, frontOffsetInches):
        super().__init__()
        self.vision = vision
        self.swerve = swerve

        self.TAG_ID = TAG_ID
        self.TAG_TO_GOAL = Transform3d(
            Translation3d(Units.inchesToMeters(frontOffsetInches), 0.0, 0.0),
            Rotation3d.fromDegrees(0.0, 0.0, 0.0)
        )

        self.targetToUse = None

        self.addRequirements(self.swerve, self.vision)
        self.addCommands(
            DeferredCommand(lambda: self.getCommand(), swerve, vision), 
            InstantCommand(lambda: self.swerve.stop())
        )

    def getCommand(self):
        robotToPose2d = self.swerve.getPose()
        robotToPose3d = Pose3d(
            robotToPose2d.X(),
            robotToPose2d.Y(),
            0.0,
            Rotation3d(0.0, 0.0, robotToPose2d.rotation().radians())
        )

        result = self.vision.getCamera().getLatestResult()

        if result.hasTargets() == False:
            return InstantCommand()
        else:
            try:
                allTargets = result.getTargets()
                for target in allTargets:
                    if target.getFiducialId() == self.TAG_ID:
                        self.targetToUse = target
                
                if self.targetToUse.getPoseAmbiguity() >= 0.2:
                    return InstantCommand()
                
                camToTarget = self.targetToUse.getBestCameraToTarget()
                cameraPose = robotToPose3d.transformBy(self.vision.robotToCamera)
                targetPose = cameraPose.transformBy(camToTarget)
                goalPose = targetPose.transformBy(self.TAG_TO_GOAL).toPose2d()

                return AutoBuilder.pathfindToPose(goalPose, PathConstraints(
                    1.5, 1,
                    Units.degreesToRadians(540), Units.degreesToRadians(720), 0
                ))
            except Exception as e:
                print(e)
                return InstantCommand()



