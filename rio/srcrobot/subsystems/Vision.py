from commands2 import Subsystem

from photonlibpy.photonCamera import PhotonCamera, VisionLEDMode
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.photonPipelineResult import PhotonPipelineResult
from lib.util.PhotonUtils import PhotonUtils

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField, loadAprilTagLayoutField

from wpimath.geometry import Pose2d, Transform2d, Transform3d, Pose3d, Rotation2d, Rotation3d, Translation2d, Translation3d

import wpimath.units as Units

from wpilib import SmartDashboard

from typing import Callable

class Vision(Subsystem):
    instance = None

    def __init__(self):
        self.camera = PhotonCamera("camera1")
        self.aprilTagFieldLayout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

        self.speakerPosition = Pose2d(-0.04, 5.55, Rotation2d())
        self.distanceSpeakerFieldToCamera = 0.0

        self.robotToCamera = Transform3d(
            Translation3d(0.0, 0.0, 0.0),
            Rotation3d.fromDegrees(0.0, 0.0, 0.0)
        )
        self.fieldToCamera = Transform3d()

        self.photonPoseEstimator = PhotonPoseEstimator(
            self.aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            self.robotToCamera 
        )
        self.photonPoseEstimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

        self.CAMERA_HEIGHT_METERS = Units.inchesToMeters(24.0)
        self.TARGET_HEIGHT_METERS = Units.feetToMeters(5.0)

        self.CAMERA_PITCH_RADIANS = Rotation2d.fromDegrees(0.0)

        self.instance : Vision = None

    @classmethod
    def getInstance(cls):
        if cls.instance == None:
            cls.instance = Vision()
        return cls.instance

    def getEstimatedGlobalPose(self):
        return self.photonPoseEstimator.update()
    
    def getDistanceToSpeakerFieldToCameraInches(self, fieldToCamera: Transform3d):
        x = fieldToCamera.X()
        y = fieldToCamera.Y()
        theta = fieldToCamera.rotation().toRotation2d()

        pose = Pose2d(x, y, theta)

        distanceToSpeakerFieldToCamera = Units.metersToInches(
            PhotonUtils.getDistanceToPose(pose, Pose2d(-0.0381, 5.547868, Rotation2d()))
        )
        return distanceToSpeakerFieldToCamera
    
    def getCamera(self):
        return self.camera
    
    def hasTargetBooleanSupplier(self):
        return lambda: self.camera.getLatestResult().hasTargets()
    
    def takeSnapshot(self):
        self.camera.takeInputSnapshot()

    def setPipeline(self, pipelineIdx):
        self.camera.setPipelineIndex(pipelineIdx)

    def setTagMode(self):
        self.setPipeline(0)

    def getBestTarget(self, result : PhotonPipelineResult):
        targets = result.getTargets()

        # sort targets by area largest to smallest
        targets.sort(key=lambda target: target.area, reverse=True)

        return targets[0]
    
    def isTargetSeen(self, tagID) -> bool:
        result = self.camera.getLatestResult()
        if result.hasTargets() == False:
            return False
        best_target = self.getBestTarget(result)
        return best_target.getFiducialId() == tagID
    
    def isTargetSeenLambda(self, tagIDSupplier: Callable[[], int]) -> bool:
        result = self.camera.getLatestResult()
        if result.hasTargets() == False:
            return False
        best_target = self.getBestTarget(result)
        return best_target.getFiducialId() == tagIDSupplier()
    
    def getAngleToTag(self, tagIDSupplier: Callable[[], int]):
        if self.isTargetSeenLambda(tagIDSupplier):
            result = self.camera.getLatestResult()
            best_target = self.getBestTarget(result)
            return best_target.getYaw()
        else:
            return 0.0
        
    def getDistanceToTag(self, tagIDSupplier: Callable[[], int]):
        if self.isTargetSeenLambda(tagIDSupplier):
            result = self.camera.getLatestResult()
            best_target = self.getBestTarget(result)
            dist = PhotonUtils.calculateDistanceToTargetMeters(
                self.CAMERA_HEIGHT_METERS,
                self.TARGET_HEIGHT_METERS,
                self.CAMERA_PITCH_RADIANS.radians(),
                Units.degreesToRadians(best_target.getPitch())
            )
            return dist
        else:
            return 0.0

    def periodic(self):
        result = self.camera.getLatestResult()

        if result.multiTagResult.estimatedPose.isPresent:
            self.fieldToCamera = result.multiTagResult.estimatedPose.best

            self.distanceToSpeakerFieldToCamera = self.getDistanceToSpeakerFieldToCameraInches(self.fieldToCamera)

        hasTargets = result.hasTargets()

        if hasTargets:        
            # get the best tag based on largest area
            bestTarget = self.getBestTarget(result)
            SmartDashboard.putNumber("tag ID", bestTarget.getFiducialId())
            SmartDashboard.putNumber("pose ambiguity", bestTarget.getPoseAmbiguity())








