from commands2 import Subsystem

from photonlibpy.photonCamera import PhotonCamera, VisionLEDMode
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.photonPipelineResult import PhotonPipelineResult
from lib.util.PhotonUtils import PhotonUtils

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField, loadAprilTagLayoutField

from wpimath.geometry import Pose2d, Transform2d, Transform3d, Pose3d, Rotation2d, Rotation3d, Translation2d, Translation3d

import wpimath.units as Units

from wpilib import SmartDashboard, DriverStation

from typing import Callable

class Vision(Subsystem):
    instance = None

    def __init__(self):
        self.camera = PhotonCamera("camera1")
        self.aprilTagFieldLayout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

        self.speakerPositionRed = Pose2d(-0.04, 5.55, Rotation2d())
        self.speakerPositionBlue = Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d())
        self.distanceSpeakerFieldToCamera = 0.0

        # Right = 6.5 in
        # Up = 11.5 in
        # Froward = (frame / 2.0) - 1.25 in

        self.robotToCamera = Transform3d(
            Translation3d(-Units.inchesToMeters((31.125 / 2.0) - 1.25), -Units.inchesToMeters(6.5), Units.inchesToMeters(11.5)),
            Rotation3d.fromDegrees(0.0, -45.0, 180.0)
        )
        self.fieldToCamera = Transform3d()

        self.photonPoseEstimator = PhotonPoseEstimator(
            self.aprilTagFieldLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            self.robotToCamera 
        )
        self.photonPoseEstimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

        self.CAMERA_HEIGHT_METERS = Units.inchesToMeters(11.5)
        self.SPEAKER_HEIGHT_METERS = 1.45 # meters

        self.CAMERA_PITCH_RADIANS = Rotation2d.fromDegrees(-45.0)

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

        speakerPos = self.speakerPositionBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            speakerPos = self.speakerPositionRed
        else:
            speakerPos = self.speakerPositionBlue

        distanceToSpeakerFieldToCamera = Units.metersToInches(
            PhotonUtils.getDistanceToPose(pose, speakerPos)
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

        if len(targets) <= 0:
            return None

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
    
    def getSortedTargetsList(self, result: PhotonPipelineResult):
        targets = result.getTargets()
        targets.sort(key=lambda target: target.area, reverse=True)
        return targets

    
    def getAngleToTag(self, tagIDSupplier: Callable[[], int]):
        # if self.isTargetSeenLambda(tagIDSupplier):
        #     result = self.camera.getLatestResult()
        #     best_target = self.getBestTarget(result)
        #     return best_target.getYaw()
        # else:
        #     return 0.0

        sortedTargets = self.getSortedTargetsList(self.camera.getLatestResult())

        for target in sortedTargets:
            if target.getFiducialId() == tagIDSupplier():
                return target.getYaw()
        return 0.0


        
    # def getDistanceToTag(self, tagIDSupplier: Callable[[], int]):
    #     if self.isTargetSeenLambda(tagIDSupplier):
    #         result = self.camera.getLatestResult()
    #         best_target = self.getBestTarget(result)
    #         # camToTarget = best_target.getBestCameraToTarget()
    #         # dist = 0.0
    #         # return dist
    #     else:
    #         return 0.0

    def periodic(self):
        result = self.camera.getLatestResult()

        if result.multiTagResult.estimatedPose.isPresent:
            self.fieldToCamera = result.multiTagResult.estimatedPose.best

            self.distanceToSpeakerFieldToCamera = self.getDistanceToSpeakerFieldToCameraInches(self.fieldToCamera)

        hasTargets = result.hasTargets()

        if hasTargets:        
            # get the best tag based on largest areaprint(f"================ {Units.inchesToMeters(self.s_Vision.getDistanceToSpeakerFieldToCameraInches(Transform3d(0.0, 0.0, 0.0, Rotation3d())))}")
            bestTarget = self.getBestTarget(result)
            if bestTarget is not None:
                SmartDashboard.putNumber("tag ID", bestTarget.getFiducialId())
                SmartDashboard.putNumber("pose ambiguity", bestTarget.getPoseAmbiguity())
                SmartDashboard.putNumber("tag transform X", bestTarget.getBestCameraToTarget().X())
                SmartDashboard.putNumber("tag transform Y", bestTarget.getBestCameraToTarget().Y())
                SmartDashboard.putNumber("tag transform Z", bestTarget.getBestCameraToTarget().Z())
                SmartDashboard.putNumber("tag transform angle", bestTarget.getBestCameraToTarget().rotation().angle_degrees)
                SmartDashboard.putNumber("tag yaw", bestTarget.getYaw())
            # SmartDashboard.putNumber("Vision Pose X", self.getEstimatedGlobalPose().estimatedPose.X())
            # SmartDashboard.putNumber("Vision Pose Y", self.getEstimatedGlobalPose().estimatedPose.Y())
            # SmartDashboard.putNumber("Vision Pose Angle", self.getEstimatedGlobalPose().estimatedPose.rotation().angle_degrees)








