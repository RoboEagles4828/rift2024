from commands2 import Subsystem

from photonlibpy.photonCamera import PhotonCamera, VisionLEDMode, Packet
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.photonPipelineResult import PhotonPipelineResult
from lib.util.PhotonUtils import PhotonUtils

from robotpy_apriltag import AprilTagFieldLayout, AprilTagField, loadAprilTagLayoutField

from wpimath.geometry import Pose2d, Transform2d, Transform3d, Pose3d, Rotation2d, Rotation3d, Translation2d, Translation3d

import wpimath.units as Units

from constants import Constants

from wpilib import SmartDashboard, DriverStation

from typing import Callable

import math

class Vision(Subsystem):
    instance = None

    def __init__(self):

        try:
            self.camera : PhotonCamera | None = PhotonCamera("camera1")
            self.camera2 : PhotonCamera | None = PhotonCamera("camera2")
        except:
            self.camera : PhotonCamera | None = None
            self.camera2 : PhotonCamera | None = None
            print("========= NO PHOTON CAMERAS FOUND =========")

        self.aprilTagFieldLayout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

        self.speakerPositionBlue = Pose2d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Rotation2d())
        self.speakerPositionRed = Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180.0))
        self.distanceSpeakerFieldToCamera = 0.0

        # Right = 6.5 in
        # Up = 11.5 in
        # Froward = (frame / 2.0) - 1.25 in

        # self.robotToCamera = Transform3d(
        #     Translation3d(-Units.inchesToMeters((31.125 / 2.0) - 1.25), -Units.inchesToMeters(6.5), Units.inchesToMeters(11.5)),
        #     Rotation3d.fromDegrees(0.0, -45.0, 180.0)
        # )
        self.robotToCamera = Transform3d(
            Translation3d(-Units.inchesToMeters((31.125 / 2.0) - 3.25), -Units.inchesToMeters(7), Units.inchesToMeters(10.5)),
            Rotation3d.fromDegrees(180.0, -45.0, 180.0)
        )
        self.robotToCamera2 = Transform3d(
            Translation3d(-Units.inchesToMeters((31.125 / 2.0) - 2.5), -Units.inchesToMeters(7), Units.inchesToMeters(9)),
            Rotation3d.fromDegrees(180.0, -15.0, 180.0)
        )
        self.fieldToCamera = Transform3d()

        if self.camera is not None and self.camera2 is not None:
            try:
                self.photonPoseEstimator = PhotonPoseEstimator(
                    self.aprilTagFieldLayout, 
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    self.camera,
                    self.robotToCamera 
                )
                self.photonPoseEstimator.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY

                self.photonPoseEstimator2 = PhotonPoseEstimator(
                    self.aprilTagFieldLayout, 
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    self.camera2,
                    self.robotToCamera2 
                )
                self.photonPoseEstimator2.multiTagFallbackStrategy = PoseStrategy.LOWEST_AMBIGUITY
            except:
                self.photonPoseEstimator = None
                self.photonPoseEstimator2 = None
                print("===== PHOTON PROBLEM (POSE ESTIMATORS) =======")

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
        if self.camera is None or self.photonPoseEstimator is None:
            return None
        return self.photonPoseEstimator.update()
    
    def getSecondEstimatedGlobalPose(self):
        if self.camera2 is None or self.photonPoseEstimator2 is None:
            return None
        return self.photonPoseEstimator2.update()
    
    def getDistanceToSpeakerFieldToCameraFeet(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        speakerPos = self.speakerPositionBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            speakerPos = self.speakerPositionRed
        else:
            speakerPos = self.speakerPositionBlue

        distanceToSpeakerFieldToCamera = Units.metersToFeet(
            PhotonUtils.getDistanceToPose(pose, speakerPos)
        )
        return distanceToSpeakerFieldToCamera - (36.37 / 12.0)
    
    def getDistanceVectorToSpeaker(self, pose: Pose2d):
        speakerPos = self.speakerPositionBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            speakerPos = self.speakerPositionRed
        else:
            speakerPos = self.speakerPositionBlue

        distanceVector = PhotonUtils.getDistanceVectorToPose(pose, speakerPos)
        return distanceVector
    
    def getAngleToSpeakerFieldToCamera(self, fieldToCamera: Pose2d):
        pose = fieldToCamera

        speakerPos = self.speakerPositionBlue

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            speakerPos = self.speakerPositionRed
        else:
            speakerPos = self.speakerPositionBlue

        dx = speakerPos.X() - pose.X()
        dy = speakerPos.Y() - pose.Y()

        angleToSpeakerFieldToCamera = Rotation2d(math.atan2(dy, dx))

        return angleToSpeakerFieldToCamera
    
    def getCamera(self):
        return self.camera
    
    def getSecondCamera(self):
        return self.camera2
    
    def hasTargetBooleanSupplier(self):
        return lambda: self.camera.getLatestResult().hasTargets() or self.camera2.getLatestResult().hasTargets()
    
    def takeSnapshot(self):
        self.camera.takeInputSnapshot()
        self.camera2.takeInputSnapshot()

    def setPipeline(self, pipelineIdx):
        self.camera.setPipelineIndex(pipelineIdx)
        self.camera2.setPipelineIndex(pipelineIdx)

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
        if self.camera is None or self.camera2 is None:
            return False
        result = self.camera.getLatestResult()
        result2 = self.camera2.getLatestResult()
        if result.hasTargets() == False and result2.hasTargets() == False:
            return False
        best_target = self.getBestTarget(result)
        second_best_target = self.getBestTarget(result2)
        return best_target.getFiducialId() == tagID or second_best_target.getFiducialId() == tagID
    
    def isTargetSeenLambda(self, tagIDSupplier: Callable[[], int]) -> bool:
        if self.camera is None or self.camera2 is None:
            return False
        result = self.camera.getLatestResult()
        result2 = self.camera2.getLatestResult()
        if result.hasTargets() == False and result2.hasTargets() == False:
            return False
        best_target = self.getBestTarget(result)
        second_best_target = self.getBestTarget(result2)
        return best_target.getFiducialId() == tagIDSupplier() or second_best_target.getFiducialId() == tagIDSupplier()
    
    def getSortedTargetsList(self, result: PhotonPipelineResult):
        targets = result.getTargets()
        targets.sort(key=lambda target: target.area, reverse=True)
        return targets

    def getAngleToTag(self, tagIDSupplier: Callable[[], int]):
        if self.camera is None:
            return 0.0
        if self.isTargetSeenLambda(tagIDSupplier):
            result = self.camera.getLatestResult()
            best_target = self.getBestTarget(result)
            if best_target is not None:
                return best_target.getYaw()
            else:
                return 0.0
        else:
            return 0.0

    # def periodic(self):
    #     # if self.camera is not None:
        #     result = self.camera.getLatestResult()

        #     if result.multiTagResult.estimatedPose.isPresent:
        #         self.fieldToCamera = result.multiTagResult.estimatedPose.best

        #     hasTargets = result.hasTargets()

        #     if hasTargets:        
        #         # get the best tag based on largest areaprint(f"================ {Units.inchesToMeters(self.s_Vision.getDistanceToSpeakerFieldToCameraInches(Transform3d(0.0, 0.0, 0.0, Rotation3d())))}")
        #         bestTarget = self.getBestTarget(result)
        #         if bestTarget is not None:
        #             SmartDashboard.putNumber("tag ID", bestTarget.getFiducialId())
        #             SmartDashboard.putNumber("pose ambiguity", bestTarget.getPoseAmbiguity())
        #             SmartDashboard.putNumber("tag transform X", bestTarget.getBestCameraToTarget().X())
        #             SmartDashboard.putNumber("tag transform Y", bestTarget.getBestCameraToTarget().Y())
        #             SmartDashboard.putNumber("tag transform Z", bestTarget.getBestCameraToTarget().Z())
        #             SmartDashboard.putNumber("tag transform angle", bestTarget.getBestCameraToTarget().rotation().angle_degrees)
        #             SmartDashboard.putNumber("tag yaw", bestTarget.getYaw())()
                # SmartDashboard.putNumber("Vision Pose X", self.getEstimatedGlobalPose().estimatedPose.X())
                # SmartDashboard.putNumber("Vision Pose Y", self.getEstimatedGlobalPose().estimatedPose.Y())
                # SmartDashboard.putNumber("Vision Pose Angle", self.getEstimatedGlobalPose().estimatedPose.rotation().angle_degrees)








