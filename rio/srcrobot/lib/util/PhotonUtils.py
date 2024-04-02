from wpimath.geometry import *
import math

class PhotonUtils:
    @staticmethod
    def calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters, cameraPitchRadians, targetPitchRadians):
            return (targetHeightMeters - cameraHeightMeters) / math.tan(cameraPitchRadians + targetPitchRadians)

    @staticmethod
    def estimateCameraToTargetTranslation(targetDistanceMeters, yaw: Rotation2d):
        return Translation2d(yaw.cos() * targetDistanceMeters, yaw.sin() * targetDistanceMeters);

    @staticmethod
    def estimateFieldToRobot(
            cameraHeightMeters,
            targetHeightMeters,
            cameraPitchRadians,
            targetPitchRadians,
            targetYaw: Rotation2d,
            gyroAngle: Rotation2d,
            fieldToTarget: Pose2d,
            cameraToRobot: Transform2d):
        return PhotonUtils.estimateFieldToRobot(
                PhotonUtils.estimateCameraToTarget(
                        PhotonUtils.estimateCameraToTargetTranslation(
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        cameraHeightMeters, targetHeightMeters, cameraPitchRadians, targetPitchRadians),
                                targetYaw),
                        fieldToTarget,
                        gyroAngle),
                fieldToTarget,
                cameraToRobot)
    
    @staticmethod
    def estimateCameraToTarget(
        cameraToTargetTranslation: Translation2d, fieldToTarget: Pose2d, gyroAngle: Rotation2d):

        return Transform2d(cameraToTargetTranslation, gyroAngle.__mul__(-1).__sub__(fieldToTarget.rotation()))

    @staticmethod
    def estimateFieldToRobot(
            cameraToTarget: Transform2d, fieldToTarget: Pose2d, cameraToRobot: Transform2d):
        return PhotonUtils.estimateFieldToCamera(cameraToTarget, fieldToTarget).transformBy(cameraToRobot)

    @staticmethod
    def estimateFieldToCamera(cameraToTarget: Transform2d, fieldToTarget: Pose2d):
        targetToCamera = cameraToTarget.inverse()
        return fieldToTarget.transformBy(targetToCamera)

    @staticmethod
    def estimateFieldToRobotAprilTag(
        cameraToTarget: Transform3d, fieldRelativeTagPose: Pose3d, cameraToRobot: Transform3d):
        return fieldRelativeTagPose.__add__(cameraToTarget.inverse()).__add__(cameraToRobot)

    @staticmethod
    def getYawToPose(robotPose: Pose2d, targetPose: Pose2d):
        relativeTrl = targetPose.relativeTo(robotPose).translation()
        angle = math.atan2(relativeTrl.Y(), relativeTrl.X())
        return Rotation2d(angle)

    @staticmethod
    def getDistanceToPose(robotPose: Pose2d, targetPose: Pose2d):
        robotTranslation = robotPose.translation()
        return robotTranslation.distance(targetPose.translation())
    
    @staticmethod
    def getDistanceVectorToPose(robotPose: Pose2d, targetPose: Pose2d):
        robotTranslation = robotPose.translation()
        return targetPose.translation().__sub__(robotTranslation)