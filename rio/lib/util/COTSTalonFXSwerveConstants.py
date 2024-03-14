from phoenix6.signals import InvertedValue;
from phoenix6.signals import SensorDirectionValue;
import math

import lib.mathlib.units as Units

class COTSTalonFXSwerveConstants:
    wheelDiameter: float
    wheelCircumference: float
    angleGearRatio: float
    driveGearRatio: float
    angleKP: float
    angleKI: float
    angleKD: float
    driveMotorInvert: InvertedValue
    angleMotorInvert: InvertedValue
    cancoderInvert: SensorDirectionValue

    def __init__(self, wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert):
        self.wheelDiameter = wheelDiameter
        self.wheelCircumference = wheelDiameter * math.pi
        self.angleGearRatio = angleGearRatio
        self.driveGearRatio = driveGearRatio
        self.angleKP = angleKP
        self.angleKI = angleKI
        self.angleKD = angleKD
        self.driveMotorInvert = driveMotorInvert
        self.angleMotorInvert = angleMotorInvert
        self.cancoderInvert = cancoderInvert

    # Swerve Drive Specialties - MK4i Module
    class MK4i: 
        # Swerve Drive Specialties - MK4i Module (Falcon 500)
        def Falcon500(driveGearRatio):
            wheelDiameter = Units.inchesToMeters(4.0)
    
            # (150 / 7) : 1
            angleGearRatio = ((150.0 / 7.0) / 1.0)
    
            angleKP = 100.0
            angleKI = 0.0
            angleKD = 0.0

            driveMotorInvert = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            angleMotorInvert = InvertedValue.CLOCKWISE_POSITIVE
            cancoderInvert = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
            return COTSTalonFXSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert)

        # Swerve Drive Specialties - MK4i Module (Kraken X60)
        def KrakenX60(driveGearRatio):
            wheelDiameter = Units.inchesToMeters(4.0)
    
            # (150 / 7) : 1
            angleGearRatio = ((150.0 / 7.0) / 1.0)
    
            angleKP = 1.0
            angleKI = 0.0
            angleKD = 0.0
    
            driveMotorInvert = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            angleMotorInvert = InvertedValue.CLOCKWISE_POSITIVE
            cancoderInvert = SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
            return COTSTalonFXSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert)

        class driveRatios:
            # SDS MK4i - (8.14 : 1)
            L1 = (8.14 / 1.0)
            # SDS MK4i - (6.75 : 1)
            L2 = (6.75 / 1.0)
            # SDS MK4i - (6.12 : 1)
            L3 = (6.12 / 1.0)

  