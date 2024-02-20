import math

class MotionMagic():
    def __init__(self, max_accel, max_vel):
        self.max_accel = max_accel
        self.max_vel = max_vel
        
        self.prevTargetPosition = 0
        self.totalDistance = 0
        self.tolerance = 0.05
        
        self.velocityInRampWindow1 = 0.5
        self.velocityInRampWindow2 = 0.66
        self.velocityInCruiseWindow = 1.0
        self.rampWindow1 = 0.3
        self.rampWindow2 = 0.8
        
        
    def getPositionDifference(self, targetPosition, sensorPosition):
        m_targetPos = math.fmod(targetPosition, 2*math.pi)
        m_sensorPos = math.fmod(sensorPosition, 2*math.pi)
        copy_targetPosition = m_targetPos
        difference = copy_targetPosition - m_sensorPos
        if difference > math.pi:
            copy_targetPosition -= 2*math.pi
        elif difference < -math.pi:
            copy_targetPosition += 2*math.pi
        
        return math.fmod(copy_targetPosition - m_sensorPos, 2*math.pi)
    
    def getNextVelocity(self, targetPosition, sensorPosition):
        error = self.getPositionDifference(targetPosition, sensorPosition)
        absError = abs(error)
        if targetPosition != self.prevTargetPosition:
            self.totalDistance = absError
            self.prevTargetPosition = targetPosition
        
        if absError < self.tolerance:
            return 0
        
        dir = 1.0
        if error < 0.0:
            dir = -1.0
            
        if absError <= self.rampWindow1:
            return dir * self.velocityInRampWindow1
        elif absError <= self.rampWindow2:
            return dir * self.velocityInRampWindow2
        else:
            return dir * self.velocityInCruiseWindow