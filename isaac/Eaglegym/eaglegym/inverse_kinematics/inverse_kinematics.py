from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Translation2d, Rotation2d
from eaglegym.inverse_kinematics.motion_magic_control import MotionMagic
import math

class InverseKinematics():
    def __init__(self, max_vel):
        self.speeds = ChassisSpeeds()
        self.module_config = {
            "front_left": {
                "wheel_joint_name": "front_left_wheel_joint",
                "wheel_motor_port": 3, #9
                "axle_joint_name": "front_left_axle_joint",
                "axle_motor_port": 1, #7
                "axle_encoder_port": 2, #8
                "encoder_offset": 18.721, # 248.203,
                "location" : Translation2d(-0.52085486, 0.52085486) # Translation2d(0.52085486, 0.52085486)
            },
            "front_right": {
                "wheel_joint_name": "front_right_wheel_joint",
                "wheel_motor_port": 6, #12
                "axle_joint_name": "front_right_axle_joint",
                "axle_motor_port": 4, #10
                "axle_encoder_port": 5, #11
                "encoder_offset": 45.439 + 180.0, #15.908, TODO: REDO ENCODER OFFSET
                "location" : Translation2d(-0.52085486, -0.52085486)# Translation2d(-0.52085486, 0.52085486)
            },
            "rear_left": {
                "wheel_joint_name": "rear_left_wheel_joint",
                "wheel_motor_port": 12, #6
                "axle_joint_name": "rear_left_axle_joint",
                "axle_motor_port": 10, #4
                "axle_encoder_port": 11, #5
                "encoder_offset": 16.084 + 180.0, #327.393, TODO: REDO ENCODER OFFSET
                "location" : Translation2d(0.52085486, 0.52085486) #Translation2d(0.52085486, -0.52085486)
            },
            "rear_right": {
                "wheel_joint_name": "rear_right_wheel_joint",
                "wheel_motor_port": 9, #3
                "axle_joint_name": "rear_right_axle_joint",
                "axle_motor_port": 7, #1
                "axle_encoder_port": 8, #2
                "encoder_offset": -9.141, #201.094,
                "location" : Translation2d(0.52085486, -0.52085486) # Translation2d(-0.52085486, -0.52085486)
            }
        }
        self.front_left_location = self.module_config["front_left"]["location"]
        self.front_right_location = self.module_config["front_right"]["location"]
        self.rear_left_location = self.module_config["rear_left"]["location"]
        self.rear_right_location = self.module_config["rear_right"]["location"]
        self.ROBOT_MAX_TRANSLATIONAL = max_vel
        self.ROBOT_MAX_ROTATIONAL = max_vel
        self.MODULE_MAX_SPEED = max_vel
        self.kinematics = SwerveDrive4Kinematics(self.front_left_location, self.front_right_location, self.rear_left_location, self.rear_right_location)
        
        self.positionCoefficient = 2.0 * math.pi / 2048.0
        self.velocityCoefficient = self.positionCoefficient * 10.0
        
        self.MAX_VEL = 18000
        self.MAX_ACCEL = 14000
        
        self.velocityConstant = 0.5
        self.accelerationConstant = 0.25
        
        self.real_mm_accel = (8.0 - 2.0) / self.accelerationConstant / self.velocityCoefficient
        self.real_mm_vel = 2.0 / self.velocityConstant / self.velocityCoefficient
        
        self.motion_magic = MotionMagic(self.ticksToRadians(self.MAX_ACCEL, "velocity") * 10, self.ticksToRadians(self.MAX_VEL, "velocity"))
        
        self.new_motion_magic_1 = MotionMagic(self.ticksToRadians(self.real_mm_accel, "velocity") * 10, self.ticksToRadians(self.real_mm_vel, "velocity"))
        self.new_motion_magic_2 = MotionMagic(self.ticksToRadians(self.real_mm_accel, "velocity") * 10, self.ticksToRadians(self.real_mm_vel, "velocity"))
        self.new_motion_magic_3 = MotionMagic(self.ticksToRadians(self.real_mm_accel, "velocity") * 10, self.ticksToRadians(self.real_mm_vel, "velocity"))
        self.new_motion_magic_4 = MotionMagic(self.ticksToRadians(self.real_mm_accel, "velocity") * 10, self.ticksToRadians(self.real_mm_vel, "velocity"))
    
    def metersToRadians(self, meters):
        wheel_rad = 0.0508
        return meters/wheel_rad
    
    def ticksToRadians(self, ticks, displacementType):
        if displacementType == "position":
            return ticks * self.positionCoefficient
        elif displacementType == "velocity":
            return ticks * self.velocityCoefficient
        else:
            return 0
    
    def getDriveJointStates(self, x, y, z, module_angles: list, imu_angle_radians):
        self.speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, Rotation2d(imu_angle_radians))
        curr_module_states = self.kinematics.toSwerveModuleStates(self.speeds)
        module_speeds = [i.speed for i in curr_module_states]
        
        # remove diagonal saturation from wheel speed
        is_above = False
        for i in module_speeds:
            if i > self.MODULE_MAX_SPEED:
                is_above = True
                break
        if is_above:
            for i in range(len(module_speeds)):
                module_speeds[i] = module_speeds[i] / max(module_speeds) * self.MODULE_MAX_SPEED
        
        module_states = []
        for index, i in enumerate(module_speeds):
            module_states.append(SwerveModuleState(i, curr_module_states[index].angle))
        
        front_left_state = module_states[0]
        front_right_state = module_states[1]
        rear_left_state = module_states[2]
        rear_right_state = module_states[3]
        
        front_left_state = SwerveModuleState.optimize(front_left_state, Rotation2d(module_angles[0]))
        front_right_state = SwerveModuleState.optimize(front_right_state, Rotation2d(module_angles[1]))
        rear_left_state = SwerveModuleState.optimize(rear_left_state, Rotation2d(module_angles[2]))
        rear_right_state = SwerveModuleState.optimize(rear_right_state, Rotation2d(module_angles[3]))
        
        front_left_speed = self.metersToRadians(front_left_state.speed)
        front_right_speed = self.metersToRadians(front_right_state.speed)
        rear_left_speed = self.metersToRadians(rear_left_state.speed)
        rear_right_speed = self.metersToRadians(rear_right_state.speed)
        
        front_left_angle = self.new_motion_magic_1.getNextVelocity(front_left_state.angle.radians(), module_angles[0])
        front_right_angle = self.new_motion_magic_2.getNextVelocity(front_right_state.angle.radians(), module_angles[1])
        rear_left_angle = self.new_motion_magic_3.getNextVelocity(rear_left_state.angle.radians(), module_angles[2])
        rear_right_angle = self.new_motion_magic_4.getNextVelocity(rear_right_state.angle.radians(), module_angles[3])
        
        return [
            front_left_speed,
            front_right_speed,
            rear_left_speed,
            rear_right_speed,
            front_left_angle,
            front_right_angle,
            rear_left_angle,
            rear_right_angle
        ]
    
    def getArmJointStates(self, names: list, target_positions: list, current_positions: list):
        vel = [0.0]*len(names)
        for j, i in enumerate(names):
            target_position = target_positions[j]
            current_position = current_positions[j]
            output_velocity = self.motion_magic.getNextVelocity(target_position, current_position)
            vel[j] = output_velocity
            
        return vel
            
            
            
            
        
        
        
        