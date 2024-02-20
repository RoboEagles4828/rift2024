import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Imu
from rclpy.time import Time, Duration
from std_msgs.msg import Header, String
import math

class IsaacDriveHardware(Node):
    def __init__(self):
        super().__init__('isaac_drive_hardware')
        self.realtime_isaac_publisher_drive = self.create_publisher(JointState, 'isaac_drive_commands', 10)
        self.realtime_isaac_publisher_arm = self.create_publisher(JointState, 'isaac_arm_commands', 10)
        self.real_imu_publisher = self.create_publisher(String, 'real_imu', 10)
        # self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        self.isaac_subscriber = self.create_subscription(JointState, 'isaac_joint_states', self.isaac_callback, 10)
        self.real_subscriber = self.create_subscription(JointState, '/real/real_joint_states', self.real_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        
        self.OKGREEN = '\033[92m'
        self.ENDC = '\033[0m'
        
        self.joint_names: list[str] = []
        self.joint_state: JointState = None
        self.joint_names2: list[str] = []
        self.joint_state2: JointState = None
        
        self.arm_joint_names = []
        self.drive_joint_names = []
        
        self.command_effort = []
        self.command_position = []
        self.empty = []
        
        self.realtime_isaac_command: JointState = JointState()
        self.joint_state_command: JointState = JointState()
        
        self.header = Header()
        
        self.get_logger().info(self.OKGREEN + "Configured and Activated Isaac Drive Hardware" + self.ENDC)
        
    def imu_callback(self, imu: Imu):
        if imu == None:
            self.get_logger().warn("Imu message recieved was null")
        imu_string = String()
        data = f"{imu.orientation.w}|{imu.orientation.x}|{imu.orientation.y}|{imu.orientation.z}|{imu.angular_velocity.x}|{imu.angular_velocity.y}|{imu.angular_velocity.z}|{imu.linear_acceleration.x}|{imu.linear_acceleration.y}|{imu.linear_acceleration.z}"
        imu_string.data = data
        self.real_imu_publisher.publish(imu_string)
        
    def real_callback(self, joint_state: JointState):
        self.joint_names = list(joint_state.name)
        self.joint_state = joint_state
        self.get_logger().info(self.OKGREEN + "Recieved Real Joint State" + self.ENDC)
        self.write()
        
    def isaac_callback(self, joint_state: JointState):
        self.joint_names2 = list(joint_state.name)
        self.joint_state2 = joint_state
        if self.joint_state2 == None:
            self.get_logger().warn("Velocity message recieved was null")
        else:
            self.read()
        
    def convertToRosPosition(self, isaac_position: float):
        if isaac_position > math.pi:
            return isaac_position - 2.0 * math.pi
        elif isaac_position < -math.pi:
            return isaac_position + 2.0 * math.pi
        return isaac_position
        
    def read(self):
        self.joint_state_command.effort = []
        names = self.joint_state2.name
        positions = self.joint_state2.position
        velocities = self.joint_state2.velocity
        efforts = self.joint_state2.effort
        
        for i in range(len(self.joint_names)-1):
            for j in range(len(names)-1):
                if names[j] == self.joint_names[i]:
                    self.joint_state_command.position.append(self.convertToRosPosition(positions[j]))
                    self.joint_state_command.velocity.append(velocities[j])
                    self.joint_state_command.effort.append(efforts[j])
                    break              
        # self.joint_state_command.header.stamp = Time(seconds=self._clock.now().seconds_nanoseconds()[0], nanoseconds=self._clock.now().seconds_nanoseconds()[1])
        # self.joint_state_publisher.publish(self.joint_state_command)
                    
    def write(self):
        self.command_effort = []
        self.command_position = []
        self.arm_joint_names.clear()
        self.drive_joint_names.clear()
        for j, i in enumerate(self.joint_names):
            if i.__contains__("wheel") or i.__contains__("axle"):
                vel = self.joint_state.velocity[j]
                self.command_effort.append(vel/10000.0)
                self.drive_joint_names.append(i)
            else:
                self.arm_joint_names.append(i)
                position = self.joint_state.position[j]/10000.0
                
                # Elevator
                if i == "arm_roller_bar_joint":
                    # split position among 2 joints
                    self.command_position.append(position)
                    self.arm_joint_names.append("elevator_outer_1_joint")
                    if position == 0.07:
                        self.command_position.append(0.2)
                    elif position == 0.0:
                        self.command_position.append(0.0)
                elif i == "top_slider_joint":
                    self.command_position.append(position)
                elif i == "top_gripper_left_arm_joint":
                    self.command_position.append(position)
                    self.arm_joint_names.append("top_gripper_right_arm_joint")
                    self.command_position.append(position)
                elif i == "elevator_center_joint":
                    elevator_max = 0.56
                    elevator_min = 0.0
                    
                    #scale position to be between 0 and 1
                    position = (position - elevator_min)/(elevator_max - elevator_min)
                    
                    self.command_position.append(position/2.0)
                    self.arm_joint_names.append("elevator_outer_2_joint")
                    self.command_position.append(position/2.0)
                        
        self.header.stamp = self._clock.now().to_msg()
        
        self.realtime_isaac_command.header = self.header
        self.realtime_isaac_command.name = self.drive_joint_names
        self.realtime_isaac_command.velocity = self.command_effort
        self.realtime_isaac_command.position = self.empty
        self.realtime_isaac_command.effort = self.empty
        
        self.realtime_isaac_publisher_drive.publish(self.realtime_isaac_command)
        
        self.header.stamp = self._clock.now().to_msg()
        
        self.realtime_isaac_command.header = self.header
        self.realtime_isaac_command.name = self.arm_joint_names
        self.realtime_isaac_command.velocity = self.empty
        self.realtime_isaac_command.position = self.command_position
        self.realtime_isaac_command.effort = self.empty
        
        self.realtime_isaac_publisher_arm.publish(self.realtime_isaac_command)
        
        
def main(args=None):
    rclpy.init(args=args)
    
    isaac_drive_hardware = IsaacDriveHardware()
    rclpy.spin(isaac_drive_hardware)
    
    isaac_drive_hardware.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
        
        
                