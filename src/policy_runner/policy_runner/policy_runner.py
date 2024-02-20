import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import torch
import torch.nn as nn
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from rl_games.algos_torch.models import ModelA2CContinuous
from rl_games.algos_torch.torch_ext import load_checkpoint
from rl_games.algos_torch.network_builder import A2CBuilder
from rl_games.common.a2c_common import ContinuousA2CBase
import yaml

class Reader(Node):
    def __init__(self):
        super().__init__("reinforcement_learning_runner")
        # self.robot_ip = robot_ip
        # self.policy = torch.load("/workspaces/rift2024/isaac/Eaglegym/eaglegym/runs/RiftK/nn/RiftK.pth")
        
        self.policy = self.load_checkpoint("/workspaces/rift2024/isaac/Eaglegym/eaglegym/runs/RiftK/nn/RiftK_1050.pth")
        
        self.joint_action_pub = self.create_publisher(String, "/real/cmd_vel", 10)
        # self.joint_trajectory_action_pub = self.create_publisher(Twist, "joint_trajectory_message", 10)
        
        self.declare_parameter("odom_topic", "/real/odom")
        self.declare_parameter("target_topic", "/real/obj_det_pose")
        
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.target_topic = self.get_parameter("target_topic").get_parameter_value().string_value
        
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.target_sub = self.create_subscription(String, self.target_topic, self.target_callback, 10)
        # self.joint_state_sub = self.create_subscription(Float32, "joint_state", self.joint_state_callback, 10)
        self.odom_msg = Odometry()
        # self.joint_state_msg = JointState()
        self.twist_msg = Twist()
        # self.cmds = JointTrajectory()
        # self.position_cmds = JointTrajectoryPoint()
        self.episode_reward = 0
        self.step = 0
        
        self.i = 0
        # self.joints = [
        #     'arm_roller_bar_joint',
        #     'elevator_center_joint',
        #     'elevator_outer_1_joint',
        #     'elevator_outer_2_joint',
        #     'top_gripper_right_arm_joint',
        #     'top_gripper_left_arm_joint',
        #     'top_slider_joint',
        #     'bottom_intake_joint',
        # ]
        
        self.target_pos = []

        self.get_logger().info("\033[92m" + "Policy Runner Started" + "\033[0m")
        
    def load_checkpoint(self, filepath):
        config = yaml.load(open("/workspaces/rift2024/isaac/Eaglegym/eaglegym/cfg/train/RiftKPPO.yaml", "r"), Loader=yaml.FullLoader)
        config = config["params"]
        state = load_checkpoint(filepath)
        state_dict = {k.replace('a2c_network.', ''): v for k, v in state['model'].items()}
        builder = A2CBuilder()
        builder.load(config["network"])
        network = builder.build("network", actions_num=10, input_shape=(13,))
        
        model_state_dict = network.state_dict()
        state_dict = {k: v for k, v in state_dict.items() if k in model_state_dict}
        
        network.load_state_dict(state_dict)
        network.eval()
        network.train(False)

        return network
        # model = checkpoint['model']
        # model.load_state_dict(checkpoint)
        # for parameter in model.parameters():
        #     parameter.requires_grad = False
        # model.eval()
        # return model

    def get_action(self, msg):
        '''
        Gym obs type for RiftK:
            [0:3] = ([target_pos] - [robot_pos]) / 3 # x, y, z
            [3:7] = [robot_rotation_quaternion] # x, y, z, w
            [7:10] = [robot_linear_velocities] / 2 # x, y, z
            [10:13] = [robot_angular_velocities] / M_PI # x, y, z
            
        Input type String:
            0,1,2,3,4,5,6,7,8,9,10,11,12,13
        '''
                
        # convert string to list
        input_obs = msg.data.split(",")
        
        obs = np.array(input_obs, dtype=np.float32)
        
        obs = torch.tensor(obs).float()
        
        obs = obs.unsqueeze(0)
        
        observation = {"obs": obs}
        
        action = self.policy(observation)

        vel = action[0].detach().numpy()[0]
        
        self.get_logger().info("Full Action: " + np.array_str(vel, precision=2))
        
        # vel = [self.limit(i) for i in vel]
    
        # ======================= convert action to twist message ===================================
        
        self.twist_msg.linear.x = float(vel[0])
        self.twist_msg.linear.y = float(vel[1])
        self.twist_msg.linear.z = float(vel[2])
        
        # self.position_cmds.positions = [
        #     action[3].detach().numpy(),
        #     action[4].detach().numpy(),
        #     action[5].detach().numpy(),
        #     action[4].detach().numpy(),
        #     action[6].detach().numpy(),
        #     action[6].detach().numpy(),
        #     action[7].detach().numpy(),
        #     action[6].detach().numpy(),
        #     action[8].detach().numpy(),
        #     action[8].detach().numpy(),
        # ]
        # self.cmds.joint_names = self.joints
        # self.cmds.points = [self.position_cmds]
        
        # self.publisher_.publish(self.cmds)
        
        
        output = String()
        output.data = f"{-vel[1]}|{-vel[0]}|{vel[2]}"
        
        # if self.target_pos == [0, 0, 0]:
        #     output.data = f"0.0|0.0|0.0"
        #     vel = [0.0, 0.0, 0.0]
        
        self.print_in_color(f"Action: {str(vel[0:3])}", "blue")
        
        self.joint_action_pub.publish(output)
        self.step += 1
        
    def limit(self, value):
        speed = 0
        if value > 1:
            speed = 1
        elif value < -1:
            speed = -1
        else:
            speed = value
            
        return speed / 10

    def odom_callback(self, msg: Odometry):
        if(msg != None and len(self.target_pos) > 0):
            self.odom_msg = msg
            obs_string = String()
            robot_pos = [
                float(self.odom_msg.pose.pose.position.x),
                float(self.odom_msg.pose.pose.position.y),
                float(self.odom_msg.pose.pose.position.z)
            ]
            robot_rot_quat = [
                float(self.odom_msg.pose.pose.orientation.x),
                float(self.odom_msg.pose.pose.orientation.y),
                float(self.odom_msg.pose.pose.orientation.z),
                float(self.odom_msg.pose.pose.orientation.w)
            ]
            robot_linear_vel = [
                float(self.odom_msg.twist.twist.linear.x),
                float(self.odom_msg.twist.twist.linear.y),
                float(self.odom_msg.twist.twist.linear.z)
            ]
            robot_angular_vel = [
                float(self.odom_msg.twist.twist.angular.x),
                float(self.odom_msg.twist.twist.angular.y),
                float(self.odom_msg.twist.twist.angular.z)
            ]
            
            obs_input = [
                (self.target_pos[0] - robot_pos[0]),
                (self.target_pos[1] - robot_pos[1]),
                (self.target_pos[2] - robot_pos[2]),
                robot_rot_quat[0],
                robot_rot_quat[1],
                robot_rot_quat[2],
                robot_rot_quat[3],
                robot_linear_vel[0],
                robot_linear_vel[1],
                robot_linear_vel[2],
                robot_angular_vel[0],
                robot_angular_vel[1],
                robot_angular_vel[2]
            ]
            
            obs_string.data = ",".join([str(i) for i in obs_input])
            
            obs_dict = {
                "target position": self.target_pos,
                "robot position": robot_pos,
                "robot rotation quaternion": robot_rot_quat,
                "robot linear velocity": robot_linear_vel,
                "robot angular velocity": robot_angular_vel
            }
            
            self.print_in_color(f"Observation: {obs_dict}", "blue")
            
            self.get_action(obs_string)
        return
    
    def target_callback(self, msg):
        if(msg != None):
            self.target_pos = msg.data.split("|")
            self.target_pos = [float(i) for i in self.target_pos]      
        return

    def get_reward():
        return
    
    def print_in_color(self, msg, color):
        if(color == "green"):
            self.get_logger().info("\033[92m" + msg + "\033[0m")
        elif(color == "blue"):
            self.get_logger().info("\033[94m" + msg + "\033[0m")
        elif(color == "red"):
            self.get_logger().info("\033[91m" + msg + "\033[0m")
        else:
            self.get_logger().info(msg)
        return

def main(args=None):
    # env = gym.create_env("RealRobot", ip=self.robot_ip)
    rclpy.init(args=args)
    reader = Reader()
    rclpy.spin(reader)
    # env.disconnect()
if __name__ == '__main__':
    main()