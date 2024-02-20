from hardware_interface.drivetrain import DriveTrain
import hardware_interface.drivetrain as dt
from hardware_interface.joystick import Joystick
from hardware_interface.armcontroller import ArmController
from commands2 import *
import wpilib
from wpilib import Field2d
from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import SuppliedFloatValueWidget
from hardware_interface.commands.drive_commands import *
from auton_selector import AutonSelector
import time
from dds.dds import DDS_Publisher, DDS_Subscriber
import os
import inspect
import logging
import traceback
import threading
from lib.mathlib.conversions import Conversions

from pathplannerlib.auto import NamedCommands

ENABLE_STAGE_BROADCASTER = True
ENABLE_ENCODER = True

# Global Variables
arm_controller : ArmController = None
drive_train : DriveTrain = None
navx_sim_data : list = None
frc_stage = "DISABLED"
fms_attached = False
stop_threads = False

# Logging
format = "%(asctime)s: %(message)s"
logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")

# XML Path for DDS configuration
curr_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
xml_path = os.path.join(curr_path, "dds/xml/ROS_RTI.xml")

############################################
## Hardware
def initDriveTrain():
    global drive_train
    if drive_train == None:
        drive_train = DriveTrain()
        logging.info("Success: DriveTrain created")
    return drive_train

def initArmController():
    global arm_controller
    if arm_controller == None:
        arm_controller = ArmController()
        logging.info("Success: ArmController created")
    return arm_controller

def initDDS(ddsAction, participantName, actionName):
    dds = None
    with rti_init_lock:
        dds = ddsAction(xml_path, participantName, actionName)
    return dds

############################################
## Threads
def threadLoop(name, dds, action):
    logging.info(f"Starting {name} thread")
    global stop_threads
    global frc_stage
    try:
        while stop_threads == False:
            if (frc_stage == 'AUTON' and name != "joystick") or (name in ["stage-broadcaster", "service", "imu", "zed"]) or (frc_stage == 'TELEOP'):
                action(dds)
            time.sleep(20/1000)
    except Exception as e:
        logging.error(f"An issue occured with the {name} thread")
        logging.error(e)
        logging.error(traceback.format_exc())
    
    logging.info(f"Closing {name} thread")
    dds.close()
    
# Generic Start Thread Function
def startThread(name) -> threading.Thread | None:
    thread = None
    # if name == "encoder":
    #     thread = threading.Thread(target=encoderThread, daemon=True)
    if name == "stage-broadcaster":
        thread = threading.Thread(target=stageBroadcasterThread, daemon=True)
    elif name == "service":
        thread = threading.Thread(target=serviceThread, daemon=True)
    elif name == "imu":
        thread = threading.Thread(target=imuThread, daemon=True)
    elif name == "zed":
        thread = threading.Thread(target=zedThread, daemon=True)
    
    thread.start()
    return thread

# Locks
rti_init_lock = threading.Lock()
drive_train_lock = threading.Lock()
arm_controller_lock = threading.Lock()
############################################

################## ENCODER ##################
ENCODER_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::encoder_info"
ENCODER_WRITER_NAME = "encoder_info_publisher::encoder_info_writer"

# def encoderThread():
#     encoder_publisher = initDDS(DDS_Publisher, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
#     threadLoop('encoder', encoder_publisher, encoderAction)

def encoderAction(publisher):
    # TODO: Make these some sort of null value to identify lost data
    data = {
        'name': [],
        'position': [],
        'velocity': [],
        'effort': []
    }

    global drive_train
    with drive_train_lock:
        drive_data = drive_train.getModuleCommand()
        data['name'] += drive_data['name']
        data['position'] += drive_data['position']
        data['velocity'] += drive_data['velocity']
    
    global arm_controller
    with arm_controller_lock:
        arm_data = arm_controller.getEncoderData()
        data['name'] += arm_data['name']
        data['position'] += arm_data['position']
        data['velocity'] += arm_data['velocity']

    publisher.write(data)
############################################

################## SERVICE ##################
SERVICE_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::service"
SERVICE_WRITER_NAME = "service_pub::service_writer"

def serviceThread():
    service_publisher = initDDS(DDS_Publisher, SERVICE_PARTICIPANT_NAME, SERVICE_WRITER_NAME)
    threadLoop('service', service_publisher, serviceAction)

def serviceAction(publisher : DDS_Publisher):
    temp_service = True
    
    publisher.write({ "data": temp_service })
############################################

################## STAGE ##################
STAGE_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::stage_broadcaster"
STAGE_WRITER_NAME = "stage_publisher::stage_writer"

def stageBroadcasterThread():
    stage_publisher = initDDS(DDS_Publisher, STAGE_PARTICIPANT_NAME, STAGE_WRITER_NAME)
    threadLoop('stage-broadcaster', stage_publisher, stageBroadcasterAction)

def stageBroadcasterAction(publisher : DDS_Publisher):
    global frc_stage
    global fms_attached
    is_disabled = wpilib.DriverStation.isDisabled()
    
    publisher.write({ "data": f"{frc_stage}|{fms_attached}|{is_disabled}" })
############################################

################## IMU #####################   
IMU_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::imu"
IMU_READER_NAME = "imu_subscriber::imu_reader"

def imuThread():
    imu_subscriber = initDDS(DDS_Subscriber, IMU_PARTICIPANT_NAME, IMU_READER_NAME)
    threadLoop("imu", imu_subscriber, imuAction)
    
def imuAction(subscriber):
    data: dict = subscriber.read()
    if data is not None:
        
        arr = data["data"].split("|")
        w = float(arr[0])
        x = float(arr[1])
        y = float(arr[2])
        z = float(arr[3])
        angular_velocity_x = float(arr[4])
        angular_velocity_y = float(arr[5])
        angular_velocity_z = float(arr[6])
        linear_acceleration_x = float(arr[7])
        linear_acceleration_y = float(arr[8])
        linear_acceleration_z = float(arr[9])
        global navx_sim_data
        navx_sim_data = [
            w, x, y, z, 
            angular_velocity_x, angular_velocity_y, angular_velocity_z, 
            linear_acceleration_x, linear_acceleration_y, linear_acceleration_z
        ]
############################################

object_pos = [0.0, 0.0, 0.0]


ZED_PARTICIPANT_NAME = "ROS2_PARTICIPANT_LIB::zed_objects"
ZED_READER_NAME = "zed_objects_subscriber::zed_objects_reader"

def zedThread():
    zed_subscriber = initDDS(DDS_Subscriber, ZED_PARTICIPANT_NAME, ZED_READER_NAME)
    threadLoop("zed", zed_subscriber, zedAction)
    
def zedAction(subscriber):
    global object_pos
    data: dict = subscriber.read()
    if data is not None:
        arr = data["data"].split("|")
        object_pos[0] = float(arr[0])
        object_pos[1] = float(arr[1])
        object_pos[2] = float(arr[2])
        
class Robot(wpilib.TimedRobot):
    def robotInit(self):
        self.use_threading = True
        wpilib.CameraServer.launch()
        logging.warning("Running in simulation!") if wpilib.RobotBase.isSimulation() else logging.info("Running in real!")
        
        self.threads = []
        if self.use_threading:
            logging.info("Initializing Threads")
            global stop_threads
            stop_threads = False
            # if ENABLE_ENCODER: self.threads.append({"name": "encoder", "thread": startThread("encoder") })
            if ENABLE_STAGE_BROADCASTER: self.threads.append({"name": "stage-broadcaster", "thread": startThread("stage-broadcaster") })
            self.threads.append({"name": "service", "thread": startThread("service") })
            self.threads.append({"name": "imu", "thread": startThread("imu") })
            self.threads.append({"name": "zed", "thread": startThread("zed") })
        else:
            # self.encoder_publisher = DDS_Publisher(xml_path, ENCODER_PARTICIPANT_NAME, ENCODER_WRITER_NAME)
            self.stage_publisher = DDS_Publisher(xml_path, STAGE_PARTICIPANT_NAME, STAGE_WRITER_NAME)
            self.service_publisher = DDS_Publisher(xml_path, SERVICE_PARTICIPANT_NAME, SERVICE_WRITER_NAME)
            self.imu_subscriber = DDS_Subscriber(xml_path, IMU_PARTICIPANT_NAME, IMU_READER_NAME)
            self.zed_subscriber = DDS_Subscriber(xml_path, ZED_PARTICIPANT_NAME, ZED_READER_NAME)
        
        self.arm_controller = initArmController()
        self.drive_train = initDriveTrain()
        self.drive_train.is_sim = self.isSimulation()
        self.joystick = Joystick("xbox")
        self.auton_selector = AutonSelector(self.arm_controller, self.drive_train)
        self.joystick_selector = wpilib.SendableChooser()
        self.joystick_selector.setDefaultOption("XBOX", "xbox")
        self.joystick_selector.addOption("PS4", "ps4")
        self.auton_run = False

        self.shuffleboard = Shuffleboard.getTab("Main")
        self.shuffleboard.add(title="AUTON", defaultValue=self.auton_selector.autonChooser)

        # self.shuffleboard.add(title="JOYSTICK", defaultValue=self.joystick_selector)

        # self.shuffleboard.add("WHINE REMOVAL", self.drive_train.whine_remove_selector)
        self.shuffleboard.addDoubleArray("MOTOR VELOCITY", lambda: (self.drive_train.motor_vels))
        self.shuffleboard.addDoubleArray("MOTOR POSITIONS", lambda: (self.drive_train.motor_pos))
        # self.shuffleboard.add("ANGLE SOURCE", self.drive_train.angle_source_selector)

        # self.shuffleboard.add("PROFILE", self.drive_train.profile_selector)
        self.shuffleboard.add("NAVX", self.drive_train.navx)
        self.shuffleboard.add("PP Auton", self.auton_selector.ppchooser)
        self.shuffleboard.addDouble("YAW", lambda: (self.drive_train.navx.getYaw()))
        self.shuffleboard.addBoolean("FIELD ORIENTED", lambda: (self.drive_train.field_oriented_value))
        # self.shuffleboard.addBoolean("SLOW", lambda: (self.drive_train.slow))
        self.shuffleboard.addDoubleArray("MOTOR TEMPS", lambda: (self.drive_train.motor_temps))
        self.shuffleboard.addDoubleArray("JOYSTICK OUTPUT", lambda: ([self.drive_train.linX, self.drive_train.linY, self.drive_train.angZ]))
        self.shuffleboard.addDoubleArray("POSE: ", lambda: ([self.auton_selector.drive_subsystem.getPose().X(), self.auton_selector.drive_subsystem.getPose().Y(), self.auton_selector.drive_subsystem.getPose().rotation().degrees()]))

        self.shuffleboard.add("PP Chooser", self.auton_selector.ppchooser)

        self.shuffleboard.addDouble("Pose X", lambda: self.auton_selector.drive_subsystem.getPose().X())
        self.shuffleboard.addDouble("Pose Y", lambda: self.auton_selector.drive_subsystem.getPose().Y())
        self.shuffleboard.addDouble("Pose Rotation", lambda: self.auton_selector.drive_subsystem.getPose().rotation().degrees())
        self.shuffleboard.addDouble("Sweeping Rotation", lambda: self.auton_selector.drive_subsystem.drivetrain.navx.getRotation2d().__mul__(-1).degrees())
        self.shuffleboard.addDoubleArray("CHASSIS SPEEDS", lambda: [
            self.auton_selector.drive_subsystem.getRobotRelativeChassisSpeeds().vx,
            self.auton_selector.drive_subsystem.getRobotRelativeChassisSpeeds().vy,
            self.auton_selector.drive_subsystem.getRobotRelativeChassisSpeeds().omega
        ])
        # self.shuffleboard.addString("AUTO TURN STATE", lambda: (self.drive_train.auto_turn_value))
        
        # self.second_order_chooser = wpilib.SendableChooser()
        # self.second_order_chooser.setDefaultOption("1st Order", False)
        # self.second_order_chooser.addOption("2nd Order", True)
        
        # self.shuffleboard.add("2nd Order", self.second_order_chooser)
        self.arm_controller.setToggleButtons()
        
        self.auton_run = False
        
        self.turn_90 = TurnToAngleCommand(self.auton_selector.drive_subsystem, 90.0)
        self.turn_180 = TurnToAngleCommand(self.auton_selector.drive_subsystem, 180.0)
        self.turn_270 = TurnToAngleCommand(self.auton_selector.drive_subsystem, 270.0)
        self.turn_0 = TurnToAngleCommand(self.auton_selector.drive_subsystem, 0.0)

        NamedCommands.registerCommand("Turn180", self.turn_180)


    def robotPeriodic(self):
        self.joystick.type = self.joystick_selector.getSelected()
        self.auton_selector.drive_subsystem.updateOdometry()
        if navx_sim_data is not None:
            self.drive_train.navx_sim.update(*navx_sim_data)


    # Auton
    def autonomousInit(self):
        self.drive_train.navx.reset()
        self.drive_train.set_navx_offset(0)
        self.auton_selector.run()
        global object_pos
        # self.cone_move = ConeMoveAuton(self.auton_selector.drive_subsystem, object_pos)
        logging.info("Entering Auton")
        global frc_stage
        frc_stage = "AUTON"

    def autonomousPeriodic(self):
        CommandScheduler.getInstance().run()
        global object_pos       
        global fms_attached
        
        # dist = math.sqrt(object_pos[0]**2 + object_pos[1]**2)
        # if dist > 0.5:
        #     self.cone_move.execute()
        #     self.cone_move.object_pos = object_pos
        # else:
        #     print(f"GIVEN POS: {object_pos} ********************")
        #     self.drive_train.swerveDriveAuton(0, 0, 0)
        #     self.drive_train.stop()(
        
        # self.drive_train.swerveDriveAuton(object_pos[0]/5.0, object_pos[1]/5.0, object_pos[2]/5.0)
        
        logging.info(f"Robot Pose: {self.auton_selector.drive_subsystem.getPose()}")
        
        fms_attached = wpilib.DriverStation.isFMSAttached()
        if self.use_threading:
            self.manageThreads()
        else:
            self.doActions()
            
    def autonomousExit(self):
        CommandScheduler.getInstance().cancelAll()
        logging.info("Exiting Auton")
        global frc_stage
        frc_stage = "AUTON"


    # Teleop
    def teleopInit(self):
        self.arm_controller.setToggleButtons()
        CommandScheduler.getInstance().cancelAll()
        logging.info("Entering Teleop")
        global frc_stage
        frc_stage = "TELEOP"

    def teleopPeriodic(self):
        self.drive_train.swerveDrive(self.joystick)
        self.arm_controller.setArm(self.joystick)
        global fms_attached
        fms_attached = wpilib.DriverStation.isFMSAttached()
        if self.use_threading:
            self.manageThreads()
        else:
            self.doActions()
        
    def manageThreads(self):
        # Check all threads and make sure they are alive
        for thread in self.threads:
            if thread["thread"].is_alive() == False:
                logging.warning(f"Thread {thread['name']} is not alive, restarting...")
                thread["thread"] = startThread(thread["name"])
                
    def doActions(self):
        # encoderAction(self.encoder_publisher)
        stageBroadcasterAction(self.stage_publisher)
        serviceAction(self.service_publisher)
        imuAction(self.imu_subscriber)
        zedAction(self.zed_subscriber)
        
    def stopThreads(self):
        global stop_threads
        stop_threads = True
        for thread in self.threads:
            thread.join()
        logging.info('All Threads Stopped')
        

if __name__ == '__main__':
    wpilib.run(Robot)