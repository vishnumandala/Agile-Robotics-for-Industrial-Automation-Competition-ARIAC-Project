#!/usr/bin/env python3
"""
File: ariac_python_node_interface.py
Author: Ankur Mahesh Chavan (achavan1@umd.edu),Datta Lohith Gannavarapu (gdatta@umd.edu),
Shail Kiritkumar Shah (sshah115@umd.edu) Vinay Krishna Bukka (vinay06@umd.edu),
Vishnu Mandala (vishnum@umd.edu)
Date: 04/29/2024
Description: Module to manage orders, perform kitting tasks, and logic for high priority orders
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from ariac_msgs.msg import Order as OrderMsg, AGVStatus, CompetitionState, BasicLogicalCameraImage,AdvancedLogicalCameraImage,  VacuumGripperState, Part
from ariac_msgs.srv import MoveAGV, SubmitOrder,ChangeGripper, VacuumGripperControl, PerformQualityCheck
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from queue import PriorityQueue
from geometry_msgs.msg import Pose,Quaternion, TransformStamped
from sensor_msgs.msg import Image
from launch_ros.substitutions import FindPackageShare
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from collections import Counter
import numpy as np
import cv2 as cv
import time
import threading
import PyKDL
import yaml

# Import custom ROS services
from robot_commander_msgs.srv import (
    EnterToolChanger,
    ExitToolChanger,
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV,
    PickPartBin,
    PlacePartTray,
    ReleasePartOnTray
)

class Kitting:
    """
    Class to represent the kitting task of an order.
    
    Attributes:
        _agv_number (int): Number of the AGV
        _tray_id (int): ID of the tray
        _parts (list): List of parts to be kitted
        _destination (str): Destination of the kitted parts
    
    Methods:
        __init__(self, order_data): Initialize the kitting task
    
    Properties:
        agv_number: Getter for agv_number
        tray_id: Getter for tray_id
        parts: Getter for parts
        destination: Getter for destination
    """

    def __init__(self, order_data):
        self._agv_number = order_data.kitting_task.agv_number
        self._tray_id = order_data.kitting_task.tray_id
        self._parts = order_data.kitting_task.parts
        self._destination = order_data.kitting_task.destination

    @property # Getter for agv_number
    def agv_number(self):
        return self._agv_number

    @property # Getter for tray_id
    def tray_id(self):
        return self._tray_id

    @property # Getter for parts
    def parts(self):
        return self._parts

    @property # Getter for destination
    def destination(self):
        return self._destination

class Assembly:
    """
    Class to represent the assembly task of an order.
    
    Attributes:
        _agv_numbers (list): List of AGV numbers
        _station (str): Station for assembly
        _parts (list): List of parts to be assembled
        
    Methods:
        __init__(self, order_data): Initialize the assembly task
    
    Properties:
        agv_numbers: Getter for agv_numbers
        station: Getter for station
        parts: Getter for parts
    """

    def __init__(self, order_data):
        self._agv_numbers = order_data.assembly_task.agv_numbers
        self._station = order_data.assembly_task.station
        self._parts = order_data.assembly_task.parts

    @property # Getter for agv_numbers
    def agv_numbers(self): 
        return self._agv_numbers

    @property # Getter for station
    def station(self):
        return self._station

    @property # Getter for parts
    def parts(self):
        return self._parts

class CombinedTask:
    """
    Class to represent the combined task of an order.

    Attributes:
        _station (str): Station for the combined task
        _parts (list): List of parts for the combined task
        
    Methods:
        __init__(self, order_data): Initialize the combined task
    
    Properties:
        station: Getter for station
        parts: Getter for parts
    """

    def __init__(self, order_data):
        self._station = order_data.combined_task.station
        self._parts = order_data.combined_task.parts

    @property # Getter for station
    def station(self):
        return self._station

    @property # Getter for parts
    def parts(self):
        return self._parts

class Order:
    """
    Class to represent an order.

    Attributes:
        _order_id (int): ID of the order
        _order_type (str): Type of the order
        _order_priority (bool): Priority of the order
        _order_task (Kitting/Assembly/CombinedTask): Task of the order
        waiting (bool): Flag to indicate if the order is in waiting period
        elapsed_wait (int): Elapsed wait time for the order
        wait_start_time (float): Start time of the wait period
        _visiting_first_time (bool): Flag to check if the order is being processed for the first time
        _order_completed_flag (bool): Flag to check if the order is completed
        _parts_status_tray (dict): Dictionary to store the parts status on tray
        _tray_pick_status (dict): Dictionary to store the tray pick status
        
    Methods:
        __init__(self, order_data): Initialize the order
    
    Properties:
        order_id: Getter for order_id
        order_type: Getter for order_type
        order_priority: Getter for order_priority
        order_task: Getter for order_task
    """

    def __init__(self, order_data):
        self._order_id = order_data.id
        self._order_type = order_data.type
        self._order_priority = order_data.priority

        self.waiting = (
            False  # Indicates if the order is currently in its waiting period
        )
        self.elapsed_wait = 0  # Track elapsed wait time for the order
        self.wait_start_time = None  # Track the start time of the wait period

        # Initialize the order task based on the order type
        if self._order_type == OrderMsg.KITTING:
            self._order_task = Kitting(order_data)
        elif self._order_type == OrderMsg.ASSEMBLY:
            self._order_task = Assembly(order_data)
        elif self._order_type == OrderMsg.COMBINED:
            self._order_task = CombinedTask(order_data)
        
        self._visiting_first_time = True # Flag to check if the order is being processed for the first time
        self._order_completed_flag = False # Flag to check if the order is completed
        self._parts_status_tray = {} # Dictionary to store the parts status on tray
        self._tray_pick_status = {} # Dictionary to store the tray pick status

class OrderManagement(Node):
    """
    Class to manage the orders, do agility challengers and service calls to move it actions.

    Inherited Class:
        Node (rclpy.node.Node): Node class
        
    Attributes:
        _agv_statuses (dict): Dictionary to store the AGV statuses
        _agv_velocities (dict): Dictionary to store the AGV velocities
        current_order (Order): Track the currently processing or waiting order
        competition_ended (bool): Flag to indicate if the competition has ended
        tables_done (dict): Dictionary to store the status of the tables
        bins_done (dict): Dictionary to store the status of the bins
        _Tray_Dictionary (dict): Dictionary to store the tray data
        _Bins_Dictionary (dict): Dictionary to store the bin data
        _Parts_Dictionary (dict): Dictionary to store the part data
        _Agvs_Dictionary (dict): Dictionary to store the AGV data
        _end_condition_thread (threading.Thread): Thread to check the end conditions
        _pick_part_from_bin (bool): Flag to indicate if the robot is picking a part from the bin
        _picked_part_from_bin (bool): Flag to indicate if the robot has picked the part from the bin
        _placing_part_on_tray (bool): Flag to indicate if the robot is placing the part on the tray
        _placed_part_on_tray (bool): Flag to indicate if the robot has placed the part on the tray
        _released_part_on_tray (bool): Flag to indicate if the robot has released the part on the tray
        _fault_gripper_flag (bool): Flag to indicate if there is a fault with the gripper
        _part_dropped_trash (bool): Flag to indicate if the part has been dropped in the trash
        _part_detached (bool): Flag to indicate if the part has been detached
        _kit_completed (bool): Flag to indicate if the kit has been completed
        _quality_check_completed (bool): Flag to indicate if the quality check has been completed
        _competition_started (bool): Flag to indicate if the competition has started
        _competition_state (CompetitionState): State of the competition
        _moved_robot_home (bool): Flag to indicate if the robot has moved to home
        _moved_robot_to_table (bool): Flag to indicate if the robot has moved to the table
        _entered_tool_changer (bool): Flag to indicate if the robot has entered the tool changer
        _changed_gripper (bool): Flag to indicate if the gripper has been changed
        _exited_tool_changer (bool): Flag to indicate if the robot has exited the tool changer
        _activated_gripper (bool): Flag to indicate if the gripper has been activated
        _deactivated_gripper (bool): Flag to indicate if the gripper has been deactivated
        _moved_robot_to_tray (bool): Flag to indicate if the robot has moved to the tray
        _moved_tray_to_agv (bool): Flag to indicate if the tray has been moved to the AGV
        _locked_agv (bool): Flag to indicate if the AGV has been locked
        _agv_moved_warehouse (bool): Flag to indicate if the AGV has moved to the warehouse
        _submitted_order (bool): Flag to indicate if the order has been submitted
        _competition_ended_flag (bool): Flag to indicate if the competition has ended
        _high_priority_orders (list): List of high priority orders
        _normal_orders (list): List of normal priority orders
        _paused_orders (list): List of paused orders    
        _start_process_order (bool): Flag to indicate if the order processing has started
        current_order_is (str): Flag to indicate the current order type
        _order_announcements_count (int): Count of order announcements
        _order_submitted_count (int): Count of order submissions
        _faults (list): List of faults

    Methods:
        __init__(self, node_name): Initialize the node
        _orders_initialization_cb(self, msg): Callback for receiving orders
        _order_priority_timer_cb(self): Function to process the orders based on priority
        _check_priority_flag(self): Check if the priority flag is set to True
        _competition_state_cb(self, msg): Callback for competition state changes
        _agv_status_cb(self, msg, agv_id): Callback for AGV status changes
        _table_camera_callback(self, message, table_id='Unknown'): Table Camera Callback
        _bin_camera_callback(self, message, side='Unknown'): Bin Camera Callback
        _agv_camera_callback(self, message, agv_id): AGV Camera Callback
        _multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose: Use KDL to multiply two poses together
        _find_unused_tray(self, tray_id): Find an unused tray in the dictionary
        _find_available_part(self, type, color): Find an available part in the bins
        _process_order(self, order): Process the order
        _check_end_conditions(self): Check the end conditions
        _order_processing_thread(self): Start the order processing thread
        _check_order_completion(self): Check if the order is completed
        _pick_part_from_bin_cb(self, response): Callback for picking a part from the bin
        _place_part_on_tray_cb(self, response): Callback for placing a part on the tray
        _release_part_on_tray_cb(self, response): Callback for releasing a part on the tray
        _move_robot_home_cb(self, response): Callback for moving the robot to home
        _move_robot_to_table_cb(self, response): Callback for moving the robot to the table
        _move_robot_to_tray_cb(self, response): Callback for moving the robot to the tray
        _move_tray_to_agv_cb(self, response): Callback for moving the tray to the AGV
        _enter_tool_changer_cb(self, response): Callback for entering the tool changer
        _exit_tool_changer_cb(self, response): Callback for exiting the tool changer
        _set_gripper_state_cb(self, response): Callback for setting the gripper state
        _change_gripper_cb(self, response): Callback for changing the gripper
        _drop_part_in_trash_cb(self, response): Callback for dropping a part in the trash
        _detach_part_planning_scene_cb(self, response): Callback for detaching a part from the planning scene
    """

    def __init__(self, node_name):
        """
        Initialize the node.

        Args:
            node_name (str): Name of the node
        """
        super().__init__(node_name)

        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        self.pkg_share = FindPackageShare("final_group1").find("final_group1")
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        
        # Create callback groups
        self.callback_groups = {
            '_order_callback_group': ReentrantCallbackGroup(),
            '_sensor_callback_group': ReentrantCallbackGroup(),
            '_competition_callback_group': ReentrantCallbackGroup(),
            '_agv_callback_group': ReentrantCallbackGroup(),
        }
        
        # Define the subscriptions
        subscriptions = {
            '_left_table_camera_subscription': ('/ariac/sensors/left_table_camera/image', BasicLogicalCameraImage, lambda msg: self._table_camera_callback(msg, 'Left'), qos_policy),
            '_right_table_camera_subscription': ('/ariac/sensors/right_table_camera/image', BasicLogicalCameraImage, lambda msg: self._table_camera_callback(msg, 'Right'), qos_policy),
            '_left_bins_camera_subscription': ('/ariac/sensors/left_bins_camera/image', BasicLogicalCameraImage, lambda msg: self._bin_camera_callback(msg, 'Left'), qos_policy),  
            '_right_bins_camera_subscription': ('/ariac/sensors/right_bins_camera/image', BasicLogicalCameraImage, lambda msg: self._bin_camera_callback(msg, 'Right'), qos_policy),
            '_right_table_rgb_subscription': ('/ariac/sensors/right_table_camera_rgb/rgb_image', Image, lambda msg: self._table_tray_callback(msg,'Right'), QoSProfile(depth=1)),
            '_left_table_rgb_subscription': ('/ariac/sensors/left_table_camera_rgb/rgb_image', Image, lambda msg: self._table_tray_callback(msg,'Left'), QoSProfile(depth=1)),
            '_left_bins_rgb_subscription': ('/ariac/sensors/left_bins_camera_rgb/rgb_image', Image, lambda msg: self._bin_part_callback(msg,'Left'), QoSProfile(depth=1)),
            '_right_bins_rgb_subscription': ('/ariac/sensors/right_bins_camera_rgb/rgb_image', Image, lambda msg: self._bin_part_callback(msg,'Right'), QoSProfile(depth=1)),
            '_robot_gripper_state_subscription': ('/ariac/floor_robot_gripper_state', VacuumGripperState, self._robot_gripper_state_subscription_cb, qos_policy),
        }  
        
        # Create the subscriptions
        for attr, (topic, msg_type, callback,qos) in subscriptions.items():
            setattr(self, attr, self.create_subscription(msg_type, topic, callback, qos_profile=qos, callback_group=self.callback_groups['_sensor_callback_group']))

        self._competition_state_subscription = self.create_subscription(CompetitionState,"/ariac/competition_state",self._competition_state_cb,qos_profile=qos_policy,callback_group=self.callback_groups['_competition_callback_group'])
        self._orders_subscription = self.create_subscription(OrderMsg,"/ariac/orders",self._orders_initialization_cb,qos_profile=qos_policy,callback_group=self.callback_groups['_order_callback_group'])
        
        self.get_logger().info(f"Node {node_name} initialized")
        
        self._agv_statuses = {}
        self._agv_velocities = {}
        self.current_order = None  # Track the currently processing or waiting order
        self.competition_ended = False
        self.tables_done = {'Left':True,'Right':True,'Left Rgb':True,'Right Rgb':True,'Lefts Rgb':True,'Rights Rgb':True} # To avoid reprocessing
        self.bins_done = {'Left':True,'Right':True,'Left Rgb':True,'Right Rgb':True,'Lefts Rgb':True,'Rights Rgb':True} # To avoid reprocessing
        self.trays={'Left':{},'Right':{}} # To store the tray ids
        self.parts={'Left':{},'Right':{}} # To store the part types and colors
        self._parts_order = {'left':[],'right':[]} # To store the parts in the order they are listed
        
        # Initialize YOLO model
        self._model = YOLO(f'{self.pkg_share}/dataset/yolo/best.pt')

        self._Tray_Dictionary = {}
        self._Bins_Dictionary = {}
        self._Parts_Dictionary={'colors':{0: 'red', 1: 'green', 2: 'blue', 3: 'orange', 4: 'purple'},
                      'types':{10:'battery', 11:'pump', 12:'sensor', 13:'regulator'}}
        self._Agvs_Dictionary = {}
        
        # Initialize the end condition thread
        self._end_condition_thread = None
        self.processing_lock = (
            threading.Condition()
        )  # Condition variable for synchronizing order processing

        ############## MoveIt Action Clients, Service Clients and Flags ################
        # Define the mappings
        self._tray_id_mapping = {i: getattr(MoveRobotToTray.Request, f"TRAY_ID{i}") for i in range(10)}
        self._agv_id_mapping = {i: getattr(MoveTrayToAGV.Request, f"AGV{i}") for i in range(1, 5)}
        self._quadrant_mapping = {i: getattr(PlacePartTray.Request, f"QUADRANT{i}") for i in range(1, 5)}
        self._part_color_mapping = {color: getattr(Part, color.upper()) for color in ["red", "green", "blue", "orange", "purple"]}
        self._part_type_mapping = {part_type: getattr(Part, part_type.upper()) for part_type in ["battery", "pump", "sensor", "regulator"]}

        self._robot_gripper_state = "part_gripper" # Initialize the robot gripper state

        # Define the service clients
        clients = {
            '_pick_part_bin_cli': (PickPartBin, "/commander/pick_part_bin"),
            '_place_part_tray_cli': (PlacePartTray, "/commander/place_part_tray"),
            '_release_part_on_tray_cli' : (ReleasePartOnTray, "/commander/release_part_on_tray"),
            '_move_robot_home_cli': (Trigger, "/commander/move_robot_home"),
            '_move_robot_to_table_cli': (MoveRobotToTable, "/commander/move_robot_to_table"),
            '_move_robot_to_tray_cli': (MoveRobotToTray, "/commander/move_robot_to_tray"),
            '_move_tray_to_agv_cli': (MoveTrayToAGV, "/commander/move_tray_to_agv"),
            '_enter_tool_changer_cli': (EnterToolChanger, "/commander/enter_tool_changer"),
            '_exit_tool_changer_cli': (ExitToolChanger, "/commander/exit_tool_changer"),
            '_set_gripper_state_cli': (VacuumGripperControl, "/ariac/floor_robot_enable_gripper"),
            '_change_gripper_cli': (ChangeGripper, "/ariac/floor_robot_change_gripper"),
            '_drop_part_in_trash_cli' : (Trigger, "/commander/drop_part_in_trash"),
            '_detach_part_planning_scene_cli' : (Trigger, "/commander/detach_part_planning_scene"),
            '_start_competition_cli': (Trigger, "/ariac/start_competition"),
        }

        # Create the service clients
        for attr, (srv_type, srv_name) in clients.items():
            setattr(self, attr, self.create_client(srv_type, srv_name))

        # Reading the yaml file
        self._get_yaml()
        
        # Initialize the flags for the order processing
        self._picking_part_from_bin = False
        self._picked_part_from_bin = False
        self._placing_part_on_tray = False
        self._placed_part_on_tray = False
        self._released_part_on_tray = False
        self._fault_gripper_flag = False
        self._part_dropped_trash = False
        self._part_detached = False
        # Flags to track the completion of actions
        self._kit_completed = False
        self._quality_check_completed = False
        self._competition_started = False
        self._competition_state = None
        self._moved_robot_home = False
        self._moved_robot_to_table = False
        self._entered_tool_changer = False
        self._changed_gripper = False
        self._exited_tool_changer = False
        self._activated_gripper = False
        self._deactivated_gripper = False
        self._moved_robot_to_tray = False
        self._moved_tray_to_agv = False
        self._locked_agv = False
        self._agv_moved_warehouse = False
        self._submitted_order = False
        self._competition_ended_flag = False
        self._yaml_read = False
        self._reprocessing_yolo = False

        self._high_priority_orders = []
        self._normal_orders =[]
        self._paused_orders = []
        self._start_process_order = False
        self.current_order_is = None
        self._order_announcements_count = 0
        self._order_submitted_count = 0
        self._faults = []
        
        # Timer for starting the competition
        self._competition_start_timer = self.create_timer(1, self._start_competition)

        # Start order processing thread
        self._order_processing_thread = threading.Thread(
                target=self._order_priority_timer_cb
            )
        self._order_processing_thread.start()

    def _start_competition(self):
        """
        Function to start the competition.
        """
        if not self._competition_started and self._yaml_read:
            if self.tables_done['Left Rgb'] and self.tables_done['Right Rgb'] and self.bins_done['Left Rgb'] and self.bins_done['Right Rgb']:
                self.get_logger().info("Starting the competition")
                time.sleep(1)
                future = self._start_competition_cli.call_async(Trigger.Request())
                future.add_done_callback(self._start_competition_cb)
            else:
                self.get_logger().info("Waiting for the sensors to be ready")
            
    def _start_competition_cb(self, future):
        """
        Callback for starting the competition.
        """
        try:
            self._competition_start_timer.cancel()
            self.get_logger().info("Competition started")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def _get_yaml(self):
        """
        Function to get the parameters from the yaml file.
        """
        self._get_yaml_cli = self.create_client(GetParameters, '/environment_startup_node/get_parameters')
        request = GetParameters.Request()
        request.names = ['trial_config_path']
        future = self._get_yaml_cli.call_async(request)
        future.add_done_callback(self._yaml_cb)

    def _yaml_cb(self, future):
        try:
            response = future.result()
            yaml_path = response.values[0].string_value
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                if 'parts' in data and 'bins' in data['parts']:
                    bins = data['parts']['bins']
                    for bin_id, parts in bins.items():
                        if bin_id in ['bin1', 'bin2', 'bin3', 'bin4']:
                            for part in parts:
                                for i in range(len(part['slots'])):
                                    part_entry = (part['color'], part['type'])
                                    self._parts_order['right'].append(part_entry)
                        else:
                            for part in parts:
                                for i in range(len(part['slots'])):
                                    part_entry = (part['color'], part['type'])
                                    self._parts_order['left'].append(part_entry)
            self.get_logger().info('Done reading yaml file!')
            self.tables_done = {x: False for x in self.tables_done}
            self.bins_done = {x: False for x in self.bins_done}
            self._yaml_read = True
        except Exception as e:
            self.get_logger().error('Failed to read yaml file!')
        

    def _orders_initialization_cb(self, msg):
        """
        Callback for receiving orders.

        Args:
            msg (any): Message received from the topic
        """
        order = Order(msg)
        self._order_announcements_count += 1
        self.get_logger().info(
            f"Received Order: {str(order._order_id)} with priority: {int(order._order_priority)}"
        )
        if(order._order_priority):
            self._high_priority_orders.append(order)
        else:
            self._normal_orders.append(order)
        
        # Create strings that list the order IDs from each list
        high_priority_order_ids = ', '.join([str(order._order_id) for order in self._high_priority_orders])
        normal_order_ids = ', '.join([str(order._order_id) for order in self._normal_orders])
        self.get_logger().info(f"High Priority Orders: {high_priority_order_ids}")
        self.get_logger().info(f"Normal Priority Orders: {normal_order_ids}")

        # Create a subscription to the AGV status topic
        agv_id = order._order_task.agv_number
        if agv_id not in self._agv_statuses:
            self.create_subscription(AGVStatus,f"/ariac/agv{agv_id}_status",lambda msg: self._agv_status_cb(msg, agv_id),QoSProfile(depth=10),callback_group=self.callback_groups['_agv_callback_group'])
    
    def _order_priority_timer_cb(self):
        """
        Function to process the orders based on priority.
        """
        yaml_read_temp = self._yaml_read
        while not yaml_read_temp:
            yaml_read_temp = self._yaml_read
            time.sleep(0.1)
        table_done_temp=self.tables_done.copy()
        bin_done_temp=self.bins_done.copy()
        while not all(table_done_temp.values()) or not all(bin_done_temp.values()):
            table_done_temp=self.tables_done.copy()
            bin_done_temp=self.bins_done.copy()
            time.sleep(0.1)
        self.get_logger().info(f"Starting the Process order thread!!!")
        if not self._competition_started:
            self._competition_started = True
            while True:
                h_len = len(self._high_priority_orders)
                n_len = len(self._normal_orders)
                if(h_len > 0):
                    self.current_order_is = "high"
                    ord_to_process = self._high_priority_orders[0]
                    self._process_order(ord_to_process)
                    self._high_priority_orders.pop(0) # Remove the order from the high priority list
                    self._order_submitted_count += 1
                elif (n_len > 0):
                    self.current_order_is = "normal"
                    ord_to_process = self._normal_orders[0]
                    self._process_order(ord_to_process)
                    if(ord_to_process._order_completed_flag):
                        self._normal_orders.pop(0) # Remove the order from the normal list
                        self._order_submitted_count += 1
                elif (self._order_submitted_count == self._order_announcements_count and self._competition_ended_flag):
                    self.get_logger().info("Ending the Process order thread!!!")
                    break

    def _check_priority_flag(self):
        """
        Check if the priority flag is set to True. If set, return True, else False.

        Args:
            None

        Returns:
            bool: True if the priority flag is set, else False
        """
        if(len(self._high_priority_orders) > 0 and self.current_order_is == "normal"):
            return True
        else:
            return False

    def _competition_state_cb(self, msg):
        """
        Callback for competition state changes. Starts the end condition checker when order announcements are done.

        Args:
            msg (competition_state): Message received from the competition state topic
        
        Returns:
            None
        """
        # Check if the competition has started
        if msg.competition_state == CompetitionState.STARTED:
            self._start_process_order = True
        if msg.competition_state == CompetitionState.ENDED:
            self._competition_ended_flag = True
        # Check if the competition has ended and set the flag
        if msg.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
            if (self._end_condition_thread is None or not self._end_condition_thread.is_alive()):
                self._end_condition_thread = threading.Thread(target=self._check_end_conditions)
                self._end_condition_thread.start()

    def _agv_status_cb(self, msg, agv_id):
        """
        Callback for AGV status changes. Updates the AGV status in the dictionary.

        Args:
            msg (AGVStatus): Message received from the AGV status topic
            agv_id (int): ID of the AGV

        Returns:
            None
        """
        # Define a mapping for AGV locations
        location_status_map = {0: "Kitting Station", 3: "WAREHOUSE"}
        status = location_status_map.get(msg.location, "OTHER")
        current_velocity = msg.velocity
        if agv_id not in self._agv_statuses :
            self._agv_statuses[agv_id] = status
        if status == "WAREHOUSE" and self._agv_statuses[agv_id] != status:
            self._agv_statuses[agv_id] = status
        if agv_id not in self._agv_velocities or self._agv_velocities[agv_id] != current_velocity:
            self._agv_velocities[agv_id] = current_velocity

    def _table_camera_callback(self, message, table_id='Unknown'):
        """
        Table Camera Callback is used to scan table cameras and store data in the dictionary.

        Args:
            message(AdvancedLogicalCameraImage): Image type from topic
            table_id: Arg passed when subscription callback created
        
        Returns:
            None
        """
        # Check if the table ID is unknown
        if table_id == 'Unknown':
            self.get_logger().warn("Unknown table ID")
            return
        
        # Check if the table is already scanned
        if self.tables_done[table_id] == False:
            self._Tray_Dictionary[table_id]={}
            tray_poses = message.tray_poses
            if len(tray_poses) > 0:
                self.tables_done[table_id] = True
                for i in range(len(tray_poses)):
                    # Get the tray ID, camera pose, and tray pose
                    tray_pose_id = self.trays[table_id][i]
                    camera_pose = Pose()
                    camera_pose.position.x = message.sensor_pose.position.x
                    camera_pose.position.y = message.sensor_pose.position.y
                    camera_pose.position.z = message.sensor_pose.position.z
                    camera_pose.orientation.x = message.sensor_pose.orientation.x
                    camera_pose.orientation.y = message.sensor_pose.orientation.y
                    camera_pose.orientation.z = message.sensor_pose.orientation.z
                    camera_pose.orientation.w = message.sensor_pose.orientation.w

                    tray_pose = Pose()
                    tray_pose.position.x = tray_poses[i].position.x
                    tray_pose.position.y = tray_poses[i].position.y
                    tray_pose.position.z = tray_poses[i].position.z
                    tray_pose.orientation.x = tray_poses[i].orientation.x
                    tray_pose.orientation.y = tray_poses[i].orientation.y
                    tray_pose.orientation.z = tray_poses[i].orientation.z
                    tray_pose.orientation.w = tray_poses[i].orientation.w
                    tray_world_pose = self._multiply_pose(camera_pose, tray_pose) # Get the world pose of the tray
                    
                    self._Tray_Dictionary[table_id][tray_pose_id]={'position': [tray_world_pose.position.x,tray_world_pose.position.y,tray_world_pose.position.z], 'orientation': [tray_world_pose.orientation.x,tray_world_pose.orientation.y,tray_world_pose.orientation.z,tray_world_pose.orientation.w], 'status':False}
                self.tables_done[table_id] = True
                
    def _table_tray_callback(self, message, side='Unknown'):
        """
        Callback for table camera images. Detects trays on the table and updates the tray dictionary.

        Args:
            message (Image): Image message from the camera
        """
        if side == 'Unknown':
            self.get_logger().warn("Unknown side ID")
            return
        
        if self.tables_done[side+'s Rgb'] == False:
            self.tables_done[side+'s Rgb'] = True
            self.get_logger().info(f"Processing trays for {side} side")

            # Placeholder for detected trays
            detected_trays = set()
            # detecting aruco markers
            bridge = CvBridge()

            try:
                cv_image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            except CvBridgeError as e:
                self.get_logger().error(e)
            image = cv_image
            image = image[210:245, 50:590]
            image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            
            # Edge detection on the image
            edges = cv.Canny(image, 50, 150)
            
            # Loop through template images of the aruco markers
            for i in range(10):
                template_image_path = f'{self.pkg_share}/dataset/aruco/id_{i}.png'  
                template_image = cv.imread(template_image_path, cv.IMREAD_GRAYSCALE)
                template_image = cv.resize(template_image, (35, 35))
                # Edge detection on the template image
                edges_template = cv.Canny(template_image, 50, 150)

                # Template matching
                res = cv.matchTemplate(edges, edges_template, cv.TM_CCOEFF_NORMED)
                threshold = 0.3
                loc = np.where(res >= threshold)
                for pt in zip(*loc[::-1]):
                    cv.rectangle(image, pt, (pt[0] + 35, pt[1] + 35), (0, 255, 0), 2)
                    detected_trays.add((i,pt))
            
            # Sorting ids based on location of the tray
            detected_trays = sorted(detected_trays, key=lambda x: x[1][0])  
            
            trays=[]
            for i in range(len(detected_trays)):
                if i == 0:
                    trays.append(detected_trays[i][0])
                else:
                    if detected_trays[i][1][0] - detected_trays[i-1][1][0] > 35:
                        trays.append(detected_trays[i][0])

            self.trays[side] = trays
            self.tables_done[side+' Rgb'] = True
         
    def _bin_camera_callback(self, message,side='Unknown'):
        """
        Bin Camera Callback is used to scan bin cameras and store data

        Args:
            message(AdvancedLogicalCameraImage): Image type from topic
            table_id: Arg passed when subscription callback created

        Returns:
            None
        """
        # Check if the side ID is unknown
        if side == 'Unknown':
            self.get_logger().warn("Unknown side ID")
            return
        
        # Check if the bins are already scanned
        if self.bins_done[side] == False:
            bin_poses = message.part_poses
            bin_camera_pose = Pose()
            bin_camera_pose.position.x = message.sensor_pose.position.x
            bin_camera_pose.position.y = message.sensor_pose.position.y
            bin_camera_pose.position.z = message.sensor_pose.position.z
            bin_camera_pose.orientation.x = message.sensor_pose.orientation.x
            bin_camera_pose.orientation.y = message.sensor_pose.orientation.y
            bin_camera_pose.orientation.z = message.sensor_pose.orientation.z
            bin_camera_pose.orientation.w = message.sensor_pose.orientation.w

            self._Bins_Dictionary[side]={}
            if len(bin_poses) > 0:
                for i in range(len(bin_poses)):
                    bin_part = self.parts[side][i]
                    bin_part_pose = Pose()
                    bin_part_pose.position.x = bin_poses[i].position.x
                    bin_part_pose.position.y = bin_poses[i].position.y
                    bin_part_pose.position.z = bin_poses[i].position.z
                    bin_part_pose.orientation.x = bin_poses[i].orientation.x
                    bin_part_pose.orientation.y = bin_poses[i].orientation.y
                    bin_part_pose.orientation.z = bin_poses[i].orientation.z
                    bin_part_pose.orientation.w = bin_poses[i].orientation.w

                    bin_world_pose = self._multiply_pose(bin_camera_pose, bin_part_pose)
                    type = bin_part[1]
                    color = bin_part[0]
                    if (type,color) in self._Bins_Dictionary[side].keys():
                        keys=self._Bins_Dictionary[side][(type,color)].keys()
                        self._Bins_Dictionary[side][(type,color)][len(keys)]={'position': [bin_world_pose.position.x,bin_world_pose.position.y,bin_world_pose.position.z],
                                                                            'orientation': [bin_world_pose.orientation.x,bin_world_pose.orientation.y,bin_world_pose.orientation.z,bin_world_pose.orientation.w],'picked': False}
                    
                    else:
                        self._Bins_Dictionary[side][(type,color)]={}
                        self._Bins_Dictionary[side][(type,color)][0]={'position': [bin_world_pose.position.x,bin_world_pose.position.y,bin_world_pose.position.z],
                                                                    'orientation': [bin_world_pose.orientation.x,bin_world_pose.orientation.y,bin_world_pose.orientation.z,bin_world_pose.orientation.w],'picked': False}
                self.bins_done[side] = True

    def _bin_part_callback(self, message, side='Unknown'):
        """ Callback for bin camera images. Detects parts in the bins and updates the parts dictionary.

        Args:
            message (Image): Image message from the camera
            side (str, optional): Side of the bin. Defaults to 'Unknown'.
        """

        if side == 'Unknown':
            self.get_logger().warn("Unknown side ID")
            return
        
        # Check if RGB processing is already done to avoid re-processing
        if self.bins_done[side+'s Rgb'] == False:
            order_of_parts = self._parts_order[side.lower()]
            self.get_logger().info(f"Order of parts for side {side}: {order_of_parts}")
            order_of_parts_counter = Counter(order_of_parts)
            
            self.bins_done[side+'s Rgb'] = True
            self.get_logger().info(f"Processing parts for {side} side")
            
            # Detecting Parts
            bridge = CvBridge()
            
            try:
                cv_image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            except CvBridgeError as e:
                self.get_logger().error(f"Bridge conversion error: {e}")
                return
            
            image = cv_image
            
            # Compute the median of the gradient magnitudes
            sobelx = cv.Sobel(image, cv.CV_64F, 1, 0, ksize=5)
            sobely = cv.Sobel(image, cv.CV_64F, 0, 1, ksize=5)
            gradient_magnitude = np.sqrt(sobelx**2 + sobely**2)
            median_val = np.median(gradient_magnitude)

            # Set lower and upper thresholds based on the median
            sigma = 0.33
            lower = int(max(0, (1.0 - sigma) * median_val))
            upper = int(min(255, (1.0 + sigma) * median_val))
            
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            
            # Edge detection on the image
            edges = cv.Canny(gray, lower, upper)
            edges = cv.cvtColor(edges, cv.COLOR_GRAY2RGB)
                
            # Detecting parts using YOLO
            res = self._model(edges)
            parts = []
            for r in res:
                boxes = r.boxes
                for box in boxes:
                    b=box.xyxy[0].to('cpu').detach().numpy().copy()
                    c=box.cls
                    class_name = self._model.names[int(c)].lower()
                    top = int(b[0])
                    left = int(b[1])
                    bottom = int(b[2])
                    right = int(b[3])
                    parts.append((class_name, top, left, bottom, right))
                        
            final_parts = []

            hsv_ranges = {
                'red': ([169, 100, 100], [189, 255, 255]),
                'green': ([57, 100, 100], [77, 255, 255]),
                'blue': ([105, 100, 100], [125, 255, 255]),
                'orange': ([3, 100, 100], [23, 255, 255]),
                'purple': ([128, 100, 100], [148, 255, 255])
            }
            
            # Detecting colors of the parts
            for part in parts:
                max_area = 0
                part_color = None
                hsv = cv.cvtColor(image[part[2]:part[4], part[1]:part[3]], cv.COLOR_BGR2HSV)
                for color, (lower, upper) in hsv_ranges.items():
                    lower = np.array(lower, dtype=np.uint8)
                    upper = np.array(upper, dtype=np.uint8)
                    mask = cv.inRange(hsv, lower, upper)
                    area = cv.countNonZero(mask)
                    if area > max_area:
                        part_color = color
                        max_area = area
                final_parts.append((part_color, part[0]))
            
            if Counter(final_parts) == order_of_parts_counter:
                self.get_logger().info(f"Parts in the {side} bins Found!")
                self.parts[side] = order_of_parts
                self.bins_done[side+' Rgb'] = True
            elif self._reprocessing_yolo:
                self.parts[side]=final_parts
            else:
                self.get_logger().info(f"Getting new image for {side} bins!")
                self.bins_done[side + 's Rgb'] = False
                self.bins_done[side+' Rgb'] = False

    def _multiply_pose(self, pose1: Pose, pose2: Pose) -> Pose:
        '''
        Use KDL to multiply two poses together.
        Args:
            pose1 (Pose): Pose of the first frame
            pose2 (Pose): Pose of the second frame
        Returns:
            Pose: Pose of the resulting frame
        '''

        orientation1 = pose1.orientation
        frame1 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation1.x, orientation1.y, orientation1.z, orientation1.w),
            PyKDL.Vector(pose1.position.x, pose1.position.y, pose1.position.z))

        orientation2 = pose2.orientation
        frame2 = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation2.x, orientation2.y, orientation2.z, orientation2.w),
            PyKDL.Vector(pose2.position.x, pose2.position.y, pose2.position.z))

        frame3 = frame1 * frame2

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame3.p.x()
        pose.position.y = frame3.p.y()
        pose.position.z = frame3.p.z()

        # Get Quaternion 
        q = frame3.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return pose

    def _find_unused_tray(self, tray_id):
        """
        Find an unused tray in the dictionary.
        
        Args:
            tray_id (int): ID of the tray
            
        Returns:
            tuple: Tuple containing the side, pose, and orientation of the tray
        """
        for key in self._Tray_Dictionary.keys():
            if tray_id in self._Tray_Dictionary[key] and not self._Tray_Dictionary[key][tray_id]['status']:
                side = key
                self._Tray_Dictionary[key][tray_id]['status'] = True
                return side.lower(), self._Tray_Dictionary[key][tray_id]['position'], self._Tray_Dictionary[key][tray_id]['orientation']

    def _find_available_part(self, type, color):
        """
        Find an available part in the bins.
        
        Args:
            type (str): Type of the part
            color (str): Color of the part
            
        Returns:
            tuple: Tuple containing the part, side, pose, and orientation
        """
        for side in ['Left', 'Right']:
            if (type, color) in self._Bins_Dictionary[side]:
                for k, part in enumerate(self._Bins_Dictionary[side][(type, color)].values()):
                    if not part['picked']:
                        self._Bins_Dictionary[side][(type, color)][k]['picked'] = True
                        return side.lower(), part['position'], part['orientation']
        return None, None, None
    
    def _process_order(self, order):
        """
        Process the order by locking the AGV tray, moving the AGV to the station, and submitting the order.

        Args:
            order (Order): Order object to process

        Returns:
            None
        """

        # Check if the order is visiting for the first time
        if(order._visiting_first_time):
            # Process the order
            order._visiting_first_time = False
            self.get_logger().info(f"Processing order: {order._order_id}.")
            
            self.get_logger().info("")
            self.get_logger().info("-"*50)
            stars=len(order._order_id) + 6
            self.get_logger().info("-" * ((50 - stars) // 2) + f"Order {order._order_id}" + "-" * ((50 - stars) // 2))
            self.get_logger().info("-"*50)
            
            # Get the tray pose and orientation
            tray_id = order._order_task.tray_id
            # Get the side, pose, and orientation of the tray
            side, tray_pose, tray_orientation = self._find_unused_tray(tray_id)
            
            self.get_logger().info("Kitting Tray:")
            self.get_logger().info(f" - ID: {tray_id}")
            self.get_logger().info(f" - Position (xyz): {tray_pose}")
            self.get_logger().info(f" - Orientation (rpy): {tray_orientation}")
            self.get_logger().info("Parts:")

            # Dictionary in Class to store the tray information along with whether they are placed on AGV or not
            order._tray_pick_status[tray_id] = {"status": False, "tray_pose": tray_pose, "tray_id": tray_id,
                                                "tray_orientation": tray_orientation, "tray_side": side}
            
            # Get the parts and their poses
            parts = order._order_task.parts
            for part in parts:
                type = self._Parts_Dictionary['types'][part.part.type]
                color = self._Parts_Dictionary['colors'][part.part.color]
                part_quadrant = part.quadrant
                bin_side, pose, orientation = self._find_available_part(type, color)
                
                # Check if the part is available in the bins and skip if not
                if bin_side is None:
                    self.get_logger().info(f"Part {color} {type} not available in bins. Skipping.")
                    continue
                
                # Dictionary in Class to store the parts information along with whether they are placed on tray or not
                order._parts_status_tray[(type, color, part_quadrant)] = {"part_status": False, "part_type": type, "part_color": color,
                                                            "pose": pose, "orientation": orientation,
                                                            "part_quadrant": part_quadrant, "bin": bin_side}
                
                self.get_logger().info(f"    - {color} {type}")
                self.get_logger().info(f"       - Position (xyz): {pose}")
                self.get_logger().info(f"       - Orientation (rpy): {orientation}")
            
            self.get_logger().info("-"*50)
            self.get_logger().info("-"*50)
            self.get_logger().info("-"*50)

        # Execute the move it tasks for the order
        self._execute_move_it_tasks(order)

    def _execute_move_it_tasks(self, order):
        """
        Execute the move it tasks for the order.
        
        Args:
            order (Order): Order object to process
        """
        tray_id = self._tray_id_mapping[order._order_task.tray_id] # Get the tray ID
        agv_id = self._agv_id_mapping[order._order_task.agv_number] # Get the AGV ID
        
        if self._check_priority_flag():
            return 
        
        ############################## 1. Move Robot to Home ##############################################
        
        self._move_robot_home()
        robot_moved_home_status_temp = self._moved_robot_home
        while not robot_moved_home_status_temp :
            robot_moved_home_status_temp = self._moved_robot_home
        self._moved_robot_home = False
        self.get_logger().info("Python Node : Robot Moved to Home")
        
        if self._check_priority_flag():
            self.get_logger().info("Order priority received after initial move robot home. So returning")
            return 
        
        ############################## 2. Pick Tray and Place on AGV ########################################
        
        if (order._tray_pick_status[tray_id]["status"] ==  False):
            if order._tray_pick_status[tray_id]["tray_side"] == "left":
                tray_side_current = "kts1"
            elif order._tray_pick_status[tray_id]["tray_side"] == "right":
                tray_side_current = "kts2"
            
            ############## 2.1. Move Robot to Table ##############
            if(tray_side_current == "kts1"):
                self._move_robot_to_table(MoveRobotToTable.Request.KTS1)
            elif (tray_side_current == "kts2"):
                self._move_robot_to_table(MoveRobotToTable.Request.KTS2)

            robot_moved_table_status_temp = self._moved_robot_to_table
            while not robot_moved_table_status_temp :
                robot_moved_table_status_temp = self._moved_robot_to_table
            self._moved_robot_to_table = False
            self.get_logger().info("Python Node : Robot Moved to Table")
            
            if self._check_priority_flag():
                self.get_logger().info("Order priority received robot moved to table for tray pickup. So returning")
                return 
            
            ############## 2.2. Performing Gripper Actions ##############
            if(self._robot_gripper_state != "tray_gripper"):
                
                ######### 2.2.1. Enter Tool Changer #########
                self._enter_tool_changer(tray_side_current, "trays")
                robot_entered_tool_changer = self._entered_tool_changer
                while not robot_entered_tool_changer :
                    robot_entered_tool_changer = self._entered_tool_changer
                self._entered_tool_changer = False

                ######### 2.2.2. Change Gripper #########
                self._change_gripper(ChangeGripper.Request.TRAY_GRIPPER)
                robot_changed_gripper = self._changed_gripper
                while not robot_changed_gripper :
                    robot_changed_gripper = self._changed_gripper
                self._changed_gripper = False

                ######### 2.2.3. Exit Tool Changer #########
                self._exit_tool_changer(tray_side_current, "trays")
                robot_exited_gripper = self._exited_tool_changer
                while not robot_exited_gripper :
                    robot_exited_gripper = self._exited_tool_changer
                self._exited_tool_changer = False

            ######### 2.2.4. Activate Gripper #########
            self._activate_gripper()
            robot_gripper_activated = self._activated_gripper
            while not robot_gripper_activated :
                robot_gripper_activated = self._activated_gripper
            self._activated_gripper = False
            
            ############## Finished Gripper Actions ##############
            
            ######### 2.3. Move Robot to Tray #########
            tray_pose_curr = order._tray_pick_status[tray_id]["tray_pose"]
            tray_orientation_curr = order._tray_pick_status[tray_id]["tray_orientation"]
            
            # Set the tray pose
            tray_pose = Pose()
            tray_pose.position.x = tray_pose_curr[0]
            tray_pose.position.y = tray_pose_curr[1]
            tray_pose.position.z = tray_pose_curr[2]
            tray_pose.orientation.x = tray_orientation_curr[0]
            tray_pose.orientation.y = tray_orientation_curr[1]
            tray_pose.orientation.z = tray_orientation_curr[2]
            tray_pose.orientation.w = tray_orientation_curr[3]
            
            self._move_robot_to_tray(tray_id, tray_pose)
            robot_moved_tray_status_temp = self._moved_robot_to_tray
            while not robot_moved_tray_status_temp :
                robot_moved_tray_status_temp = self._moved_robot_to_tray
            self._moved_robot_to_tray = False
            self.get_logger().info("Python Node : Robot Moved to Tray")

            ############## 2.4. Place Tray on AGV ##############
            self._move_tray_to_agv(agv_id)
            robot_moved_tray_to_agv = self._moved_tray_to_agv
            while not robot_moved_tray_to_agv :
                robot_moved_tray_to_agv = self._moved_tray_to_agv
            self._moved_tray_to_agv = False

            ############## 2.5. Deactivate Gripper ##############
            self._deactivate_gripper()
            robot_deactivate_gripper = self._deactivated_gripper
            while not robot_deactivate_gripper :
                robot_deactivate_gripper = self._deactivated_gripper
            self._deactivated_gripper = False
            
            ############## 2.6. Update Tray Pick Status ##############
            order._tray_pick_status[tray_id]["status"] =  True
            
            ############## 2.7. Lock Tray ##############
            self._lock_tray(agv_id)
            agv_lock_status_temp = self._locked_agv
            while not agv_lock_status_temp :
                agv_lock_status_temp = self._locked_agv
            self._locked_agv = False

            # Check if the priority flag is set
            if self._check_priority_flag():
                self.get_logger().info("Order priority tray is placed on agv. So returning")
                return 

        if self._check_priority_flag():
            self.get_logger().info("Order priority received after tray placed on agv and moved home. So returning")
            return 
        
        ############################## 3. Pick Part from Bin and Place on Tray ########################################

        self._pick_place_parts_on_tray(order, agv_id)

        ############################## 4. Check Order Completion ########################################
        
        order_completed_flag = True
        for k,v in order._parts_status_tray.items():
            if(v["part_status"] ==  False):
                order_completed_flag = False
                break

        ############################## 5. Process Order ##################################################
        if(order_completed_flag):
            order._order_completed_flag = True
            
            ############## 5.1. Move AGV to Shipping Station ##############
            self._move_agv(agv_id, order._order_task.destination)
            move_agv_status_temp = self._agv_moved_warehouse
            while not move_agv_status_temp :
                move_agv_status_temp = self._agv_moved_warehouse
            self._agv_moved_warehouse = False
            
            ############## 5.2. Move Robot to Home ##############
            self._move_robot_home()
            robot_moved_home_status_temp = self._moved_robot_home
            while not robot_moved_home_status_temp :
                robot_moved_home_status_temp = self._moved_robot_home
            self._moved_robot_home = False
            self.get_logger().info("Python Node : Robot Moved to Home")
            
            ############## 5.3. Submit Order ##############
            self._submit_order(agv_id, order._order_id)
            submit_order_temp_status = self._submitted_order
            while not submit_order_temp_status :
                submit_order_temp_status = self._submitted_order
            self._submitted_order = False
            
            self.get_logger().info(f"Order {order._order_id} processed and shipped.")

        return 

    def _quality_check(self, order_id):
        """
        Perform a quality check on the part.
        
        Args:
            part (Part): Part object to check
        """
        self.get_logger().info(f"Performing quality check on Order: {order_id}")
        # Perform quality check here
        self._quality_check_client = self.create_client(PerformQualityCheck, "/ariac/perform_quality_check")
        request = PerformQualityCheck.Request()
        request.order_id = order_id
        future = self._quality_check_client.call_async(request)
        future.add_done_callback(lambda future: self._quality_check_cb(future))        

    def _quality_check_cb(self, future):
        """
        Callback for the quality check service. 
        
        Args:
            future: Future object
            
        Returns:
            None
        """
        quadrants = []
        response = future.result()
        if response:
            self.valid_id = response.valid_id
            self.all_passed = response.all_passed
            self.incorrect_tray = response.incorrect_tray
            quadrants.append(response.quadrant1)
            quadrants.append(response.quadrant2)
            quadrants.append(response.quadrant3)
            quadrants.append(response.quadrant4)
            for i in range(4):
                if not quadrants[i].all_passed and quadrants[i].faulty_part:
                    self._faults.append(i+1)
            self.get_logger().info(f"Faulty Quadrants: {self._faults}") if len(self._faults) > 0 else self.get_logger().info("No Faulty Quadrants")
            self._quality_check_completed = True
        else:
            self.get_logger().warn("Quality check failed. No response received.")
        
    def _pick_place_parts_on_tray(self, order,agv_id):
        """
        Pick and place the part on the tray.
        
        It picks the part from the bin, places it on the tray, and performs quality checks.
        1. Pick the part from the bin
        2. Place the part on the tray
        3. Perform the quality check
        4. Drop the part in the trash if faulty
        5. Pick a new part from the bin
        
        Args:
            order (Order): Order object to process
            agv_id (int): ID of the AGV to place the part on
        
        Returns:
            None
        """

        while True:
            for v in order._parts_status_tray.values():
                if(v["part_status"] == False):
                    part_details = Part()
                    part_details.color = self._part_color_mapping[v["part_color"]]
                    part_details.type = self._part_type_mapping[v["part_type"]]
                    part_pose_curr = v["pose"]
                    part_pose_curr_orient = v["orientation"]
                    part_bin_side = v["bin"]
                    part_quadrant = self._quadrant_mapping[v["part_quadrant"]]
                    
                    part_pose = Pose()
                    part_pose.position.x = part_pose_curr[0]
                    part_pose.position.y = part_pose_curr[1]
                    part_pose.position.z = part_pose_curr[2]
                    part_pose.orientation.x = part_pose_curr_orient[0]
                    part_pose.orientation.y = part_pose_curr_orient[1]
                    part_pose.orientation.z = part_pose_curr_orient[2]
                    part_pose.orientation.w = part_pose_curr_orient[3]

                    ############## 3.1. Pick Part from Bin ##############
                    
                    time.sleep(1)
                    self._robot_pick_part_from_bin(part_details, part_pose, part_bin_side)
                    robot_picked_part_bin_temp = self._picked_part_from_bin
                    while not robot_picked_part_bin_temp :
                        robot_picked_part_bin_temp = self._picked_part_from_bin
                    self._picked_part_from_bin = False

                    ############## 3.2. Place Part on Tray ##############
                    
                    time.sleep(3)
                    a = 1 # Flag to check if the part is placed on tray
                    self._robot_place_part_on_tray(agv_id,part_quadrant)
                    robot_placed_part_tray_temp = self._placed_part_on_tray
                    
                    ############## 3.2.1. Faulty Gripper Challenge ##############
                    fault_gripper_flag_temp = self._fault_gripper_flag
                    while not robot_placed_part_tray_temp or not fault_gripper_flag_temp :
                        if (fault_gripper_flag_temp):
                            self.get_logger().info("Faulty Gripper Challenge Initiated")
                            a = 0 # Reset the flag to check if the part is placed on tray
                            break
                        if robot_placed_part_tray_temp:
                            self.get_logger().info("Part placing...")
                            break
                        fault_gripper_flag_temp = self._fault_gripper_flag
                        robot_placed_part_tray_temp = self._placed_part_on_tray
                    self._placed_part_on_tray = False
                    self._fault_gripper_flag =  False
                    
                    if a == 0:
                        self._parts_order[part_bin_side].remove((v["part_color"], v["part_type"]))
                        self._faulty_gripper_challenge()
                        bin_side, pose, orientation = self._find_available_part(v["part_type"], v["part_color"])
                        if bin_side is None:
                            self.get_logger().info(f"Part {v['part_color']} {v['part_type']} not available in bins. Skipping.")
                            v["part_status"] = True
                            continue
                        v["pose"] = pose
                        v["orientation"] = orientation
                        v["bin"] = bin_side
                        a = 1
                        continue
                    
                    ################### 3.2.3. Quality Check ###################
                    self._quality_check(order._order_id)
                    
                    quality_check_temp_flag = self._quality_check_completed
                    while not quality_check_temp_flag:
                        quality_check_temp_flag = self._quality_check_completed
                    self._quality_check_completed = False
                    
                    if part_quadrant in self._faults:
                        self.get_logger().info("Removing Faulty Part")
                        part_dropped_trash_temp = self._part_dropped_trash
                        
                        ############## 3.2.3.1. Drop Part in Trash ##############
                        self._drop_part_in_trash()
                        while not part_dropped_trash_temp:
                            part_dropped_trash_temp = self._part_dropped_trash
                        self._part_dropped_trash = False
                        
                        ############## 3.2.3.2. Pick New Part from Bin ##############
                        bin_side, pose, orientation = self._find_available_part(v["part_type"], v["part_color"])
                        if bin_side is None:
                            self.get_logger().info(f"Part {v['part_color']} {v['part_type']} not available in bins. Skipping.")
                            self._parts_order[part_bin_side].remove((v["part_color"], v["part_type"]))
                            v["part_status"] = True
                            continue
                        v["pose"] = pose
                        v["orientation"] = orientation
                        v["bin"] = bin_side

                    else:
                        self.get_logger().info(f"Part Placed on Tray")
                        self._parts_order[part_bin_side].remove((v["part_color"], v["part_type"]))
                        self._release_part_on_tray(agv_id, part_quadrant)
                        v["part_status"] = True     # Mark the part as placed on tray
                    self._faults = []
                    
                    if self._check_priority_flag():
                        self.get_logger().info("Order priority received after a part is placed. So returning")
                        return 

            if all(v["part_status"] for v in order._parts_status_tray.values()):
                break
            
    def _faulty_gripper_challenge(self):
        """
        Perform the faulty gripper challenge.
        
        Args:
            None
        
        Returns:
            None
        """
        part_detached_temp = self._part_detached
        self._detach_part()
        while not part_detached_temp:
            part_detached_temp = self._part_detached
        self._part_detached = True
        self.bins_done = {x: False for x in self.bins_done.keys()} 
        self._reprocessing_yolo = True
        while not (self.bins_done.values()):
            time.sleep(0.1)
            
    def _lock_tray(self, agv):
        """Function to lock the tray

        Args:
            agv (str): Name of the agv
        """
        # Lock the tray to AGV
        self.get_logger().info(f"Lock Tray service called")
        self._lock_trays_client = self.create_client(
            Trigger, f"/ariac/agv{agv}_lock_tray")
        request = Trigger.Request()
        future = self._lock_trays_client.call_async(request)
        future.add_done_callback(lambda future: self._lock_tray_cb(future, agv))

    def _lock_tray_cb(self,future,agv):
        """
        Callback function of lock tray where the async response from server is handled

        Args:
            future: The Service Client Request
            agv (str): Name of the agv

        Returns:
            None
        """

        if future.result() is not None:
            response = future.result()
            if response:
                self._locked_agv = True
                self.get_logger().info(f"AGV {agv} locked")
        else:
            self.get_logger().warn(f"Unable to lock AGV {agv}")

    def _move_agv(self, agv, destination):
        """Function to move the agv to the shipping station

        Args:
            agv (str): Name of the agv
            destination (str): Destination of the agv
        
        Returns:
            None
        """
        # Move the AGV to the destination
        self.get_logger().info(f"Move AGV service called")
        self._move_agv_client = self.create_client(
            MoveAGV, f"/ariac/move_agv{agv}")
        request = MoveAGV.Request()
        request.location = destination
        future = self._move_agv_client.call_async(request)
        future.add_done_callback(lambda future: self._move_agv_cb(future, agv,destination))

    def _move_agv_cb(self,future, agv,destination):
        """
        Callback function of Move AGV where the async response from server is handled

        Args:
            future: The Service Client Request
            agv (str): Name of the agv
            destination(int): The destination of order to be processed
        
        Returns:

        """
        # rclpy.spin_until_future_complete(self, future)
        destination = "Warehouse" if destination == 3 else destination
        if future.result() is not None:
            response = future.result()
            if response:
                self._agv_moved_warehouse = True
                self.get_logger().info(f"AGV: {agv} moved to {destination}")
                # self._agv_statuses[agv] = 'WAREHOUSE'
        else:
            self.get_logger().warn(f"Service call failed {future.exception()}")
            self.get_logger().warn(f"Failed to move AGV: {agv} to {destination}")

    def _submit_order(self, agv_id, order_id):
        """
        Submit the order to the competition.

        Args:
            agv_id (int): ID of the AGV
            order_id (str): ID of the order
        """
        self.get_logger().info("Submit Order service called")

        current_status = self._agv_statuses.get(agv_id)
        current_agv_velocity = self._agv_velocities.get(agv_id)

        # Wait until the AGV is in warehouse and has stopped
        while (current_status != "WAREHOUSE" or current_agv_velocity > 0.0):
            self.get_logger().info(f"In status {current_status},{current_agv_velocity}")
            current_status = self._agv_statuses.get(agv_id)
            current_agv_velocity = self._agv_velocities.get(agv_id)

        self._submit_order_client = self.create_client(
            SubmitOrder, "/ariac/submit_order")
        request = SubmitOrder.Request()
        request.order_id = order_id
        future = self._submit_order_client.call_async(request)
        future.add_done_callback(lambda future: self._submit_order_cb(future))

    def _submit_order_cb(self, future):
        """
        Callback function of Submit AGV where the async response from server is handled

        Args:
            future: The Service Client Request
        
        Returns:
            None
        """
        if future.result() is not None:
            response = future.result()
            if response:
                self._submitted_order = True
                self.get_logger().info("Order submitted")
        else:
            self.get_logger().warn("Unable to submit order")

    def _check_end_conditions(self):
        """
        Periodically check if all orders are processed and AGVs are in warehouse, then end competition.

        Args:
            None
        
        Returns:
            None
        """
        while not self.competition_ended and rclpy.ok():
            # Check if all orders are processed and AGVs are in warehouse
            if (all(status == "WAREHOUSE" for status in self._agv_statuses.values()) and self._order_submitted_count == self._order_announcements_count):
                self.get_logger().info("All orders processed and AGVs at destination. Preparing to end competition.")
                self._end_competition()
                self._order_processing_thread.join()
            time.sleep(5)  # Check every 5 seconds

    def _end_competition(self):
        """
        End the competition if all conditions are met.

        Args:
            None
        
        Returns:
            None
        """
        if not self.competition_ended:
            self.competition_ended = True
            self.get_logger().info("End competition service called")
            
            self._end_competition_client = self.create_client(Trigger, "/ariac/end_competition", callback_group=self.callback_groups['_competition_callback_group'])
            request = Trigger.Request()
            future = self._end_competition_client.call_async(request)
            future.add_done_callback(lambda future: self._end_competition_cb(future))

    def _end_competition_cb(self, future):
        """
        Callback function of End Competition where the async response from server is handled

        Args:
            future: The Service Client Request
        
        Returns:
            None
        """
        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info("Competition ended")
        else:
            self.get_logger().warn("Unable to end competition")

    #####################################################################
    #################### MoveIt Task Functions ##########################
    #####################################################################

    def _move_robot_home(self, end_demo=False):
        """
        Move the floor robot to its home position

        Args:
            end_demo (bool): If True, end the demo
        
        Returns:
            None
        """

        self.get_logger().info("👉 Moving robot home...")
        if end_demo:
            self._ending_demo = True
        else:
            self._moving_robot_home = True

        while not self._move_robot_home_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Move Robot Service not available, waiting...")

        request = Trigger.Request()
        future = self._move_robot_home_cli.call_async(request)
        future.add_done_callback(self._move_robot_home_done_cb)

    def _move_robot_home_done_cb(self, future):
        """
        Client callback for the service /competitor/floor_robot/go_home. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_robot_home = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _move_robot_to_table(self, table_id):
        """
        Move the floor robot to a table

        Args:
            table_id (int): 1 for kts1 and 2 for kts2

        Returns:
            None
        """

        self.get_logger().info("👉 Moving robot to changing station...")
        self._moving_robot_to_table = True
        while not self._move_robot_to_table_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = MoveRobotToTable.Request()
        request.kts = table_id
        future = self._move_robot_to_table_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_table_done_cb)

    def _move_robot_to_table_done_cb(self, future):
        """
        Client callback for the service /commander/move_robot_to_table. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object

        Returns:
            None
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_robot_to_table = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _enter_tool_changer(self, station, gripper_type):
        """
        Move the end effector inside a tool changer

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'
        """
        self.get_logger().info("👉 Entering tool changer...")
        self._entering_tool_changer = True
        while not self._enter_tool_changer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = EnterToolChanger.Request()
        request.changing_station = station
        request.gripper_type = gripper_type
        future = self._enter_tool_changer_cli.call_async(request)
        future.add_done_callback(self._enter_tool_changer_done_cb)

    def _enter_tool_changer_done_cb(self, future):
        """
        Client callback for the service /commander/enter_tool_changer. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
        
        Returns:
            None
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._entered_tool_changer = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _change_gripper(self, gripper_type):
        """
        Change the gripper

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'

        Returns:
            None
        """
        self.get_logger().info("👉 Changing gripper...")
        self._changing_gripper = True
        while not self._change_gripper_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = ChangeGripper.Request()
        request.gripper_type = gripper_type
        future = self._change_gripper_cli.call_async(request)
        future.add_done_callback(self._change_gripper_done_cb)

    def _change_gripper_done_cb(self, future):
        """
        Client callback for the service /ariac/floor_robot_change_gripper. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object

        Returns:
            None
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info("✅ Gripper changed")
            self._changed_gripper = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _exit_tool_changer(self, station, gripper_type):
        """
        Move the end effector outside a tool changer

        Args:
            station (str): 'kts1' or 'kts2'
            gripper_type (str): 'parts' or 'trays'

        Returns:
            None
        """
        self.get_logger().info("👉 Exiting tool changer...")
        self._exiting_tool_changer = True
        while not self._exit_tool_changer_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = ExitToolChanger.Request()
        request.changing_station = station
        request.gripper_type = gripper_type
        future = self._exit_tool_changer_cli.call_async(request)
        future.add_done_callback(self._exit_tool_changer_done_cb)

    def _exit_tool_changer_done_cb(self, future):
        """
        Client callback for the service /commander/exit_tool_changer. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
            
        Returns:
            None
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._exited_tool_changer = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _activate_gripper(self):
        """
        Activate the gripper. 

        Args:
            None
        
        Returns:
            None
        """
        self.get_logger().info("👉 Activating gripper...")
        self._activating_gripper = True
        
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = VacuumGripperControl.Request()
        request.enable = True
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._activate_gripper_done_cb)

    def _activate_gripper_done_cb(self, future):
        """
        Client callback for the service /ariac/floor_robot_enable_gripper. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
        """
        if future.result().success:
            self.get_logger().info("✅ Gripper activated")
            self._activated_gripper = True  
        else:
            self.get_logger().fatal("💀 Gripper not activated")

    def _deactivate_gripper(self):
        """
        Deactivate the gripper
        
        Args:
            None
        
        Returns:
            None
        """
        self.get_logger().info("👉 Deactivating gripper...")
        self._deactivating_gripper = True
        while not self._set_gripper_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = VacuumGripperControl.Request()
        request.enable = False
        future = self._set_gripper_state_cli.call_async(request)
        future.add_done_callback(self._deactivate_gripper_done_cb)

    def _deactivate_gripper_done_cb(self, future):
        """
        Client callback for the service /ariac/floor_robot_enable_gripper. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
        """
        if future.result().success:
            self.get_logger().info("✅ Gripper deactivated")
            self._deactivated_gripper = True
        else:
            self.get_logger().fatal("💀 Gripper not deactivated")

    def _move_robot_to_tray(self, tray_id, tray_pose):
        """
        Move the floor robot to a tray to pick it up.
        
        Args:
            tray_id (int): ID of the tray
            tray_pose (Pose): Pose of the tray in world frame
        
        Returns:
            None
        """
        self.get_logger().info("👉 Moving robot to tray...")
        self._moving_robot_to_tray = True
        
        while not self._move_robot_to_tray_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service not available, waiting...")

        request = MoveRobotToTray.Request()
        request.tray_id = tray_id
        request.tray_pose_in_world = tray_pose
        future = self._move_robot_to_tray_cli.call_async(request)
        future.add_done_callback(self._move_robot_to_tray_done_cb)

    def _move_robot_to_tray_done_cb(self, future):
        """
        Client callback for the service /commander/move_robot_to_tray. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_robot_to_tray = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _move_tray_to_agv(self, agv_number):
        """
        Move the tray to the AGV.
        
        Args:
            agv_number (int): Number of the AGV
        
        Returns:
            None
        """
        self.get_logger().info("👉 Moving tray to AGV...")
        self._moving_tray_to_agv = True

        while not self._move_tray_to_agv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = MoveTrayToAGV.Request()
        request.agv_number = agv_number
        future = self._move_tray_to_agv_cli.call_async(request)
        future.add_done_callback(self._move_tray_to_agv_done_cb)

    def _move_tray_to_agv_done_cb(self, future):
        """
        Client callback for the service /commander/move_tray_to_agv. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
        
        Returns:
            None
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_tray_to_agv = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _robot_gripper_state_subscription_cb(self, message):
        """
        Callback function for the subscription to /ariac/gripper/state topic.
        
        Args:
            message (GripperState): Message received from the topic
        
        Returns:
            None
        """
        current_gripper_state = message.type
        if (current_gripper_state != self._robot_gripper_state):
            self._robot_gripper_state = current_gripper_state
            
        if (self._robot_gripper_state == "part_gripper"):
            if message.enabled:
                if not message.attached:
                    self._fault_gripper_flag = True
                else:
                    self._fault_gripper_flag = False
            
    def _robot_pick_part_from_bin(self, part,pose,bin):
        """
        Make the robot pick part from bin

        Args:
            part: the part to be picked
            pose: the pose of the part
            bin: the bin from which part is to be picked
        """

        self.get_logger().info("👉 Picking Part from bin")
        self._picking_part_from_bin = True
        while not self._pick_part_bin_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = PickPartBin.Request()
        request.part = part
        request.pose = pose
        request.bin_side = bin
        future = self._pick_part_bin_cli.call_async(request)
        future.add_done_callback(self._pick_part_bin_cli_cb)

    def _pick_part_bin_cli_cb(self, future):
        """
        Client callback for the service /commander/pick_part_bin

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._picked_part_from_bin = True
        else:
            self.get_logger().fatal(f"💀 {message}")
    
    def _robot_place_part_on_tray(self, agv_number,quadrant):
        """
        Make the robot pick part from bin

        Args:
            agv_number: the agv on which part is placed
            quadrant: the quadrant on tray where part is placed

        Returns:
            None
        """

        self.get_logger().info("👉 Placing Part on Tray")
        self._placing_part_on_tray = True
        while not self._place_part_tray_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = PlacePartTray.Request()
        request.agv_number = agv_number
        request.quadrant = quadrant
        future = self._place_part_tray_cli.call_async(request)
        future.add_done_callback(self._place_part_tray_cli_cb)

    def _place_part_tray_cli_cb(self, future):
        """
        Client callback for the service /commander/pick_part_tray. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
        
        Returns:
            None
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._placed_part_on_tray = True
        else:
            self._fault_gripper_flag = True
            self.get_logger().fatal(f"💀 {message}")

    def _release_part_on_tray(self, agv_number, quadrant):
        """
        Release part on Tray.

        Args:
            agv_number: the agv on which part is placed
            quadrant: the quadrant on tray where part is placed

        Returns:
            None
        """

        self.get_logger().info("👉 Releasing Part on Tray")
        self._releasing_part_on_tray = True
        while not self._release_part_on_tray_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = ReleasePartOnTray.Request()
        request.agv_number = agv_number
        request.quadrant = quadrant
        future = self._release_part_on_tray_cli.call_async(request)
        future.add_done_callback(self._release_part_on_tray_cli_cb)

    def _release_part_on_tray_cli_cb(self, future):
        """
        Client callback for the service /commander/release_part_on_tray. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object

        Returns:
            None
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._released_part_on_tray = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _drop_part_in_trash(self):
        """
        Drop the Part in trash.

        Args:
            None
        
        Returns:
            None
        """

        self.get_logger().info("👉 Dropping part in Trash...")
        while not self._drop_part_in_trash_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = Trigger.Request()
        future = self._drop_part_in_trash_cli.call_async(request)
        future.add_done_callback(self._drop_part_in_trash_cb)

    def _drop_part_in_trash_cb(self, future):
        """
        Client callback for the service /commander/drop_part_in_trash. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._part_dropped_trash = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _detach_part(self):
        """
        Detach the part from planning scene.

        Args:
            None
        
        Returns:
            None
        """
        self.get_logger().info("👉 Detaching part from planning scene...")
        while not self._detach_part_planning_scene_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = Trigger.Request()
        future = self._detach_part_planning_scene_cli.call_async(request)
        future.add_done_callback(self._detach_part_cb)

    def _detach_part_cb(self, future):
        """
        Client callback for the service /commander/drop_part_in_trash. Prints the result of the service call and sets the flag.

        Args:
            future (Future): A future object

        Returns:
            None
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._part_detached = True
        else:
            self.get_logger().fatal(f"💀 {message}")