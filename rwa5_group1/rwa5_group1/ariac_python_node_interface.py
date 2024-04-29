#!/usr/bin/env python3
"""
File: order_management_interface.py
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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ariac_msgs.msg import Order as OrderMsg, AGVStatus, CompetitionState, BasicLogicalCameraImage,AdvancedLogicalCameraImage,  VacuumGripperState, Part
from ariac_msgs.srv import MoveAGV, SubmitOrder,ChangeGripper, VacuumGripperControl
from std_srvs.srv import Trigger
from queue import PriorityQueue
from geometry_msgs.msg import Pose,Quaternion, TransformStamped
from sensor_msgs.msg import Image
from launch_ros.substitutions import FindPackageShare
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import cv2 as cv
import time
import threading
import PyKDL

# Import custom ROS services
from robot_commander_msgs.srv import (
    EnterToolChanger,
    ExitToolChanger,
    MoveRobotToTable,
    MoveRobotToTray,
    MoveTrayToAGV,
    PickPartBin,
    PlacePartTray
)



class Kitting:
    """
    Class to represent the kitting task of an order.
    """

    def __init__(self, order_data):
        self._agv_number = order_data.kitting_task.agv_number
        self._tray_id = order_data.kitting_task.tray_id
        self._parts = order_data.kitting_task.parts
        self._destination = order_data.kitting_task.destination

    @property
    def agv_number(self):
        return self._agv_number

    @property
    def tray_id(self):
        return self._tray_id

    @property
    def parts(self):
        return self._parts

    @property
    def destination(self):
        return self._destination


class Assembly:
    """
    Class to represent the assembly task of an order.
    """

    def __init__(self, order_data):
        self._agv_numbers = order_data.assembly_task.agv_numbers
        self._station = order_data.assembly_task.station
        self._parts = order_data.assembly_task.parts

    @property
    def agv_numbers(self):
        return self._agv_numbers

    @property
    def station(self):
        return self._station

    @property
    def parts(self):
        return self._parts


class CombinedTask:
    """
    Class to represent the combined task of an order.
    """

    def __init__(self, order_data):
        self._station = order_data.combined_task.station
        self._parts = order_data.combined_task.parts

    @property
    def station(self):
        return self._station

    @property
    def parts(self):
        return self._parts


class Order:
    """
    Class to represent an order.
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

        if self._order_type == OrderMsg.KITTING:
            self._order_task = Kitting(order_data)
        elif self._order_type == OrderMsg.ASSEMBLY:
            self._order_task = Assembly(order_data)
        elif self._order_type == OrderMsg.COMBINED:
            self._order_task = CombinedTask(order_data)
        
        self._visting_first_time = True
        self._order_completed_flag = False
        self._parts_status_tray = {}
        self._tray_pick_status = {}


class OrderManagement(Node):
    """
    Class to manage the orders, do agility challengers and service calls to move it actions.

    Inherited Class:
        Node (rclpy.node.Node): Node class
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


        self.pkg_share = FindPackageShare("rwa5_group1").find("rwa5_group1")
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
            '_left_table_camera_subscription': ('/ariac/sensors/left_table_camera/image', AdvancedLogicalCameraImage, lambda msg: self._table_camera_callback(msg, 'Left')),
            '_right_table_camera_subscription': ('/ariac/sensors/right_table_camera/image', AdvancedLogicalCameraImage, lambda msg: self._table_camera_callback(msg, 'Right')),
            '_left_bins_camera_subscription': ('/ariac/sensors/left_bins_camera/image', AdvancedLogicalCameraImage, lambda msg: self._bin_camera_callback(msg, 'Left')),  
            '_right_bins_camera_subscription': ('/ariac/sensors/right_bins_camera/image', AdvancedLogicalCameraImage, lambda msg: self._bin_camera_callback(msg, 'Right')),
            '_robot_gripper_state_subscription': ('/ariac/floor_robot_gripper_state', VacuumGripperState, self._robot_gripper_state_subscription_cb),
        }  
        
        # Create the subscriptions
        for attr, (topic, msg_type, callback) in subscriptions.items():
            setattr(self, attr, self.create_subscription(msg_type, topic, callback, qos_profile=qos_policy, callback_group=self.callback_groups['_sensor_callback_group']))
            
        self._competition_state_subscription = self.create_subscription(CompetitionState,"/ariac/competition_state",self._competition_state_cb,qos_profile=qos_policy,callback_group=self.callback_groups['_competition_callback_group'])
        self._orders_subscription = self.create_subscription(OrderMsg,"/ariac/orders",self._orders_initialization_cb,qos_profile=qos_policy,callback_group=self.callback_groups['_order_callback_group'])
        
        self.get_logger().info(f"Node {node_name} initialized")
        self._orders_queue = PriorityQueue()
        self._agv_statuses = {}
        self._agv_velocities = {}
        self.current_order = None  # Track the currently processing or waiting order
        self.competition_ended = False
        self.tables_done = {'Left':False,'Right':False}
        self.bins_done = {'Left':False,'Right':False}

        
        
        self._Tray_Dictionary = {}
        self._Bins_Dictionary = {}
        self._Parts_Dictionary={'colors':{0: 'Red', 1: 'Green', 2: 'Blue', 3: 'Orange', 4: 'Purple'},
                      'types':{10:'Battery', 11:'Pump', 12:'Sensor', 13:'Regulator'}}

        
        
        self._end_condition_thread = None
        self.processing_lock = (
            threading.Condition()
        )  # Condition variable for synchronizing order processing

        ### MY Clients, Subscribers and variables for move it communication
        # Define the mappings
        self._tray_id_mapping = {i: getattr(MoveRobotToTray.Request, f"TRAY_ID{i}") for i in range(10)}
        self._agv_id_mapping = {i: getattr(MoveTrayToAGV.Request, f"AGV{i}") for i in range(1, 5)}
        self._quadrant_mapping = {i: getattr(PlacePartTray.Request, f"QUADRANT{i}") for i in range(1, 5)}
        self._part_color_mapping = {color: getattr(Part, color.upper()) for color in ["Red", "Green", "Blue", "Orange", "Purple"]}
        self._part_type_mapping = {part_type: getattr(Part, part_type.upper()) for part_type in ["Battery", "Pump", "Sensor", "Regulator"]}

        self._robot_gripper_state = "part_gripper"

        # Define the service clients
        clients = {
            '_pick_part_bin_cli': (PickPartBin, "/commander/pick_part_bin"),
            '_place_part_tray_cli': (PlacePartTray, "/commander/place_part_tray"),
            '_move_robot_home_cli': (Trigger, "/commander/move_robot_home"),
            '_move_robot_to_table_cli': (MoveRobotToTable, "/commander/move_robot_to_table"),
            '_move_robot_to_tray_cli': (MoveRobotToTray, "/commander/move_robot_to_tray"),
            '_move_tray_to_agv_cli': (MoveTrayToAGV, "/commander/move_tray_to_agv"),
            '_enter_tool_changer_cli': (EnterToolChanger, "/commander/enter_tool_changer"),
            '_exit_tool_changer_cli': (ExitToolChanger, "/commander/exit_tool_changer"),
            '_set_gripper_state_cli': (VacuumGripperControl, "/ariac/floor_robot_enable_gripper"),
            '_change_gripper_cli': (ChangeGripper, "/ariac/floor_robot_change_gripper"),
        }

        # Create the service clients
        for attr, (srv_type, srv_name) in clients.items():
            setattr(self, attr, self.create_client(srv_type, srv_name))


        # Initialize the flags for the order processing
        self._picking_part_from_bin = False
        self._picked_part_from_bin = False
        self._placing_part_on_tray = False
        self._placed_part_on_tray = False

        # Flags to track the completion of actions
        self._kit_completed = False
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


        self._high_priority_orders = []
        self._normal_orders =[]
        self._paused_orders = []
        self._start_process_order = False
        self.current_order_is = None
        self._order_announcements_count = -1
        self._order_submitted_count = 0


        # Start order processing thread
        self._order_processing_thread = threading.Thread(
                target=self.order_priority_timer_cb
            )
        self._order_processing_thread.start()

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
    
    def order_priority_timer_cb(self):
        """
        Function to process the orders based on priority.
        """
        while False in self.tables_done.values() or False in self.bins_done.values():
            pass
        else:
            if not self._competition_started:
                self._competition_started = True
                while True:
                    h_len = len(self._high_priority_orders)
                    n_len = len(self._normal_orders)
                    self.get_logger().info(f"High priority {self._high_priority_orders} ")
                    self.get_logger().info(f"Normal Priority {self._normal_orders} ")
                    if(h_len > 0):
                        self.current_order_is = "high"
                        ord_to_process = self._high_priority_orders[0]
                        self._process_order(ord_to_process)
                        self._high_priority_orders.pop(0)   
                        self._order_submitted_count += 1
                    elif (n_len > 0):
                        self.current_order_is = "normal"
                        ord_to_process = self._normal_orders[0]
                        self._process_order(ord_to_process)
                        if(ord_to_process._order_completed_flag):
                            self._normal_orders.pop(0)
                            self._order_submitted_count += 1
                    elif (self._order_submitted_count == self._order_announcements_count+1):
                        self.get_logger().info(f"Ending the Process order thread!!!")
                        break

    def _check_priority_flag(self):
        """
        Check if the priority flag is set to True. If set, return True, else False.
        """
        if(len(self._high_priority_orders) > 0 and self.current_order_is == "normal"):
            return True
        else:
            return False
        
    def _competition_state_cb(self, msg):
        """
        Callback for competition state changes. Starts the end condition checker when order announcements are done.
        """
        # Check if the competition has started
        if msg.competition_state == CompetitionState.STARTED:
            self._start_process_order = True
            
        # Check if the competition has ended and set the flag 
        if msg.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
            if (self._end_condition_thread is None or not self._end_condition_thread.is_alive()):
                self._end_condition_thread = threading.Thread(target=self._check_end_conditions)
                self._end_condition_thread.start()

    def _agv_status_cb(self, msg, agv_id):
        """
        Callback for AGV status changes. Updates the AGV status in the dictionary.

        Args:
            msg (any): Message received from the topic
            agv_id (int): ID of the AGV
        """
        # Define a mapping for AGV locations
        location_status_map = {0: "Kitting Station", 3: "WAREHOUSE"}
        status = location_status_map.get(msg.location, "OTHER")
        current_velocity = msg.velocity
        if agv_id not in self._agv_statuses or self._agv_statuses[agv_id] != status:
            self._agv_statuses[agv_id] = status
        if agv_id not in self._agv_velocities or self._agv_velocities[agv_id] != current_velocity:
            self._agv_velocities[agv_id] = current_velocity

    def _table_camera_callback(self, message, table_id='Unknown'):
        """
        Table Camera Callback is used to scan table cameras and store data

        Args:
            message: Image type from topic
            table_id: Arg passed when subscription callback created
        """
        if table_id == 'Unknown':
            self.get_logger().warn("Unknown table ID")
            return
        
        if self.tables_done[table_id] == False:
            
            self._Tray_Dictionary[table_id]={}
            tray_poses = message.tray_poses
            # self.get_logger().info(f" Tray Poses {tray_poses}")
            if len(tray_poses) > 0:
                self.tables_done[table_id] = True
                for tray in range(len(tray_poses)):
                    tray_pose_id = tray_poses[tray].id
                    # self.get_logger().info(f" Tray Poses ID {tray_pose_id}")
                    camera_pose = Pose()
                    camera_pose.position.x = message.sensor_pose.position.x
                    camera_pose.position.y = message.sensor_pose.position.y
                    camera_pose.position.z = message.sensor_pose.position.z
                    camera_pose.orientation.x = message.sensor_pose.orientation.x
                    camera_pose.orientation.y = message.sensor_pose.orientation.y
                    camera_pose.orientation.z = message.sensor_pose.orientation.z
                    camera_pose.orientation.w = message.sensor_pose.orientation.w

                    tray_pose = Pose()
                    tray_pose.position.x = tray_poses[tray].pose.position.x
                    tray_pose.position.y = tray_poses[tray].pose.position.y
                    tray_pose.position.z = tray_poses[tray].pose.position.z
                    tray_pose.orientation.x = tray_poses[tray].pose.orientation.x
                    tray_pose.orientation.y = tray_poses[tray].pose.orientation.y
                    tray_pose.orientation.z = tray_poses[tray].pose.orientation.z
                    tray_pose.orientation.w = tray_poses[tray].pose.orientation.w
                    tray_world_pose = self._multiply_pose(camera_pose, tray_pose)
                    if self._Tray_Dictionary[table_id] is None:
                        self._Tray_Dictionary[table_id] = {}
                    else:
                        self._Tray_Dictionary[table_id].update({tray_pose_id:{'position': [tray_world_pose.position.x,tray_world_pose.position.y,tray_world_pose.position.z], 'orientation': [tray_world_pose.orientation.x,tray_world_pose.orientation.y,tray_world_pose.orientation.z,tray_world_pose.orientation.w], 'status':False}})
                    # self.get_logger().info(f"    - {self._Tray_Dictionary}")
    
                
    def _bin_camera_callback(self, message,side='Unknown'):
        """
        Bin Camera Callback is used to scan bin cameras and store data

        Args:
            message: Image type from topic
            table_id: Arg passed when subscription callback created
        """
        if side == 'Unknown':
            self.get_logger().warn("Unknown side ID")
            return
        
        if self.bins_done[side] == False:
            bin_poses = message.part_poses
            if(len(bin_poses) > 0):
                self.bins_done[side] = True
                bin_camera_pose = Pose()
                bin_camera_pose.position.x = message.sensor_pose.position.x
                bin_camera_pose.position.y = message.sensor_pose.position.y
                bin_camera_pose.position.z = message.sensor_pose.position.z
                bin_camera_pose.orientation.x = message.sensor_pose.orientation.x
                bin_camera_pose.orientation.y = message.sensor_pose.orientation.y
                bin_camera_pose.orientation.z = message.sensor_pose.orientation.z
                bin_camera_pose.orientation.w = message.sensor_pose.orientation.w

                self._Bins_Dictionary[side]={}
                for i in range(len(bin_poses)):
                    bin_part = bin_poses[i].part
                    bin_part_pose = Pose()
                    bin_part_pose.position.x = bin_poses[i].pose.position.x
                    bin_part_pose.position.y = bin_poses[i].pose.position.y
                    bin_part_pose.position.z = bin_poses[i].pose.position.z
                    bin_part_pose.orientation.x = bin_poses[i].pose.orientation.x
                    bin_part_pose.orientation.y = bin_poses[i].pose.orientation.y
                    bin_part_pose.orientation.z = bin_poses[i].pose.orientation.z
                    bin_part_pose.orientation.w = bin_poses[i].pose.orientation.w

                    bin_world_pose = self._multiply_pose(bin_camera_pose, bin_part_pose)
                    type = self._Parts_Dictionary['types'][bin_part.type]
                    color = self._Parts_Dictionary['colors'][bin_part.color]
                    
                    if (type,color) in self._Bins_Dictionary[side].keys():
                        keys=self._Bins_Dictionary[side][(type,color)].keys()
                        self._Bins_Dictionary[side][(type,color)][len(keys)]={'position': [bin_world_pose.position.x,bin_world_pose.position.y,bin_world_pose.position.z], 'orientation': [bin_world_pose.orientation.x,bin_world_pose.orientation.y,bin_world_pose.orientation.z,bin_world_pose.orientation.w],'picked': False}
                    
                    else:
                        self._Bins_Dictionary[side][(type,color)]={}
                        self._Bins_Dictionary[side][(type,color)][0]={'position': [bin_world_pose.position.x,bin_world_pose.position.y,bin_world_pose.position.z], 'orientation': [bin_world_pose.orientation.x,bin_world_pose.orientation.y,bin_world_pose.orientation.z,bin_world_pose.orientation.w],'picked': False}
                    # self.get_logger().info(f"    - {self._Bins_Dictionary}")

    
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

        # Getting Roll, Pitch, and Yaw from the rotation matrix
        # q = frame3.M.GetRPY()
        # pose.orientation.x = q[0]
        # pose.orientation.y = q[1]
        # pose.orientation.z = q[2]
        # pose.orientation.w = 0.0

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
                return side, self._Tray_Dictionary[key][tray_id]['position'], self._Tray_Dictionary[key][tray_id]['orientation']

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
                        return part, side.lower(), part['position'], part['orientation']
        return None, None, None, None
    

    def _process_order(self, order):
        """
        Process the order by locking the AGV tray, moving the AGV to the station, and submitting the order.

        Args:
            order (Order): Order object to process
        """

        if(order._visting_first_time):
            # Process the order
            order._visting_first_time = False
            self.get_logger().info(f"Processing order: {order._order_id}.")
            
            self.get_logger().info("")
            self.get_logger().info("-"*50)
            stars=len(order._order_id) + 6
            self.get_logger().info("-" * ((50 - stars) // 2) + f"Order {order._order_id}" + "-" * ((50 - stars) // 2))
            self.get_logger().info("-"*50)
            # self.get_logger().info(f"Bin Dict {self._Bins_Dictionary}")
            # self.get_logger().info(f"Tray Dict {self._Tray_Dictionary}")
            # Get the tray pose and orientation
            tray_id = order._order_task.tray_id
            # To get the Unuesd tray
            side, tray_pose, tray_orientation = self._find_unused_tray(tray_id)
            
            self.get_logger().info("Kitting Tray:")
            self.get_logger().info(f" - ID: {tray_id}")
            self.get_logger().info(f" - Position (xyz): {tray_pose}")
            self.get_logger().info(f" - Orientation (rpy): {tray_orientation}")
            self.get_logger().info("Parts:")

            # Tray Picked Status for AGV. Value in form = [Picked Status, Tray Pose, Tray Orientation]
            order._tray_pick_status[tray_id] = {"status": False, "tray_pose": tray_pose, "tray_id": tray_id,
                                                "tray_orientation": tray_orientation, "tray_side": side}
            
            index = 0
            # Get the parts and their poses
            parts = order._order_task.parts
            for part in parts:
                type = self._Parts_Dictionary['types'][part.part.type]
                color = self._Parts_Dictionary['colors'][part.part.color]
                part_quadrant = part.quadrant
                final_part, bin_side, pose, orientation = self._find_available_part(type, color)
                
                # Dictionary in Class to store the parts information along with whether they are placed on tray or not
                order._parts_status_tray[(type, color, index)] = {"part_status": False, "part_type": type, "part_color": color,
                                                            "pose": pose, "orientation": orientation,
                                                            "part_quadrant": part_quadrant, "bin": bin_side}
                index += 1
                
                self.get_logger().info(f"    - {color} {type}")
                self.get_logger().info(f"       - Position (xyz): {pose}")
                self.get_logger().info(f"       - Orientation (rpy): {orientation}")
            
            self.get_logger().info("-"*50)
            self.get_logger().info("-"*50)
            self.get_logger().info("-"*50)

        self.execute_move_it_tasks(order)
        


    def execute_move_it_tasks(self,order):
        """
        Execute the move it tasks for the order.
        
        Args:
            order (Order): Order object to process
        """
        tray_id = self._tray_id_mapping[order._order_task.tray_id]
        agv_id = self._agv_id_mapping[order._order_task.agv_number]
        
        # self.get_logger().info(f"Order priority currently running is {str(order._order_id)} with priority: {int(order._order_priority)} with tray pick status {order._tray_pick_status}")
        
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
            if order._tray_pick_status[tray_id]["tray_side"] == "Left":
                tray_side_current = "kts1"
            elif order._tray_pick_status[tray_id]["tray_side"] == "Right":
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

            ############## 2.6. Move Robot to Home ##############
            # self._move_robot_home()
            # robot_moved_home_status_temp = self._moved_robot_home
            # while not robot_moved_home_status_temp :
            #     robot_moved_home_status_temp = self._moved_robot_home
            # self._moved_robot_home = False
            # self.get_logger().info("Python Node : Robot Moved to Home")
            
            ############## 2.7. Update Tray Pick Status ##############
            order._tray_pick_status[tray_id]["status"] =  True
            
            if self._check_priority_flag():
                self.get_logger().info("Order priority tray is placed on agv. So returning")
                return 
            
        # self.get_logger().info(f"Tray Pick Status {order._tray_pick_status}")

        if self._check_priority_flag():
            self.get_logger().info("Order priority received after tray placed on agv and moved home. So returning")
            return 
        
        ############################## 3. Pick Part from Bin and Place on Tray ########################################

        for key, v in order._parts_status_tray.items():            
            # self.get_logger().info("Python Node : Robot Moved to Home")
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
                part_pose.orientation.y = part_pose_curr_orient[0]
                part_pose.orientation.z = part_pose_curr_orient[0]
                part_pose.orientation.w = part_pose_curr_orient[0]

                ############## 3.1. Move Robot to Home ##############
                # self._move_robot_home()
                # robot_moved_home_status_temp = self._moved_robot_home
                # while not robot_moved_home_status_temp :
                #     robot_moved_home_status_temp = self._moved_robot_home
                # self._moved_robot_home = False
                # self.get_logger().info("Python Node : Robot Moved to Home")

                ############## 3.2. Pick Part from Bin ##############
                # time.sleep(3)
                self._robot_pick_part_from_bin(part_details, part_pose, part_bin_side)
                robot_picked_part_bin_temp = self._picked_part_from_bin
                while not robot_picked_part_bin_temp :
                    robot_picked_part_bin_temp = self._picked_part_from_bin
                self._picked_part_from_bin = False
                # self.get_logger().info("Python Node : Part Picked From Bin")

                ############## 3.3. Place Part on Tray ##############
                # time.sleep(3)
                self._robot_place_part_on_tray(agv_id,part_quadrant)
                robot_placed_part_tray_temp = self._placed_part_on_tray
                while not robot_placed_part_tray_temp :
                    robot_placed_part_tray_temp = self._placed_part_on_tray
                self._placed_part_on_tray = False
                # self.get_logger().info("Python Node : Part Placed on Tray")
                
                v["part_status"] = True     # Mark the part as placed on tray
                
                if self._check_priority_flag():
                    self.get_logger().info("Order priority received after a part is placed. So returning")
                    return 
        ############################## 4. Check Order Completion ########################################
        
        # self.get_logger().info(f"Order Status{order._parts_status_tray} ")
        order_completed_flag = True
        for k,v in order._parts_status_tray.items():
            if(v["part_status"] ==  False):
                order_completed_flag = False
                break

        ############################## 5. Process Order ##################################################
        if(order_completed_flag):
            order._order_completed_flag = True
            
            ############## 5.1. Lock Tray ##############
            self._lock_tray(agv_id)
            agv_lock_status_temp = self._locked_agv
            while not agv_lock_status_temp :
                agv_lock_status_temp = self._locked_agv
            self._locked_agv = False
            
            ############## 5.2. Move AGV to Shipping Station ##############
            self._move_agv(agv_id, order._order_task.destination)
            move_agv_status_temp = self._agv_moved_warehouse
            while not move_agv_status_temp :
                move_agv_status_temp = self._agv_moved_warehouse
            self._agv_moved_warehouse = False
            
            ############## 5.3. Submit Order ##############
            self._submit_order(agv_id, order._order_id)
            submit_order_temp_status = self._submitted_order
            while not submit_order_temp_status :
                submit_order_temp_status = self._submitted_order
            self._submitted_order = False
            
            self.get_logger().info(f"Order {order._order_id} processed and shipped.")

        return 

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
        """
        # rclpy.spin_until_future_complete(self, future)
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
        self.get_logger().info(f"Submit Order service called")

        current_status = self._agv_statuses.get(agv_id)
        current_agv_velocity = self._agv_velocities.get(agv_id)

        while (current_status != "WAREHOUSE" or current_agv_velocity > 0.0):
            self.get_logger().info(f"In status{current_status},{current_agv_velocity}")
            current_status = self._agv_statuses.get(agv_id)
            current_agv_velocity = self._agv_velocities.get(agv_id)

        self._submit_order_client = self.create_client(
            SubmitOrder, "/ariac/submit_order")
        request = SubmitOrder.Request()
        request.order_id = order_id
        future = self._submit_order_client.call_async(request)
        future.add_done_callback(lambda future: self._submit_order_cb(future))

        
    def _submit_order_cb(self,future):
        """
        Callback function of Submit AGV where the async response from server is handled
        """
        # rclpy.spin_until_future_complete(self, future)
        # rclpy.spin_once(self)
        if future.result() is not None:
            response = future.result()
            if response:
                self._submitted_order = True
                self.get_logger().info(f"Order submitted")
        else:
            self.get_logger().warn(f"Unable to submit order")

    def _check_end_conditions(self):
        """
        Periodically check if all orders are processed and AGVs are in warehouse, then end competition.
        """
        while not self.competition_ended and rclpy.ok():
            if (all(status == "WAREHOUSE" for status in self._agv_statuses.values()) and self._order_submitted_count == self._order_announcements_count+1):
                self.get_logger().info("All orders processed and AGVs at destination. Preparing to end competition.")
                self._end_competition()
                self._order_processing_thread.join()
            time.sleep(5)  # Check every 5 seconds

    def _end_competition(self):
        """
        End the competition if all conditions are met.
        """
        if not self.competition_ended:
            self.competition_ended = True
            self.get_logger().info("End competition service called")
            self._end_competition_client = self.create_client(Trigger, "/ariac/end_competition", callback_group=self.callback_groups['_competition_callback_group'])
            request = Trigger.Request()
            future = self._end_competition_client.call_async(request)
            future.add_done_callback(lambda future: self._end_competition_cb(future))

    def _end_competition_cb(self,future):
        """
        Callback function of End Competition where the async response from server is handled
        """
        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info("Competition ended")
        else:
            self.get_logger().warn("Unable to end competition")

    
    #####################################################################
    ####################Move IT Demo Functions###########################
    #####################################################################

    
    def _move_robot_home(self, end_demo=False):
        """
        Move the floor robot to its home position
        """

        self.get_logger().info("👉 Moving robot home...")
        if end_demo:
            self._ending_demo = True
        else:
            self._moving_robot_home = True

        while not self._move_robot_home_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")

        request = Trigger.Request()
        future = self._move_robot_home_cli.call_async(request)
        future.add_done_callback(self._move_robot_home_done_cb)

    def _move_robot_home_done_cb(self, future):
        """
        Client callback for the service /competitor/floor_robot/go_home

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
        Client callback for the service /commander/move_robot_to_table

        Args:
            future (Future): A future object
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
        Client callback for the service /commander/enter_tool_changer

        Args:
            future (Future): A future object
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
        Client callback for the service /ariac/floor_robot_change_gripper

        Args:
            future (Future): A future object
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
        Client callback for the service /commander/exit_tool_changer

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._exited_tool_changer = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    def _activate_gripper(self):
        """
        Activate the gripper
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
        Client callback for the service /ariac/floor_robot_enable_gripper

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
        Client callback for the service /ariac/floor_robot_enable_gripper

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
        Move the floor robot to a tray to pick it up
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
        Client callback for the service /commander/move_robot_to_tray

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_robot_to_tray = True
        else:
            self.get_logger().fatal(f"💀 {message}")

    # @brief Move the floor robot to its home position
    def _move_tray_to_agv(self, agv_number):
        
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
        Client callback for the service /commander/move_tray_to_agv

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._moved_tray_to_agv = True
        else:
            self.get_logger().fatal(f"💀 {message}")


    ########### My Functions for MoveIt Implementation################

    def _robot_gripper_state_subscription_cb(self,message):
        current_gripper_state = message.type
        if (current_gripper_state != self._robot_gripper_state):
            self._robot_gripper_state = current_gripper_state


    def _robot_pick_part_from_bin(self, part,pose,bin):
        """
        Make the robot pick part from bin

        Args:
            part: the type and color of part
            pose: The pose of the part in world frame
            bin: The bin where parts lie
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
        Client callback for the service /commander/pick_part_tray

        Args:
            future (Future): A future object
        """
        message = future.result().message
        if future.result().success:
            self.get_logger().info(f"✅ {message}")
            self._placed_part_on_tray = True
        else:
            self.get_logger().fatal(f"💀 {message}")
