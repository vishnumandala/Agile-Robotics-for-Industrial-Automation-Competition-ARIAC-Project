import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from ariac_msgs.msg import Order as OrderMsg, AGVStatus, CompetitionState
from ariac_msgs.srv import MoveAGV, SubmitOrder
from std_srvs.srv import Trigger
from queue import PriorityQueue
import time
import threading

class Kitting:
    """
    Class to represent the kitting task of an order.
    """
    def __init__(self,order_data):
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
    def __init__(self,order_data):
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
    def __init__(self,order_data):
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
    def __init__(self,order_data):
        self._order_id = order_data.id
        self._order_type = order_data.type
        self._order_priority = order_data.priority
        if (self._order_type == OrderMsg.KITTING):
            self._order_task = Kitting(order_data)
        elif (self._order_type == OrderMsg.ASSEMBLY):
            self._order_task = Assembly(order_data)
        elif (self._order_type == OrderMsg.COMBINED):
            self._order_task = CombinedTask(order_data)

class OrderManagement(Node):
    """
    Class to manage the orders and competition state.

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
        self._callback_group = ReentrantCallbackGroup()
        self._service_group = ReentrantCallbackGroup()

        # Subscriptions
        self.orders_subscription = self.create_subscription(
            OrderMsg, '/ariac/orders', self._orders_initialization_cb,
            QoSProfile(depth=10), callback_group=self._callback_group)

        self.competition_state_subscription = self.create_subscription(
            CompetitionState, '/ariac/competition_state',
            self._competition_state_cb, QoSProfile(depth=10),
            callback_group=self._callback_group)

        # Timer to process orders
        self._process_order_timer = self.create_timer(15, self._process_order_timer_cb, callback_group=self._callback_group)

        # Initialize variables
        self.get_logger().info(f"Node {node_name} initialized")
        self._orders_queue = PriorityQueue()
        self._agv_statuses = {}
        self._order_processing_thread = None
        self._end_condition_thread = None
        self.competition_ended = False
  
    def _orders_initialization_cb(self, msg):
        """
        Callback for receiving orders.

        Args:
            msg (any): Message received from the topic
        """
        # Add the order to the queue
        order_priority = -1 if msg.priority else 1
        order = Order(msg)
        self._orders_queue.put((order_priority, order))
        self.get_logger().info(f"Received Order: {str(order._order_id)} with priority: {int(order._order_priority)}")
        
        # Reset the timer after receiving an order
        self._process_order_timer.reset()
        
        # Create a subscription to the AGV status topic 
        agv_id = order._order_task.agv_number
        if agv_id not in self._agv_statuses:
            self.create_subscription(
                AGVStatus,
                f'/ariac/agv{agv_id}_status',
                lambda msg: self._agv_status_cb(msg, agv_id),
                QoSProfile(depth=10),
                callback_group=self._callback_group)
            
    def _competition_state_cb(self, msg):
        """
        Callback for competition state changes. Starts the end condition checker when order announcements are done.
        """
        # Start the end condition checker when order announcements are done
        if msg.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
            if self._end_condition_thread is None or not self._end_condition_thread.is_alive():
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
        self._agv_statuses[agv_id] = status

    def _process_order_timer_cb(self):
        """
        Callback function to process the orders.
        """
        # If no order is currently being processed, start processing
        if self._order_processing_thread is None or not self._order_processing_thread.is_alive():
            self._order_processing_thread = threading.Thread(target=self._process_orders)
            self._order_processing_thread.start()

    def _process_orders(self):
        """
        Process all orders in the queue.
        """
        while rclpy.ok() and not self._orders_queue.empty():
            _, order = self._orders_queue.get()
            self._process_order(order)

    def _process_order(self, order):
        """
        Process the order by locking the AGV tray, moving the AGV to the station, and submitting the order.

        Args:
            order (Order): Order object to process
        """
        # Process the order
        self.get_logger().info(f"Processing order: {order._order_id}.")
        agv_id = order._order_task.agv_number
        self._lock_tray(agv_id)
        self._move_agv(agv_id, order._order_task.destination)
        self._submit_order(agv_id, order._order_id)
        self.get_logger().info(f"Order {order._order_id} processed and shipped.")

    def _lock_tray(self, agv):
        """Function to lock the tray

        Args:
            agv (str): Name of the agv
        """        
        # Lock the tray to AGV
        self.get_logger().info(f'Lock Tray service called')
        self._lock_trays_client = self.create_client(Trigger, f'/ariac/agv{agv}_lock_tray', callback_group=self._service_group)
        request = Trigger.Request()
        future = self._lock_trays_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info(f'AGV {agv} locked')
        else:
            self.get_logger().warn(f'Unable to lock AGV {agv}')
            
    def _move_agv(self, agv, destination):
        """Function to move the agv to the shipping station

        Args:
            agv (str): Name of the agv
            destination (str): Destination of the agv
        """        
        # Move the AGV to the destination
        self.get_logger().info(f'Move AGV service called')
        self._move_agv_client = self.create_client(MoveAGV, f'/ariac/move_agv{agv}', callback_group=self._service_group)
        request = MoveAGV.Request()
        request.location = destination
        future = self._move_agv_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        destination = "Warehouse" if destination == 3 else destination
        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info(f'AGV: {agv} moved to {destination}')
        else:
            self.get_logger().warn(f'Service call failed {future.exception()}')
            self.get_logger().warn(f'Failed to move AGV: {agv} to {destination}')

    def _submit_order(self, agv_id, order_id):
        """
        Submit the order to the competition.

        Args:
            agv_id (int): ID of the AGV
            order_id (str): ID of the order
        """
        self.get_logger().info(f'Submit Order service called')
        
        # Wait until the AGV is in the warehouse
        while self._agv_statuses.get(agv_id) != 'WAREHOUSE':
            time.sleep(1)
        
        self._submit_order_client = self.create_client(SubmitOrder, '/ariac/submit_order', callback_group=self._service_group)
        request = SubmitOrder.Request()
        request.order_id = order_id
        future = self._submit_order_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response:
                self.get_logger().info(f'Order submitted')
        else:
            self.get_logger().warn(f'Unable to submit order')

    def _check_end_conditions(self):
        """
        Periodically check if all orders are processed and AGVs are in warehouse, then end competition.
        """
        while not self.competition_ended and rclpy.ok():
            if self._orders_queue.empty() and all(status == "WAREHOUSE" for status in self._agv_statuses.values()):
                self.get_logger().info("All orders processed and AGVs at destination. Preparing to end competition.")
                self._end_competition()
            time.sleep(5)  # Check every 5 seconds

    def _end_competition(self):
        """
        End the competition if all conditions are met.
        """
        if not self.competition_ended:
            self.competition_ended = True
            self.get_logger().info(f'End competition service called')
            self._end_competition_client = self.create_client(Trigger, '/ariac/end_competition', callback_group=self._service_group)
            request = Trigger.Request()
            future = self._end_competition_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                response = future.result()
                if response:
                    self.get_logger().info(f'Competition ended')
            else:
                self.get_logger().warn(f'Unable to end competition')