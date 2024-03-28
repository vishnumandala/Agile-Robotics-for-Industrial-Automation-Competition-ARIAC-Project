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
    def __init__(self,order_data):
        self._order_id = order_data.id
        self._order_type = order_data.type
        self._order_priority = order_data.priority
        
        self.waiting = False  # Indicates if the order is currently in its waiting period
        self.elapsed_wait = 0  # Track elapsed wait time for the order
        self.wait_start_time = None  # Track the start time of the wait period
        
        if (self._order_type == OrderMsg.KITTING):
            self._order_task = Kitting(order_data)
        elif (self._order_type == OrderMsg.ASSEMBLY):
            self._order_task = Assembly(order_data)
        elif (self._order_type == OrderMsg.COMBINED):
            self._order_task = CombinedTask(order_data)

class OrderManagement(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()

        # Subscriptions
        self.orders_subscription = self.create_subscription(
            OrderMsg, '/ariac/orders', self._orders_initialization_cb,
            QoSProfile(depth=10), callback_group=self.callback_group)

        self.competition_state_subscription = self.create_subscription(
            CompetitionState, '/ariac/competition_state',
            self._competition_state_cb, QoSProfile(depth=10),
            callback_group=self.callback_group)

        # Service clients
        self.submit_order_client = self.create_client(SubmitOrder, '/ariac/submit_order', callback_group=self.service_callback_group)
        self.end_competition_client = self.create_client(Trigger, '/ariac/end_competition', callback_group=self.service_callback_group)

        self.get_logger().info(f"Node {node_name} initialized")
        
        self._orders_queue = PriorityQueue()
        self.agv_statuses = {}
        self.current_order = None  # Track the currently processing or waiting order
        self.competition_ended = False
        
        self._order_processing_thread = None
        self._end_condition_thread = None
        self.competition_ended = False
        self.processing_lock = threading.Condition()  # Condition variable for synchronizing order processing
        self.current_order = None  # Track the currently processing or waiting order

    def _competition_state_cb(self, msg):
        """Callback for competition state changes. Starts the end condition checker when order announcements are done."""
        if msg.competition_state == CompetitionState.ORDER_ANNOUNCEMENTS_DONE:
            if self._end_condition_thread is None or not self._end_condition_thread.is_alive():
                self._end_condition_thread = threading.Thread(target=self._check_end_conditions)
                self._end_condition_thread.start()

    def _check_end_conditions(self):
        """Periodically check if all orders are processed and AGVs are in warehouse, then end competition."""
        while not self.competition_ended and rclpy.ok():
            if self._orders_queue.empty() and all(status == "WAREHOUSE" for status in self.agv_statuses.values()):
                self.get_logger().info("All orders processed and AGVs at destination. Preparing to end competition.")
                self._end_competition()
            time.sleep(5)  # Check every 5 seconds

    def _agv_status_cb(self, msg, agv_id):
        # Define a mapping for AGV locations
        location_status_map = {0: "Kitting Station", 3: "WAREHOUSE"}
        status = location_status_map.get(msg.location, "OTHER")
        self.agv_statuses[agv_id] = status

    def _end_competition(self):
        if not self.competition_ended:
            """End the competition if all conditions are met."""
            self.get_logger().info("Ending competition now.")
            request = Trigger.Request()
            future = self.end_competition_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response.success:
                self.get_logger().info("Competition ended successfully.")
                self.competition_ended = True
            else:
                self.get_logger().error("Failed to end the competition.")

    def _submit_order(self, agv_id, order_id):
        # Check the AGV status every second until it reaches the destination
        while self.agv_statuses.get(agv_id) != 'WAREHOUSE':
            time.sleep(1)

        # Directly using the submit_order_client to call the service
        request = SubmitOrder.Request()
        request.order_id = order_id
        future = self.submit_order_client.call_async(request)
        
        # Wait for the service call to complete
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            self.get_logger().info(f"Order submitted for AGV {agv_id}.")
        else:
            self.get_logger().error(f"Failed to submit order for AGV {agv_id}.")

    def _process_order(self, order):
        self.get_logger().info(f"Processing order: {order._order_id}.")
        agv_id = order._order_task.agv_number
        self._lock_agv_tray(agv_id)
        self._move_agv_to_station(agv_id, order._order_task.destination)
        self._submit_order(agv_id, order._order_id)
        self.get_logger().info(f"Order {order._order_id} processed and shipped.")

    def _process_orders(self):
        while rclpy.ok():
            with self.processing_lock:
                while not self.current_order and self._orders_queue.empty():
                    self.processing_lock.wait()  # Wait for an order to be queued or for the current order to be set

                if not self.current_order:  # No current order, get the next one from the queue
                    _, self.current_order = self._orders_queue.get()

            # Process the current order (either a new one or a resumed one)
            self._wait_and_process_current_order()

    def _wait_and_process_current_order(self):
        order = self.current_order
        if order.elapsed_wait > 0:
            # Calculate the remaining wait time for a resuming order
            remaining_wait = max(15 - order.elapsed_wait, 0)
            self.get_logger().info(f"Order {order._order_id} resuming wait with {remaining_wait:.2f} seconds remaining.")
        else:
            # Full wait time for a first-time wait
            remaining_wait = 15
        order.wait_start_time = time.time()

        order.waiting = True  # Ensure order.waiting is set to True whether it's a new wait or a resumed one
        self.get_logger().info(f"Order {order._order_id}: Waiting for {remaining_wait:.2f} seconds before processing.")

        while remaining_wait > 0:
            start_wait = time.time()
            time.sleep(min(0.1, remaining_wait))
            with self.processing_lock:
                if not order.waiting:
                    # If the order's waiting is interrupted, adjust the elapsed wait time and pause the wait
                    paused_wait = time.time() - start_wait
                    order.elapsed_wait += paused_wait
                    self.get_logger().info(f"Order {order._order_id}: Wait paused at {paused_wait:.2f} seconds. Total elapsed wait time: {order.elapsed_wait:.2f} seconds.")
                    return  # Exit the wait loop if the order is paused

            actual_waited = time.time() - start_wait
            remaining_wait -= actual_waited

        total_waited = time.time() - order.wait_start_time
        order.elapsed_wait += total_waited  # Update the total elapsed wait time
        order.waiting = False  # Set waiting to False after the wait is completed

        self.get_logger().info(f"Order {order._order_id}: Total elapsed wait time before processing: {order.elapsed_wait:.2f} seconds.")
        self._process_order(order)  # Proceed to process the order

        with self.processing_lock:
            self.current_order = None  # Clear the current order after processing
            self.processing_lock.notify()  # Notify potentially waiting threads that the current order has been processed

    def _lock_agv_tray(self, agv_id):
        self.get_logger().info(f"Locking tray for AGV{agv_id}...")
        service_name = f'/ariac/agv{agv_id}_lock_tray'
        client = self.create_client(Trigger, service_name, callback_group=self.service_callback_group)
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"AGV{agv_id} tray locked.")
        else:
            self.get_logger().error(f"Failed to lock AGV{agv_id} tray.")
            
    def _move_agv_to_station(self, agv_id, station):
        self.get_logger().info(f"Moving AGV{agv_id}...")
        service_name = f'/ariac/move_agv{agv_id}'
        client = self.create_client(MoveAGV, service_name, callback_group=self.service_callback_group)
        request = MoveAGV.Request()
        if station == 3:
            station = "Warehouse"
            request.location = MoveAGV.Request.WAREHOUSE
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"AGV{agv_id} moved to {station}.")
        else:
            self.get_logger().error(f"Failed to move AGV{agv_id} to {station}.")

    def _orders_initialization_cb(self, msg):
        order_priority = -1 if msg.priority else 1
        order = Order(msg)
        # self._orders_queue.put((order_priority, order))
        self.get_logger().info(f"Order {msg.id} received with {'high' if msg.priority else 'low'} priority and queued.")

        with self.processing_lock:
            if msg.priority and self.current_order and self.current_order.waiting:  # High-priority order interrupts the current order
                # Update the elapsed_wait for the current order before pausing
                current_time = time.time()
                if hasattr(self.current_order, 'wait_start_time'):
                    interrupted_wait = current_time - self.current_order.wait_start_time
                    self.current_order.elapsed_wait += interrupted_wait
                    self.get_logger().info(f"Order {self.current_order._order_id}: Paused for high-priority order {msg.id}. Total elapsed wait time: {self.current_order.elapsed_wait:.2f} seconds.")
                self.current_order.waiting = False  # Pause the current order's waiting
                self._orders_queue.put((1, self.current_order))  # Re-queue the paused order
                self.current_order = order  # Set high-priority order as current order to be processed immediately
            else:
                self._orders_queue.put((1 if msg.priority else 2, order))  # Regular queueing for orders

            self.processing_lock.notify()

        # If no order is currently being processed, start processing
        if self._order_processing_thread is None or not self._order_processing_thread.is_alive():
            self._order_processing_thread = threading.Thread(target=self._process_orders)
            self._order_processing_thread.start()
            
        agv_id = order._order_task.agv_number
        if agv_id not in self.agv_statuses:
            self.create_subscription(
                AGVStatus,
                f'/ariac/agv{agv_id}_status',
                lambda msg: self._agv_status_cb(msg, agv_id),
                QoSProfile(depth=10),
                callback_group=self.callback_group)