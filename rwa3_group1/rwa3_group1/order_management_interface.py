from rclpy.node import Node
from ariac_msgs.msg import (Order as OrderMsg)

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
        if (self._order_type == OrderMsg.KITTING):
            self._order_task = Kitting(order_data)
        elif (self._order_type == OrderMsg.ASSEMBLY):
            self._order_task = Assembly(order_data)
        elif (self._order_type == OrderMsg.COMBINED):
            self._order_task = CombinedTask(order_data)



class OrderManagement(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.orders_subscription = self.create_subscription(OrderMsg,'/ariac/orders',self._orders_initialization_cb,10)
        self.get_logger().info(f"{node_name} initialized")
        self._orders_list = []

    def _orders_initialization_cb(self, msg):
        self.current_order = Order(msg)
        self._orders_list.append(self.current_order)
        self.get_logger().info(str(self.current_order._order_id))

    