import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
ok = False

def mysubcallback(msg):
    global ok
    if msg.data == "pass":
        ok=True
    
def execute(self, inputs, outputs, gvm):
    global ok
    node_handle = gvm.get_variable('node',per_reference=True)
    self.logger.info("Sahil"+str(node_handle))
    try:
        if gvm.variable_exist("ros_node_initialized") or gvm.get_variable("ros_node_initialized"):
            subscriber = node_handle.create_subscription(String, 'mysecondtopic', mysubcallback, 10)
        while not ok:
            rclpy.spin_once(node_handle)
        return 0
    except Exception as e:
        self.logger.error("Unexpected error:"+str(e))
        return -1