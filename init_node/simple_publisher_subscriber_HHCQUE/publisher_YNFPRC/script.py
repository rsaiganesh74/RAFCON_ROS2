import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

def execute(self, inputs, outputs, gvm):
    node_handle = gvm.get_variable('node',per_reference=True)
    #self.logger("Unexpected "+str(node_handle))
    try:
        if gvm.variable_exist("ros_node_initialized") or gvm.get_variable("ros_node_initialized"):
            sub = node_handle.create_publisher(String, 'myfirsttopic', 1)
        msg=String()
        msg.data="Hello"

        sub.publish(msg)
        time.sleep(0.5)
    except Exception as e:
        self.logger.error("Unexpected error:"+str(e))

    return 0