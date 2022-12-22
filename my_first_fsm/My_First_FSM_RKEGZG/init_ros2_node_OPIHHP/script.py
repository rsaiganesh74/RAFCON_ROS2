import rclpy
from rclpy.node import Node
import sys
import traceback

def execute(self, inputs, outputs, gvm):
        node_name = inputs["node_name"]
        if type(node_name).__name__ == "unicode":
            node_name = node_name.encode('ascii', 'ignore')
            #node_name = new_ros2_node
        try:
            if not gvm.variable_exist("ros_node_initialized") or not gvm.get_variable("ros_node_initialized"):
                gvm.set_variable("ros_node_initialized", True)
                rclpy.init(args=None)
                node = rclpy.create_node(node_name)
                gvm.set_variable(node_name, node, per_reference=True)
                self.logger.info("Creating node: " + node_name)                           
        except Exception as e:
            self.logger.error("Unexpected error:" + str(e) + str(traceback.format_exc()))
        return 0