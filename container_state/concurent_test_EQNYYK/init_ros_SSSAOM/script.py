import rclpy
from rclpy.node import Node

def execute(self, inputs, outputs, gvm):
    node_name = inputs["node_name"]
    print(node_name)
    if type(node_name).__name__ == "unicode":
        node_name = node_name.encode('ascii', 'ignore')
        # print node_name
                
    self.logger.info("Creating node: " + node_name)
    try:
        try:
            rclpy.init()
        except:
            pass
        if not gvm.variable_exist("ros_node_initialized") or not gvm.get_variable("ros_node_initialized"):
            gvm.set_variable("ros_node_initialized", True)
            publishernode = rclpy.create_node('myfirstpublisher')
            gvm.set_variable(node_name,publishernode,per_reference=True)
            return 0
        else:
            return 0
    except Exception as e:
        self.logger.error("Unexpected error: "+str(e))
        return -1