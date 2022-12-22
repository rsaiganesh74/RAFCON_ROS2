from std_msgs.msg import String

def execute(self, inputs, outputs, gvm):
    node = gvm.get_variable("new_ros2_node", True)
    test_pub = node.create_publisher(String, '/chatter', 10)
    msg = String()
    msg.data = 'I am in State 1'
    test_pub.publish(msg)
    return "success"
