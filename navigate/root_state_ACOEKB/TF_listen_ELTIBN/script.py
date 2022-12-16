import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
ok = False

def lookup_cb():
    global ok,tf_buffer,t
    try:
        t = tf_buffer.lookup_transform('map','can4',rclpy.time.Time())
        print('Received Lookup, exiting')
        ok = 'pass'
    except:
        print("Not getting Lookup")
    
def execute(self, inputs, outputs, gvm):
    global ok,tf_buffer,t
    node_handle = gvm.get_variable('node',per_reference=True)
    self.logger.info("TF"+str(node_handle))
    try:
        if gvm.variable_exist("ros_node_initialized") or gvm.get_variable("ros_node_initialized"):
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, node_handle)
            timer = node_handle.create_timer(0.1,lookup_cb)
        while not ok:
            rclpy.spin_once(node_handle)
        outputs['tf']=t
        return 0
    except Exception as e:
        self.logger.error("Unexpected error:"+str(e))
        return -1