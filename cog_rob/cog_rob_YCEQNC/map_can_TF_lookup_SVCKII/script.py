import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped

class Lookup(Node):
    def __init__(self):
        super().__init__('lookup')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(TransformStamped,'/looked_up',10)
        self.timer = self.create_timer(0.1,self.lookup_cb)
        self.tf_received = False
        self.max=0
    def lookup_cb(self):
        global lookup_Success
        try:
            self.t = self.tf_buffer.lookup_transform('map','can5',rclpy.time.Time())
            self.tf_received = True
        except:
            self.max+=1 
def execute(self, inputs, outputs, gvm):
    lookup = Lookup()
    try:
        while not lookup.tf_received:
            if lookup.max <50:
                rclpy.spin_once(lookup)
            else:
                break
        if lookup.max <50:
            outputs['tf']=lookup.t
            self.logger.info(str(lookup.t))
            return 0
        else:
            return -1
    except:
        return -1
