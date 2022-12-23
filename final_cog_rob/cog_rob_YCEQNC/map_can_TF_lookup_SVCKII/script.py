import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
global num
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
        global num
        self.can = "can"+str(num)
        try:
            self.tf_buffer.wait_for_transform_async('map',self.can, rclpy.time.Time())
            self.t = self.tf_buffer.lookup_transform('map',self.can, rclpy.time.Time())
            self.tf_received = True
        except:
            self.max+=1 
def execute(self, inputs, outputs, gvm):
    global num
    num  = inputs['can_num']
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
            outputs['can_no'] = inputs['can_num']
            return 0
        else:
            return -1
    except:
        return -1
