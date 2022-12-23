import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
global num
class Distance(Node):
    def __init__(self):
        super().__init__('orient')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(0.1,self.distance_cb)
        self.translate_y=False
        self.vel = Twist()
    def distance_cb(self):
        global num
        self.can = "can"+str(num)
        try:
            self.tf_buffer.wait_for_transform_async('base_link',self.can, rclpy.time.Time())
            self.t = self.tf_buffer.lookup_transform('base_link',self.can, rclpy.time.Time())
            if not self.translate_y:
                if self.t.transform.translation.y>0.0:
                    if not 0.0<self.t.transform.translation.y<0.04:
                        self.vel.linear.y = 0.3
                        self.pub.publish(self.vel)
                    else:
                        self.vel.linear.y = 0.0
                        self.pub.publish(self.vel)
                        self.translate_y = True
                else:
                    if not 0.0<self.t.transform.translation.y<0.04:
                        self.vel.linear.y = -0.3
                        self.pub.publish(self.vel)
                    else:
                        self.vel.linear.y = 0.0
                        self.pub.publish(self.vel)
                        self.translate_y = True
        except:
            pass

def execute(self, inputs, outputs, gvm):
    global num
    num  = inputs['can_num']
    try:
        rclpy.init()
    except:
        pass
    distance = Distance()
    while not distance.translate_y:
        rclpy.spin_once(distance)
    if inputs["num_tries"]!=3:
        outputs["num"]=inputs["num_tries"]+1
        return -2
    else:
        outputs["num"]=0
        return 0