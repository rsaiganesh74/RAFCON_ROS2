import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
from spot_msgs.srv import SpotMotion
ok = False

class Lookup(Node):
    def __init__(self):
        super().__init__('lookup')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(0.1,self.lookup_cb)
        self.vel = Twist()
        self.x_align = False
        self.t = None


    def lookup_cb(self):
        global ok
        try:

            print(self.x_align)
            self.tf_buffer.wait_for_transform_async('Robotiq3fGripper','can4', rclpy.time.Time())
            self.t = self.tf_buffer.lookup_transform('Robotiq3fGripper','can4', rclpy.time.Time())
            #print('Received Lookup, exiting')
            #print(self.t)
            
        except:
            print("Not getting Lookup")

        if self.x_align == False:
            print(self.x_align)
            if not 0.08<self.t.transform.translation.y<0.12:
                print("aligning x")
                self.vel.linear.x = 0.2
                self.pub.publish(self.vel)
            else:
                self.vel.linear.x = 0.0
                self.pub.publish(self.vel)
                self.x_align = True

    def close_gripper(self):     
        global ok
     
        self.srv = self.create_client(SpotMotion,'/Spot/close_gripper')
     

    
        self.req = SpotMotion.Request()
        self.future = self.srv.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return
  


def execute(self, inputs, outputs, gvm):
    global ok
    try:
        rclpy.init()
    except:
        pass
    lookup = Lookup()
    # Get info from /joint_states
    while not lookup.x_align:
        rclpy.spin_once(lookup)
    lookup.close_gripper()
    if ok:
        return 0
    else:
        return -1