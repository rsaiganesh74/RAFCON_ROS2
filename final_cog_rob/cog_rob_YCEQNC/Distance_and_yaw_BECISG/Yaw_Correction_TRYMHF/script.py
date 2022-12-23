import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
global num
class Orient(Node):
    def __init__(self):
        super().__init__('orient')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.timer = self.create_timer(0.1,self.orient_cb)
        self.rotate_complete = False
        self.translate_x=False
        self.translate_y=False
        self.vel = Twist()
    def orient_cb(self):
        global num
        self.can = "can"+str(num)
        try:
            self.tf_buffer.wait_for_transform_async('base_link',self.can, rclpy.time.Time())
            self.t = self.tf_buffer.lookup_transform('base_link',self.can, rclpy.time.Time())
            if not self.rotate_complete:
                if num == 5:
                    if self.t.transform.rotation.z>=0:
                        if not -0.01<self.t.transform.rotation.z<0.01:
                            self.vel.angular.z=0.3
                            self.pub.publish(self.vel)
                        else:
                            self.vel.angular.z = -0.0
                            self.pub.publish(self.vel)
                            self.rotate_complete = True    
                    else:
                        if not -0.01<self.t.transform.rotation.z<0.01:
                            self.vel.angular.z=-0.3
                            self.pub.publish(self.vel)
                        else:
                            self.vel.angular.z = -0.0
                            self.pub.publish(self.vel)
                            self.rotate_complete = True
                elif num == 4 or num == 3:
                    if self.t.transform.rotation.z<=-0.707:
                        if not -0.71<self.t.transform.rotation.z<-0.69:
                            self.vel.angular.z=-0.3
                            self.pub.publish(self.vel)
                        else:
                            self.vel.angular.z = -0.0
                            self.pub.publish(self.vel)
                            self.rotate_complete = True    
                    else:
                        if not -0.69<self.t.transform.rotation.z<-0.71:
                            self.vel.angular.z=0.3
                            self.pub.publish(self.vel)
                        else:
                            self.vel.angular.z = -0.0
                            self.pub.publish(self.vel)
                            self.rotate_complete = True
                elif num == 1:
                    if self.t.transform.rotation.z>=1:
                        if not 0.99<self.t.transform.rotation.z<1.01:
                            self.vel.angular.z=-0.3
                            self.pub.publish(self.vel)
                        else:
                            self.vel.angular.z = -0.0
                            self.pub.publish(self.vel)
                            self.rotate_complete = True    
                    else:
                        if not 0.99<self.t.transform.rotation.z<1.01:
                            self.vel.angular.z=0.3
                            self.pub.publish(self.vel)
                        else:
                            self.vel.angular.z = -0.0
                            self.pub.publish(self.vel)
                            self.rotate_complete = True

        except:
            pass

def execute(self, inputs, outputs, gvm):
    global num
    num  = inputs['can_num']
    self.logger.info(num)
    try:
        rclpy.init()
    except:
        pass
    orient = Orient()
    while not orient.rotate_complete:
        rclpy.spin_once(orient)
    return 0