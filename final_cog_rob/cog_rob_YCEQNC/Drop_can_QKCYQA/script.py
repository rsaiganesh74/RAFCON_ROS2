import rclpy
from spot_msgs.srv import SpotMotion
from rclpy.node import Node
var = False

class Open(Node):
    def __init__(self):
        super().__init__("open_gripper")
        global var
        self.srv = self.create_client(SpotMotion,'/Spot/open_gripper')
        self.req = SpotMotion.Request()
        self.future = self.srv.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        var = True


def execute(self, inputs, outputs, gvm):
    global var
    try:
        rclpy.init()
    except:
        pass
    open_gripper = Open()
    while not var:
        rclpy.spin_once(open_gripper)
    if inputs['can_num'] != 3:
        outputs['can_no'] = inputs['can_num']- 1
        if var:
            return -2
        else:
            return -1
    elif inputs['can_num'] == 1:
        if var:
            return 0
        else:
            return -1
    else:
        outputs['can_no'] = inputs['can_num']- 2
        if var:
            return -2
        else:
            return -1

