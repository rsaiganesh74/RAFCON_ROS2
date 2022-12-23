import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Point, Quaternion

global_joint_states = None
global status_second
global status_third 

class ComputeIKtwo(Node):
    def __init__(self):
        super().__init__('service_and_actiontwo')
        self.req = GetPositionIK.Request()
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)
        self.srv = self.create_client(GetPositionIK,'/compute_ik')
        self.req.ik_request.group_name = 'spot_ur3e_arm'
        self.req.ik_request.robot_state.is_diff = True

    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state
        self.req.ik_request.robot_state.joint_state = joint_state

    def send_request(self):
        global status_second,status_third
        pose_can_point  = Point()
        pose_can_orientation = Quaternion()

        pose_can_point.x = 0.7464211994107804
        pose_can_point.y = 0.01
        pose_can_point.z = 0.0
        pose_can_orientation.x = 0.0
        pose_can_orientation.y = 0.0
        pose_can_orientation.z = -0.707
        pose_can_orientation.w = 0.707

        self.req.ik_request.pose_stamped.pose.position = pose_can_point
        self.req.ik_request.pose_stamped.pose.orientation = pose_can_orientation
        try:
            self.req.ik_request.pose_stamped.header.frame_id = 'base_link'
            self.req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            self.req.ik_request.avoid_collisions = False
            self.req.ik_request.timeout = Duration()
            self.req.ik_request.timeout.sec = 20
            while not self.srv.wait_for_service(timeout_sec=1.0):
                pass
            self.future = self.srv.call_async(self.req)
            rclpy.spin_until_future_complete(self,self.future)
            return self.future.result()
        except:
            return False

def execute(self, inputs, outputs, gvm):
    global status_second,status_third
    try:
        rclpy.init()
    except:
        pass
    compute_ik = ComputeIKtwo()
    while global_joint_states==None:
        rclpy.spin_once(compute_ik)
    outputs['joint_states']=global_joint_states

    response = compute_ik.send_request()

    if (response.error_code.val != -31) and not (response == False):
        self.logger.info("SUCCESS")
        outputs['compute_ik_response'] = response
        return 0
    else:
        self.logger.info("Failed compute")
        return -1