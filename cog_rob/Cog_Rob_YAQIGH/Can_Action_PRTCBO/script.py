import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup

from copy import deepcopy

global_joint_states = None
lookup_done = False
get_angles = False


class MoveGroupActionClient(Node):

    def __init__(self):
        super().__init__('moveit_plan_execute_python')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1,self.lookup_cb)

        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)


    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state

    def send_goal(self):

        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.5
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.5
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.5
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.5
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.5
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.5
        self.motion_plan_request.start_state.is_diff = True

        self.jc = JointConstraint()
        self.jc.tolerance_above = 0.0001
        self.jc.tolerance_below = 0.0001
        self.jc.weight = 1.0

        joints = {}
        joints['shoulder_pan_joint'] = self.angles[0]
        joints['shoulder_lift_joint'] = self.angles[1]
        joints['elbow_joint'] = self.angles[2]
        joints['wrist_1_joint'] = self.angles[3]
        joints['wrist_2_joint'] = self.angles[4]
        joints['wrist_3_joint'] = self.angles[5]

        constraints = Constraints()
        for (joint, angle) in joints.items():
            self.jc.joint_name = joint
            self.jc.position = angle
            constraints.joint_constraints.append(deepcopy(self.jc))

        self.motion_plan_request.goal_constraints.append(constraints)

        self.motion_plan_request.pipeline_id = 'move_group'
        self.motion_plan_request.group_name = 'spot_ur3e_arm'
        self.motion_plan_request.num_planning_attempts = 5
        self.motion_plan_request.allowed_planning_time = 15.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0

        self.planning_options = PlanningOptions()
        self.planning_options.plan_only = False
        self.planning_options.look_around = False
        self.planning_options.look_around_attempts = 0
        self.planning_options.max_safe_execution_cost = 0.
        self.planning_options.replan = True
        self.planning_options.replan_attempts = 10
        self.planning_options.replan_delay = 0.1

        
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.motion_plan_request.start_state.joint_state = self.joint_state

        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.planning_options = self.planning_options

        self._action_client.wait_for_server()
        print("waiting")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
   


    def lookup_cb(self):
        global lookup_done
        try:
            self.tf_buffer.wait_for_transform_async('base_link','can4', rclpy.time.Time())
            self.t = self.tf_buffer.lookup_transform('base_link','can4',rclpy.time.Time()) 
            while lookup_done == False:
                #print('Received Lookup, exiting')
                print(self.t.transform.translation.z)
                if -0.060<self.t.transform.translation.z<-0.050:
                    self.key = -0.055
                    self.joint_angles(self.key)
                elif -0.050 < self.t.transform.translation.z < -0.040:
                    self.key = -0.046
                    self.joint_angles(self.key)
                elif 0.240 < self.t.transform.translation.z < 0.250:
                    self.key = 0.248
                    self.joint_angles(self.key)
                elif 0.250 < self.t.transform.translation.z < 0.260:
                    self.key = 0.254
                    self.joint_angles(self.key)
        except:
            print("Not getting Lookup")

    def joint_angles(self, key):
        global get_angles, lookup_done
        self.angle_val = {-0.055 : [1.1021957076032016, -0.17248525117034186, 1.7602054848751538, 4.695393383535243, 1.1017290218565028, -3.1472797573696605],
            -0.046 : [-1.0286563487178404, -2.9471373911172947, -1.7788700326189506, -1.5538880379676083, -1.0289534302696344, 3.1399130639653143],
            0.248 : [-5.1807660382063645, -0.7382727144594636, 2.0415024749905446, 1.8383681303505592, -1.1024246039727346, 0.0028362194177750464],
            0.254 : [-5.180767579168836, -0.7659777878197128, 2.0466537486542555, 1.8609218355554278, -1.1024231075469837, -6.280348911199471]}
        self.angles = self.angle_val[key]
        get_angles = True
        print(self.angles)
        print(get_angles)
        self.timer.destroy()
        lookup_done = True

def execute(self, inputs, outputs, gvm):
    global lookup_done,get_angles
    try:
        rclpy.init()
    except:
        pass
    action_client = MoveGroupActionClient()
    for i in range(100):
        if lookup_done and get_angles:
            rclpy.spin_once(action_client)
            action_client.send_goal()
            rclpy.spin_once(action_client)
            break
        else:
            rclpy.spin_once(action_client)
            print(lookup_done)
        i += 1
    return 0 
