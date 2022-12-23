import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup
from action_msgs.msg import GoalStatus

from copy import deepcopy

global_joint_states = None
var = False
class MoveGroupActionClient_two(Node):

    def __init__(self):
        super().__init__('moveit_plan_execute')

        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True
        self.var = False

        jc = JointConstraint()
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        joints = {}
        joints['shoulder_pan_joint'] = 0.0
        joints['shoulder_lift_joint'] = 0.0
        joints['elbow_joint'] = 0.0
        joints['wrist_1_joint'] = 0.0
        joints['wrist_2_joint'] = 0.0
        joints['wrist_3_joint'] = 0.0

        constraints = Constraints()
        for (joint, angle) in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        self.motion_plan_request.goal_constraints.append(constraints)

        self.motion_plan_request.pipeline_id = 'move_group'
        self.motion_plan_request.group_name = 'spot_ur3e_arm'
        self.motion_plan_request.num_planning_attempts = 10
        self.motion_plan_request.allowed_planning_time = 5.0
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
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state
        
    def send_goal(self):
        global var
        self.motion_plan_request.start_state.joint_state = self.joint_state

        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.planning_options = self.planning_options
        while not self._action_client.wait_for_server(timeout_sec=2.0):
            continue
        self.send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, self.send_goal_future)
        goal_handle = self.send_goal_future.result()
        if not goal_handle.accepted:
            self.var = False
            return
        self.get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.get_result_future)
        status = self.get_result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            var = True
        return

def execute(self, inputs, outputs, gvm):
    global var,global_joint_states
    try:
        rclpy.init()
    except:
        self.logger.info(".")
    action_client_two = MoveGroupActionClient_two()
    while global_joint_states is None:
        rclpy.spin_once(action_client_two)
    action_client_two.send_goal()
    if var:
        return 0
    else:
        return -1
