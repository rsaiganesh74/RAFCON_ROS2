import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup
from copy import deepcopy
from action_msgs.msg import GoalStatus

global_joint_states = None
var = False
global status_second
global status_third
class ManipulatorMove(Node):
    def __init__(self):
        super().__init__('manipulator')
        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        self.motion_plan_request.workspace_parameters.min_corner.x = -4.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -4.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -4.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 4.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 4.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 4.0
        self.motion_plan_request.start_state.is_diff = True 
        self.jc = JointConstraint()
        self.jc.tolerance_above = 0.01
        self.jc.tolerance_below = 0.01
        self.jc.weight = 1.0
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
        self.planning_options.replan_attempts = 5
        self.planning_options.replan_delay = 0.1                              

        self._action_client = ActionClient(self, MoveGroup, '/move_action')

    def build_constraint(self,response):
        joints = {}
        joints['shoulder_pan_joint'] = response.solution.joint_state.position[0]
        joints['shoulder_lift_joint'] = response.solution.joint_state.position[1]
        joints['elbow_joint'] = response.solution.joint_state.position[2]
        joints['wrist_1_joint'] = response.solution.joint_state.position[3]
        joints['wrist_2_joint'] = response.solution.joint_state.position[4]
        joints['wrist_3_joint'] = response.solution.joint_state.position[5]

        constraints = Constraints()
        for (joint, angle) in joints.items():
            self.jc.joint_name = joint
            self.jc.position = angle
            constraints.joint_constraints.append(deepcopy(self.jc))

        self.motion_plan_request.goal_constraints.append(constraints)
        self.send_goal()
  
    def send_goal(self):
        global var
        self.motion_plan_request.start_state.joint_state = self.joint_stated

        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.planning_options = self.planning_options

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
    
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        goal_handle = self._send_goal_future.result()
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
    global var,status_second,status_third
    try:
        rclpy.init()
    except:
        pass
    manipulator_move = ManipulatorMove()
    response = inputs['compute_ik_response']

    manipulator_move.joint_stated = inputs['joint_states']
    manipulator_move.build_constraint(response)
    if var:
        return 0

    else:
        return -1