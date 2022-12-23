import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from spot_msgs.srv import SpotMotion

enter_lookup = False
odom_enter = False

class Navigate(Node):
    def __init__(self):
        super().__init__("navigate_to_pose")
        self.nav_client = ActionClient(self,NavigateToPose,"/navigate_to_pose")
        self.path_to_pose_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            print("Server still not available; waiting...")
        self.count = 1
        self.tf = TransformStamped()
        self.status = "success"

    def navigate(self):
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            print("\nServer still not available; waiting...")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = 10.459-1.14
        goal.pose.pose.position.y = 7.780
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = 0.0
        goal.pose.pose.orientation.y = 0.0
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0
    
        send_goal_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
                return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            return False

def execute(self, inputs, outputs, gvm):    
    try:
        rclpy.init()
    except:
        pass
    navigate = Navigate()
    var = navigate.navigate()
    if var :
        return 0 
    else:
        return -1

