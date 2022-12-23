import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped

enter_lookup = False
odom_enter = False
global num

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
        global num
        while not self.nav_client.wait_for_server(timeout_sec=2.0):
            print("\nServer still not available; waiting...")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        print(num)
        if num == 5:
            goal.pose.pose.position.x = self.tf.transform.translation.x-1.3
            goal.pose.pose.position.y = self.tf.transform.translation.y
            goal.pose.pose.position.z = 0.0
            goal.pose.pose.orientation = self.tf.transform.rotation
        elif num == 4 or num == 3:
            goal.pose.pose.position.x = self.tf.transform.translation.x
            goal.pose.pose.position.y = self.tf.transform.translation.y - 1.3
            goal.pose.pose.position.z = 0.0
            goal.pose.pose.orientation.x = 0.0
            goal.pose.pose.orientation.y = 0.0
            goal.pose.pose.orientation.z = 0.7071068
            goal.pose.pose.orientation.w = 0.7071068
        elif num == 1:
            goal.pose.pose.position.x = self.tf.transform.translation.x + 1.3
            goal.pose.pose.position.y = self.tf.transform.translation.y
            goal.pose.pose.position.z = 0.0
            goal.pose.pose.orientation.x = 0.0
            goal.pose.pose.orientation.y = 0.0
            goal.pose.pose.orientation.z = 1.0
            goal.pose.pose.orientation.w = 0.0

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
    global num
    num = inputs['can_num']
    try:
        rclpy.init()
    except:
        pass
    navigate = Navigate()
    navigate.tf = inputs['tf']
    var = navigate.navigate()
    if var:
        return 0
    else:
        return -1
