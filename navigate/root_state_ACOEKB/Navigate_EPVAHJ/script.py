import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

def navigate(node,client,t):
    while not client.wait_for_server(timeout_sec=2.0):
        print("\nServer still not available; waiting...")

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = t.transform.translation.x-0.5
    goal.pose.pose.position.y = t.transform.translation.y-2.6
    goal.pose.pose.position.z = 0.0
    goal.pose.pose.orientation = t.transform.rotation

    send_goal_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    if not goal_handle.accepted:
            return False
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    status = result_future.result().status
    if status == GoalStatus.STATUS_SUCCEEDED:
        return True
    else:
        return False

def execute(self, inputs, outputs, gvm):
    node_handle = gvm.get_variable('node',per_reference=True)
    try:
        if gvm.variable_exist("ros_node_initialized") or gvm.get_variable("ros_node_initialized"):
            t=inputs['tf_nav']
            nav_client = ActionClient(node_handle,NavigateToPose,"/navigate_to_pose")
            while not nav_client.wait_for_server(timeout_sec=2.0):
                self.logger.info("Server still not available; waiting...")
            var = navigate(node_handle,nav_client,t)
            if var:
                return 0
            else:
                return -1
    except Exception as e:
        self.logger.error("Unexpected error:"+str(e))
        return -1