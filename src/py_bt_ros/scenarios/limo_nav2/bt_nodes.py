import rclpy
import py_trees
import py_trees_ros
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# ---------------------------------------------------------
# 1. Main Node (데이터 공유용)
# ---------------------------------------------------------
class BTNode(Node):
    def __init__(self):
        super().__init__("bt_controller_node")
        
        # [중요] 현재 목표를 기억할 변수
        self.current_goal_msg = None 
        self.new_goal_received = False
        
        # RViz에서 찍는 목표(/goal_pose)를 가로채서 듣기
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

    def goal_callback(self, msg):
        self.get_logger().info(f"📍 새로운 목표 수신: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.current_goal_msg = msg
        self.new_goal_received = True

# ---------------------------------------------------------
# 2. Condition: 장애물 감지
# ---------------------------------------------------------
class IsObstacleNear(py_trees.behaviour.Behaviour):
    def __init__(self, name, topic_name="/scan", threshold=0.45):
        super(IsObstacleNear, self).__init__(name=name)
        self.topic_name = topic_name
        self.threshold = threshold
        self.node = None
        self.scan_data = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        qos = QoSProfile(depth=10)
        self.node.create_subscription(LaserScan, self.topic_name, self.callback, qos)

    def callback(self, msg):
        self.scan_data = msg

    def update(self):
        if self.scan_data is None:
            return py_trees.common.Status.FAILURE

        # 전방 60도 (중앙 기준 좌우 30개 데이터)
        num_ranges = len(self.scan_data.ranges)
        mid_idx = num_ranges // 2
        window = 30 
        
        # 유효 데이터 필터링 (0.01m ~ 100m)
        ranges = [r for r in self.scan_data.ranges[mid_idx-window : mid_idx+window] if r > 0.01]
        
        if not ranges:
            return py_trees.common.Status.FAILURE

        min_dist = min(ranges)

        if min_dist < self.threshold:
            # 장애물이 있으면 SUCCESS -> 상위에서 정지 로직 발동
            self.node.get_logger().info(f"🚨 장애물 발견! 거리: {min_dist:.2f}m")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

# ---------------------------------------------------------
# 3. Action: 로봇 정지
# ---------------------------------------------------------
class StopRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(StopRobot, self).__init__(name=name)
        self.publisher = None
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)

    def update(self):
        msg = Twist()
        # 0.0을 지속적으로 보내서 강제 정지 유지
        self.publisher.publish(msg)
        return py_trees.common.Status.SUCCESS

# ---------------------------------------------------------
# 4. Action: Nav2 주행 (동적 목표 처리)
# ---------------------------------------------------------
class Nav2DynamicGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Nav2DynamicGoal, self).__init__(name=name)
        self.node = None
        self.action_client = None
        self.goal_handle = None
        self.sent_goal = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.action_client = rclpy.action.ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        
        self.node.get_logger().info("Nav2 서버 연결 대기 중...")
        self.action_client.wait_for_server()
        self.node.get_logger().info("Nav2 연결 완료! RViz에서 목표를 설정하세요.")

    def initialise(self):
        # 트리가 다시 이 노드로 돌아왔을 때 (장애물 회피 후 복귀 시)
        # 만약 이미 목표를 보내놓고 달리는 중이었다면 재전송 방지
        pass

    def update(self):
        # 1. 목표가 아예 설정되지 않았으면 대기 (Idle)
        if self.node.current_goal_msg is None:
            return py_trees.common.Status.FAILURE

        # 2. 새로운 목표가 들어왔거나(RViz 클릭), 장애물 때문에 멈췄다가 다시 시작해야 하는 경우
        if self.node.new_goal_received or not self.sent_goal:
            return self.send_new_goal()

        # 3. 이미 주행 중이라면 상태 유지
        return py_trees.common.Status.RUNNING

    def send_new_goal(self):
        # 목표 메시지 구성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.node.current_goal_msg.pose
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map" # 맵 좌표계 기준

        self.node.get_logger().info(f"🚀 Nav2 목표 전송/재개")
        
        send_future = self.action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)
        
        self.sent_goal = True
        self.node.new_goal_received = False # 새 목표 처리 완료 플래그
        return py_trees.common.Status.RUNNING
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('❌ 목표가 거부되었습니다.')
            return
        self.goal_handle = goal_handle
        
        # 결과 대기 (도착 확인용)
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info('🎉 도착 완료!')
            self.node.current_goal_msg = None # 목표 달성했으므로 초기화
            self.sent_goal = False

    def terminate(self, new_status):
        # 장애물이 나타나서 이 노드가 취소될 때 (INVALID 상태로 변경됨)
        if new_status == py_trees.common.Status.INVALID and self.goal_handle:
            self.node.get_logger().info("⚠️ 장애물 회피를 위해 Nav2 일시 중지 (Cancel)")
            self.goal_handle.cancel_goal_async()
            self.sent_goal = False # 이렇게 해야 장애물이 사라지면 다시 목표를 보냄

# ---------------------------------------------------------
# 5. 트리 구성 및 실행
# ---------------------------------------------------------
def create_tree(ros_node):
    # Root: Selector (우선순위 결정)
    root = py_trees.composites.Selector(name="Hospital_Robot_Behavior", memory=False)

    # 1. [긴급] 장애물 회피 시퀀스
    # 장애물 감지 -> 정지 -> 3초 대기 (Wait Decorator 사용)
    obstacle_seq = py_trees.composites.Sequence(name="Obstacle_Response", memory=True)
    
    check_obstacle = IsObstacleNear(name="Check_Obstacle", threshold=0.45)
    stop_action = StopRobot(name="Stop_Immediately")
    
    # 3초 대기를 위한 Timer 데코레이터 적용
    wait_stop = py_trees.decorators.Timeout(
        child=stop_action,
        duration=3.0
    )
    # 주의: Timeout은 시간 지나면 Failure를 낼 수 있음. 
    # 간단히: 장애물 있으면 -> StopRobot(Success) 계속 실행됨 -> 장애물 없어지면 -> Nav2 실행
    
    obstacle_seq.add_children([check_obstacle, stop_action])

    # 2. [기본] Nav2 주행
    nav_behavior = Nav2DynamicGoal(name="Nav2_Dynamic_Goal")

    root.add_children([obstacle_seq, nav_behavior])
    return root

def main(args=None):
    rclpy.init(args=args)
    
    # 커스텀 노드 생성
    node = BTNode()
    root = create_tree(node)
    
    # 트리 셋업
    py_trees.trees.BehaviourTree(root).setup(node=node)

    try:
        # 주기 실행 (10Hz)
        while rclpy.ok():
            root.tick_once()
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
