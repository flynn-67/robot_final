import rclpy
import json
import math
import time
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose
from action_msgs.msg import GoalStatus

import sys
import os
import asyncio 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../')))

from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback
from modules.base_bt_nodes_ros import ActionWithROSAction

BTNodeList.ACTION_NODES.extend([
    'NavigateToPoseNode',
    'SelectBestHospital',
    'WaitForButton',
    'WaitForStart',
    'SetHomeTarget',
    'StopRobot',        # ✅ 추가
])

BTNodeList.CONDITION_NODES.extend([
    'CheckStringValue',
    'ObstacleNear',     # ✅ 추가
])


#좌표만 바꾸면 됨
HOSPITAL_COORDS = {
    "내과": {"x": 6.455, "y": 2.609},
    "외과": {"x": 7.358, "y": 0.297},
    "이비인후과": {"x": 5.0, "y": 1.0},
    "치과": {"x": 2.0, "y": 2.0}
}
# -----------------------------
# 장애물 감지 + 정지용 노드들
# -----------------------------
OBSTACLE_LIMIT = 0.45   # 0.45m 이내면 앞에 장애물 있다고 판단 (원하면 조절 가능)


class ObstacleNear(Node):
    """
    /scan 을 구독해서 전방에 장애물이 가까이 있으면 SUCCESS, 없으면 FAILURE 를 리턴하는 Condition 노드
    """
    def __init__(self, tag_name, agent, name=None):
        super().__init__(name if name else tag_name)
        self.agent = agent
        self.node = agent.ros_bridge.node

        # 라이다 구독
        self.sub = self.node.create_subscription(
            LaserScan, '/scan', self.lidar_cb, 10
        )

        self.is_danger = False

    def lidar_cb(self, msg: LaserScan):
        # 정면 기준 +/- 30개 데이터만 사용 (총 60개)
        mid_index = len(msg.ranges) // 2
        range_width = 30

        front = msg.ranges[mid_index - range_width: mid_index + range_width]

        # 0.0 이거나 inf 같은 값은 제외
        valid = [r for r in front if r > 0.01]

        if not valid:
            self.is_danger = False
            return

        min_dist = min(valid)

        if min_dist < OBSTACLE_LIMIT:
            self.is_danger = True
        else:
            self.is_danger = False

    async def run(self, agent, blackboard):
        # BT에서 매 tick마다 불림
        if self.is_danger:
            # 장애물 가까이 있으면 Sequence 에서 SUCCESS 처리
            return Status.SUCCESS
        # 장애물 없으면 FAILURE → Fallback 구조에서 "다음 자식(이동)" 실행
        return Status.FAILURE
    
class StopRobot(Node):
    """
    /cmd_vel 로 0 속도 Twist 를 계속 발행해서 로봇을 강제로 멈추는 Action 노드
    """
    def __init__(self, tag_name, agent, name=None):
        super().__init__(name if name else tag_name)
        self.agent = agent
        self.node = agent.ros_bridge.node

        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.last_log_time = 0.0

    async def run(self, agent, blackboard):
        # 속도 0 명령 발행
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        # 1초에 한 번만 로그 찍기 (스팸 방지)
        now = time.time()
        if now - self.last_log_time > 1.0:
            self.node.get_logger().info(f"[{self.name}] 장애물 감지 → 로봇 정지 중...")
            self.last_log_time = now

        # 장애물이 있는 동안에는 계속 RUNNING 으로 유지
        return Status.RUNNING


class WaitForStart(Node):
    def __init__(self, tag_name, agent, name=None):
        super().__init__(name if name else tag_name)
        self.agent = agent
        self.sub = agent.ros_bridge.node.create_subscription(String, '/hospital_data', self.cb, 10)
        self.started = False

    def cb(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('command') == 'start':
                self.started = True
        except:
            pass

    async def run(self, agent, blackboard):
        if self.started:
            self.started = False
            if not hasattr(agent, 'blackboard'): agent.blackboard = {}
            agent.blackboard['visited_hospitals'] = []
            
            pub = agent.ros_bridge.node.create_publisher(Bool, '/exam_finished', 10)
            pub.publish(Bool(data=False))
            
            agent.ros_bridge.node.get_logger().info("🔔 시작 신호 수신! 시스템 초기화 완료.")
            return Status.SUCCESS
        return Status.RUNNING

class SelectBestHospital(Node):
    def __init__(self, tag_name, agent, name=None):
        super().__init__(name if name else tag_name)
        self.agent = agent
        self.ros_node = agent.ros_bridge.node
        self.sub = self.ros_node.create_subscription(String, '/hospital_data', self.data_cb, 10)
        self.latest_data = {}

    def data_cb(self, msg):
        try:
            data = json.loads(msg.data)
            if 'command' in data: del data['command']
            self.latest_data = data
        except:
            pass

    async def run(self, agent, blackboard):
        visited = blackboard.get('visited_hospitals', [])
        if not self.latest_data: return Status.FAILURE

        candidates = {}
        for name, count in self.latest_data.items():
            if name not in visited:
                candidates[name] = count

        if not candidates: return Status.FAILURE

        best_hospital = min(candidates, key=candidates.get)
        wait_count = candidates[best_hospital]
        target_xy = HOSPITAL_COORDS[best_hospital]
        
        blackboard['target_x'] = target_xy['x']
        blackboard['target_y'] = target_xy['y']
        blackboard['current_hospital_name'] = best_hospital.strip()

        visited.append(best_hospital)
        blackboard['visited_hospitals'] = visited

        self.ros_node.get_logger().info(f"\n[결정] ✨ {best_hospital} 선택! (대기: {wait_count}명) -> 출발")
        return Status.SUCCESS

class SetHomeTarget(Node):
    def __init__(self, tag_name, agent, name=None):
        super().__init__(name if name else tag_name)
    
    async def run(self, agent, blackboard):
        if not hasattr(agent, 'blackboard'): agent.blackboard = {}
        agent.blackboard['target_x'] = 0.0
        agent.blackboard['target_y'] = 0.0
        
        # 이름 기억 삭제 (이러면 NavigateToPoseNode가 집으로 인식)
        agent.blackboard['current_hospital_name'] = None 
        
        return Status.SUCCESS

class NavigateToPoseNode(Node): 
    def __init__(self, tag_name, agent, name=None):
        actual_name = name if name else tag_name
        super().__init__(actual_name)
        self.ros_node = agent.ros_bridge.node
        self.timer = 0
        
        self.pub_hospital = self.ros_node.create_publisher(String, '/current_hospital', 10)
        self.pub_finish = self.ros_node.create_publisher(Bool, '/exam_finished', 10)

    async def run(self, agent, blackboard):
        tx = getattr(agent, 'blackboard', {}).get('target_x', 0)
        ty = getattr(agent, 'blackboard', {}).get('target_y', 0)
        
        if self.timer == 0:
            self.ros_node.get_logger().info(f"[{self.name}] 🚀 이동 시작... ({tx}, {ty})")
        
        self.timer += 1
        
        if self.timer > 15: # 도착 시점
            hospital_name = getattr(agent, 'blackboard', {}).get('current_hospital_name', None)
            
            self.ros_node.get_logger().info(f"[{self.name}] ✨ 도착 완료!")
            
            if hospital_name: 
                # [상황 1] 병원 도착 -> 화면 전환 신호 (1회 전송)
                msg = String()
                msg.data = str(hospital_name).strip()
                print(f" >>> [BT] 병원 도착! 화면 전환 신호 전송 ({msg.data})")
                self.pub_hospital.publish(msg)

            else:
                # [상황 2] 집 도착 -> 최종 결과 신호 (1회 전송)
                msg = Bool(data=True)
                print(f"\n >>> [BT] 🏠 집 도착 완료! 최종 결과표 팝업 신호 전송! \n")
                self.pub_finish.publish(msg)

            self.timer = 0
            return Status.SUCCESS
            
        return Status.RUNNING

class WaitForButton(Node):
    def __init__(self, tag_name, agent, name=None, topic_name='/doctor_confirm'):
        actual_name = name if name else tag_name
        super().__init__(actual_name)
        self.topic_name = topic_name
        self.sub = agent.ros_bridge.node.create_subscription(Bool, topic_name, self.cb, 10)
        self.is_pressed = False
        self.has_reset = False

    def cb(self, msg):
        if msg.data: self.is_pressed = True

    async def run(self, agent, blackboard):
        if not self.has_reset:
            self.is_pressed = False
            self.has_reset = True
            return Status.RUNNING
        if self.is_pressed:
            return Status.SUCCESS
        return Status.RUNNING

class CheckStringValue(Node):
    async def run(self, a, b): return Status.FAILURE
