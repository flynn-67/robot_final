import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# --- 설정값 (병원 환경에 맞춰 조정하세요) ---
OBSTACLE_LIMIT = 0.40  # 40cm 이내 장애물 감지 시 정지
STOP_DURATION = 3.0    # 장애물 감지 시 3초간 대기
GOAL_X = 2.0           # 목표 지점 X 좌표 (맵 기준)
GOAL_Y = 0.0           # 목표 지점 Y 좌표 (맵 기준)
# ----------------------------------------

class HospitalBot(Node):
    def __init__(self):
        super().__init__('hospital_nurse_bot')
        
        # 1. 장애물 감지를 위한 라이다 구독
        qos = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        
        # 2. 강제 정지를 위한 속도 명령 발행
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 3. 상태 플래그
        self.is_danger = False

    def lidar_callback(self, msg):
        # 전방 60도 부채꼴 범위 내의 가장 가까운 장애물 찾기
        # LIMO 라이다 데이터 배열의 중간 지점이 정면이라고 가정
        mid_index = len(msg.ranges) // 2
        range_width = 30  # 좌우 데이터 개수 (범위)
        
        # 전방 데이터 슬라이싱 (유효하지 않은 0.0 값 제외)
        front_ranges = [r for r in msg.ranges[mid_index-range_width : mid_index+range_width] if r > 0.01]
        
        if not front_ranges:
            return

        min_dist = min(front_ranges)
        
        # 설정한 거리보다 가까우면 위험 신호
        if min_dist < OBSTACLE_LIMIT:
            self.is_danger = True
        else:
            self.is_danger = False

    def force_stop(self):
        # 로봇을 그 자리에 즉시 멈춤
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)

def main():
    rclpy.init()
    
    # 노드와 Nav2 네비게이터 생성
    bot = HospitalBot()
    navigator = BasicNavigator()

    # Nav2가 켜질 때까지 대기
    print("🏥 병원 로봇 시스템: Nav2 연결 대기 중...")
    navigator.waitUntilNav2Active()
    print("✅ Nav2 연결 완료! 임무를 시작합니다.")

    # 목표 지점 설정
    goal_pose = navigator.getPoseStamped([GOAL_X, GOAL_Y], 0.0)
    
    # 최초 주행 시작
    navigator.goToPose(goal_pose)
    current_goal = goal_pose  # 현재 목표 기억

    while rclpy.ok():
        rclpy.spin_once(bot, timeout_sec=0.1) # 센서 데이터 업데이트

        # [핵심 로직] 주행 중 장애물이 발견되면?
        if not navigator.isTaskComplete():
            if bot.is_danger:
                print(f"🚨 장애물 감지! ({OBSTACLE_LIMIT}m 이내)")
                
                # 1. 현재 Nav2 주행 취소 (멈춰!)
                navigator.cancelTask()
                
                # 2. 확실하게 정지 명령 전송 (미끄러짐 방지)
                for _ in range(10):
                    bot.force_stop()
                    time.sleep(0.05)
                
                # 3. 지정된 시간만큼 대기 (3초)
                print(f"⏳ {STOP_DURATION}초간 대기합니다...")
                time.sleep(STOP_DURATION)
                
                # 4. 장애물이 여전한지 체크 (선택사항) 후 재주행
                print("▶️ 다시 주행을 시도합니다.")
                navigator.goToPose(current_goal)
                
                # 재주행 명령이 들어갈 때까지 잠시 대기
                time.sleep(1.0)
        
        # 도착 여부 확인
        elif navigator.getResult() == TaskResult.SUCCEEDED:
            print("🎉 목적지에 도착했습니다!")
            break

    bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()