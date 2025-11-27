import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time

def main():
    rclpy.init()
    
    # 1. 네비게이터 객체 생성
    navigator = BasicNavigator()

    # 2. Nav2 활성화 대기
    print("Nav2가 준비될 때까지 대기 중입니다... (RViz에서 초기 위치를 잡아주세요)")
    navigator.waitUntilNav2Active()

    # 3. 이동할 웨이포인트 목록 설정
    points = [
        (6.455062389373779, 2.609785318374634),   # 첫 번째 경유지
        (7.358321666717529, 0.2972927689552307),   # 두 번째 경유지
        (0.0, 0.0)    # 다시 원점(시작점)으로 복귀
    ]

    # PoseStamped 메시지 리스트로 변환
    goal_poses = []
    for x, y in points:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        goal_poses.append(goal_pose)

    print("\n" + "="*40)
    print(f"총 {len(goal_poses)}개의 웨이포인트가 준비되었습니다.")
    print("이제부터 각 구간마다 엔터 키를 눌러야 이동합니다.")
    print("="*40 + "\n")

    # 4. 반복문으로 하나씩 이동 처리
    for i, goal_pose in enumerate(goal_poses):
        # --- [수정된 부분] 사용자 입력 대기 ---
        input(f">>> {i+1}번째 웨이포인트로 출발하려면 [Enter] 키를 누르세요 <<<")
        
        print(f"이동 시작: {i+1}번 목표지점으로 가는 중...")
        
        # followWaypoints 대신 goToPose 사용 (한 지점만 이동)
        navigator.goToPose(goal_pose)

        # 이동 완료될 때까지 대기
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            # 필요하면 남은 거리 출력 (너무 자주 출력되지 않게 주석 처리함)
            # print(f'남은 거리: {feedback.distance_remaining:.2f}m')
            # time.sleep(0.1) 

        # 결과 확인
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"★ {i+1}번 웨이포인트 도착 완료! ★\n")
        else:
            print(f"ERROR: {i+1}번 이동 중 실패 또는 취소되었습니다. (상태 코드: {result})")
            # 실패 시 프로그램을 종료하고 싶으면 아래 주석 해제
            # break 

    print("모든 일정이 종료되었습니다.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()