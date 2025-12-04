import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import sys, termios, tty

class LimoSpeedController(Node):
    def __init__(self):
        super().__init__('limo_speed_remote')
        self.cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        
        self.get_logger().info("⏳ Nav2 컨트롤러 연결 중...")
        # 1초 기다렸다가 안 되면 그냥 메시지 띄우고 진행 (무한 대기 방지)
        if not self.cli.wait_for_service(timeout_sec=2.0):
             self.get_logger().warn("⚠️ Nav2 컨트롤러 서버(/controller_server)를 찾을 수 없습니다. Nav2가 켜져 있나요?")
        else:
            self.get_logger().info("✅ 연결 완료! 키보드로 속도를 조절하세요.")

    def change_speed(self, speed):
        req = SetParameters.Request()
        # DWBLocalPlanner의 속도 파라미터
        param_name = "FollowPath.max_vel_x"
        
        val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(speed))
        req.parameters.append(Parameter(name=param_name, value=val))

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            result = future.result()
            if result.results[0].successful:
                print(f"\n🚀 속도 변경 완료: {speed} m/s")
            else:
                print(f"\n❌ 변경 실패. 파라미터 이름({param_name})이 맞는지 확인하세요.")
        except Exception as e:
            print(f"\n❌ 오류 발생: {e}")

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init()
    node = LimoSpeedController()

    print("""
    =========================================
    🎮 LIMO 실시간 속도 조절 리모컨
    =========================================
    [1] 🐢 안전 모드 (0.2 m/s)
    [2] 🚶 보통 모드 (0.4 m/s)
    [3] 🐇 고속 모드 (0.7 m/s)
    [q] 종료
    =========================================
    """)

    try:
        while True:
            key = get_key()
            if key == '1': node.change_speed(0.2)
            elif key == '2': node.change_speed(0.4)
            elif key == '3': node.change_speed(0.7)
            elif key == 'q': break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()