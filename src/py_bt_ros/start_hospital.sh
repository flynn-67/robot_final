#!/bin/bash

cleanup() {
    echo ""
    echo "🛑 종료 중... 프로세스를 정리합니다."
    kill $REPORT_PID 2>/dev/null
    kill $DASH_PID 2>/dev/null
    exit
}
trap cleanup SIGINT

echo "=============================================="
echo "   🏥 스마트 병원 시스템 부팅   "
echo "=============================================="

# 1. 환자용 키오스크 (Patient UI) 먼저 실행 -> 화면에 바로 뜸
echo "📄 [1/3] 환자 키오스크 실행 중... (화면에 표시됨)"
python3 scenarios/limo_nav2/patient_report_ui.py &
REPORT_PID=$!

sleep 1

# 2. 의료진 대시보드 (Doctor UI) 실행 -> 백그라운드에서 숨김 상태로 대기
echo "👨‍⚕️ [2/3] 의료진 대시보드 준비 중... (시작 신호 대기)"
python3 scenarios/limo_nav2/dashboard_ui.py &
DASH_PID=$!

sleep 1

# 3. 메인 프로그램 실행
echo "🤖 [3/3] 리모(Limo) 두뇌 가동! (Ctrl+C로 종료)"
echo "----------------------------------------------"
echo " 👉 환자 화면에서 '접수 시작' -> '작성 완료' -> '로봇 연동'을 누르면 의사 화면이 뜹니다."
echo "----------------------------------------------"
python3 main.py

# 4. 속도 조절 리모컨 실행
echo "🎮 속도 조절 리모컨을 실행합니다."
# 기존 py_bt_ros가 아니라, 새로 만든 'limo_controller' 패키지를 실행합니다.
ros2 run limo_controller speed_ctrl

cleanup