#!/bin/bash

# ==========================================
# [중요] 스크립트 시작 전 필수 설정


cleanup() {
    echo ""
    echo "🛑 종료 중... 프로세스를 정리합니다."
    if [ -n "$REPORT_PID" ]; then kill $REPORT_PID 2>/dev/null; fi
    if [ -n "$DASH_PID" ]; then kill $DASH_PID 2>/dev/null; fi
    exit
}
trap cleanup SIGINT

echo "=============================================="
echo "    🏥 스마트 병원 시스템 부팅    "
echo "=============================================="
echo "📂 실행 위치: $(pwd)" 
echo "----------------------------------------------"

# 1. 환자용 키오스크 (Patient UI)
echo "📄 [1/3] 환자 키오스크 실행 중..."
# 파일이 있는지 확인 후 실행
if [ -f "scenarios/limo_nav2/patient_report_ui.py" ]; then
    python3 scenarios/limo_nav2/patient_report_ui.py &
    REPORT_PID=$!
else
    echo "❌ 에러: patient_report_ui.py 파일을 찾을 수 없습니다."
fi

# 2. 의료진 대시보드 (Doctor UI)
echo "👨‍⚕️ [2/3] 의료진 대시보드 준비 중..."
if [ -f "scenarios/limo_nav2/dashboard_ui.py" ]; then
    python3 scenarios/limo_nav2/dashboard_ui.py &
    DASH_PID=$!
else
    echo "❌ 에러: dashboard_ui.py 파일을 찾을 수 없습니다."
fi

# 3. 메인 프로그램 실행
echo "🤖 [3/3] 리모(Limo) 두뇌 가동! (Ctrl+C로 종료)"
echo "----------------------------------------------"
if [ -f "main.py" ]; then
    python3 main.py
else
    echo "❌ 에러: main.py 파일을 찾을 수 없습니다."
fi

cleanup
