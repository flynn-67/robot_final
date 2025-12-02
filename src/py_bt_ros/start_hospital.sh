#!/bin/bash

echo "🏥 병원 진료 시스템을 시작합니다..."

# 1. 환자용 결과 보고서 실행
# (이제 실행하자마자 화면에 뜰 겁니다)
echo "📄 환자 결과 보고서(Patient UI) 실행 중..."
python3 scenarios/limo_nav2/patient_report_ui.py &
REPORT_PID=$!

# 2. 의사용 대시보드 실행
echo "👨‍⚕️ 의사 대시보드(Doctor UI) 실행 중..."
python3 scenarios/limo_nav2/dashboard_ui.py &
DASH_PID=$!

sleep 2

# 3. 로봇 두뇌 실행
echo "🤖 리모 두뇌(BT) 실행 중..."
python3 main.py

# 종료 처리
echo "🛑 시스템 종료 중... 모든 창을 닫습니다."
kill $REPORT_PID
kill $DASH_PID