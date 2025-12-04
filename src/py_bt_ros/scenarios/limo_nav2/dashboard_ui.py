#!/usr/bin/env python3
import faulthandler
faulthandler.enable()

import os
import sys
import time
import threading
import queue
import json
import requests
import tkinter as tk

# [중요] GUI 충돌 방지
os.environ["GDK_BACKEND"] = "x11"

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# ==========================================
# [설정] API 키 및 의사용 폼 ID
# ==========================================
API_KEY = "e57cc1f435fe873f0fdf8ada20298ba1"
DOCTOR_FORM_ID = "253293055163051"  # 의사용 문진표 ID (QR코드가 연결된 곳)

# [중요] 의사 폼의 필드 ID (JotForm에서 확인 필요, 틀리면 데이터 안 뜸)
# URL 예시: ...?input_3=환자ID
FIELD_PATIENT_ID = "3"      # 환자 고유 번호 (input_3)
FIELD_DIAGNOSIS = "4"       # 진료 소견 (input_4 라고 가정 - 텍스트상자)
FIELD_DEPT = "5"            # 진료과 (input_5 라고 가정 - 드롭다운)

class DoctorDashboard:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("의료진 통합 대시보드")
        self.root.geometry("400x600")
        self.root.configure(bg="white")

        self.running = True
        self.last_submission_id = None
        self.target_patient_id = None # 현재 진료 중인 환자 ID

        # ROS Publishers
        self.pub_record = self.node.create_publisher(String, '/medical_record', 10)
        self.pub_finish = self.node.create_publisher(Bool, '/exam_finished', 10)
        
        # ROS Subscribers
        self.create_ui()
        
        # 의사 폼 감시 스레드 시작
        self.monitor_thread = threading.Thread(target=self.loop_check_doctor_form, daemon=True)
        self.monitor_thread.start()

    def create_ui(self):
        tk.Label(self.root, text="JotForm 데이터 대기 중...", font=("Helvetica", 16, "bold"), bg="white").pack(pady=20)
        
        self.lbl_status = tk.Label(self.root, text="수신된 환자 ID: -", font=("Helvetica", 14), fg="blue", bg="white")
        self.lbl_status.pack(pady=10)

        self.txt_log = tk.Text(self.root, height=15, width=40, font=("Helvetica", 10))
        self.txt_log.pack(padx=20, pady=20)
        self.txt_log.insert(tk.END, "[모바일에서 작성된 진료 소견]\n핸드폰으로 QR을 스캔하여 소견서를 제출하면\n여기에 자동으로 내용이 뜹니다.\n\n")

    def loop_check_doctor_form(self):
        """ 주기적으로 의사 폼(JotForm)을 확인하여 새 제출이 있는지 감시 """
        print(">>> [Dashboard] 의사 소견서 감시 시작...")
        
        while self.running:
            try:
                # 1. JotForm API로 최신 제출물 1개 가져오기
                url = f"https://api.jotform.com/form/{DOCTOR_FORM_ID}/submissions?apiKey={API_KEY}&limit=1&orderby=created_at"
                response = requests.get(url, timeout=5)

                if response.status_code == 200:
                    data = response.json().get("content", [])
                    if data:
                        submission = data[0]
                        sub_id = submission.get("id")
                        
                        # 새로운 제출이 발견되면 처리
                        if sub_id != self.last_submission_id:
                            self.last_submission_id = sub_id
                            self.process_submission(submission)
                
            except Exception as e:
                print(f"[API Error] {e}")
            
            time.sleep(3) # 3초마다 확인

    def process_submission(self, sub):
        """ 의사가 제출한 데이터를 분석해서 ROS로 전송 """
        answers = sub.get("answers", {})
        
        # 1. 데이터 추출 (ID값은 폼 설정에 따라 다를 수 있음)
        p_id = answers.get(FIELD_PATIENT_ID, {}).get("answer", "Unknown")
        diagnosis = answers.get(FIELD_DIAGNOSIS, {}).get("answer", "소견 없음")
        dept = answers.get(FIELD_DEPT, {}).get("answer", "일반의")

        # UI 업데이트 (메인 스레드에서 실행되도록 after 사용 권장이나 간단히 처리)
        log_msg = f"------------------------\n[수신] 환자 ID: {p_id}\n과목: {dept}\n소견: {diagnosis}\n"
        self.txt_log.insert(tk.END, log_msg)
        self.txt_log.see(tk.END)
        self.lbl_status.config(text=f"수신된 환자 ID: {p_id}")

        # 2. 로봇/환자 UI로 데이터 전송 (ROS2)
        # JSON 형태로 묶어서 보냄
        record_data = {
            "id": p_id,
            "dept": dept,
            "diagnosis": diagnosis,
            "timestamp": time.time()
        }
        
        # (A) 환자용 키오스크 화면 업데이트용
        self.pub_record.publish(String(data=json.dumps(record_data)))
        print(f">>> [ROS 전송] /medical_record: {p_id} 진료 완료")

        # (B) 로봇에게 다음 동작 지시 (예: 진료 끝났으니 복귀해라)
        self.pub_finish.publish(Bool(data=True))
        print(f">>> [ROS 전송] /exam_finished: True")

# ==========================================
# ROS 스레드 및 메인 실행
# ==========================================
def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = Node('doctor_dashboard_node')
    
    root = tk.Tk()
    app = DoctorDashboard(root, node)
    
    t = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    t.start()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        app.running = False
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        try:
            root.destroy()
        except:
            pass

if __name__ == '__main__':
    main()
