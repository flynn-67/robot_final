#!/usr/bin/env python3
import tkinter as tk
from tkinter import messagebox, scrolledtext
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import threading
import queue
import requests
import time

# ==========================================
# [설정] 의사용 JotForm 정보 (제공해주신 ID 적용됨)
# ==========================================
API_KEY = "e57cc1f435fe873f0fdf8ada20298ba1"  
DOCTOR_FORM_ID = "253293055163051"           # 의사 소견서 폼 ID

# 질문 번호 (Unique Name이 input_3 이면 ID는 보통 "3" 입니다)
FIELD_ID_PATIENT_NUM = "3" 

# 진료 소견 질문 번호 (보통 그 다음 번호인 "4"일 확률이 높음)
# 만약 소견이 안 뜨면 터미널 로그를 확인해서 번호를 수정하세요.
FIELD_ID_OPINION = "4" 
# ==========================================

class DoctorDashboard:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title("👨‍⚕️ 의료진 통합 대시보드")
        self.root.geometry("600x600")
        
        self.current_patient_id = None
        self.last_submission_id = None
        self.event_queue = queue.Queue()
        self.running = True

        self.setup_ui()

        # ROS 통신
        self.pub_record = self.node.create_publisher(String, '/medical_record', 10)
        self.pub_finish = self.node.create_publisher(Bool, '/exam_finished', 10)
        
        # JotForm 감시 스레드 시작
        self.jotform_thread = threading.Thread(target=self.loop_check_jotform, daemon=True)
        self.jotform_thread.start()

        self.root.after(100, self.update_loop)
        print(">>> [Doctor UI] 실행됨 (JotForm 감시 중)")

    def setup_ui(self):
        # 상태바
        self.lbl_status = tk.Label(self.root, text="모바일 소견서 대기 중...", font=("Arial", 16, "bold"), bg="lightgray", pady=10)
        self.lbl_status.pack(fill="x")

        # 환자 정보
        frame_info = tk.Frame(self.root, pady=10)
        frame_info.pack()
        tk.Label(frame_info, text="수신된 환자 ID:", font=("Arial", 12)).pack(side="left")
        self.lbl_patient_id = tk.Label(frame_info, text="-", font=("Arial", 14, "bold"), fg="blue")
        self.lbl_patient_id.pack(side="left", padx=10)

        # 소견서 작성 영역
        tk.Label(self.root, text="[모바일에서 작성된 진료 소견]", font=("Arial", 12, "bold"), fg="#4f46e5").pack(pady=(20, 5))
        
        self.txt_diagnosis = scrolledtext.ScrolledText(self.root, height=12, width=55, font=("Arial", 11))
        self.txt_diagnosis.pack(padx=20)
        
        self.txt_diagnosis.insert("1.0", "핸드폰으로 QR을 스캔하여 소견서를 제출하면\n여기에 자동으로 내용이 뜹니다.")
        self.txt_diagnosis.config(state="disabled", bg="#f0f4f8") 

        # 버튼
        frame_btn = tk.Frame(self.root, pady=20)
        frame_btn.pack()
        self.btn_save = tk.Button(frame_btn, text="최종 확정 및 전송", bg="#00CC66", fg="white", 
                                  font=("Arial", 12, "bold"), state="disabled", command=self.send_report)
        self.btn_save.pack()

    def loop_check_jotform(self):
        """ 백그라운드에서 의사용 폼 제출을 주기적으로 확인 """
        while self.running:
            try:
                # 최신 제출 1개 가져오기
                url = f"https://api.jotform.com/form/{DOCTOR_FORM_ID}/submissions?apiKey={API_KEY}&limit=1&orderby=created_at"
                response = requests.get(url, timeout=5)
                
                if response.status_code == 200:
                    content = response.json().get("content", [])
                    if content:
                        sub = content[0]
                        sub_id = sub.get("id")
                        
                        # 새로운 제출이 발견되면
                        if sub_id != self.last_submission_id:
                            self.last_submission_id = sub_id
                            # 디버깅을 위해 전체 데이터를 출력 (ID 번호 확인용)
                            print(f"\n[DEBUG] JotForm 데이터 수신함. 데이터 구조:")
                            print(json.dumps(sub.get("answers", {}), indent=2, ensure_ascii=False))
                            
                            self.event_queue.put(("jotform_data", sub))
            except Exception as e:
                print(f"[API 오류] {e}")
            
            time.sleep(3) # 3초마다 확인

    def update_loop(self):
        try:
            while True:
                msg_type, data = self.event_queue.get_nowait()
                if msg_type == "jotform_data":
                    self.display_jotform_data(data)
        except queue.Empty: pass
        self.root.after(100, self.update_loop)

    def display_jotform_data(self, submission):
        """ 수신된 데이터를 화면에 표시 """
        answers = submission.get("answers", {})
        
        # 1. 환자 ID 파싱
        p_id = answers.get(FIELD_ID_PATIENT_NUM, {}).get("answer", "Unknown")
        # 2. 의사 소견 파싱
        opinion = answers.get(FIELD_ID_OPINION, {}).get("answer", "")

        self.current_patient_id = p_id
        
        # UI 업데이트
        self.lbl_status.config(text="✅ 모바일 소견서 수신 완료!", bg="#D1FAE5", fg="#065F46")
        self.lbl_patient_id.config(text=self.current_patient_id)
        
        self.txt_diagnosis.config(state="normal", bg="white") 
        self.txt_diagnosis.delete("1.0", tk.END)
        self.txt_diagnosis.insert(tk.END, opinion)
        
        self.btn_save.config(state="normal") 
        
        messagebox.showinfo("알림", f"환자(ID: {p_id})의 소견서가 도착했습니다.\n[최종 확정]을 눌러 로봇을 복귀시키세요.")

    def send_report(self):
        diagnosis = self.txt_diagnosis.get("1.0", tk.END).strip()
        if not diagnosis:
            messagebox.showwarning("경고", "내용이 비어있습니다.")
            return

        # ROS 데이터 전송
        record = {"id": self.current_patient_id, "dept": "내과", "diagnosis": diagnosis}
        self.pub_record.publish(String(data=json.dumps(record)))
        self.pub_finish.publish(Bool(data=True)) # 진료 완료 신호
        
        messagebox.showinfo("성공", "환자에게 최종 소견서를 전송했습니다.")
        
        # 초기화
        self.lbl_status.config(text="다음 데이터 대기 중...", bg="lightgray", fg="black")
        self.lbl_patient_id.config(text="-")
        self.txt_diagnosis.delete("1.0", tk.END)
        self.txt_diagnosis.insert("1.0", "대기 중...")
        self.txt_diagnosis.config(state="disabled", bg="#f0f4f8")
        self.btn_save.config(state="disabled")
        self.current_patient_id = None

def ros_thread(node): rclpy.spin(node)

def main():
    if not rclpy.ok(): rclpy.init()
    node = Node('doctor_dashboard_node')
    root = tk.Tk()
    app = DoctorDashboard(root, node)
    
    t = threading.Thread(target=ros_thread, args=(node,), daemon=True)
    t.start()
    
    try: root.mainloop()
    except KeyboardInterrupt: pass
    finally:
        app.running = False
        if rclpy.ok(): node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
