#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import tkinter as tk
from tkinter import messagebox
import threading
import sys
import os

class DoctorDashboard:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("의료진용 컨트롤 패널")
        self.root.geometry("450x550")
        self.root.configure(bg="#333") 

        self.current_hospital = None
        
        # UI 구성
        tk.Label(root, text="👨‍⚕️ 진료실 대시보드", font=("Arial", 18, "bold"), bg="#333", fg="white").pack(pady=20)

        # 상태창
        self.status_frame = tk.Frame(root, bg="#444", padx=20, pady=20)
        self.status_frame.pack(fill="x", padx=20)
        self.lbl_loc = tk.Label(self.status_frame, text="상태: 시스템 대기 중", font=("Arial", 14), bg="#444", fg="#aaa")
        self.lbl_loc.pack()

        # 진료 입력창
        tk.Label(root, text="진료 소견 작성", font=("Arial", 12), bg="#333", fg="white", anchor="w").pack(fill="x", padx=20, pady=(20, 5))
        self.text_diagnosis = tk.Text(root, height=8, font=("Arial", 11))
        self.text_diagnosis.pack(fill="x", padx=20)
        self.text_diagnosis.config(state="disabled", bg="#eee")

        # 완료 버튼
        self.btn_confirm = tk.Button(root, text="진료 완료 및 내보내기", font=("Arial", 14, "bold"), 
                                     bg="#555", fg="white", state="disabled", command=self.complete_treatment)
        self.btn_confirm.pack(fill="x", padx=20, pady=20)

        self.lbl_emergency = tk.Label(root, text="", font=("Arial", 14, "bold"), bg="#333", fg="red")
        self.lbl_emergency.pack(pady=10)

        # ROS 통신
        self.sub_start = self.node.create_subscription(String, '/hospital_data', self.start_cb, 10)
        self.sub_loc = self.node.create_subscription(String, '/current_hospital', self.location_cb, 10)
        self.sub_emg = self.node.create_subscription(Bool, '/emergency', self.emergency_cb, 10)
        self.pub_confirm = self.node.create_publisher(Bool, '/doctor_confirm', 10) 
        self.pub_record = self.node.create_publisher(String, '/medical_record', 10) 

        # 처음 실행 시 창을 숨김
        self.root.withdraw()
        print(">>> [Doctor UI] 실행됨 (대기 상태 - 창 숨김)")

    # --- ROS 콜백 (스레드 안전 처리) ---
    def start_cb(self, msg):
        # 메인 스레드에서 UI 업데이트를 수행하도록 예약
        self.root.after(0, self._handle_start, msg)

    def _handle_start(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('command') == 'start':
                self.root.deiconify() # 창 띄우기 (메인 스레드에서 실행됨)
                self.lbl_loc.config(text="상태: 로봇 이동 중...", fg="#4f46e5")
                print(">>> [Doctor UI] 화면 표시됨")
        except: pass

    def location_cb(self, msg):
        self.root.after(0, self._handle_location, msg)

    def _handle_location(self, msg):
        self.current_hospital = msg.data.strip()
        self.root.deiconify() # 안전장치
        self.lbl_loc.config(text=f"환자 도착: {self.current_hospital}", fg="#10b981", font=("Arial", 16, "bold"))
        self.text_diagnosis.config(state="normal", bg="white")
        self.text_diagnosis.delete("1.0", tk.END)
        self.btn_confirm.config(state="normal", bg="#4f46e5")
        print(f">>> [Doctor] 환자 도착: {self.current_hospital}")

    def emergency_cb(self, msg):
        if msg.data:
            self.root.after(0, self._handle_emergency)

    def _handle_emergency(self):
        self.root.deiconify()
        self.lbl_emergency.config(text="🚨 긴급 호출 발생! 🚨")
        messagebox.showwarning("긴급", "환자 긴급 호출 발생!")

    def complete_treatment(self):
        diagnosis_text = self.text_diagnosis.get("1.0", tk.END).strip()
        if not diagnosis_text:
            messagebox.showwarning("경고", "진료 소견을 입력해주세요.")
            return

        # 기록 전송
        record_data = {"dept": self.current_hospital, "diagnosis": diagnosis_text}
        self.pub_record.publish(String(data=json.dumps(record_data)))

        # 로봇 출발 신호
        self.pub_confirm.publish(Bool(data=True))

        # UI 리셋
        self.lbl_loc.config(text=f"{self.current_hospital} 완료. 이동 중...", fg="#aaa", font=("Arial", 14))
        self.text_diagnosis.delete("1.0", tk.END)
        self.text_diagnosis.config(state="disabled", bg="#eee")
        self.btn_confirm.config(state="disabled", bg="#555")
        self.lbl_emergency.config(text="")
        print(f">>> [Doctor] 진료 완료 처리됨 ({self.current_hospital})")

def ros_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = Node('doctor_ui_node')
    root = tk.Tk()
    app = DoctorDashboard(root, node)
    
    t = threading.Thread(target=ros_thread, args=(node,))
    t.daemon = True
    t.start()
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()