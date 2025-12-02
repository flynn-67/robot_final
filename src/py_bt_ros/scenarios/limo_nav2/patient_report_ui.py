#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import tkinter as tk
from tkinter import messagebox
import threading
from datetime import datetime
import random
import sys
import os

# 병원 진료 과목 정의
DEPARTMENTS = [
    {"name": "내과", "desc": "혈압 및 기본 검사"},
    {"name": "외과", "desc": "신체 외상 검사"},
    {"name": "이비인후과", "desc": "호흡기 정밀 검사"},
    {"name": "치과", "desc": "구강 건강 검진"}
]

class SmartHospitalApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.node = ros_node
        self.root.title("스마트 병원 환자용 키오스크")
        self.root.geometry("500x750")
        self.root.configure(bg="#f0f4f8")

        # 데이터 초기화
        self.patient_name = ""
        self.medical_records = []
        self.waiting_counts = {}
        self.dept_labels = {}

        # 메인 프레임
        self.main_frame = tk.Frame(root, bg="#f0f4f8")
        self.main_frame.pack(fill="both", expand=True, padx=20, pady=20)
        
        # 초기 화면 로드
        self.show_home_screen()

        # ROS 통신 설정
        self.pub_start = self.node.create_publisher(String, '/hospital_data', 10)
        self.pub_emergency = self.node.create_publisher(Bool, '/emergency', 10)
        
        self.sub_loc = self.node.create_subscription(String, '/current_hospital', self.update_location_cb, 10)
        self.sub_fin = self.node.create_subscription(Bool, '/exam_finished', self.finish_cb, 10)
        self.sub_record = self.node.create_subscription(String, '/medical_record', self.receive_record_cb, 10)

        print(">>> [Patient UI] 환자용 키오스크 실행됨")

    def clear_frame(self):
        for widget in self.main_frame.winfo_children():
            widget.destroy()

    def show_home_screen(self):
        self.clear_frame()
        self.medical_records = []
        
        # 헤더
        tk.Label(self.main_frame, text="🏥 스마트 병원", font=("Arial", 24, "bold"), bg="#f0f4f8", fg="#4f46e5").pack(pady=40)
        tk.Label(self.main_frame, text="환자 맞춤형 대기 시스템", font=("Arial", 12), bg="#f0f4f8", fg="#666").pack(pady=(0, 20))
        
        # 버튼
        tk.Button(self.main_frame, text="접수 시작", font=("Arial", 14, "bold"), bg="#4f46e5", fg="white", height=2, 
                  command=self.show_questionnaire).pack(fill="x", pady=10)
        
        tk.Button(self.main_frame, text="시설 안내도", font=("Arial", 12), bg="white", command=lambda: messagebox.showinfo("안내", "화장실: 복도 끝\n비상구: 1층 정문")).pack(fill="x", pady=5)
        
        tk.Button(self.main_frame, text="🚨 긴급 호출", font=("Arial", 12, "bold"), bg="#ef4444", fg="white", 
                  command=self.send_emergency).pack(side="bottom", fill="x", pady=20)

    def show_questionnaire(self):
        self.clear_frame()
        tk.Label(self.main_frame, text="📝 기초 문진표", font=("Arial", 18, "bold"), bg="#f0f4f8").pack(pady=20)
        
        tk.Label(self.main_frame, text="성함", bg="#f0f4f8", anchor="w").pack(fill="x")
        self.entry_name = tk.Entry(self.main_frame, font=("Arial", 12))
        self.entry_name.pack(fill="x", pady=5)
        
        tk.Button(self.main_frame, text="작성 완료", bg="#4f46e5", fg="white", font=("Arial", 12, "bold"), 
                  command=self.submit_questionnaire).pack(fill="x", pady=20)

    def show_qr_simulation(self):
        self.clear_frame()
        tk.Label(self.main_frame, text=f"{self.patient_name}님 접수증", font=("Arial", 18, "bold"), bg="#f0f4f8").pack(pady=20)
        
        qr_box = tk.Frame(self.main_frame, bg="black", width=200, height=200)
        qr_box.pack(pady=20)
        tk.Label(qr_box, text="QR CODE", fg="white", bg="black").place(relx=0.5, rely=0.5, anchor="center")
        
        tk.Label(self.main_frame, text="로봇 카메라에 QR을 보여주세요", bg="#f0f4f8").pack()
        
        # 시뮬레이션 버튼
        tk.Button(self.main_frame, text="▶ 로봇 연동 시작 (터치)", bg="#10b981", fg="white", font=("Arial", 12, "bold"), 
                  command=self.start_robot_system).pack(fill="x", pady=30)

    def show_progress_view(self):
        self.clear_frame()
        tk.Label(self.main_frame, text="실시간 대기 현황", font=("Arial", 18, "bold"), bg="#f0f4f8").pack(pady=20)
        self.dept_labels = {}

        for dept in DEPARTMENTS:
            name = dept['name']
            count = self.waiting_counts.get(name, 0)
            
            frame = tk.Frame(self.main_frame, bg="white", padx=10, pady=10)
            frame.pack(fill="x", pady=5)
            
            tk.Label(frame, text=name, font=("Arial", 14, "bold"), bg="white", width=8, anchor="w").pack(side="left")
            
            status_text = f"대기: {count}명"
            lbl_status = tk.Label(frame, text=status_text, font=("Arial", 12), fg="#e11d48", bg="white")
            lbl_status.pack(side="right")
            
            self.dept_labels[name] = lbl_status

        tk.Button(self.main_frame, text="🚨 SOS", bg="#ef4444", fg="white", command=self.send_emergency).pack(side="bottom", anchor="e", pady=20)

    def show_final_report(self):
        self.clear_frame()
        tk.Label(self.main_frame, text="📋 통합 진료 결과", font=("Arial", 20, "bold"), bg="#f0f4f8", fg="#4f46e5").pack(pady=20)
        
        text_area = tk.Text(self.main_frame, font=("Arial", 12), padx=10, pady=10)
        text_area.pack(fill="both", expand=True)
        
        text_area.insert(tk.END, f"환자명: {self.patient_name}\n일자: {datetime.now().strftime('%Y-%m-%d')}\n\n" + "="*35 + "\n\n")
        
        for record in self.medical_records:
            text_area.insert(tk.END, f"[{record['dept']}] 진료 완료\n👨‍⚕️ 소견: {record['diagnosis']}\n" + "-"*35 + "\n\n")
        
        text_area.config(state="disabled")
        tk.Button(self.main_frame, text="처음으로", bg="#333", fg="white", command=self.show_home_screen).pack(fill="x", pady=10)

    def submit_questionnaire(self):
        if not self.entry_name.get(): return
        self.patient_name = self.entry_name.get()
        self.show_qr_simulation()

    def start_robot_system(self):
        # [핵심] 랜덤 대기 인원 생성
        self.waiting_counts = {
            "내과": random.randint(1, 6),
            "외과": random.randint(1, 6),
            "이비인후과": random.randint(1, 6),
            "치과": random.randint(1, 6)
        }
        
        data = self.waiting_counts.copy()
        data['command'] = 'start'
        msg = String()
        msg.data = json.dumps(data)
        self.pub_start.publish(msg)
        print(f">>> [UI] 시작 신호 전송 완료 (대기인원: {self.waiting_counts})")
        self.show_progress_view()

    def update_location_cb(self, msg):
        hospital_name = msg.data.strip()
        if hospital_name in self.dept_labels:
            self.dept_labels[hospital_name].config(text="진료 중 🩺", fg="#4f46e5", font=("Arial", 12, "bold"))

    def receive_record_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.medical_records.append(data)
            dept = data.get('dept')
            if dept in self.dept_labels:
                self.dept_labels[dept].config(text="완료 ✅", fg="#10b981")
        except: pass

    def finish_cb(self, msg):
        if msg.data: self.show_final_report()

    def send_emergency(self):
        self.pub_emergency.publish(Bool(data=True))
        messagebox.showwarning("긴급", "호출 신호 전송 완료!")

def ros_thread(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = Node('patient_ui_node')
    root = tk.Tk()
    app = SmartHospitalApp(root, node)
    
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