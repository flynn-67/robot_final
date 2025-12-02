import tkinter as tk
from tkinter import ttk, messagebox
import random
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import threading

HOSPITAL_COORDS = {
    "내과": {"x": 6.455, "y": 2.609},
    "외과": {"x": 7.358, "y": 0.297},
    "이비인후과": {"x": 5.0, "y": 1.0},
    "치과": {"x": 2.0, "y": 2.0}
}

class DashboardNode(Node):
    def __init__(self, app):
        super().__init__('hospital_dashboard')
        self.app = app
        self.pub_data = self.create_publisher(String, '/hospital_data', 10)
        self.pub_confirm = self.create_publisher(Bool, '/doctor_confirm', 10)
        self.pub_record = self.create_publisher(String, '/medical_record', 10)
        
        # 리스너
        self.create_subscription(String, '/current_hospital', self.hospital_sub_callback, 10)
        
        self.hospital_status = {name: random.randint(1, 50) for name in HOSPITAL_COORDS.keys()}

    def hospital_sub_callback(self, msg):
        hospital_name = msg.data.strip()
        # [디버그용 로그] 이게 터미널에 떠야 합니다!
        print(f"\n >>> [UI] 신호 받음: '{hospital_name}' -> 화면 전환 시도\n")
        self.app.show_doctor_page(hospital_name)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps(self.hospital_status)
        self.pub_data.publish(msg)

    def send_confirm(self):
        self.pub_confirm.publish(Bool(data=True))

    def send_medical_record(self, target_key, value_text):
        msg = String()
        data = {"target": target_key, "value": value_text}
        msg.data = json.dumps(data)
        self.pub_record.publish(msg)

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("🏥 스마트 병원 의사 전용 모니터")
        self.root.geometry("600x600")
        self.node = None 

        style = ttk.Style()
        style.configure("Header.TLabel", font=("Malgun Gothic", 15, "bold"), foreground="blue")
        style.configure("BigButton.TButton", font=("Malgun Gothic", 12))

        # 상단
        top_frame = ttk.Frame(root)
        top_frame.pack(fill="x", padx=10, pady=10)
        ttk.Button(top_frame, text="▶ 문진 완료 및 로봇 호출 (시작)", command=self.start_system).pack(side="left", fill="x", expand=True)
        self.status_lbl = ttk.Label(root, text="[대기현황] 준비 중...", font=("Malgun Gothic", 10))
        self.status_lbl.pack(pady=5)

        # 메인
        self.main_frame = ttk.Frame(root, borderwidth=2, relief="groove")
        self.main_frame.pack(fill="both", expand=True, padx=10, pady=10)

        self.pages = {}
        self.create_waiting_page()
        
        # 페이지들 생성 (이름이 정확해야 함)
        self.create_hospital_page("내과", [("혈압", "bp", "120/80"), ("혈당", "blood_sugar", "95"), ("소견", "internal_comment", "정상")])
        self.create_hospital_page("외과", [("수술부위", "surgery_check", "이상무"), ("소견", "surgery_comment", "양호")])
        self.create_hospital_page("안과", [("시력", "vision", "1.0/1.0"), ("안압", "eye_pressure", "정상"), ("소견", "eye_comment", "이상무")])
        self.create_hospital_page("이비인후과", [("청력", "hearing", "정상"), ("소견", "ent_comment", "처방완료")])
        self.create_hospital_page("치과", [("구강검진", "dental", "충치1"), ("소견", "dental_comment", "치료요망")])

        self.switch_frame("waiting")
        self.update_loop()

    def set_node(self, node):
        self.node = node

    def create_waiting_page(self):
        frame = ttk.Frame(self.main_frame)
        ttk.Label(frame, text="🚑 환자가 이동 중입니다...", font=("Malgun Gothic", 20)).pack(expand=True)
        ttk.Label(frame, text="로봇이 도착하면 입력창이 뜹니다.", font=("Malgun Gothic", 12)).pack()
        self.pages["waiting"] = frame

    def create_hospital_page(self, hospital_name, inputs):
        frame = ttk.Frame(self.main_frame)
        ttk.Label(frame, text=f"[{hospital_name}] 진료실", style="Header.TLabel").pack(pady=20)
        
        entries = {}
        input_container = ttk.Frame(frame)
        input_container.pack(fill="x", padx=50)

        for label, key, default in inputs:
            row = ttk.Frame(input_container)
            row.pack(fill="x", pady=5)
            ttk.Label(row, text=label, width=15, anchor="e").pack(side="left", padx=5)
            ent = ttk.Entry(row)
            ent.pack(side="left", fill="x", expand=True)
            entries[key] = ent
        
        btn = ttk.Button(frame, text=f"✅ {hospital_name} 진료 완료", style="BigButton.TButton",
                         command=lambda: self.on_finish(hospital_name, entries))
        btn.pack(side="bottom", fill="x", padx=20, pady=20, ipady=10)
        self.pages[hospital_name] = frame

    def switch_frame(self, page_name):
        for frame in self.pages.values(): frame.pack_forget()
        
        if page_name in self.pages:
            self.pages[page_name].pack(fill="both", expand=True)
        else:
            print(f"⚠️ 경고: '{page_name}' 페이지를 찾을 수 없음! (waiting으로 대체)")
            self.pages["waiting"].pack(fill="both", expand=True)

    def show_doctor_page(self, hospital_name):
        # 스레드 안전하게 화면 전환
        self.root.after(0, lambda: self.switch_frame(hospital_name))

    def on_finish(self, hospital_name, entries):
        for key, ent in entries.items():
            val = ent.get().strip()
            if val: self.node.send_medical_record(key, val)
            ent.delete(0, tk.END)
        
        self.node.send_confirm()
        for name in self.node.hospital_status: self.node.hospital_status[name] = random.randint(1, 50)
        self.switch_frame("waiting")

    def start_system(self):
        data = self.node.hospital_status.copy()
        data['command'] = 'start'
        msg = String()
        msg.data = json.dumps(data)
        self.node.pub_data.publish(msg)

    def update_loop(self):
        if self.node:
            self.node.publish_status()
            status_text = " | ".join([f"{k}:{v}명" for k, v in self.node.hospital_status.items()])
            self.status_lbl.config(text=f"[대기현황] {status_text}")
        self.root.after(1000, self.update_loop)

def main():
    rclpy.init()
    root = tk.Tk()
    app = App(root)
    node = DashboardNode(app)
    app.set_node(node)
    
    t = threading.Thread(target=lambda: rclpy.spin(node))
    t.daemon = True
    t.start()
    
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()