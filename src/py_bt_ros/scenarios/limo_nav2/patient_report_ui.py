import tkinter as tk
from tkinter import ttk, font, messagebox
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
import threading
import queue 

class PatientReportNode(Node):
    def __init__(self, gui_queue):
        super().__init__('patient_report_ui')
        self.gui_queue = gui_queue
        
        self.create_subscription(String, '/medical_record', self.record_callback, 10)
        self.create_subscription(Bool, '/exam_finished', self.finish_callback, 10)

    def record_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.gui_queue.put(("data", data))
        except: pass

    def finish_callback(self, msg):
        print(f"\n !!! [UI] 검사 종료 신호 수신 !!! \n")
        if msg.data:
            self.gui_queue.put(("finish", True))
        else:
            self.gui_queue.put(("reset", True))

class ReportApp:
    def __init__(self, root, gui_queue):
        self.root = root
        self.gui_queue = gui_queue
        
        self.root.title("건강검진 결과 보고서 (환자용)")
        self.root.geometry("500x750")
        self.root.configure(bg="white")

        # [핵심 1] 프로그램 시작 시 창을 숨깁니다 (백그라운드 실행)
        self.root.withdraw()
        print(">>> [UI] 환자 보고서가 백그라운드에서 대기 중입니다...")

        self.header_font = font.Font(family="Malgun Gothic", size=20, weight="bold")
        self.value_font = font.Font(family="Malgun Gothic", size=12, weight="bold")

        self.title_lbl = tk.Label(root, text="건강검진 결과 보고서", font=self.header_font, bg="white", fg="#333333")
        self.title_lbl.pack(pady=30)

        self.table_frame = tk.Frame(root, bg="white")
        self.table_frame.pack(fill="both", expand=True, padx=30, pady=10)

        # 헤더
        tk.Label(self.table_frame, text="검사 항목", width=15, height=2, bg="#4a4a4a", fg="white", font=("Malgun Gothic", 12)).grid(row=0, column=0, sticky="nsew")
        tk.Label(self.table_frame, text="검사 결과", width=25, height=2, bg="#4a4a4a", fg="white", font=("Malgun Gothic", 12)).grid(row=0, column=1, sticky="nsew")

        self.result_labels = {}
        items = [
            ("혈압", "bp"), ("혈당", "blood_sugar"), ("내과 소견", "internal_comment"),
            ("시력", "vision"), ("안압", "eye_pressure"), ("안과 소견", "eye_comment"),
            ("청력", "hearing"), ("이비인후과 소견", "ent_comment"),
            ("구강 상태", "dental"), ("치과 소견", "dental_comment"),
            ("수술 부위", "surgery_check"), ("외과 소견", "surgery_comment")
        ]

        for i, (label_text, key) in enumerate(items):
            row = i + 1
            tk.Label(self.table_frame, text=label_text, height=2, bg="#f9f9f9", font=("Malgun Gothic", 11), borderwidth=1, relief="solid").grid(row=row, column=0, sticky="nsew")
            lbl = tk.Label(self.table_frame, text="", height=2, bg="white", font=self.value_font, borderwidth=1, relief="solid", fg="blue")
            lbl.grid(row=row, column=1, sticky="nsew")
            self.result_labels[key] = lbl

        self.footer_lbl = tk.Label(root, text="검사가 진행 중입니다...", font=("Malgun Gothic", 14), bg="white", fg="gray")
        self.footer_lbl.pack(pady=30)

        self.root.after(100, self.check_queue)

    def check_queue(self):
        try:
            while True:
                task_type, content = self.gui_queue.get_nowait()
                
                if task_type == "data":
                    self.update_table(content)
                elif task_type == "finish":
                    self.show_final_report()
                elif task_type == "reset":
                    self.reset_report()
        except queue.Empty:
            pass
        self.root.after(100, self.check_queue)

    def update_table(self, data):
        target = data.get('target')
        value = data.get('value')
        if target in self.result_labels:
            self.result_labels[target].config(text=value)

    def show_final_report(self):
        # [핵심 2] 로봇이 복귀 신호를 보내면 그때 창을 보여줍니다!
        self.root.deiconify() # 숨김 해제
        self.root.attributes('-topmost', True) # 화면 맨 앞으로 강제 이동
        
        self.footer_lbl.config(text="✅ 최종 보고서 작성 완료", fg="green", font=("Malgun Gothic", 16, "bold"))
        self.title_lbl.config(fg="green", text="[완료] 건강검진 결과표")
        
        messagebox.showinfo("검사 종료", "모든 검사가 완료되었습니다.\n환자분께서는 귀가하셔도 좋습니다.", parent=self.root)
        
        # 팝업 닫은 후 최상위 속성 해제 (다른 창 쓸 수 있게)
        self.root.attributes('-topmost', False)

    def reset_report(self):
        # 리셋할 때는 다시 숨김 (선택 사항 - 여기선 숨기도록 설정)
        self.root.withdraw()
        for lbl in self.result_labels.values():
            lbl.config(text="")
        self.footer_lbl.config(text="검사 대기 중...", fg="gray", font=("Malgun Gothic", 14))
        self.title_lbl.config(fg="#333333", text="건강검진 결과 보고서")

def main():
    rclpy.init()
    root = tk.Tk()
    gui_queue = queue.Queue()
    
    app = ReportApp(root, gui_queue)
    node = PatientReportNode(gui_queue)
    
    t = threading.Thread(target=lambda: rclpy.spin(node))
    t.daemon = True
    t.start()
    
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()