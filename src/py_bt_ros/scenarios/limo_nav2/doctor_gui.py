import sys
import threading
import tkinter as tk
from tkinter import font
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class DoctorGuiNode(Node):
    def __init__(self):
        super().__init__('doctor_gui_node')
        self.publisher = self.create_publisher(Bool, '/doctor_confirm', 10)

    def send_confirm(self):
        msg = Bool()
        msg.data = True
        self.publisher.publish(msg)
        self.get_logger().info("진단 완료 신호 전송함!")

def gui_thread(node):
    root = tk.Tk()
    root.title("병원 진료 시스템")
    root.geometry("300x200")
    
    label = tk.Label(root, text="진료가 끝나면\n아래 버튼을 눌러주세요", font=("Arial", 15))
    label.pack(pady=20)

    btn = tk.Button(root, text="진단 완료 (다음 환자)", 
                    command=node.send_confirm, 
                    bg="lightblue", height=3, width=20)
    btn.pack(pady=10)
    
    root.mainloop()

def main():
    rclpy.init()
    node = DoctorGuiNode()
    
    # GUI가 멈추지 않게 스레드 분리
    t = threading.Thread(target=gui_thread, args=(node,))
    t.daemon = True
    t.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()