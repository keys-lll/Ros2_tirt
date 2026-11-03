#!/usr/bin/env python3
"""Laptop operator display; the vehicle runs the sole mission controller."""
import json
import queue
import threading
import time
import tkinter as tk
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from std_msgs.msg import String

class Window:
    def __init__(self, root, publisher):
        self.root, self.publisher = root, publisher
        self.events = queue.Queue()
        self.last_state = 0
        self.pose_lock = threading.Lock()
        self.pose_text = "等待 ROS 定位座標…"
        self.prompt_id = None
        self.buttons = []
        root.title('送餐上位機｜車輛控制')
        root.configure(bg='#101b2c')
        root.geometry(f'{min(1100, root.winfo_screenwidth()-40)}x{min(850, root.winfo_screenheight()-80)}+20+40')
        f = 'Noto Sans CJK TC'
        tk.Label(root, text='送餐上位機', font=(f, 26, 'bold'), bg='#101b2c', fg='white').pack(anchor='w', padx=24, pady=12)
        body = tk.Frame(root, bg='#101b2c'); body.pack(fill='both', expand=True, padx=24)
        left = tk.Frame(body, bg='#1b2b40', padx=20, pady=20); left.pack(side='left', fill='both', expand=True, padx=(0, 10))
        self.status = tk.Label(left, text='等待車上任務連線…', font=(f, 21, 'bold'), bg='#1b2b40', fg='#66d8c2', wraplength=460, justify='left'); self.status.pack(anchor='w', fill='x')
        self.detail = tk.Label(left, text='', font=(f, 15), bg='#1b2b40', fg='white', wraplength=460, justify='left'); self.detail.pack(anchor='w', fill='x', pady=12)
        self.position = tk.Label(left, text='等待 ROS 定位座標…', font=(f, 22, 'bold'), bg='#1b2b40', fg='#66d8c2', justify='left', anchor='w'); self.position.pack(anchor='w', fill='x', pady=12)
        self.extra = tk.Label(left, text='加點：等待中', font=(f, 20, 'bold'), bg='#1b2b40', fg='#ffd878', wraplength=460, justify='left'); self.extra.pack(anchor='w', fill='x', pady=12)
        self.controls = tk.Frame(left, bg='#1b2b40'); self.controls.pack(fill='x', pady=12)
        self.qr = tk.Label(left, text='QR Code：等待掃描', bg='#1b2b40', fg='#ffd878', font=(f, 17), wraplength=750, justify='left', anchor='w'); self.qr.pack(fill='x', pady=10)
        self.connection = tk.Label(root, text='等待連線', bg='#101b2c', fg='#c2d2e4', font=(f, 12)); self.connection.pack(anchor='w', padx=24, pady=12)
        root.after(80, self.drain)
    def send(self, prompt, value):
        if time.monotonic() - self.last_state > 2 or prompt != self.prompt_id: return
        msg = String(); msg.data = json.dumps({'prompt_id':prompt, 'value':value}); self.publisher.publish(msg)
        for b in self.buttons: b.config(state='disabled')
    def drain(self):
        try:
            while True:
                state = self.events.get_nowait()
                self.last_state = time.monotonic()
                self.status.config(text=state['status']); self.detail.config(text=state['detail'])
                self.extra.config(text='加點：' + state['extra']); self.qr.config(text='QR Code：' + state['qr'])
                prompt = state.get('prompt_id')
                if prompt != self.prompt_id:
                    self.prompt_id = prompt
                    for b in self.buttons: b.destroy()
                    self.buttons = []
                    for i, (label, value) in enumerate(state.get('choices', [])):
                        b = tk.Button(self.controls, text=label, command=lambda p=prompt,v=value: self.send(p,v), font=('Noto Sans CJK TC', 14, 'bold'), bg='#54d6bb', relief='flat', padx=10, pady=8)
                        b.grid(row=i//2, column=i%2, sticky='ew', padx=4, pady=4); self.controls.grid_columnconfigure(i%2, weight=1); self.buttons.append(b)
        except queue.Empty: pass
        with self.pose_lock:
            self.position.config(text=self.pose_text)
        connected = time.monotonic() - self.last_state <= 2
        self.connection.config(text='● 已連線車上任務' if connected else '● 與車上任務的連線中斷', fg='#66d8c2' if connected else '#ff9b9b')
        if not connected:
            for b in self.buttons: b.config(state='disabled')
            self.prompt_id = None
        self.root.after(80, self.drain)

def main():
    rclpy.init()
    node = rclpy.create_node('laptop_mission_operator')
    pub = node.create_publisher(String, '/car_mission/command', 10)
    root = tk.Tk(); app = Window(root, pub)
    def state(msg):
        try: app.events.put(json.loads(msg.data))
        except ValueError: pass
    node.create_subscription(String, '/car_mission/state', state, 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node, spin_thread=False)
    def update_pose():
        text = '等待定位座標（map / odom）'
        for frame in ('map', 'odom'):
            try:
                transform = buffer.lookup_transform(frame, 'base_link', Time())
                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
                age = node.get_clock().now().nanoseconds / 1e9 - stamp
                if stamp == 0 or age < -2 or age > 2:
                    continue
                position = transform.transform.translation
                name = '地圖座標' if frame == 'map' else '里程計座標'
                text = f'{name}（{frame}）\nX：{position.x:.2f} m    Y：{position.y:.2f} m'
                break
            except TransformException:
                pass
        with app.pose_lock:
            app.pose_text = text
    node.create_timer(0.2, update_pose)
    executor = SingleThreadedExecutor(); executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    try: root.mainloop()
    finally:
        executor.shutdown(); thread.join(timeout=3); node.destroy_node(); rclpy.try_shutdown()
if __name__ == '__main__': main()
