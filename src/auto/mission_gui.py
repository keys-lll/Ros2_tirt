#!/usr/bin/env python3
"""Desktop operator interface for the single-file delivery mission."""
import queue
import os
import signal
import subprocess
import tempfile
import json
import uuid
import threading
import time
import fcntl
import sys
import re
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.time import Time
from tf2_ros import TransformException
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageTk
import cv2
import run_mission


def initialize_localization(node, point, stop_event, say, timeout=30.0):
    """Publish the saved start pose on demand; block navigation until confirmed."""
    say('正在初始化定位：使用已記錄的 start，等待 AMCL 與 TF…')
    received = []
    requested_ns = node.get_clock().now().nanoseconds

    def pose_received(msg):
        stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        if stamp_ns >= requested_ns:
            received.append(stamp_ns)

    subscription = node.create_subscription(
        PoseWithCovarianceStamped, '/amcl_pose', pose_received, qos_profile_sensor_data)
    deadline = time.monotonic() + timeout
    next_publish = 0.0
    try:
        while time.monotonic() < deadline:
            if stop_event.is_set():
                raise run_mission.MissionAborted('按 Q 中止初始化')
            now = time.monotonic()
            if not received and now >= next_publish:
                pose = run_mission.pose_from_point(point, 'map', node.get_clock().now().to_msg())
                node.setInitialPose(pose)
                next_publish = now + 1.0
            rclpy.spin_once(node, timeout_sec=0.1)
            if received:
                try:
                    transform = node.buffer.lookup_transform('map', 'base_link', Time())
                    stamp = transform.header.stamp
                    age = (node.get_clock().now().nanoseconds -
                           stamp.sec * 1000000000 - stamp.nanosec) / 1e9
                    if -0.2 <= age <= 1.0:
                        return
                except TransformException:
                    pass
        raise RuntimeError('初始化逾時，尚未取得新的 AMCL 定位與有效 TF；任務停止，請檢查導航／雷達後重新開始。')
    finally:
        node.destroy_subscription(subscription)


class CameraStream:
    """Own the camera/QR launch only while a stopped handoff needs it."""

    def __init__(self, events, command=None):
        self.events = events
        self.command = command or [
            'ros2', 'launch', 'mecanum_navigation', 'delivery_camera.launch.py',
            'video_device:=/dev/video0',
        ]
        self.condition = threading.Condition()
        self.desired = False
        self.closing = False
        self.process = None
        self.stopped = threading.Event()
        self.stopped.set()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def set_active(self, active):
        with self.condition:
            self.desired = active
            if active:
                self.stopped.clear()
            self.condition.notify_all()

    @staticmethod
    def stop_process(process):
        for sig, timeout in ((signal.SIGINT, 3), (signal.SIGTERM, 1), (signal.SIGKILL, 1)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue
        process.wait()

    def run(self):
        try:
            while True:
                with self.condition:
                    if self.closing:
                        break
                    desired = self.desired
                if self.process is not None and (not desired or self.process.poll() is not None):
                    crashed = desired
                    self.stop_process(self.process)
                    self.process = None
                    if crashed:
                        self.events.put(('camera_error', '相機／QR 程式已停止；請查看相機紀錄，可按 Enter 手動放行。'))
                        with self.condition:
                            self.desired = False
                elif desired and self.process is None:
                    try:
                        with tempfile.NamedTemporaryFile(mode='w', prefix='mecanum_camera_', suffix='.log', delete=False) as log:
                            self.process = subprocess.Popen(
                                self.command, stdin=subprocess.DEVNULL, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
                            print('相機／QR 紀錄：' + log.name, flush=True)
                    except OSError as exc:
                        self.events.put(('camera_error', '相機啟動失敗：' + str(exc)))
                        with self.condition:
                            self.desired = False
                with self.condition:
                    if self.process is None and not self.desired:
                        self.stopped.set()
                    # Wake immediately when the GUI changes its camera demand.
                    if not self.closing and self.desired == desired:
                        self.condition.wait(timeout=0.2)
        finally:
            if self.process is not None:
                self.stop_process(self.process)
                self.process = None
            self.stopped.set()

    def close(self):
        with self.condition:
            self.closing = True
            self.condition.notify_all()
        self.thread.join()


class MissionWindow:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.camera_stream = CameraStream(self.events)
        self.answers = queue.Queue()
        self.stop_event = threading.Event()
        self.waiting = False
        self.scanning = False
        self.scan_kind = None
        self.scan_started = None
        self.current_extra = '尚未掃描'
        self.phase = '任務準備'
        self.last_message = ''
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.bridge = CvBridge()
        self.root.title('送餐機器人｜任務控制｜鍵盤版')
        self.root.configure(bg='#101b2c')
        width = min(1000, self.root.winfo_screenwidth() - 40)
        height = min(1750, self.root.winfo_screenheight() - 80)
        self.root.geometry(f'{width}x{height}+20+40')
        self.root.minsize(min(700, width), min(700, height))
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        font = 'Noto Sans CJK TC'
        header = tk.Frame(root, bg='#101b2c', padx=36, pady=24)
        header.pack(fill='x')
        tk.Label(header, text='送餐機器人', font=(font, 27, 'bold'), fg='#f5f8ff', bg='#101b2c').pack(side='left')
        tk.Label(header, text='任務控制中心', font=(font, 16), fg='#8da6c4', bg='#101b2c').pack(side='left', padx=22)
        body = tk.Frame(root, bg='#101b2c', padx=36)
        body.pack(fill='both', expand=True)
        upper = tk.Frame(body, bg='#101b2c')
        upper.pack(fill='both', expand=True)
        self.card = tk.Frame(upper, bg='#1b2b40', padx=30, pady=25)
        self.card.pack(side='top', fill='x', pady=(0, 16))
        camera = tk.Frame(upper, bg='#1b2b40', padx=20, pady=20)
        camera.pack(side='top', fill='both', expand=True)
        tk.Label(camera, text='相機畫面', font=(font, 16, 'bold'), fg='#66d8c2', bg='#1b2b40').pack(anchor='w')
        self.camera_label = tk.Label(camera, text='到達桌邊後開啟相機', font=(font, 18), fg='#a8c7dd', bg='#101b2c')
        self.camera_label.pack(fill='both', expand=True, pady=(10, 0))
        self.photo = None
        qr_panel = tk.Frame(camera, bg='#263d50', padx=20, pady=14)
        qr_panel.pack(fill='x', pady=(14, 0))
        tk.Label(qr_panel, text='QR Code 辨識結果', font=(font, 16, 'bold'), fg='#a8c7dd', bg='#263d50').pack(anchor='w')
        self.qr_label = tk.Label(qr_panel, text='等待掃描…', font=(font, 22, 'bold'), fg='#ffd878', bg='#263d50', wraplength=850, justify='left', anchor='w')
        self.qr_label.pack(fill='x')
        tk.Label(self.card, text='目前流程', font=(font, 16, 'bold'), fg='#66d8c2', bg='#1b2b40').pack(anchor='w')
        self.status = tk.Label(self.card, text='正在啟動…', font=(font, 25, 'bold'), fg='white', bg='#1b2b40', wraplength=950, justify='left', anchor='w')
        self.status.pack(fill='x', pady=(10, 18))
        self.detail = tk.Label(self.card, text='', font=(font, 16), fg='#c2d2e4', bg='#1b2b40', wraplength=950, justify='left', anchor='w')
        self.detail.pack(fill='x')
        extra = tk.Frame(body, bg='#263d50', padx=25, pady=18)
        extra.pack(fill='x', pady=16)
        tk.Label(extra, text='加點餐點', font=(font, 16, 'bold'), fg='#a8c7dd', bg='#263d50').pack(anchor='w')
        self.extra_label = tk.Label(extra, text=self.current_extra, font=(font, 27, 'bold'), fg='#ffd878', bg='#263d50', wraplength=950, justify='left', anchor='w')
        self.extra_label.pack(fill='x')
        controls = tk.Frame(body, bg='#101b2c')
        controls.pack(fill='x', pady=(0, 24))
        self.controls = controls
        self.buttons = []
        self.choices = []
        self.prompt_id = None
        self.publish_state = None
        self.last_state = 0.0
        self.restart = None
        self.root.bind('<Return>', self.on_enter)
        self.root.bind('<KP_Enter>', self.on_enter)
        self.root.bind('<KeyPress-q>', self.on_abort)
        self.root.bind('<KeyPress-Q>', self.on_abort)
        self.root.bind('<KeyPress>', self.on_keypress)
        self.root.after(50, self.drain)

    def say(self, text):
        self.events.put(('say', str(text)))

    def ask(self, prompt):
        if self.stop_event.is_set():
            return 'q'
        self.events.put(('ask', (prompt, time.monotonic())))
        answer = self.answers.get()
        # Keep motion blocked until the camera/QR launch has fully shut down.
        self.camera_stream.stopped.wait()
        return 'q' if self.stop_event.is_set() else answer

    @staticmethod
    def qr_prompt_kind(prompt):
        """Identify the three table-side QR handoffs."""
        for marker, kind in (
            ('請裁判給點餐卡', '點餐卡'),
            ('請裁判給加點卡', '加點卡'),
            ('請裁判給「不須加點」卡', '不須加點卡'),
        ):
            if marker in prompt:
                return kind
        return None

    def qr(self, text):
        self.events.put(('qr', (time.monotonic(), text)))

    def frame(self, msg):
        if not self.scanning:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
            with self.frame_lock:
                if self.scanning:
                    self.latest_frame = image
        except Exception:
            pass

    def show_buttons(self, prompt):
        for button in self.buttons:
            button.destroy()
        self.buttons = []
        if prompt == '__restart__':
            choices = [('重新開始任務', '__restart__')]
        elif '第一回合抽到幾桌' in prompt or '第二回合抽到幾桌' in prompt:
            available = re.search(r'(?:可選|只能) ([1-5、]+)', prompt)
            numbers = available.group(1).split('、') if available else list('12345')
            choices = [(f'[{n}] {n} 號桌', n) for n in numbers]
            if '第二回合' in prompt:
                choices.append(('結束回合', ''))
        elif '這一步沒成功' in prompt:
            choices = [('重試', 'r'), ('已手動到位', 's')]
        elif '要去哪些點' in prompt:
            choices = [('開始比賽：初始化 → start', '')]
        elif self.scanning:
            choices = [('Enter 手動放行', '')]
        else:
            choices = [('確認 / 繼續', '')]
        if prompt != '__restart__':
            choices.append(('結束任務', 'q'))
        self.choices = choices
        self.prompt_id = uuid.uuid4().hex
        for index, (label, value) in enumerate(choices):
            button = tk.Button(self.controls, text=label, command=lambda v=value: self.submit(v),
                               font=('Noto Sans CJK TC', 15, 'bold'), bg='#54d6bb' if value != 'q' else '#526174',
                               fg='#102b32' if value != 'q' else 'white', relief='flat', padx=14, pady=10)
            button.grid(row=index // 3, column=index % 3, sticky='ew', padx=(0, 10), pady=5)
            self.controls.grid_columnconfigure(index % 3, weight=1)
            self.buttons.append(button)

    def on_keypress(self, event):
        value = getattr(event, 'char', '').translate(str.maketrans('１２３４５', '12345'))
        keypad = {'KP_End': '1', 'KP_Down': '2', 'KP_Next': '3',
                  'KP_Left': '4', 'KP_Begin': '5'}
        if not value:
            value = keypad.get(event.keysym, event.keysym.removeprefix('KP_'))
        if value in ('1', '2', '3', '4', '5'):
            return self.on_table_key(value)

    def terminal_key(self, value):
        if value.lower() == 'q':
            self.on_abort()
        elif value in ('1', '2', '3', '4', '5'):
            self.on_table_key(value)
        elif self.waiting and value.lower() in ('r', 's') and any(
                answer == value.lower() for label, answer in self.choices):
            self.submit(value.lower())
        elif not value:
            self.on_enter()

    def on_table_key(self, value):
        """Select a table only when its choice is available in this prompt."""
        if self.waiting and any(answer == value for label, answer in self.choices):
            self.submit(value)
        return 'break'

    def on_abort(self, event=None):
        """End the mission, including a goal currently being navigated."""
        if self.choices == [('重新開始任務', '__restart__')]:
            return 'break'
        self.stop_event.set()
        self.detail.config(text='正在中止任務，取消導航…')
        if self.waiting:
            self.submit('q')
        return 'break'

    def on_enter(self, event=None):
        """Confirm a prompt, or activate the button selected with Tab."""
        if not self.waiting:
            return 'break'
        focused = self.root.focus_get()
        for button in self.buttons:
            if focused is button:
                button.invoke()
                return 'break'
        primary = [(label, value) for label, value in self.choices if value != 'q']
        # Multiple choices (tables / retry) must be selected explicitly.
        if len(primary) == 1:
            self.submit(primary[0][1])
        return 'break'

    def submit(self, value=None):
        if not self.waiting:
            return
        answer = '' if value is None else value
        if self.scan_kind == '加點卡' and answer.strip().lower() not in ('q', 'quit'):
            self.current_extra = answer.strip() or '未輸入'
            self.extra_label.config(text=self.current_extra)
        self.waiting = False
        self.scanning = False
        self.camera_stream.set_active(False)
        with self.frame_lock:
            self.latest_frame = None
        self.photo = None
        self.camera_label.config(image='', text='相機已關閉；到點後重新開啟')
        self.scan_kind = None
        self.scan_started = None
        self.prompt_id = None
        self.choices = []
        for button in self.buttons:
            button.destroy()
        self.buttons = []
        if answer == '__restart__' and self.restart:
            self.phase = '任務準備'
            self.current_extra = '尚未掃描'
            self.extra_label.config(text=self.current_extra)
            self.status.config(text='正在重新開始…')
            self.detail.config(text='')
            self.restart()
        else:
            self.answers.put(answer)

    def close(self):
        if self.waiting:
            if messagebox.askyesno('結束任務', '確定要結束送餐任務嗎？'):
                self.submit('q')
        else:
            messagebox.showinfo('任務執行中', '請等目前導航或停等完成，再結束任務。')

    def drain(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == 'say':
                    if value.startswith('====='):
                        self.phase = value.strip('= ')
                        self.status.config(text=self.phase)
                        if '第一趟' in value:
                            self.current_extra = '尚未掃描'
                            self.extra_label.config(text=self.current_extra)
                    else:
                        self.last_message = value
                        self.detail.config(text=value)
                        if value.startswith(('正在前往 ', '正在初始化定位')):
                            self.status.config(text=self.phase + '｜' + value)
                elif kind == 'ask':
                    value, requested_at = value
                    self.waiting = True
                    self.scan_kind = self.qr_prompt_kind(value)
                    self.scanning = self.scan_kind is not None
                    self.scan_started = requested_at if self.scanning else None
                    self.camera_stream.set_active(self.scanning)
                    if self.scanning:
                        with self.frame_lock:
                            self.latest_frame = None
                        self.qr_label.config(text='等待本次掃描…')
                        self.camera_label.config(image='', text='正在啟動相機／QR…')
                    self.status.config(text=self.phase + ('｜等待 QR Code：' + self.scan_kind if self.scanning else '｜等待操作'))
                    if '要去哪些點' in value:
                        self.detail.config(text='按「開始比賽」或 Enter 才會初始化定位，接著自動先到 start。\n開始前請將車子放在已記錄的 start 位置與朝向。')
                    elif '這一步沒成功' in value:
                        self.status.config(text=self.phase + '｜導航未完成')
                        self.detail.config(text=self.last_message + '\n\n' + value)
                    else:
                        self.detail.config(text=(value + '\n請掃描' + self.scan_kind + ' QR Code；或按 Enter 手動放行。\n未掃到且未按 Enter，會持續停留等待。' if self.scanning else (value + '\n可按數字 1～5 選桌' if '抽到幾桌' in value else value)))
                    self.show_buttons(value)
                    if self.stop_event.is_set():
                        self.submit('q')
                elif kind == 'qr':
                    observed_at, code = value
                    if code.strip():
                        self.qr_label.config(text=code.strip())
                        if self.waiting and self.scanning and observed_at >= self.scan_started:
                            kind = self.scan_kind
                            self.submit(code.strip())
                            self.detail.config(text='已掃描' + kind + '：' + code.strip())
                elif kind == 'camera_error':
                    if self.scanning:
                        self.camera_label.config(image='', text=value)
                elif kind == 'keyboard':
                    self.terminal_key(value)
                elif kind == 'remote':
                    try:
                        command = json.loads(value)
                        valid = [item[1] for item in self.choices]
                        if self.waiting and command.get('prompt_id') == self.prompt_id and command.get('value') in valid:
                            self.submit(command['value'])
                    except (ValueError, TypeError, AttributeError):
                        pass
                elif kind == 'done':
                    self.camera_stream.set_active(False)
                    self.scanning = False
                    self.waiting = False
                    for button in self.buttons:
                        button.destroy()
                    self.buttons = []
                    self.status.config(text='任務已結束')
                    self.detail.config(text=value)
                    self.waiting = True
                    self.show_buttons('__restart__')
                    self.root.protocol('WM_DELETE_WINDOW', self.root.destroy)
        except queue.Empty:
            pass
        with self.frame_lock:
            frame = self.latest_frame
            self.latest_frame = None
        if frame is not None:
            width = max(1, self.camera_label.winfo_width())
            height = max(1, self.camera_label.winfo_height())
            if width > 1 and height > 1:
                image = PILImage.fromarray(frame)
                image.thumbnail((width, height))
                self.photo = ImageTk.PhotoImage(image)
                self.camera_label.config(image=self.photo, text='')
        if self.publish_state and time.monotonic() - self.last_state >= 0.5:
            self.last_state = time.monotonic()
            self.publish_state(json.dumps({'status': self.status.cget('text'), 'detail': self.detail.cget('text'),
                'extra': self.extra_label.cget('text'), 'qr': self.qr_label.cget('text'),
                'prompt_id': self.prompt_id, 'choices': self.choices}, ensure_ascii=False))
        self.root.after(50, self.drain)


def main():
    lock = open('/tmp/mecanum_mission_gui.lock', 'w')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('已有送餐介面在執行，請先結束原本的任務。', file=sys.stderr)
        return 1
    root = tk.Tk()
    app = MissionWindow(root)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    qr_node = rclpy.create_node('mission_gui_qr')
    state_pub = qr_node.create_publisher(String, '/car_mission/state', 10)
    def publish_state(text):
        msg = String()
        msg.data = text
        state_pub.publish(msg)
    app.publish_state = publish_state
    qr_node.create_subscription(String, '/car_mission/command', lambda msg: app.events.put(('remote', msg.data)), 10)
    qr_node.create_subscription(String, '/zbar_opencv_code', lambda msg: app.qr(msg.data), 10)
    qr_node.create_subscription(Image, '/zbar_opencv', app.frame, qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(qr_node)
    qr_thread = threading.Thread(target=executor.spin, daemon=True)
    qr_thread.start()

    def worker():
        node = None
        try:
            node = run_mission.Navigator()
            node.abort_requested = app.stop_event.is_set
            def navigate(name, point):
                app.say('正在前往 ' + name + '…')
                if app.stop_event.is_set():
                    raise run_mission.MissionAborted('按 Q 中止')
                result = node.navigate(name, point)
                if app.stop_event.is_set():
                    raise run_mission.MissionAborted('按 Q 中止')
                return result
            node.navigate_for_gui = navigate
            class Adapter:
                def navigate(self, name, point):
                    return node.navigate_for_gui(name, point)
            def interruptible_sleep(seconds):
                if app.stop_event.wait(seconds):
                    raise run_mission.MissionAborted('按 Q 中止')
            points = run_mission.all_points()
            def prepare_start():
                initialize_localization(node, points['start'], app.stop_event, app.say)
            run_mission.run(Adapter(), points, app.ask, app.say, interruptible_sleep,
                            prepare_start=prepare_start)
            result = '可按重新開始任務，或關閉視窗。'
        except Exception as exc:
            result = '程式發生錯誤：' + str(exc)
        finally:
            if node is not None:
                node.close()
        app.events.put(('done', result))
    def start_worker():
        app.stop_event.clear()
        app.root.protocol('WM_DELETE_WINDOW', app.close)
        threading.Thread(target=worker, daemon=True).start()
    app.restart = start_worker
    start_worker()
    def terminal_keyboard():
        print('鍵盤操作：桌號 1～5 後按 Enter；桌邊掃 QR 或 Enter 手動放行；備餐區 Enter 出發；其他步驟 Enter 下一步；q 後按 Enter 中止。', flush=True)
        for line in sys.stdin:
            app.events.put(('keyboard', line.strip()))
    if sys.stdin.isatty():
        threading.Thread(target=terminal_keyboard, daemon=True).start()
    try:
        root.mainloop()
    finally:
        app.camera_stream.close()
        executor.shutdown()
        qr_thread.join(timeout=3)
        qr_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
