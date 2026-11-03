"""Terminal QR/Enter operator; camera runs only at stopped handoffs."""
import os
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String


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

class TerminalOperator:
    def __init__(self):
        self.events = queue.Queue()
        self.codes = queue.Queue()
        self.keyboard = queue.Queue()
        self.stop_event = threading.Event()
        self.eof = False
        self.camera = CameraStream(self.events)
        self.node = rclpy.create_node('mission_terminal_qr')
        self.node.create_subscription(String, '/zbar_opencv_code', self.qr, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.keyboard_thread = threading.Thread(target=self.read_terminal, daemon=True)
        self.keyboard_thread.start()

    def read_terminal(self):
        for line in sys.stdin:
            text = line.strip()
            if text.lower() in ('q', 'quit', 'exit'):
                self.stop_event.set()
            self.keyboard.put((time.monotonic(), text))
        self.eof = True
        self.stop_event.set()
        self.keyboard.put((time.monotonic(), None))

    def reset(self):
        while True:
            try:
                self.keyboard.get_nowait()
            except queue.Empty:
                break
        if not self.eof:
            self.stop_event.clear()

    def restart_requested(self):
        self.reset()
        while True:
            answer = self.ask('任務已結束。Enter 重新開始；q + Enter 結束程式 > ')
            if answer.lower() in ('q', 'quit', 'exit'):
                return False
            if not answer:
                self.reset()
                return True
            print('請按 Enter 重新開始，或輸入 q 結束。', flush=True)

    def qr(self, msg):
        self.codes.put((time.monotonic(), msg.data.strip()))

    @staticmethod
    def needs_camera(prompt):
        return any(marker in prompt for marker in (
            '請裁判給點餐卡', '請裁判給加點卡', '請裁判給「不須加點」卡',
        ))

    def ask(self, prompt):
        started = time.monotonic()
        scanning = self.needs_camera(prompt)
        if scanning:
            self.camera.set_active(True)
        print(prompt, flush=True)
        if scanning:
            print('相機／QR 啟動中：掃 QR 自動繼續；Enter 手動放行；q + Enter 中止。', flush=True)
        try:
            while True:
                if self.stop_event.is_set():
                    return 'q'
                try:
                    observed, text = self.keyboard.get(timeout=0.05)
                    if text is None:
                        return 'q'
                    # Ignore non-abort input entered before this prompt existed.
                    if observed >= started:
                        return text
                except queue.Empty:
                    pass
                if not scanning:
                    continue
                try:
                    while True:
                        observed, text = self.codes.get_nowait()
                        if observed >= started and text:
                            print('已掃描 QR：' + text, flush=True)
                            return text
                except queue.Empty:
                    pass
                try:
                    while True:
                        kind, text = self.events.get_nowait()
                        if kind == 'camera_error':
                            print(text, flush=True)
                except queue.Empty:
                    pass
        finally:
            if scanning:
                self.camera.set_active(False)
                self.camera.stopped.wait()
                print('相機／QR 已關閉。', flush=True)

    def close(self):
        self.camera.close()
        self.executor.shutdown()
        self.thread.join(timeout=3)
        self.node.destroy_node()
