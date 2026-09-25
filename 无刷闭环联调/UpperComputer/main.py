# -*- coding: utf-8 -*-
"""
Cubli 三轴无刷电机联调上位机 (PyQt5 + pyqtgraph + pyserial)
-------------------------------------------------------------------------
适用硬件: STM32F407ZGT6 + 3x DRV8313 (M1/M2/M3) + LSM6DSRTR / MPU-6050
通信协议: USART2 (PD5-TX, PD6-RX), 115200 8N1
主要特性:
  1. 无多余 emoji，工控简洁高对比度深色风格
  2. 左侧电机调控与参数面板，右侧 3D 姿态立方体与电机转速实时曲线
  3. 修复转速曲线显示异常问题 (采用示波器式平滑滚动时间窗，修复 autoRange 导致的数据出界)
  4. 纯 QPainter 高性能 3D 立方体引擎，与陀螺仪/加速度计互补滤波姿态实时同步，默认水平朝上
  5. 飞轮转速联动动画、坐标轴指示与鼠标自由旋转视角
"""

import sys
import time
import math
import re
from collections import deque

import serial
import serial.tools.list_ports

from PyQt5.QtCore import (
    Qt, QThread, pyqtSignal, QTimer, QMutex, QMutexLocker, QPointF
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QComboBox,
    QSlider, QDoubleSpinBox, QCheckBox, QTextEdit, QLineEdit,
    QSplitter, QFrame, QMessageBox, QTabWidget, QStatusBar, QProgressBar,
    QScrollArea
)
from PyQt5.QtGui import (
    QFont, QColor, QPalette, QPainter, QPolygonF, QPen, QBrush,
    QLinearGradient, QRadialGradient, QPainterPath
)

import pyqtgraph as pg

# 配置 pyqtgraph 全局样式
pg.setConfigOption('background', '#181b20')
pg.setConfigOption('foreground', '#d0d7de')
pg.setConfigOption('antialias', True)


# =========================================================================
# 串口后台通信工作线程 (QThread)
# =========================================================================
class SerialWorker(QThread):
    """串口数据收发与协议解析线程"""
    sig_connected = pyqtSignal(str, int)
    sig_disconnected = pyqtSignal()
    sig_error = pyqtSignal(str)
    sig_log_rx = pyqtSignal(str)
    sig_log_tx = pyqtSignal(str)
    sig_telemetry = pyqtSignal(dict)
    sig_test_step = pyqtSignal(str)     # 自检单步进度: "M1", "M2", "M3"
    sig_test_report = pyqtSignal(dict)  # 自检总结结果字典

    # 自检协议正则:
    # 步骤帧: $TEST_STEP,M1#
    # 总结帧: $TEST_REPORT,M1:PASS:100:432.1,M2:PASS:100:428.9,M3:PASS:100:435.0,ALL:PASS#
    TEST_STEP_PATTERN = re.compile(r'\$TEST_STEP,(M[123])#')
    TEST_REPORT_PATTERN = re.compile(
        r'\$TEST_REPORT,'
        r'M1:([A-Za-z0-9_()]+):(\d+):([0-9.-]+),'
        r'M2:([A-Za-z0-9_()]+):(\d+):([0-9.-]+),'
        r'M3:([A-Za-z0-9_()]+):(\d+):([0-9.-]+),'
        r'ALL:(PASS|FAIL)#'
    )

    # 遥测帧正则:
    # 基础帧: $TELE,M1:%.2f,M2:%.2f,M3:%.2f,Vq:%.2f,EN:%d%d%d,A1:%.2f,A2:%.2f,A3:%.2f#
    # 扩展帧1 (IMU): ...,Gx:%.1f,Gy:%.1f,Gz:%.1f,Ax:%.2f,Ay:%.2f,Az:%.2f#
    # 扩展帧2 (IMU + Hall): ...,H1:%.1f,H2:%.1f,H3:%.1f#
    # 扩展帧3 (Closed-loop Speed): ...,S1:%.2f,S2:%.2f,S3:%.2f#
    TELE_PATTERN = re.compile(
        r'\$TELE,'
        r'M1:([+-]?\d+(?:\.\d+)?),'
        r'M2:([+-]?\d+(?:\.\d+)?),'
        r'M3:([+-]?\d+(?:\.\d+)?),'
        r'Vq:([+-]?\d+(?:\.\d+)?),'
        r'EN:([01])([01])([01]),'
        r'A1:([+-]?\d+(?:\.\d+)?),'
        r'A2:([+-]?\d+(?:\.\d+)?),'
        r'A3:([+-]?\d+(?:\.\d+)?)(?:,'
        r'Gx:([+-]?\d+(?:\.\d+)?),'
        r'Gy:([+-]?\d+(?:\.\d+)?),'
        r'Gz:([+-]?\d+(?:\.\d+)?),'
        r'Ax:([+-]?\d+(?:\.\d+)?),'
        r'Ay:([+-]?\d+(?:\.\d+)?),'
        r'Az:([+-]?\d+(?:\.\d+)?))?'
        r'(?:,H1:([+-]?\d+(?:\.\d+)?),'
        r'H2:([+-]?\d+(?:\.\d+)?),'
        r'H3:([+-]?\d+(?:\.\d+)?))?'
        r'(?:,S1:([+-]?\d+(?:\.\d+)?),'
        r'S2:([+-]?\d+(?:\.\d+)?),'
        r'S3:([+-]?\d+(?:\.\d+)?))?#'
    )

    def __init__(self):
        super().__init__()
        self.ser = None
        self.is_running = False
        self.mutex = QMutex()
        self.tx_queue = deque()

    def connect_port(self, port_name, baud_rate=115200):
        """请求连接串口"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.is_running = True
            if not self.isRunning():
                self.start()
            self.sig_connected.emit(port_name, baud_rate)
        except Exception as e:
            self.sig_error.emit(f"无法打开串口 {port_name}: {str(e)}")

    def disconnect_port(self):
        """请求断开串口"""
        self.is_running = False
        with QMutexLocker(self.mutex):
            self.tx_queue.clear()
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.sig_disconnected.emit()

    def send_cmd(self, cmd_str):
        """添加待发送命令到队列 (自动补充换行符)"""
        cmd_str = cmd_str.strip()
        if not cmd_str:
            return
        payload = (cmd_str + "\r\n").encode('utf-8', errors='ignore')
        with QMutexLocker(self.mutex):
            self.tx_queue.append((cmd_str, payload))

    def run(self):
        rx_buffer = bytearray()
        while self.is_running:
            if not (self.ser and self.ser.is_open):
                time.sleep(0.05)
                continue

            # 1. 处理发送队列
            cmd_to_log = None
            with QMutexLocker(self.mutex):
                if self.tx_queue:
                    cmd_to_log, payload = self.tx_queue.popleft()
            if cmd_to_log:
                try:
                    self.ser.write(payload)
                    self.sig_log_tx.emit(cmd_to_log)
                except Exception as e:
                    self.sig_error.emit(f"发送失败: {str(e)}")

            # 2. 读取接收数据
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(waiting)
                    if chunk:
                        rx_buffer.extend(chunk)
                        while b'\n' in rx_buffer:
                            line_end = rx_buffer.index(b'\n')
                            raw_line = rx_buffer[:line_end]
                            rx_buffer = rx_buffer[line_end + 1:]
                            decoded = raw_line.decode('utf-8', errors='ignore').strip()
                            if decoded:
                                self._process_received_line(decoded)
                else:
                    time.sleep(0.005)
            except Exception as e:
                self.sig_error.emit(f"串口读取异常: {str(e)}")
                self.disconnect_port()
                break

    def _process_received_line(self, line):
        """解析接收到的单行报文"""
        match = self.TELE_PATTERN.search(line)
        if match:
            try:
                m1_spd = float(match.group(1))
                m2_spd = float(match.group(2))
                m3_spd = float(match.group(3))
                vq = float(match.group(4))
                en1 = int(match.group(5)) == 1
                en2 = int(match.group(6)) == 1
                en3 = int(match.group(7)) == 1
                a1 = float(match.group(8))
                a2 = float(match.group(9))
                a3 = float(match.group(10))

                gx = float(match.group(11)) if match.group(11) is not None else 0.0
                gy = float(match.group(12)) if match.group(12) is not None else 0.0
                gz = float(match.group(13)) if match.group(13) is not None else 0.0
                ax = float(match.group(14)) if match.group(14) is not None else 0.0
                ay = float(match.group(15)) if match.group(15) is not None else 0.0
                az = float(match.group(16)) if match.group(16) is not None else 0.0
                has_imu = match.group(11) is not None

                h1 = float(match.group(17)) if match.group(17) is not None else 0.0
                h2 = float(match.group(18)) if match.group(18) is not None else 0.0
                h3 = float(match.group(19)) if match.group(19) is not None else 0.0
                has_hall = match.group(17) is not None

                s1 = float(match.group(20)) if match.group(20) is not None else 0.0
                s2 = float(match.group(21)) if match.group(21) is not None else 0.0
                s3 = float(match.group(22)) if match.group(22) is not None else 0.0
                has_speed = match.group(20) is not None

                tele_data = {
                    'm1_spd': m1_spd, 'm2_spd': m2_spd, 'm3_spd': m3_spd,
                    'vq': vq,
                    'en1': en1, 'en2': en2, 'en3': en3,
                    'a1': a1, 'a2': a2, 'a3': a3,
                    'gx': gx, 'gy': gy, 'gz': gz,
                    'ax': ax, 'ay': ay, 'az': az,
                    'h1': h1, 'h2': h2, 'h3': h3,
                    's1': s1, 's2': s2, 's3': s3,
                    'has_imu': has_imu,
                    'has_hall': has_hall,
                    'has_speed': has_speed,
                    'time': time.time()
                }
                self.sig_telemetry.emit(tele_data)
            except Exception:
                pass

        # 解析自检单步步骤帧: $TEST_STEP,M1#
        match_step = self.TEST_STEP_PATTERN.search(line)
        if match_step:
            self.sig_test_step.emit(match_step.group(1))

        # 解析自检汇总报表帧: $TEST_REPORT,M1:PASS:100:432.1,M2:PASS:100:428.9,M3:PASS:100:435.0,ALL:PASS#
        match_rep = self.TEST_REPORT_PATTERN.search(line)
        if match_rep:
            try:
                rep_data = {
                    'm1': {'status': match_rep.group(1), 'comm': int(match_rep.group(2)), 'deg': float(match_rep.group(3))},
                    'm2': {'status': match_rep.group(4), 'comm': int(match_rep.group(5)), 'deg': float(match_rep.group(6))},
                    'm3': {'status': match_rep.group(7), 'comm': int(match_rep.group(8)), 'deg': float(match_rep.group(9))},
                    'all_pass': match_rep.group(10) == 'PASS'
                }
                self.sig_test_report.emit(rep_data)
            except Exception:
                pass
        elif "ALL 3 MOTORS & ENCODERS ARE HEALTHY (PASS)" in line:
            # 兼容终端回退文本
            self.sig_test_report.emit({
                'all_pass': True,
                'm1': {'status': 'PASS', 'comm': 100, 'deg': 360.0},
                'm2': {'status': 'PASS', 'comm': 100, 'deg': 360.0},
                'm3': {'status': 'PASS', 'comm': 100, 'deg': 360.0}
            })
        elif "ENCODER/MOTOR ANOMALY DETECTED (FAIL)" in line:
            self.sig_test_report.emit({
                'all_pass': False,
                'm1': {'status': 'FAIL', 'comm': 0, 'deg': 0.0},
                'm2': {'status': 'FAIL', 'comm': 0, 'deg': 0.0},
                'm3': {'status': 'FAIL', 'comm': 0, 'deg': 0.0}
            })

        self.sig_log_rx.emit(line)


# =========================================================================
# 三维姿态立方体组件 (Cubli3DWidget - 纯 QPainter 高性能投影)
# =========================================================================
class Cubli3DWidget(QWidget):
    """
    Cubli 三维立方体渲染组件:
    - 纯基于 PyQt5 原生 QPainter 实现 3D 透视投影与画家算法深度着色
    - 与陀螺仪/加速度计互补滤波姿态实时同步
    - 默认初始状态: 水平朝上 (Roll=0, Pitch=0, Yaw=0, Z轴垂直向上)
    - 动量轮旋转动画跟随三电机目标转速
    - 支持鼠标拖拽旋转观察视角，双击恢复默认等轴测视角
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 260)
        self.setSizePolicy(self.sizePolicy().Expanding, self.sizePolicy().Expanding)

        # 欧拉角 (度, 对应机体姿态)
        self.roll = 0.0    # 绕 X 轴旋转 (横滚)
        self.pitch = 0.0   # 绕 Y 轴旋转 (俯仰)
        self.yaw = 0.0     # 绕 Z 轴旋转 (航向)

        # 姿态零位偏移校准
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0

        # 相机观察视角 (度)
        self.cam_elev = 20.0  # 俯仰观察角 (固定 20.0° 低仰视角，符合大致 30° 以下标准)
        self.cam_azim = 35.0  # 方位观察角

        # 鼠标交互
        self._last_mouse_pos = None

        # 动量轮角度 (弧度)
        self.wheel_angles = [0.0, 0.0, 0.0]
        self.encoder_angles = [0.0, 0.0, 0.0]  # MT6701 实时角度 (度)
        self.motor_speeds = [0.0, 0.0, 0.0]

        # 互补滤波相关
        self.last_filter_time = None
        self.alpha = 0.95  # 互补滤波系数 (0.95 陀螺仪积分 + 0.05 加速度计重力)

        # 立方体半边长
        self.cube_size = 75.0

    def update_encoder_angles(self, h1, h2, h3):
        """直接使用 MT6701 实时角度驱动 3D 飞轮旋转"""
        self.encoder_angles = [h1, h2, h3]
        self.wheel_angles[0] = math.radians(h1)
        self.wheel_angles[1] = math.radians(h2)
        self.wheel_angles[2] = math.radians(h3)
        self.update()

    def reset_attitude_zero(self):
        """将当前姿态设为零位"""
        self.roll_offset = self.roll + self.roll_offset
        self.pitch_offset = self.pitch + self.pitch_offset
        self.yaw_offset = self.yaw + self.yaw_offset
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.update()

    def reset_view(self):
        """恢复默认观察视角 (固定 20.0° 低仰俯视视角)"""
        self.cam_elev = 20.0
        self.cam_azim = 35.0
        self.update()

    def update_imu_data(self, gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g):
        """输入 6 轴数据，通过互补滤波解算欧拉角"""
        now = time.time()
        if self.last_filter_time is None:
            dt = 0.05
        else:
            dt = now - self.last_filter_time
            if dt <= 0.0 or dt > 0.2:
                dt = 0.05
        self.last_filter_time = now

        # 加速度计静态姿态解算 (默认水平朝上: Az ≈ +1.0g)
        norm_a = math.sqrt(ax_g * ax_g + ay_g * ay_g + az_g * az_g)
        if norm_a > 0.1:
            ax = ax_g / norm_a
            ay = ay_g / norm_a
            az = az_g / norm_a

            # 俯仰角与横滚角 (度)
            pitch_acc = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * (180.0 / math.pi)
            roll_acc  = math.atan2(ay, az) * (180.0 / math.pi)

            # 互补滤波
            self.pitch = self.alpha * (self.pitch + gy_dps * dt) + (1.0 - self.alpha) * pitch_acc
            self.roll  = self.alpha * (self.roll + gx_dps * dt) + (1.0 - self.alpha) * roll_acc
        else:
            self.pitch += gy_dps * dt
            self.roll  += gx_dps * dt

        # 航向角积分 (陀螺仪 Z 轴)
        self.yaw += gz_dps * dt

        # 累加动量轮转角动画
        for i in range(3):
            self.wheel_angles[i] += self.motor_speeds[i] * dt * 2.0
            if self.wheel_angles[i] > 2 * math.pi:
                self.wheel_angles[i] -= 2 * math.pi
            elif self.wheel_angles[i] < -2 * math.pi:
                self.wheel_angles[i] += 2 * math.pi

        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self._last_mouse_pos is not None:
            dx = event.x() - self._last_mouse_pos.x()
            self._last_mouse_pos = event.pos()

            # 仅改变水平观察方向 (方位角 cam_azim)，俯仰角保持固定不变，地面始终保持绝对水平静止
            self.cam_azim = (self.cam_azim + dx * 0.5) % 360.0
            self.update()

    def mouseReleaseEvent(self, event):
        self._last_mouse_pos = None

    def mouseDoubleClickEvent(self, event):
        self.reset_view()

    # ---------------------------------------------------------------------
    # 3D 数学变换与渲染
    # ---------------------------------------------------------------------
    @staticmethod
    def _rot_x(v, deg):
        rad = math.radians(deg)
        c, s = math.cos(rad), math.sin(rad)
        return (v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c)

    @staticmethod
    def _rot_y(v, deg):
        rad = math.radians(deg)
        c, s = math.cos(rad), math.sin(rad)
        return (v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c)

    @staticmethod
    def _rot_z(v, deg):
        rad = math.radians(deg)
        c, s = math.cos(rad), math.sin(rad)
        return (v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2])

    def _world_to_cam(self, pt_world):
        """世界坐标系 -> 相机视点空间 (标准右手投影，固定 20.0° 低仰视角，地面基准绝对水平静止)"""
        xw, yw, zw = pt_world
        phi = math.radians(self.cam_azim)
        theta = math.radians(self.cam_elev)

        # 1. 绕世界铅垂 Z 轴旋转水平方位角 phi
        x1 = xw * math.cos(phi) - yw * math.sin(phi)
        y1 = xw * math.sin(phi) + yw * math.cos(phi)
        z1 = zw

        # 2. 俯视视角变换 (相机从水平面上方 theta 度向下俯视原点)
        xc = x1
        yc = z1 * math.cos(theta) + y1 * math.sin(theta)
        zc = -z1 * math.sin(theta) + y1 * math.cos(theta)
        return (xc, yc, zc)

    def _body_to_world(self, pt_body):
        """机体局部坐标系 -> 世界坐标系 (由姿态互补滤波解算欧拉角驱动)"""
        p = self._rot_x(pt_body, self.roll - self.roll_offset)
        p = self._rot_y(p, self.pitch - self.pitch_offset)
        p = self._rot_z(p, self.yaw - self.yaw_offset)
        return p

    def _transform_point(self, pt_body):
        """机体局部顶点 -> 世界坐标系 -> 相机投影空间"""
        pt_world = self._body_to_world(pt_body)
        return self._world_to_cam(pt_world)

    def _project(self, p_cam, cx, cy, d_cam=450.0):
        """透视投影变换"""
        z_eff = p_cam[2] + d_cam
        if z_eff < 10.0:
            z_eff = 10.0
        scale = d_cam / z_eff
        sx = cx + p_cam[0] * scale
        sy = cy - p_cam[1] * scale
        return QPointF(sx, sy), z_eff

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.TextAntialiasing, True)

        w = self.width()
        h = self.height()
        cx = w / 2.0
        cy = h / 2.0

        # 背景渐变 (素雅深色工控夜空质感)
        bg_grad = QRadialGradient(cx, cy, max(w, h) * 0.7)
        bg_grad.setColorAt(0.0, QColor(24, 28, 35))
        bg_grad.setColorAt(1.0, QColor(14, 16, 20))
        painter.fillRect(0, 0, w, h, bg_grad)

        s = self.cube_size

        # 1. 绘制绝对静止的参考水平地面 (处于世界坐标系，不随机体旋转，仅响应水平观察视角)
        self._draw_ground(painter, cx, cy)

        # 2. 立方体 8 个局部坐标顶点
        # 约定: Z轴向上 (顶面为+Z), Y轴向前 (前面为+Y), X轴向右 (右面为+X)
        verts_local = [
            (-s, -s, -s),  # 0
            ( s, -s, -s),  # 1
            ( s,  s, -s),  # 2
            (-s,  s, -s),  # 3
            (-s, -s,  s),  # 4
            ( s, -s,  s),  # 5
            ( s,  s,  s),  # 6
            (-s,  s,  s),  # 7
        ]

        # 变换顶点 (机体姿态 -> 世界 -> 相机)
        verts_cam = [self._transform_point(v) for v in verts_local]
        verts_2d = []
        for v in verts_cam:
            p2d, _ = self._project(v, cx, cy)
            verts_2d.append(p2d)

        # 3. 定义 6 个面及对应的飞轮
        # 每个面: (顶点索引[4个], 面法向量(局部), 基础色, 标签名称, 动量轮索引[或None])
        faces = [
            # 顶面 (+Z, M3 动量轮)
            ([4, 5, 6, 7], (0, 0, 1), QColor(41, 128, 185), f"M3: {self.encoder_angles[2]:05.1f}°", 2),
            # 底面 (-Z)
            ([3, 2, 1, 0], (0, 0, -1), QColor(52, 73, 94), "BOTTOM", None),
            # 右侧面 (+X, M1 动量轮)
            ([1, 2, 6, 5], (1, 0, 0), QColor(22, 160, 133), f"M1: {self.encoder_angles[0]:05.1f}°", 0),
            # 左侧面 (-X)
            ([0, 4, 7, 3], (-1, 0, 0), QColor(44, 62, 80), "LEFT", None),
            # 前侧面 (+Y, M2 动量轮)
            ([2, 3, 7, 6], (0, 1, 0), QColor(211, 84, 0), f"M2: {self.encoder_angles[1]:05.1f}°", 1),
            # 后侧面 (-Y)
            ([0, 1, 5, 4], (0, -1, 0), QColor(52, 73, 94), "BACK", None),
        ]

        # 4. 计算每个面的平均相机景深 (Z) 并按深度排序 (画家算法: 远 -> 近)
        face_render_list = []
        for face_idx, (v_idx, n_local, base_color, label, wheel_idx) in enumerate(faces):
            avg_z = sum(verts_cam[i][2] for i in v_idx) / 4.0

            # 变换面法线计算光照
            n_cam = self._transform_point(n_local)
            origin_cam = self._transform_point((0, 0, 0))
            norm = (n_cam[0] - origin_cam[0], n_cam[1] - origin_cam[1], n_cam[2] - origin_cam[2])
            length = math.sqrt(norm[0]**2 + norm[1]**2 + norm[2]**2)
            if length > 0.001:
                norm = (norm[0]/length, norm[1]/length, norm[2]/length)

            # 光源方向 (右上斜向朝场景入射)
            light = (0.35, 0.65, -0.65)
            l_len = math.sqrt(light[0]**2 + light[1]**2 + light[2]**2)
            light = (light[0]/l_len, light[1]/l_len, light[2]/l_len)

            dot = norm[0] * light[0] + norm[1] * light[1] + norm[2] * light[2]
            intensity = max(0.35, min(1.0, 0.55 + dot * 0.45))

            face_render_list.append({
                'avg_z': avg_z,
                'v_idx': v_idx,
                'color': base_color,
                'intensity': intensity,
                'label': label,
                'wheel_idx': wheel_idx,
                'norm_z': norm[2]
            })

        # 按相机 Z 降序 (远 -> 近)
        face_render_list.sort(key=lambda item: item['avg_z'], reverse=True)

        # 5. 绘制各个面
        for item in face_render_list:
            v_idx = item['v_idx']
            pts = [verts_2d[i] for i in v_idx]
            poly = QPolygonF(pts)

            # 颜色明暗调制
            c = item['color']
            shaded_color = QColor(
                int(c.red() * item['intensity']),
                int(c.green() * item['intensity']),
                int(c.blue() * item['intensity']),
                220
            )

            # 填充多边形
            painter.setBrush(QBrush(shaded_color))
            painter.setPen(QPen(QColor(230, 240, 255, 180), 1.5))
            painter.drawPolygon(poly)

            # 如果朝向观察者 (norm_z < 0.25)，绘制面中心文字与动量轮
            if item['norm_z'] < 0.25:
                fc_x = sum(p.x() for p in pts) / 4.0
                fc_y = sum(p.y() for p in pts) / 4.0

                # 绘制动量轮圆盘 (如果有)
                if item['wheel_idx'] is not None:
                    self._draw_momentum_wheel(painter, pts, fc_x, fc_y, item['wheel_idx'])

                # 绘制面文字标号
                painter.setPen(QColor(255, 255, 255, 230))
                font = QFont("Segoe UI", 8, QFont.Bold)
                painter.setFont(font)
                painter.drawText(int(fc_x - 40), int(fc_y - 8), 80, 16, Qt.AlignCenter, item['label'])

        # 6. 在立方体中心绘制机体姿态 RGB 坐标轴
        self._draw_axes(painter, cx, cy)

        # 7. 绘制左上角 HUD 信息 (欧拉角数值与鼠标提示)
        self._draw_hud(painter, w, h)

    def _draw_ground(self, painter, cx, cy):
        """
        绘制绝对静止的参考水平地面 (World Ground Plane):
        - 位于世界坐标系 Zw = -ground_height 处 (略低于立方体底面)
        - 保持水平静止，不随机体姿态 (Roll/Pitch/Yaw) 发生任何旋转倾斜
        - 鼠标水平拖拽只改变水平环视方位角 (cam_azim)，俯仰观察角固定为 <=30° (20°)
        - 绘制半透明基台、正交参考网格、世界坐标基准指示与机体投影阴影
        """
        s = self.cube_size
        ground_z = -s * 1.35  # 地面固定高度 (约 -101)
        grid_half = s * 2.5   # 地面半边长 (约 188)
        steps = 6
        step_sz = grid_half / steps

        # 1. 绘制地面半透明台面基板 (世界坐标)
        corners_world = [
            (-grid_half, -grid_half, ground_z),
            ( grid_half, -grid_half, ground_z),
            ( grid_half,  grid_half, ground_z),
            (-grid_half,  grid_half, ground_z),
        ]
        corners_2d = []
        for pt_w in corners_world:
            p_cam = self._world_to_cam(pt_w)
            p_2d, _ = self._project(p_cam, cx, cy)
            corners_2d.append(p_2d)

        ground_poly = QPolygonF(corners_2d)
        painter.setBrush(QBrush(QColor(18, 22, 28, 190)))
        painter.setPen(QPen(QColor(48, 58, 74, 180), 1.2))
        painter.drawPolygon(ground_poly)

        # 2. 绘制立方体在静止地面上的柔和投影阴影 (增强空间高度感与着地感)
        center_ground_cam = self._world_to_cam((0, 0, ground_z))
        cg_2d, _ = self._project(center_ground_cam, cx, cy)

        painter.setPen(Qt.NoPen)
        shadow_grad = QRadialGradient(cg_2d.x(), cg_2d.y(), s * 1.2)
        shadow_grad.setColorAt(0.0, QColor(8, 10, 14, 180))
        shadow_grad.setColorAt(0.65, QColor(12, 15, 20, 90))
        shadow_grad.setColorAt(1.0, QColor(18, 22, 28, 0))
        painter.setBrush(QBrush(shadow_grad))
        painter.drawEllipse(cg_2d, s * 0.95, s * 0.50)

        # 3. 绘制地面正交参考网格 (仅由世界坐标投射，绝不随机体姿态倾斜)
        pen_grid = QPen(QColor(38, 48, 62, 130), 1, Qt.SolidLine)
        pen_axis = QPen(QColor(64, 82, 108, 200), 1.4, Qt.SolidLine)

        for i in range(-steps, steps + 1):
            is_center = (i == 0)
            painter.setPen(pen_axis if is_center else pen_grid)

            # 平行于 Y_world 轴的纵向网格线
            p1_cam = self._world_to_cam((i * step_sz, -grid_half, ground_z))
            p2_cam = self._world_to_cam((i * step_sz,  grid_half, ground_z))
            p1_2d, _ = self._project(p1_cam, cx, cy)
            p2_2d, _ = self._project(p2_cam, cx, cy)
            painter.drawLine(p1_2d, p2_2d)

            # 平行于 X_world 轴的横向网格线
            p3_cam = self._world_to_cam((-grid_half, i * step_sz, ground_z))
            p4_cam = self._world_to_cam(( grid_half, i * step_sz, ground_z))
            p3_2d, _ = self._project(p3_cam, cx, cy)
            p4_2d, _ = self._project(p4_cam, cx, cy)
            painter.drawLine(p3_2d, p4_2d)

        # 4. 地面中心同心基准圆环
        painter.setPen(QPen(QColor(75, 95, 122, 140), 1.0, Qt.DashLine))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(cg_2d, s * 0.40, s * 0.20)

        # 5. 静止地面台角基准文字与世界水平标识
        corner_label_cam = self._world_to_cam((grid_half - 12, -grid_half + 12, ground_z))
        cl_2d, _ = self._project(corner_label_cam, cx, cy)
        painter.setPen(QColor(94, 114, 138, 160))
        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.drawText(int(cl_2d.x() - 110), int(cl_2d.y() - 14), 120, 14, Qt.AlignRight, "水平静止地面基准")

    def _draw_momentum_wheel(self, painter, pts, fc_x, fc_y, wheel_idx):
        """在面上绘制动量轮圆盘、旋转辐条与 MT6701 角度指针"""
        r1 = math.hypot(pts[0].x() - fc_x, pts[0].y() - fc_y)
        r2 = math.hypot(pts[1].x() - fc_x, pts[1].y() - fc_y)
        r = (r1 + r2) * 0.35

        painter.setPen(QPen(QColor(255, 255, 255, 160), 2))
        painter.setBrush(QBrush(QColor(20, 25, 32, 180)))
        painter.drawEllipse(QPointF(fc_x, fc_y), r, r)

        # 旋转辐条
        ang = self.wheel_angles[wheel_idx]
        painter.setPen(QPen(QColor(241, 196, 15, 200), 1.5))
        for k in range(4):
            spoke_a = ang + k * (math.pi / 2.0)
            sx = fc_x + r * 0.85 * math.cos(spoke_a)
            sy = fc_y + r * 0.85 * math.sin(spoke_a)
            painter.drawLine(QPointF(fc_x, fc_y), QPointF(sx, sy))

        # 红色指针精准指示当前 MT6701 实际角度
        px = fc_x + r * 0.90 * math.cos(ang)
        py = fc_y + r * 0.90 * math.sin(ang)
        painter.setPen(QPen(QColor(231, 76, 60), 2.5))
        painter.drawLine(QPointF(fc_x, fc_y), QPointF(px, py))

    def _draw_axes(self, painter, cx, cy):
        """在中心绘制 X(红), Y(绿), Z(蓝) 空间坐标轴"""
        axis_len = self.cube_size * 1.45
        origin_cam = self._transform_point((0, 0, 0))
        o_2d, _ = self._project(origin_cam, cx, cy)

        axes = [
            ((axis_len, 0, 0), QColor(231, 76, 60), "X"),
            ((0, axis_len, 0), QColor(46, 204, 113), "Y"),
            ((0, 0, axis_len), QColor(52, 152, 219), "Z"),
        ]

        for pt_local, color, name in axes:
            p_cam = self._transform_point(pt_local)
            p_2d, _ = self._project(p_cam, cx, cy)

            painter.setPen(QPen(color, 2))
            painter.drawLine(o_2d, p_2d)

            painter.setFont(QFont("Segoe UI", 9, QFont.Bold))
            painter.drawText(int(p_2d.x() + 4), int(p_2d.y() + 4), name)

    def _draw_hud(self, painter, w, h):
        """绘制左上角姿态 HUD 与操作提示"""
        painter.setFont(QFont("Consolas", 9, QFont.Bold))

        # 姿态数值
        r_deg = self.roll - self.roll_offset
        p_deg = self.pitch - self.pitch_offset
        y_deg = self.yaw - self.yaw_offset

        hud_bg = QColor(20, 24, 30, 210)
        painter.fillRect(10, 10, 240, 106, hud_bg)
        painter.setPen(QPen(QColor(50, 60, 75), 1))
        painter.drawRect(10, 10, 240, 106)

        painter.setPen(QColor(231, 76, 60))
        painter.drawText(18, 28, f"Roll (X) : {r_deg:+.1f} deg")
        painter.setPen(QColor(46, 204, 113))
        painter.drawText(18, 48, f"Pitch(Y) : {p_deg:+.1f} deg")
        painter.setPen(QColor(52, 152, 219))
        painter.drawText(18, 68, f"Yaw  (Z) : {y_deg:+.1f} deg")

        painter.setPen(QColor(16, 185, 129))
        painter.drawText(18, 92, f"MT6701: {self.encoder_angles[0]:05.1f}°|{self.encoder_angles[1]:05.1f}°|{self.encoder_angles[2]:05.1f}°")

        # 底部操作提示 (更新说明：地面保持绝对静止，鼠标仅改变水平环视观察方向，视角为低仰视角 20°)
        painter.setFont(QFont("Segoe UI", 8))
        painter.setPen(QColor(130, 145, 165))
        painter.drawText(10, h - 10, "水平拖拽鼠标改变环视方向 | 固定低仰观察视角(20°) | 地面基准水平静止 | 双击重置视角")


# =========================================================================
# 单电机独立控制卡片组件 (MotorControlCard)
# =========================================================================
class MotorControlCard(QGroupBox):
    """单个电机的控制面板，支持滑条、数值框、RPM换算、使能开关与预设按钮"""
    sig_speed_changed = pyqtSignal(int, float)  # (motor_id, speed_rad_s)
    sig_enable_changed = pyqtSignal(int, bool)   # (motor_id, enabled)

    def __init__(self, motor_id, title, color="#3498db", parent=None):
        super().__init__(title, parent)
        self.motor_id = motor_id  # 1, 2, or 3
        self.color = color
        self._block_signals = False

        self.init_ui()

    def init_ui(self):
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #363d4a;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 12px;
                background-color: #1a1e24;
                color: #e2e8f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #cbd5e1;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(8)

        # 顶部：使能状态与 RPM 换算显示
        top_row = QHBoxLayout()
        self.chk_enable = QCheckBox("驱动使能 (DRV8313)")
        self.chk_enable.setChecked(True)
        self.chk_enable.setStyleSheet("font-weight: bold; color: #34d399;")
        self.chk_enable.toggled.connect(self._on_enable_toggled)

        self.lbl_rpm = QLabel("0.0 RPM")
        self.lbl_rpm.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_rpm.setStyleSheet("font-size: 13px; font-weight: bold; color: #e2e8f0;")

        top_row.addWidget(self.chk_enable)
        top_row.addStretch()
        top_row.addWidget(self.lbl_rpm)
        layout.addLayout(top_row)

        # 霍尔/MT6701 实时角度仪表盒
        hall_box = QFrame()
        hall_box.setStyleSheet("""
            QFrame {
                background-color: #16191f;
                border: 1px solid #2d333f;
                border-radius: 4px;
                padding: 4px 6px;
            }
        """)
        hall_layout = QVBoxLayout(hall_box)
        hall_layout.setContentsMargins(4, 4, 4, 4)
        hall_layout.setSpacing(4)

        hall_header = QHBoxLayout()
        lbl_h_title = QLabel("MT6701 角度/转速:")
        lbl_h_title.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8;")
        self.lbl_hall_status = QLabel("[离线]")
        self.lbl_hall_status.setStyleSheet("font-size: 10px; font-weight: bold; color: #94a3b8;")
        self.lbl_act_speed = QLabel("0.0 rad/s")
        self.lbl_act_speed.setStyleSheet("font-size: 11px; font-weight: bold; color: #38bdf8; margin-right: 4px;")
        self.lbl_hall_val = QLabel("0.0°")
        self.lbl_hall_val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_hall_val.setStyleSheet("font-size: 13px; font-weight: bold; color: #e2e8f0;")

        hall_header.addWidget(lbl_h_title)
        hall_header.addWidget(self.lbl_hall_status)
        hall_header.addStretch()
        hall_header.addWidget(self.lbl_act_speed)
        hall_header.addWidget(self.lbl_hall_val)
        hall_layout.addLayout(hall_header)

        # 0 ~ 360° 角度进度条
        self.progress_angle = QProgressBar()
        self.progress_angle.setRange(0, 3600)  # 0.1° 精度
        self.progress_angle.setValue(0)
        self.progress_angle.setTextVisible(False)
        self.progress_angle.setFixedHeight(5)
        self.progress_angle.setStyleSheet("""
            QProgressBar {
                background-color: #121519;
                border: 1px solid #242a34;
                border-radius: 2px;
            }
            QProgressBar::chunk {
                background-color: #4a5d78;
                border-radius: 1px;
            }
        """)
        hall_layout.addWidget(self.progress_angle)
        layout.addWidget(hall_box)

        # 中部：SpinBox + 滑条调节
        mid_row = QHBoxLayout()
        lbl_target = QLabel("目标转速:")
        lbl_target.setStyleSheet("font-size: 12px; color: #cbd5e1;")

        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(-30.0, 30.0)
        self.spin_speed.setSingleStep(0.2)
        self.spin_speed.setDecimals(2)
        self.spin_speed.setSuffix(" rad/s")
        self.spin_speed.setValue(0.0)
        self.spin_speed.setFixedHeight(26)
        self.spin_speed.setMinimumWidth(100)
        self.spin_speed.setStyleSheet("""
            QDoubleSpinBox {
                background-color: #232832;
                border: 1px solid #363d4a;
                border-radius: 4px;
                padding: 2px 6px;
                font-size: 12px;
                color: #ffffff;
            }
        """)
        self.spin_speed.valueChanged.connect(self._on_spin_changed)

        mid_row.addWidget(lbl_target)
        mid_row.addWidget(self.spin_speed)
        layout.addLayout(mid_row)

        # 滑条 (-30.0 ~ +30.0，以 10 倍整数映射: -300 ~ 300)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(-300, 300)
        self.slider.setValue(0)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(50)
        self.slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #363d4a;
                height: 6px;
                background: #232832;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #475569;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #94a3b8;
                border: 1px solid #cbd5e1;
                width: 14px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 7px;
            }
            QSlider::handle:horizontal:hover {
                background: #e2e8f0;
            }
        """)
        self.slider.valueChanged.connect(self._on_slider_changed)
        layout.addWidget(self.slider)

        # 刻度标记
        scale_row = QHBoxLayout()
        lbl_scale_min = QLabel("-30")
        lbl_scale_min.setStyleSheet("font-size: 10px; color: #64748b;")
        lbl_scale_mid = QLabel("0")
        lbl_scale_mid.setStyleSheet("font-size: 10px; color: #64748b;")
        lbl_scale_max = QLabel("+30 rad/s")
        lbl_scale_max.setStyleSheet("font-size: 10px; color: #64748b;")

        scale_row.addWidget(lbl_scale_min)
        scale_row.addStretch()
        scale_row.addWidget(lbl_scale_mid)
        scale_row.addStretch()
        scale_row.addWidget(lbl_scale_max)
        layout.addLayout(scale_row)

        # 底部快捷按钮行 (紧凑素雅，适配窄屏)
        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)
        btn_row.setContentsMargins(0, 0, 0, 0)

        self.preset_buttons = []
        presets = [("-10", -10.0), ("-5", -5.0), ("0 停", 0.0), ("+5", 5.0), ("+10", 10.0)]
        for text, val in presets:
            btn = QPushButton(text)
            btn.setFixedHeight(26)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #242932;
                    border: 1px solid #363d4a;
                    border-radius: 4px;
                    color: #cbd5e1;
                    font-size: 11px;
                    padding: 2px 4px;
                    min-width: 32px;
                }
                QPushButton:hover {
                    background-color: #323946;
                    border-color: #4f596b;
                    color: #ffffff;
                }
                QPushButton:pressed {
                    background-color: #1a1e24;
                }
                QPushButton:disabled {
                    background-color: #1a1e24;
                    border-color: #262b35;
                    color: #475569;
                }
            """)
            btn.clicked.connect(lambda _, v=val: self.set_speed(v, emit=True))
            self.preset_buttons.append(btn)
            btn_row.addWidget(btn)

        self.btn_rev = QPushButton("反转")
        self.btn_rev.setFixedHeight(26)
        self.btn_rev.setStyleSheet("""
            QPushButton {
                background-color: #242932;
                border: 1px solid #363d4a;
                border-radius: 4px;
                color: #cbd5e1;
                font-size: 11px;
                padding: 2px 4px;
                min-width: 36px;
            }
            QPushButton:hover {
                background-color: #323946;
                border-color: #4f596b;
                color: #ffffff;
            }
            QPushButton:pressed {
                background-color: #1a1e24;
            }
            QPushButton:disabled {
                background-color: #1a1e24;
                border-color: #262b35;
                color: #475569;
            }
        """)
        self.btn_rev.clicked.connect(self._on_reverse_clicked)
        btn_row.addWidget(self.btn_rev)

        layout.addLayout(btn_row)

    def set_operable(self, operable: bool):
        """启用或禁用所有主动控制控件 (滑条、数值框、使能复选框、预设按键)，但保持遥测读数显示"""
        self.chk_enable.setEnabled(operable)
        self.spin_speed.setEnabled(operable)
        self.slider.setEnabled(operable)
        if hasattr(self, 'btn_rev'):
            self.btn_rev.setEnabled(operable)
        if hasattr(self, 'preset_buttons'):
            for btn in self.preset_buttons:
                btn.setEnabled(operable)

    def _on_enable_toggled(self, checked):
        if checked:
            self.chk_enable.setStyleSheet("font-weight: bold; color: #34d399;")
        else:
            self.chk_enable.setStyleSheet("font-weight: bold; color: #f87171;")
        self.sig_enable_changed.emit(self.motor_id, checked)
        self.sig_enable_changed.emit(self.motor_id, checked)

    def _on_spin_changed(self, val):
        if self._block_signals:
            return
        self._block_signals = True
        self.slider.setValue(int(round(val * 10)))
        self._update_rpm_label(val)
        self._block_signals = False
        self.sig_speed_changed.emit(self.motor_id, val)

    def _on_slider_changed(self, int_val):
        if self._block_signals:
            return
        val = int_val / 10.0
        self._block_signals = True
        self.spin_speed.setValue(val)
        self._update_rpm_label(val)
        self._block_signals = False
        self.sig_speed_changed.emit(self.motor_id, val)

    def _on_reverse_clicked(self):
        curr = self.spin_speed.value()
        self.set_speed(-curr, emit=True)

    def _update_rpm_label(self, rad_s):
        rpm = rad_s * 9.5493
        self.lbl_rpm.setText(f"{rpm:+.1f} RPM")

    def set_hall_angle(self, angle_deg, online=True):
        """更新霍尔/磁编码角度读数显示"""
        # 规整到 0 ~ 359.9°
        norm_deg = angle_deg % 360.0
        if norm_deg < 0: norm_deg += 360.0

        if online:
            self.lbl_hall_status.setText("[正常]")
            self.lbl_hall_status.setStyleSheet("font-size: 10px; font-weight: bold; color: #34d399;")
            self.lbl_hall_val.setText(f"{norm_deg:05.1f}°")
            self.lbl_hall_val.setStyleSheet("font-size: 13px; font-weight: bold; color: #e2e8f0;")
            self.progress_angle.setValue(int(norm_deg * 10))
        else:
            self.lbl_hall_status.setText("[离线]")
            self.lbl_hall_status.setStyleSheet("font-size: 10px; font-weight: bold; color: #94a3b8;")
            self.lbl_hall_val.setText(f"{norm_deg:05.1f}°")
            self.lbl_hall_val.setStyleSheet("font-size: 13px; font-weight: bold; color: #64748b;")
            self.progress_angle.setValue(0)

    def set_actual_speed(self, speed_rad_s):
        """更新实测反馈转速显示"""
        self.lbl_act_speed.setText(f"{speed_rad_s:+.2f} rad/s")

    def set_speed(self, val, emit=True):
        """由外部程序更新当前速度设定"""
        self._block_signals = True
        self.spin_speed.setValue(val)
        self.slider.setValue(int(round(val * 10)))
        self._update_rpm_label(val)
        self._block_signals = False
        if emit:
            self.sig_speed_changed.emit(self.motor_id, val)

    def set_enable_state(self, enabled):
        """外部同步使能状态"""
        self.chk_enable.blockSignals(True)
        self.chk_enable.setChecked(enabled)
        if enabled:
            self.chk_enable.setStyleSheet("font-weight: bold; color: #34d399;")
        else:
            self.chk_enable.setStyleSheet("font-weight: bold; color: #f87171;")
        self.chk_enable.blockSignals(False)


# =========================================================================
# 硬件自检与安全互锁卡片组件 (SelfTestCard)
# =========================================================================
class SelfTestCard(QGroupBox):
    """三轴电机与霍尔磁编码器安全自检与互锁控制面板"""
    sig_start_test = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__("硬件安全检测与互锁状态", parent)
        self.init_ui()

    def init_ui(self):
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #363d4a;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                background-color: #1a1e24;
                color: #e2e8f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #cbd5e1;
            }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 12, 10, 10)
        layout.setSpacing(8)

        # 1. 顶部互锁状态标签
        status_row = QHBoxLayout()
        status_row.setSpacing(6)
        lbl_status_title = QLabel("互锁状态:")
        lbl_status_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #94a3b8;")
        status_row.addWidget(lbl_status_title)

        self.lbl_badge = QLabel("未自检 (电机控制锁定)")
        self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #f59e0b;")
        status_row.addWidget(self.lbl_badge)
        status_row.addStretch()
        layout.addLayout(status_row)

        # 2. 执行检测主按钮
        self.btn_run_test = QPushButton("开始硬件自检")
        self.btn_run_test.setFixedHeight(28)
        self.btn_run_test.setStyleSheet("""
            QPushButton {
                background-color: #242932;
                border: 1px solid #363d4a;
                border-radius: 4px;
                font-size: 12px;
                font-weight: bold;
                color: #e2e8f0;
            }
            QPushButton:hover {
                background-color: #323946;
                border-color: #4f596b;
                color: #ffffff;
            }
            QPushButton:disabled {
                background-color: #1a1e24;
                border-color: #262b35;
                color: #64748b;
            }
        """)
        self.btn_run_test.clicked.connect(self.sig_start_test.emit)
        layout.addWidget(self.btn_run_test)

        # 3. 三轴诊断明细网格
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(4)
        grid.setContentsMargins(2, 4, 2, 4)

        self.lbl_m_tags = []
        self.lbl_m_details = []

        motors_meta = [
            ("M1", "TIM1 / I2C1"),
            ("M2", "TIM3 / I2C2"),
            ("M3", "TIM4 / I2C3"),
        ]

        for i, (name, bus) in enumerate(motors_meta):
            lbl_name = QLabel(f"{name} ({bus}):")
            lbl_name.setStyleSheet("font-size: 11px; font-weight: bold; color: #cbd5e1;")
            grid.addWidget(lbl_name, i, 0)

            lbl_status = QLabel("待检测")
            lbl_status.setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8;")
            lbl_status.setAlignment(Qt.AlignCenter)
            grid.addWidget(lbl_status, i, 1)
            self.lbl_m_tags.append(lbl_status)

            lbl_det = QLabel("通信: -- | 转角: --")
            lbl_det.setStyleSheet("font-size: 11px; color: #64748b;")
            lbl_det.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(lbl_det, i, 2)
            self.lbl_m_details.append(lbl_det)

        layout.addLayout(grid)

        # 4. 底部说明提示标签
        self.lbl_hint = QLabel("提示: 控制电机前必须先执行检测，确认三轴电机与霍尔编码器正常。")
        self.lbl_hint.setWordWrap(True)
        self.lbl_hint.setStyleSheet("font-size: 10px; color: #64748b;")
        layout.addWidget(self.lbl_hint)

    def set_testing_state(self, step_str=""):
        self.btn_run_test.setEnabled(False)
        self.btn_run_test.setText("正在执行检测...")
        msg = f"自检中 ({step_str})" if step_str else "自检中 (三轴低速开环测试)"
        self.lbl_badge.setText(msg)
        self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #38bdf8;")
        self.lbl_hint.setText("正在低速驱动电机换向并采样霍尔角度，请勿触碰动量轮。")
        self.lbl_hint.setStyleSheet("font-size: 10px; color: #94a3b8;")

    def set_step(self, motor_str):
        try:
            idx = int(motor_str[1]) - 1
            if 0 <= idx < 3:
                self.lbl_m_tags[idx].setText("检测中...")
                self.lbl_m_tags[idx].setStyleSheet("font-size: 11px; font-weight: bold; color: #38bdf8;")
                self.lbl_badge.setText(f"正在检测 {motor_str} (低速开环与I2C采样)...")
        except Exception:
            pass

    def set_result(self, report):
        self.btn_run_test.setEnabled(True)
        self.btn_run_test.setText("重新自检")

        all_pass = report.get('all_pass', False)
        for i, key in enumerate(['m1', 'm2', 'm3']):
            info = report.get(key, {})
            st = info.get('status', 'FAIL')
            comm = info.get('comm', 0)
            deg = info.get('deg', 0.0)

            if st == 'PASS':
                self.lbl_m_tags[i].setText("正常")
                self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #34d399;")
                self.lbl_m_details[i].setText(f"通信: {comm}% | 转角: {deg:.1f}°")
                self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #34d399;")
            elif st in ('SKIP', 'NONE', 'UNINSTALLED'):
                self.lbl_m_tags[i].setText("未安装")
                self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #64748b;")
                self.lbl_m_details[i].setText("未安装 (已跳过)")
                self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #64748b;")
            else:
                tag = "离线" if "I2C" in st else "堵转"
                self.lbl_m_tags[i].setText(tag)
                self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #f87171;")
                self.lbl_m_details[i].setText(f"通信: {comm}% | 转角: {deg:.1f}°")
                self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #f87171;")

        if all_pass:
            self.lbl_badge.setText("自检通过 (已解锁)")
            self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #34d399;")
            self.lbl_hint.setText("已安装的电机与霍尔编码器自检合格，通道已自动校准，控制功能已解锁。")
            self.lbl_hint.setStyleSheet("font-size: 10px; color: #94a3b8;")
        else:
            self.lbl_badge.setText("自检失败 (控制锁定)")
            self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #f87171;")
            self.lbl_hint.setText("存在异常轴，请检查对应I2C连接、3.3V供电、上拉电阻或电机相线后重测。")
            self.lbl_hint.setStyleSheet("font-size: 10px; color: #94a3b8;")

    def reset_state(self, connected=False):
        self.btn_run_test.setEnabled(connected)
        self.btn_run_test.setText("开始硬件自检")
        if connected:
            self.lbl_badge.setText("待自检 (控制锁定)")
            self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #f59e0b;")
            self.lbl_hint.setText("串口已连接。请点击上方“开始硬件自检”以激活电机控制权限。")
            self.lbl_hint.setStyleSheet("font-size: 10px; color: #94a3b8;")
        else:
            self.lbl_badge.setText("串口未连接 (控制锁定)")
            self.lbl_badge.setStyleSheet("font-size: 12px; font-weight: bold; color: #94a3b8;")
            self.lbl_hint.setText("提示: 请先打开串口，然后执行硬件自检以解锁电机控制。")
            self.lbl_hint.setStyleSheet("font-size: 10px; color: #64748b;")

        for i in range(3):
            self.lbl_m_tags[i].setText("待检测")
            self.lbl_m_tags[i].setStyleSheet("font-size: 11px; font-weight: bold; color: #94a3b8;")
            self.lbl_m_details[i].setText("通信: -- | 转角: --")
            self.lbl_m_details[i].setStyleSheet("font-size: 11px; color: #64748b;")


# =========================================================================
# 主窗口 (MainWindow)
# =========================================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Cubli 三轴无刷电机联调与 3D 姿态上位机 (SimpleFOC & USART2)")
        self.resize(1360, 880)
        self.setMinimumSize(1100, 720)

        # 串口后台工作线程
        self.worker = SerialWorker()
        self.worker.sig_connected.connect(self.on_serial_connected)
        self.worker.sig_disconnected.connect(self.on_serial_disconnected)
        self.worker.sig_error.connect(self.on_serial_error)
        self.worker.sig_log_rx.connect(self.on_log_rx)
        self.worker.sig_log_tx.connect(self.on_log_tx)
        self.worker.sig_telemetry.connect(self.on_telemetry_received)
        self.worker.sig_test_step.connect(self.on_self_test_step)
        self.worker.sig_test_report.connect(self.on_self_test_report)

        # 硬件自检与安全互锁状态 (True: 自检通过，允许操作; False: 锁定拦截)
        self.self_test_passed = False
        self.self_test_running = False

        # 波形历史数据队列 (示波器式平滑滚动缓冲)
        self.MAX_POINTS = 800
        self.history_time = deque(maxlen=self.MAX_POINTS)
        self.history_m1 = deque(maxlen=self.MAX_POINTS)
        self.history_m2 = deque(maxlen=self.MAX_POINTS)
        self.history_m3 = deque(maxlen=self.MAX_POINTS)

        # 闭环实测转速队列 (rad/s)
        self.history_s1 = deque(maxlen=self.MAX_POINTS)
        self.history_s2 = deque(maxlen=self.MAX_POINTS)
        self.history_s3 = deque(maxlen=self.MAX_POINTS)

        # MT6701 霍尔编码器角度历史队列 (0 ~ 360°)
        self.history_h1 = deque(maxlen=self.MAX_POINTS)
        self.history_h2 = deque(maxlen=self.MAX_POINTS)
        self.history_h3 = deque(maxlen=self.MAX_POINTS)

        # IMU 6轴数据
        self.history_gx = deque(maxlen=self.MAX_POINTS)
        self.history_gy = deque(maxlen=self.MAX_POINTS)
        self.history_gz = deque(maxlen=self.MAX_POINTS)
        self.history_ax = deque(maxlen=self.MAX_POINTS)
        self.history_ay = deque(maxlen=self.MAX_POINTS)
        self.history_az = deque(maxlen=self.MAX_POINTS)

        self.start_time = None
        self.tele_fps_counter = 0
        self.tele_last_fps_time = time.time()

        # 滑条防抖定时器 (50ms)
        self.debounce_timer = QTimer()
        self.debounce_timer.setSingleShot(True)
        self.debounce_timer.setInterval(50)
        self.debounce_timer.timeout.connect(self._flush_speed_command)
        self.pending_speed = [0.0, 0.0, 0.0]
        self.pending_flag = False

        # 波形与 3D 渲染定时器 (30 FPS)
        self.plot_timer = QTimer()
        self.plot_timer.setInterval(33)
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start()

        # 串口扫描定时器
        self.port_scan_timer = QTimer()
        self.port_scan_timer.setInterval(2000)
        self.port_scan_timer.timeout.connect(self.refresh_ports)
        self.port_scan_timer.start()

        self.init_ui()
        self.refresh_ports()

    def init_ui(self):
        # 全局深色现代工控风格 (素雅朴素、无多余彩色条框、无 emoji)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #121417;
            }
            QWidget {
                color: #e2e8f0;
                font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
            }
            QFrame#TopBar {
                background-color: #1a1e24;
                border-bottom: 1px solid #2d333f;
                padding: 4px 8px;
            }
            QLabel {
                font-size: 12px;
                color: #cbd5e1;
            }
            QPushButton {
                background-color: #242932;
                border: 1px solid #363d4a;
                border-radius: 4px;
                padding: 4px 8px;
                color: #e2e8f0;
                font-size: 12px;
                min-height: 20px;
            }
            QPushButton:hover {
                background-color: #323946;
                border-color: #4f596b;
                color: #ffffff;
            }
            QPushButton:pressed {
                background-color: #1a1e24;
                border-color: #242932;
            }
            QPushButton:disabled {
                background-color: #1a1e24;
                border-color: #2d333f;
                color: #64748b;
            }
            QComboBox {
                background-color: #20242c;
                border: 1px solid #363d4a;
                border-radius: 4px;
                padding: 3px 6px;
                color: #ffffff;
                font-size: 12px;
                min-width: 80px;
                min-height: 20px;
            }
            QComboBox:hover {
                border-color: #4f596b;
            }
            QDoubleSpinBox {
                background-color: #20242c;
                border: 1px solid #363d4a;
                border-radius: 4px;
                padding: 2px 4px;
                color: #ffffff;
                font-size: 12px;
            }
            QDoubleSpinBox:hover {
                border-color: #4f596b;
            }
            QLineEdit {
                background-color: #1a1e24;
                border: 1px solid #363d4a;
                border-radius: 4px;
                padding: 3px 6px;
                color: #ffffff;
                font-size: 12px;
            }
            QLineEdit:hover {
                border-color: #4f596b;
            }
            QTextEdit {
                background-color: #121418;
                border: 1px solid #282d37;
                border-radius: 4px;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 11px;
                color: #94a3b8;
            }
            QStatusBar {
                background-color: #1a1e24;
                color: #94a3b8;
                font-size: 11px;
                border-top: 1px solid #282d37;
            }
            QCheckBox {
                font-size: 12px;
                color: #cbd5e1;
                spacing: 6px;
            }
            QCheckBox::indicator {
                width: 14px;
                height: 14px;
                border: 1px solid #475569;
                border-radius: 3px;
                background-color: #1a1e24;
            }
            QCheckBox::indicator:checked {
                background-color: #3b82f6;
                border-color: #60a5fa;
            }
        """)

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 1. 顶部操作栏
        top_bar = self._create_top_bar()
        main_layout.addWidget(top_bar)

        # 2. 中部主分割窗格 (左侧调控面板, 右侧 3D 与波形监控)
        splitter_main = QSplitter(Qt.Horizontal)
        splitter_main.setHandleWidth(4)

        # 左侧控制面板 (带自适应滚动区域)
        left_widget = self._create_left_panel()
        splitter_main.addWidget(left_widget)

        # 右侧图表与 3D 姿态监控面板
        right_widget = self._create_right_panel()
        splitter_main.addWidget(right_widget)

        splitter_main.setSizes([450, 910])
        main_layout.addWidget(splitter_main)

        # 3. 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪 - 请选择串口并打开连接")

        # 初始安全互锁：锁定所有电机致动控制，等待串口连接并执行自检
        self.set_motor_controls_locked(True)
        self.card_self_test.reset_state(connected=False)

    def _create_top_bar(self):
        bar = QFrame()
        bar.setObjectName("TopBar")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(6)

        # 标题标签 (素雅高亮中性白，无多余彩色)
        lbl_brand = QLabel("Cubli 联调监控")
        lbl_brand.setStyleSheet("font-size: 14px; font-weight: bold; color: #f1f5f9; margin-right: 6px;")
        layout.addWidget(lbl_brand)

        # 串口选择
        layout.addWidget(QLabel("串口:"))
        self.combo_ports = QComboBox()
        self.combo_ports.setMinimumWidth(130)
        layout.addWidget(self.combo_ports)

        btn_refresh = QPushButton("刷新")
        btn_refresh.setFixedHeight(26)
        btn_refresh.clicked.connect(self.refresh_ports)
        layout.addWidget(btn_refresh)

        # 波特率选择
        layout.addWidget(QLabel("波特率:"))
        self.combo_baud = QComboBox()
        for baud in [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]:
            self.combo_baud.addItem(str(baud), baud)
        self.combo_baud.setCurrentText("115200")
        self.combo_baud.setMinimumWidth(80)
        layout.addWidget(self.combo_baud)

        # 打开/关闭按钮 (素雅工控风)
        self.btn_connect = QPushButton("打开串口")
        self.btn_connect.setFixedHeight(26)
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background-color: #242932;
                border: 1px solid #363d4a;
                font-weight: bold;
                padding: 3px 10px;
            }
            QPushButton:hover {
                background-color: #323946;
                border-color: #4f596b;
            }
        """)
        self.btn_connect.clicked.connect(self.toggle_connection)
        layout.addWidget(self.btn_connect)

        # 状态指示标签 (纯文本素雅显示)
        self.lbl_status_led = QLabel("[离线]")
        self.lbl_status_led.setStyleSheet("color: #94a3b8; font-weight: bold; margin-left: 4px;")
        layout.addWidget(self.lbl_status_led)

        layout.addStretch()

        # 3D 姿态归零按钮 (标准工控按钮)
        btn_zero_att = QPushButton("姿态归零")
        btn_zero_att.setFixedHeight(26)
        btn_zero_att.clicked.connect(self.action_reset_cube_attitude)
        layout.addWidget(btn_zero_att)

        # 全部归零 (平稳停止)
        self.btn_stop_all = QPushButton("全部归零")
        self.btn_stop_all.setFixedHeight(26)
        self.btn_stop_all.clicked.connect(self.action_stop_all)
        layout.addWidget(self.btn_stop_all)

        # 紧急停止按钮 (素雅工控暗红警示，避免刺眼彩色外框)
        btn_estop = QPushButton("紧急停止 [ESTOP]")
        btn_estop.setFixedHeight(26)
        btn_estop.setStyleSheet("""
            QPushButton {
                background-color: #991b1b;
                border: 1px solid #b91c1c;
                font-weight: bold;
                font-size: 12px;
                padding: 3px 10px;
                border-radius: 4px;
                color: #ffffff;
            }
            QPushButton:hover {
                background-color: #b91c1c;
                border-color: #dc2626;
            }
            QPushButton:pressed {
                background-color: #7f1d1d;
            }
        """)
        btn_estop.clicked.connect(self.action_emergency_stop)
        layout.addWidget(btn_estop)

        return bar

    def _create_left_panel(self):
        panel = QWidget()
        panel.setObjectName("LeftPanelContent")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 1. 硬件自检与安全互锁卡片 (处于最顶端，必须首先自检通过方可解锁操作)
        self.card_self_test = SelfTestCard()
        self.card_self_test.sig_start_test.connect(self.action_start_self_test)
        layout.addWidget(self.card_self_test)

        # 2. 三轴独立控制卡片 (素雅工控风，标题清晰注明定时器和引脚)
        self.card_m1 = MotorControlCard(1, "电机 1 (M1) - [TIM1: PE9/11/13 | EN: PD0]")
        self.card_m2 = MotorControlCard(2, "电机 2 (M2) - [TIM3: PA6/7 PB0 | EN: PD1]")
        self.card_m3 = MotorControlCard(3, "电机 3 (M3) - [TIM4: PD12/13/14 | EN: PD2]")

        self.card_m1.sig_speed_changed.connect(self.on_motor_speed_change)
        self.card_m2.sig_speed_changed.connect(self.on_motor_speed_change)
        self.card_m3.sig_speed_changed.connect(self.on_motor_speed_change)

        self.card_m1.sig_enable_changed.connect(self.on_motor_enable_change)
        self.card_m2.sig_enable_changed.connect(self.on_motor_enable_change)
        self.card_m3.sig_enable_changed.connect(self.on_motor_enable_change)

        layout.addWidget(self.card_m1)
        layout.addWidget(self.card_m2)
        layout.addWidget(self.card_m3)

        # 3. 三轴联动同步卡片 (去除刺眼紫色边框和背景，统一素雅风格)
        self.sync_group = QGroupBox("三轴同步联动 (Master / Sync)")
        self.sync_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #363d4a;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                background-color: #1a1e24;
                color: #e2e8f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #cbd5e1;
            }
        """)
        sync_layout = QVBoxLayout(self.sync_group)
        sync_layout.setContentsMargins(10, 12, 10, 10)
        sync_layout.setSpacing(8)

        row_sync = QHBoxLayout()
        row_sync.setSpacing(8)
        lbl_s_target = QLabel("同步目标:")
        lbl_s_target.setStyleSheet("font-size: 12px; color: #cbd5e1;")
        row_sync.addWidget(lbl_s_target)

        self.spin_sync = QDoubleSpinBox()
        self.spin_sync.setRange(-30.0, 30.0)
        self.spin_sync.setSingleStep(0.5)
        self.spin_sync.setDecimals(2)
        self.spin_sync.setSuffix(" rad/s")
        self.spin_sync.setValue(0.0)
        self.spin_sync.setFixedHeight(26)
        self.spin_sync.setStyleSheet("""
            QDoubleSpinBox {
                background-color: #232832;
                border: 1px solid #363d4a;
                border-radius: 4px;
                padding: 2px 6px;
                color: #ffffff;
            }
        """)
        row_sync.addWidget(self.spin_sync)

        btn_sync_apply = QPushButton("一键同步三轴")
        btn_sync_apply.setFixedHeight(26)
        btn_sync_apply.clicked.connect(self.action_apply_sync)
        row_sync.addWidget(btn_sync_apply)
        sync_layout.addLayout(row_sync)

        # 同步滑条 (素雅灰蓝槽体与滑块)
        self.slider_sync = QSlider(Qt.Horizontal)
        self.slider_sync.setRange(-300, 300)
        self.slider_sync.setValue(0)
        self.slider_sync.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px;
                background: #232832;
                border: 1px solid #363d4a;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #475569;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #94a3b8;
                border: 1px solid #cbd5e1;
                width: 14px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 7px;
            }
            QSlider::handle:horizontal:hover {
                background: #e2e8f0;
            }
        """)
        self.slider_sync.valueChanged.connect(lambda v: self.spin_sync.setValue(v / 10.0))
        self.spin_sync.valueChanged.connect(lambda v: self.slider_sync.setValue(int(round(v * 10))))
        sync_layout.addWidget(self.slider_sync)

        layout.addWidget(self.sync_group)

        # 4. 电源与相电压安全参数 (去除彩色边框与刺眼青绿背景，规范排版与尺寸)
        self.param_group = QGroupBox("参数配置与驱动限制")
        self.param_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #363d4a;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                background-color: #1a1e24;
                color: #e2e8f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #cbd5e1;
            }
        """)
        param_layout = QGridLayout(self.param_group)
        param_layout.setVerticalSpacing(8)
        param_layout.setHorizontalSpacing(8)
        param_layout.setContentsMargins(10, 12, 10, 10)

        # 相电压限制 Vq limit
        lbl_vq = QLabel("相电压限制 (Vq):")
        self.spin_vq = QDoubleSpinBox()
        self.spin_vq.setRange(0.5, 6.0)
        self.spin_vq.setSingleStep(0.1)
        self.spin_vq.setValue(2.5)
        self.spin_vq.setSuffix(" V")
        self.spin_vq.setFixedHeight(26)
        self.btn_set_vq = QPushButton("设定Vq")
        self.btn_set_vq.setFixedHeight(26)
        self.btn_set_vq.clicked.connect(self.action_set_voltage_limit)

        param_layout.addWidget(lbl_vq, 0, 0)
        param_layout.addWidget(self.spin_vq, 0, 1)
        param_layout.addWidget(self.btn_set_vq, 0, 2)

        # 速度上限 limit
        lbl_vlim = QLabel("开环极速限制:")
        self.spin_vlim = QDoubleSpinBox()
        self.spin_vlim.setRange(1.0, 50.0)
        self.spin_vlim.setSingleStep(1.0)
        self.spin_vlim.setValue(20.0)
        self.spin_vlim.setSuffix(" rad/s")
        self.spin_vlim.setFixedHeight(26)
        self.btn_set_vlim = QPushButton("设定Limit")
        self.btn_set_vlim.setFixedHeight(26)
        self.btn_set_vlim.clicked.connect(self.action_set_velocity_limit)

        param_layout.addWidget(lbl_vlim, 1, 0)
        param_layout.addWidget(self.spin_vlim, 1, 1)
        param_layout.addWidget(self.btn_set_vlim, 1, 2)

        # 遥测输出与状态查询
        self.chk_tele = QCheckBox("开启固件遥测流 ($TELE)")
        self.chk_tele.setChecked(True)
        self.chk_tele.toggled.connect(self.action_toggle_tele)
        btn_query = QPushButton("查询状态 (?)")
        btn_query.setFixedHeight(26)
        btn_query.clicked.connect(lambda: self.worker.send_cmd("?"))

        param_layout.addWidget(self.chk_tele, 2, 0, 1, 2)
        param_layout.addWidget(btn_query, 2, 2)

        # 传感器与编码器查询按钮行 (并排对齐，素雅标准按钮)
        sensor_row = QHBoxLayout()
        sensor_row.setSpacing(6)
        sensor_row.setContentsMargins(0, 0, 0, 0)

        btn_query_imu = QPushButton("查询IMU")
        btn_query_imu.setFixedHeight(26)
        btn_query_imu.clicked.connect(lambda: self.worker.send_cmd("IMU"))

        btn_sw_sensor = QPushButton("切换传感器")
        btn_sw_sensor.setFixedHeight(26)
        btn_sw_sensor.clicked.connect(self.action_switch_sensor)

        btn_query_hall = QPushButton("查询编码器(HALL)")
        btn_query_hall.setFixedHeight(26)
        btn_query_hall.clicked.connect(lambda: self.worker.send_cmd("HALL"))

        sensor_row.addWidget(btn_query_imu)
        sensor_row.addWidget(btn_sw_sensor)
        sensor_row.addWidget(btn_query_hall)

        param_layout.addLayout(sensor_row, 3, 0, 1, 3)

        layout.addWidget(self.param_group)

        # 5. 速度闭环与电角度标定卡片 (MT6701)
        self.cl_group = QGroupBox("速度闭环控制与电角度标定 (MT6701)")
        self.cl_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #363d4a;
                border-radius: 6px;
                margin-top: 8px;
                padding-top: 10px;
                background-color: #1a1e24;
                color: #e2e8f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #cbd5e1;
            }
        """)
        cl_layout = QVBoxLayout(self.cl_group)
        cl_layout.setContentsMargins(10, 12, 10, 10)
        cl_layout.setSpacing(8)

        # 模式切换与校准按键行
        mode_btn_row = QHBoxLayout()
        mode_btn_row.setSpacing(6)

        self.btn_mode_toggle = QPushButton("闭环调速模式 [已开启]")
        self.btn_mode_toggle.setFixedHeight(26)
        self.btn_mode_toggle.setStyleSheet("""
            QPushButton {
                background-color: #1e3a5f;
                border: 1px solid #2563eb;
                font-weight: bold;
                color: #60a5fa;
            }
            QPushButton:hover {
                background-color: #2563eb;
                color: #ffffff;
            }
        """)
        self.btn_mode_toggle.clicked.connect(self.action_toggle_control_mode)

        btn_align = QPushButton("电角度零位对齐 (ALIGN)")
        btn_align.setFixedHeight(26)
        btn_align.clicked.connect(self.action_align_sensors)

        mode_btn_row.addWidget(self.btn_mode_toggle)
        mode_btn_row.addWidget(btn_align)
        cl_layout.addLayout(mode_btn_row)

        # PID 参数在线微调行
        pid_row = QHBoxLayout()
        pid_row.setSpacing(4)

        lbl_p = QLabel("P:")
        lbl_p.setStyleSheet("font-size: 11px; color: #94a3b8;")
        self.spin_p = QDoubleSpinBox()
        self.spin_p.setRange(0.01, 5.0)
        self.spin_p.setSingleStep(0.02)
        self.spin_p.setDecimals(2)
        self.spin_p.setValue(0.15)
        self.spin_p.setFixedHeight(24)

        lbl_i = QLabel("I:")
        lbl_i.setStyleSheet("font-size: 11px; color: #94a3b8;")
        self.spin_i = QDoubleSpinBox()
        self.spin_i.setRange(0.0, 20.0)
        self.spin_i.setSingleStep(0.05)
        self.spin_i.setDecimals(2)
        self.spin_i.setValue(0.35)
        self.spin_i.setFixedHeight(24)

        lbl_d = QLabel("D:")
        lbl_d.setStyleSheet("font-size: 11px; color: #94a3b8;")
        self.spin_d = QDoubleSpinBox()
        self.spin_d.setRange(0.0, 1.0)
        self.spin_d.setSingleStep(0.001)
        self.spin_d.setDecimals(4)
        self.spin_d.setValue(0.0000)
        self.spin_d.setFixedHeight(24)

        btn_send_pid = QPushButton("下发PID")
        btn_send_pid.setFixedHeight(24)
        btn_send_pid.clicked.connect(self.action_send_pid)

        pid_row.addWidget(lbl_p)
        pid_row.addWidget(self.spin_p)
        pid_row.addWidget(lbl_i)
        pid_row.addWidget(self.spin_i)
        pid_row.addWidget(lbl_d)
        pid_row.addWidget(self.spin_d)
        pid_row.addWidget(btn_send_pid)
        cl_layout.addLayout(pid_row)

        layout.addWidget(self.cl_group)
        layout.addStretch()

        # 使用 QScrollArea 容器包裹左侧控制面板，彻底避免小屏下垂直挤压导致的文字变形与截断
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setStyleSheet("""
            QScrollArea {
                background-color: transparent;
                border: none;
            }
            QWidget#LeftPanelContent {
                background-color: #121417;
            }
            QScrollBar:vertical {
                background: #121417;
                width: 8px;
                margin: 0px;
            }
            QScrollBar::handle:vertical {
                background: #2e3542;
                min-height: 24px;
                border-radius: 4px;
            }
            QScrollBar::handle:vertical:hover {
                background: #434d5f;
            }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
                height: 0px;
            }
        """)
        scroll.setWidget(panel)

        return scroll

    def _create_right_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # 1. 顶部指标栏 (采用 2行x3列 网格布局，彻底解决单行 6 卡片文字挤压截断问题)
        status_card_bar = QGridLayout()
        status_card_bar.setSpacing(6)
        status_card_bar.setContentsMargins(0, 0, 0, 0)

        self.card_stat_m1 = self._create_metric_box("M1 目标 / 实测转速", "0.0 / 0.0 rad/s")
        self.card_stat_m2 = self._create_metric_box("M2 目标 / 实测转速", "0.0 / 0.0 rad/s")
        self.card_stat_m3 = self._create_metric_box("M3 目标 / 实测转速", "0.0 / 0.0 rad/s")
        self.card_stat_hall = self._create_metric_box("MT6701 角度 (H1 / H2 / H3)", "0.0° / 0.0° / 0.0°")
        self.card_stat_gyro = self._create_metric_box("陀螺仪 (Gx / Gy / Gz)", "0.0 / 0.0 / 0.0 dps")
        self.card_stat_fps = self._create_metric_box("通信状态与帧率", "0 FPS")

        status_card_bar.addWidget(self.card_stat_m1['box'], 0, 0)
        status_card_bar.addWidget(self.card_stat_m2['box'], 0, 1)
        status_card_bar.addWidget(self.card_stat_m3['box'], 0, 2)
        status_card_bar.addWidget(self.card_stat_hall['box'], 1, 0)
        status_card_bar.addWidget(self.card_stat_gyro['box'], 1, 1)
        status_card_bar.addWidget(self.card_stat_fps['box'], 1, 2)
        layout.addLayout(status_card_bar)

        # 2. 中部左右/上下布局 (左上 3D 立方体，右上/下方曲线)
        # 上半区: 3D 姿态立方体 (与陀螺仪同步，默认水平朝上)
        cube_group = QGroupBox("Cubli 空间三维姿态 (陀螺仪/加速度计同步 | 默认水平朝上)")
        cube_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #363d4a;
                border-radius: 6px;
                margin-top: 6px;
                padding-top: 8px;
                background-color: #16181d;
                color: #e2e8f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #cbd5e1;
            }
        """)
        cube_layout = QVBoxLayout(cube_group)
        cube_layout.setContentsMargins(4, 4, 4, 4)

        self.cube_widget = Cubli3DWidget()
        cube_layout.addWidget(self.cube_widget)
        layout.addWidget(cube_group, stretch=3)

        # 下半区: 实时动态波形监控选项卡 (默认展示 MT6701 实时角度曲线)
        curve_group = QGroupBox("实时动态波形监控 (示波器式平滑滚动)")
        curve_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #363d4a;
                border-radius: 6px;
                margin-top: 4px;
                padding-top: 6px;
                background-color: #16181d;
                color: #e2e8f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #cbd5e1;
            }
        """)
        curve_layout = QVBoxLayout(curve_group)
        curve_layout.setContentsMargins(4, 4, 4, 4)

        self.tab_curves = QTabWidget()
        self.tab_curves.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #363d4a;
                border-radius: 4px;
                background-color: #16181d;
            }
            QTabBar::tab {
                background-color: #1a1e24;
                border: 1px solid #363d4a;
                padding: 4px 12px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                color: #94a3b8;
                font-weight: bold;
                font-size: 11px;
            }
            QTabBar::tab:selected {
                background-color: #242932;
                color: #f1f5f9;
                border-bottom-color: #242932;
            }
        """)

        # Tab 1: MT6701 实时角度波形 (0° ~ 360°)
        self.plot_angle = pg.PlotWidget(title="三轴 MT6701 实时编码器角度 (0° ~ 360°)")
        self.plot_angle.showGrid(x=True, y=True, alpha=0.35)
        self.plot_angle.addLegend(offset=(10, 10))
        self.plot_angle.setLabel('left', 'Angle', units='deg')
        self.plot_angle.setLabel('bottom', 'Time', units='s')
        self.plot_angle.setYRange(0.0, 360.0, padding=0.02)
        self.plot_angle.setXRange(0.0, 10.0, padding=0.0)

        for pos in [90.0, 180.0, 270.0]:
            self.plot_angle.addItem(pg.InfiniteLine(pos=pos, angle=0, pen=pg.mkPen('#334155', width=1, style=Qt.DashLine)))

        self.curve_h1 = self.plot_angle.plot(pen=pg.mkPen('#00bcd4', width=2.4), name="M1 MT6701 (I2C1)")
        self.curve_h2 = self.plot_angle.plot(pen=pg.mkPen('#ff9800', width=2.4), name="M2 MT6701 (I2C2)")
        self.curve_h3 = self.plot_angle.plot(pen=pg.mkPen('#e91e63', width=2.4), name="M3 MT6701 (I2C3)")
        self.tab_curves.addTab(self.plot_angle, "MT6701 角度波形 (0~360°)")

        # Tab 2: 电机目标与闭环实测转速波形
        self.plot_speed = pg.PlotWidget(title="三轴电机速度闭环监控 (rad/s | 虚线:目标设定, 实线:实测反馈)")
        self.plot_speed.showGrid(x=True, y=True, alpha=0.35)
        self.plot_speed.addLegend(offset=(10, 10))
        self.plot_speed.setLabel('left', 'Speed', units='rad/s')
        self.plot_speed.setLabel('bottom', 'Time', units='s')
        self.plot_speed.setYRange(-25, 25, padding=0.05)
        self.plot_speed.setXRange(0, 10, padding=0.0)
        self.plot_speed.addItem(pg.InfiniteLine(pos=0.0, angle=0, pen=pg.mkPen('#445566', width=1.2, style=Qt.DashLine)))

        self.curve_m1_tgt = self.plot_speed.plot(pen=pg.mkPen('#00bcd4', width=1.5, style=Qt.DashLine), name="M1 目标")
        self.curve_s1_act = self.plot_speed.plot(pen=pg.mkPen('#38bdf8', width=2.4), name="M1 实测")
        self.curve_m2_tgt = self.plot_speed.plot(pen=pg.mkPen('#ff9800', width=1.5, style=Qt.DashLine), name="M2 目标")
        self.curve_s2_act = self.plot_speed.plot(pen=pg.mkPen('#facc15', width=2.4), name="M2 实测")
        self.curve_m3_tgt = self.plot_speed.plot(pen=pg.mkPen('#e91e63', width=1.5, style=Qt.DashLine), name="M3 目标")
        self.curve_s3_act = self.plot_speed.plot(pen=pg.mkPen('#f43f5e', width=2.4), name="M3 实测")
        self.tab_curves.addTab(self.plot_speed, "电机转速波形 (rad/s)")

        # Tab 3: 陀螺仪角速度波形
        self.plot_gyro = pg.PlotWidget(title="陀螺仪三轴角速度 (dps = °/s)")
        self.plot_gyro.showGrid(x=True, y=True, alpha=0.35)
        self.plot_gyro.addLegend(offset=(10, 10))
        self.plot_gyro.setLabel('left', 'Angular Velocity', units='dps')
        self.plot_gyro.setLabel('bottom', 'Time', units='s')
        self.plot_gyro.setYRange(-200, 200, padding=0.05)
        self.plot_gyro.setXRange(0, 10, padding=0.0)
        self.curve_gx = self.plot_gyro.plot(pen=pg.mkPen('#00e676', width=2.0), name="Gx (Roll)")
        self.curve_gy = self.plot_gyro.plot(pen=pg.mkPen('#ffeb3b', width=2.0), name="Gy (Pitch)")
        self.curve_gz = self.plot_gyro.plot(pen=pg.mkPen('#38bdf8', width=2.0), name="Gz (Yaw)")
        self.tab_curves.addTab(self.plot_gyro, "陀螺仪角速度 (dps)")

        # Tab 4: 加速度计波形
        self.plot_accel = pg.PlotWidget(title="三轴重力加速度 (g)")
        self.plot_accel.showGrid(x=True, y=True, alpha=0.35)
        self.plot_accel.addLegend(offset=(10, 10))
        self.plot_accel.setLabel('left', 'Acceleration', units='g')
        self.plot_accel.setLabel('bottom', 'Time', units='s')
        self.plot_accel.setYRange(-2.0, 2.0, padding=0.05)
        self.plot_accel.setXRange(0, 10, padding=0.0)
        self.curve_ax = self.plot_accel.plot(pen=pg.mkPen('#ff5252', width=2.0), name="Ax")
        self.curve_ay = self.plot_accel.plot(pen=pg.mkPen('#69f0ae', width=2.0), name="Ay")
        self.curve_az = self.plot_accel.plot(pen=pg.mkPen('#448aff', width=2.0), name="Az")
        self.tab_curves.addTab(self.plot_accel, "三轴加速度 (g)")

        curve_layout.addWidget(self.tab_curves)
        layout.addWidget(curve_group, stretch=3)

        # 3. 底部终端原始报文日志
        log_group = QGroupBox("串口通信监控与终端指令")
        log_group.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #363d4a;
                border-radius: 6px;
                margin-top: 4px;
                padding-top: 6px;
                background-color: #16181d;
                color: #e2e8f0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 12px;
                padding: 0 4px;
                color: #cbd5e1;
            }
        """)
        log_layout = QVBoxLayout(log_group)
        log_layout.setContentsMargins(6, 6, 6, 6)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setMaximumHeight(100)
        log_layout.addWidget(self.txt_log)

        log_ctrl_row = QHBoxLayout()
        log_ctrl_row.setSpacing(8)
        self.chk_autoscroll = QCheckBox("自动滚动")
        self.chk_autoscroll.setChecked(True)
        log_ctrl_row.addWidget(self.chk_autoscroll)

        self.chk_filter_tele = QCheckBox("隐藏遥测帧")
        self.chk_filter_tele.setChecked(True)
        log_ctrl_row.addWidget(self.chk_filter_tele)

        btn_clear_log = QPushButton("清空日志")
        btn_clear_log.setFixedHeight(26)
        btn_clear_log.clicked.connect(self.txt_log.clear)
        log_ctrl_row.addWidget(btn_clear_log)

        log_ctrl_row.addSpacing(10)
        lbl_cmd = QLabel("指令:")
        lbl_cmd.setStyleSheet("font-size: 12px; color: #cbd5e1;")
        log_ctrl_row.addWidget(lbl_cmd)
        self.edt_custom_cmd = QLineEdit()
        self.edt_custom_cmd.setFixedHeight(26)
        self.edt_custom_cmd.setPlaceholderText("例如: M 5.0 -3.0 0.0 或 STOP 或 IMU")
        self.edt_custom_cmd.returnPressed.connect(self.action_send_custom_cmd)
        log_ctrl_row.addWidget(self.edt_custom_cmd)

        btn_send_cmd = QPushButton("发送")
        btn_send_cmd.setFixedHeight(26)
        btn_send_cmd.clicked.connect(self.action_send_custom_cmd)
        log_ctrl_row.addWidget(btn_send_cmd)

        log_layout.addLayout(log_ctrl_row)
        layout.addWidget(log_group, stretch=1)

        return panel

    def _create_metric_box(self, title, initial_val, color=None):
        """创建素雅朴素的指标卡片，彻底去除刺眼彩色边框"""
        box = QFrame()
        box.setStyleSheet("""
            QFrame {
                background-color: #1a1e24;
                border: 1px solid #363d4a;
                border-radius: 4px;
                padding: 4px 8px;
            }
        """)
        l = QVBoxLayout(box)
        l.setContentsMargins(2, 2, 2, 2)
        l.setSpacing(2)

        lbl_t = QLabel(title)
        lbl_t.setStyleSheet("font-size: 11px; color: #94a3b8;")

        lbl_v = QLabel(initial_val)
        lbl_v.setStyleSheet("font-size: 13px; font-weight: bold; color: #f1f5f9;")

        l.addWidget(lbl_t)
        l.addWidget(lbl_v)
        return {'box': box, 'title': lbl_t, 'val': lbl_v}

    # =====================================================================
    # 串口连接控制 (无 emoji)
    # =====================================================================
    def refresh_ports(self):
        current = self.combo_ports.currentText()
        ports = serial.tools.list_ports.comports()
        self.combo_ports.blockSignals(True)
        self.combo_ports.clear()
        found_current = False
        for p in ports:
            display = f"{p.device} ({p.description})"
            self.combo_ports.addItem(display, p.device)
            if p.device == current or display == current:
                found_current = True

        if found_current:
            self.combo_ports.setCurrentText(current)
        elif self.combo_ports.count() > 0:
            self.combo_ports.setCurrentIndex(0)
        self.combo_ports.blockSignals(False)

    def toggle_connection(self):
        if self.worker.ser and self.worker.ser.is_open:
            self.worker.disconnect_port()
        else:
            port = self.combo_ports.currentData()
            if not port:
                port = self.combo_ports.currentText().split()[0]
            if not port:
                QMessageBox.warning(self, "提示", "未检测到可用串口！")
                return
            baud = int(self.combo_baud.currentText())
            self.worker.connect_port(port, baud)

    def on_serial_connected(self, port, baud):
        self.btn_connect.setText("关闭串口")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background-color: #242932;
                border: 1px solid #4f596b;
                font-weight: bold;
                padding: 3px 10px;
                color: #f87171;
            }
            QPushButton:hover {
                background-color: #323946;
                border-color: #ef4444;
            }
        """)
        self.lbl_status_led.setText(f"[在线 {port}@{baud}]")
        self.lbl_status_led.setStyleSheet("color: #34d399; font-weight: bold; margin-left: 4px;")
        self.status_bar.showMessage(f"已连接串口: {port}，系统处于安全锁定状态，请先执行硬件自检！")

        # 连接成功后，重置自检状态并锁定电机操作
        self.self_test_passed = False
        self.self_test_running = False
        self.set_motor_controls_locked(True)
        self.card_self_test.reset_state(connected=True)

        # 重置时间原点，确保曲线从 t=0 开始平稳推进
        self.start_time = time.time()
        self.history_time.clear()
        self.history_m1.clear()
        self.history_m2.clear()
        self.history_m3.clear()
        self.history_s1.clear()
        self.history_s2.clear()
        self.history_s3.clear()
        self.history_gx.clear()
        self.history_gy.clear()
        self.history_gz.clear()
        self.history_ax.clear()
        self.history_ay.clear()
        self.history_az.clear()

        QTimer.singleShot(300, lambda: self.worker.send_cmd("?"))

    def on_serial_disconnected(self):
        self.btn_connect.setText("打开串口")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background-color: #242932;
                border: 1px solid #363d4a;
                font-weight: bold;
                padding: 3px 10px;
                color: #e2e8f0;
            }
            QPushButton:hover {
                background-color: #323946;
                border-color: #4f596b;
            }
        """)
        self.lbl_status_led.setText("[离线]")
        self.lbl_status_led.setStyleSheet("color: #94a3b8; font-weight: bold; margin-left: 4px;")
        self.status_bar.showMessage("串口已断开")

        # 断开连接时，恢复互锁锁定状态
        self.self_test_passed = False
        self.self_test_running = False
        self.set_motor_controls_locked(True)
        self.card_self_test.reset_state(connected=False)

    def on_serial_error(self, err_msg):
        self.status_bar.showMessage(f"错误: {err_msg}")
        self.append_log(f"<span style='color: #e74c3c;'>[ERROR] {err_msg}</span>")

    # =====================================================================
    # 硬件自检与安全互锁机制
    # =====================================================================
    def set_motor_controls_locked(self, locked: bool, active_motors=(True, True, True)):
        """设置电机主动控制互锁状态 (True: 锁定禁止操作; False: 解锁允许操作，仅激活已安装电机)"""
        if hasattr(self, 'card_m1'):
            self.card_m1.set_operable(not locked and active_motors[0])
        if hasattr(self, 'card_m2'):
            self.card_m2.set_operable(not locked and active_motors[1])
        if hasattr(self, 'card_m3'):
            self.card_m3.set_operable(not locked and active_motors[2])
        if hasattr(self, 'sync_group'):
            self.sync_group.setEnabled(not locked)
        if hasattr(self, 'cl_group'):
            self.cl_group.setEnabled(not locked)
        if hasattr(self, 'btn_set_vq'):
            self.btn_set_vq.setEnabled(not locked)
        if hasattr(self, 'btn_set_vlim'):
            self.btn_set_vlim.setEnabled(not locked)
        if hasattr(self, 'btn_stop_all'):
            self.btn_stop_all.setEnabled(not locked)
        # 注意: 紧急停止按键 btn_estop 始终保持启用，绝不互锁锁定！

    def action_start_self_test(self):
        """用户点击开始自检按钮"""
        if not (self.worker.ser and self.worker.ser.is_open):
            QMessageBox.warning(self, "未连接串口", "请先在上方选择并打开串口，然后再执行硬件自检！")
            return

        self.self_test_passed = False
        self.self_test_running = True
        self.set_motor_controls_locked(True)
        self.card_self_test.set_testing_state()
        self.worker.send_cmd("TEST")
        self.status_bar.showMessage("已向单片机发送自检指令 (TEST)，正在执行低速开环与编码器测试...")
        self.append_log("<span style='color: #38bdf8;'>[自检] 开始三轴电机与霍尔磁编码器硬件自检...</span>")

    def on_self_test_step(self, step_str):
        """自检步骤更新: M1 / M2 / M3"""
        self.card_self_test.set_step(step_str)
        self.status_bar.showMessage(f"正在自检 {step_str} 电机及对应霍尔编码器...")

    def on_self_test_report(self, report_data):
        """自检完成汇总数据接收与结果展示"""
        self.self_test_running = False
        all_pass = report_data.get('all_pass', False)
        self.self_test_passed = all_pass
        self.card_self_test.set_result(report_data)

        m1_active = report_data.get('m1', {}).get('status') == 'PASS'
        m2_active = report_data.get('m2', {}).get('status') == 'PASS'
        m3_active = report_data.get('m3', {}).get('status') == 'PASS'

        if all_pass:
            self.set_motor_controls_locked(False, (m1_active, m2_active, m3_active))
            self.status_bar.showMessage("硬件自检通过：已安装的电机与编码器正常，控制权限已解锁。")
            self.append_log("<span style='color: #34d399; font-weight: bold;'>[自检] 自检通过：已安装的电机与编码器均正常，控制功能已解锁。</span>")
        else:
            self.set_motor_controls_locked(True)
            self.status_bar.showMessage("硬件自检未通过：检测到异常，保持电机控制安全锁定。")
            self.append_log("<span style='color: #f87171; font-weight: bold;'>[自检] 自检失败：检测到异常轴，请检查硬件连接后重试。</span>")

    # =====================================================================
    # 电机控制动作
    # =====================================================================
    def on_motor_speed_change(self, motor_id, speed):
        self.pending_speed[motor_id - 1] = speed
        self.pending_flag = True
        self.cube_widget.motor_speeds[motor_id - 1] = speed
        if not self.debounce_timer.isActive():
            self.debounce_timer.start()

    def _flush_speed_command(self):
        if self.pending_flag:
            v1, v2, v3 = self.pending_speed
            cmd = f"M {v1:.2f} {v2:.2f} {v3:.2f}"
            self.worker.send_cmd(cmd)
            self.pending_flag = False

    def on_motor_enable_change(self, motor_id, enabled):
        val = 1 if enabled else 0
        cmd = f"EN{motor_id} {val}"
        self.worker.send_cmd(cmd)

    def action_apply_sync(self):
        val = self.spin_sync.value()
        self.card_m1.set_speed(val, emit=False)
        self.card_m2.set_speed(val, emit=False)
        self.card_m3.set_speed(val, emit=False)
        self.pending_speed = [val, val, val]
        self.cube_widget.motor_speeds = [val, val, val]
        cmd = f"M {val:.2f} {val:.2f} {val:.2f}"
        self.worker.send_cmd(cmd)

    def action_stop_all(self):
        self.card_m1.set_speed(0.0, emit=False)
        self.card_m2.set_speed(0.0, emit=False)
        self.card_m3.set_speed(0.0, emit=False)
        self.spin_sync.setValue(0.0)
        self.pending_speed = [0.0, 0.0, 0.0]
        self.cube_widget.motor_speeds = [0.0, 0.0, 0.0]
        self.worker.send_cmd("STOP")

    def action_emergency_stop(self):
        self.card_m1.set_speed(0.0, emit=False)
        self.card_m2.set_speed(0.0, emit=False)
        self.card_m3.set_speed(0.0, emit=False)
        self.card_m1.set_enable_state(False)
        self.card_m2.set_enable_state(False)
        self.card_m3.set_enable_state(False)
        self.spin_sync.setValue(0.0)
        self.pending_speed = [0.0, 0.0, 0.0]
        self.cube_widget.motor_speeds = [0.0, 0.0, 0.0]
        self.worker.send_cmd("STOP")
        self.worker.send_cmd("EN 0 0 0")
        self.status_bar.showMessage("紧急停止已触发: 电机转速归零并切断驱动使能！")

    def action_set_voltage_limit(self):
        val = self.spin_vq.value()
        self.worker.send_cmd(f"U {val:.2f}")

    def action_set_velocity_limit(self):
        val = self.spin_vlim.value()
        self.worker.send_cmd(f"L {val:.2f}")

    def action_switch_sensor(self):
        self.worker.send_cmd("SENSOR 1")
        self.status_bar.showMessage("已向固件发送激活 LSM6DSRTR (SPI2) 指令")

    def action_toggle_tele(self, checked):
        val = 1 if checked else 0
        self.worker.send_cmd(f"TELE {val}")

    def action_toggle_control_mode(self):
        if "已开启" in self.btn_mode_toggle.text():
            self.worker.send_cmd("MODE 0")
            self.btn_mode_toggle.setText("开环模式 [已切换]")
            self.btn_mode_toggle.setStyleSheet("""
                QPushButton {
                    background-color: #242932;
                    border: 1px solid #363d4a;
                    font-weight: bold;
                    color: #94a3b8;
                }
            """)
            self.status_bar.showMessage("已向固件发送切换开环速度模式指令 (MODE 0)")
        else:
            self.worker.send_cmd("MODE 1")
            self.btn_mode_toggle.setText("闭环调速模式 [已开启]")
            self.btn_mode_toggle.setStyleSheet("""
                QPushButton {
                    background-color: #1e3a5f;
                    border: 1px solid #2563eb;
                    font-weight: bold;
                    color: #60a5fa;
                }
                QPushButton:hover {
                    background-color: #2563eb;
                    color: #ffffff;
                }
            """)
            self.status_bar.showMessage("已向固件发送切换闭环速度模式指令 (MODE 1)")

    def action_align_sensors(self):
        self.worker.send_cmd("ALIGN")
        self.status_bar.showMessage("已触发三轴电角度零位自动校准 (ALIGN)，请等待转子静止")

    def action_send_pid(self):
        p = self.spin_p.value()
        i = self.spin_i.value()
        d = self.spin_d.value()
        self.worker.send_cmd(f"PID {p:.3f} {i:.3f} {d:.4f}")
        self.status_bar.showMessage(f"已更新速度环 PID: P={p:.3f}, I={i:.3f}, D={d:.4f}")

    def action_reset_cube_attitude(self):
        """重置 3D 立方体姿态零位"""
        self.cube_widget.reset_attitude_zero()
        self.status_bar.showMessage("3D 立方体姿态已重置归零")

    def action_send_custom_cmd(self):
        cmd = self.edt_custom_cmd.text().strip()
        if not cmd:
            return

        # 互锁拦截：如果尚未通过硬件自检，拦截致动类电机控制指令，保护设备安全
        if not self.self_test_passed:
            cmd_upper = cmd.upper()
            allowed_prefixes = ("TEST", "CHECK", "?", "H", "HELP", "IMU", "HALL", "ENC", "TELE", "SENSOR", "STOP")
            if not any(cmd_upper.startswith(p) for p in allowed_prefixes):
                self.status_bar.showMessage("操作已被互锁拦截：必须先执行并通过硬件自检后方可控制电机。")
                self.append_log(f"<span style='color: #f59e0b;'>[拦截] 未通过硬件自检，拒绝执行电机控制指令: {cmd}</span>")
                self.edt_custom_cmd.clear()
                return

        self.worker.send_cmd(cmd)
        self.edt_custom_cmd.clear()

    # =====================================================================
    # 遥测与波形数据处理 (解决曲线显示异常与滑动视口)
    # =====================================================================
    def on_telemetry_received(self, data):
        """接收并存储遥测数据点"""
        if self.start_time is None:
            self.start_time = data['time']

        t_rel = data['time'] - self.start_time

        self.history_time.append(t_rel)
        self.history_m1.append(data['m1_spd'])
        self.history_m2.append(data['m2_spd'])
        self.history_m3.append(data['m3_spd'])

        # 记录 6 轴 IMU 数据
        self.history_gx.append(data['gx'])
        self.history_gy.append(data['gy'])
        self.history_gz.append(data['gz'])
        self.history_ax.append(data['ax'])
        self.history_ay.append(data['ay'])
        self.history_az.append(data['az'])

        # 同步更新 3D 立方体姿态
        if data['has_imu']:
            self.cube_widget.update_imu_data(
                data['gx'], data['gy'], data['gz'],
                data['ax'], data['ay'], data['az']
            )

        # 同步更新三轴霍尔/磁编码角度到对应控制卡片、3D立方体与顶部仪表栏
        if data.get('has_hall', False):
            self.card_m1.set_hall_angle(data['h1'], online=True)
            self.card_m2.set_hall_angle(data['h2'], online=True)
            self.card_m3.set_hall_angle(data['h3'], online=True)
            self.history_h1.append(data['h1'])
            self.history_h2.append(data['h2'])
            self.history_h3.append(data['h3'])
            self.card_stat_hall['val'].setText(f"{data['h1']:05.1f}° / {data['h2']:05.1f}° / {data['h3']:05.1f}°")
            # 实时同步更新 3D 立方体动量轮与表面角度标签
            self.cube_widget.update_encoder_angles(data['h1'], data['h2'], data['h3'])

        # 同步动量轮速度动画
        self.cube_widget.motor_speeds = [data['m1_spd'], data['m2_spd'], data['m3_spd']]

        # 同步更新三轴实测闭环转速
        if data.get('has_speed', False):
            s1 = data['s1']
            s2 = data['s2']
            s3 = data['s3']
            self.history_s1.append(s1)
            self.history_s2.append(s2)
            self.history_s3.append(s3)
            self.card_m1.set_actual_speed(s1)
            self.card_m2.set_actual_speed(s2)
            self.card_m3.set_actual_speed(s3)
            self.card_stat_m1['val'].setText(f"{data['m1_spd']:+.1f} / {s1:+.1f} rad/s")
            self.card_stat_m2['val'].setText(f"{data['m2_spd']:+.1f} / {s2:+.1f} rad/s")
            self.card_stat_m3['val'].setText(f"{data['m3_spd']:+.1f} / {s3:+.1f} rad/s")
        else:
            self.card_stat_m1['val'].setText(f"{data['m1_spd']:+.2f} rad/s")
            self.card_stat_m2['val'].setText(f"{data['m2_spd']:+.2f} rad/s")
            self.card_stat_m3['val'].setText(f"{data['m3_spd']:+.2f} rad/s")

        if data['has_imu']:
            self.card_stat_gyro['val'].setText(f"{data['gx']:+.1f}/{data['gy']:+.1f}/{data['gz']:+.1f} dps")

        # 统计通信帧率
        self.tele_fps_counter += 1
        elapsed = time.time() - self.tele_last_fps_time
        if elapsed >= 1.0:
            fps = self.tele_fps_counter / elapsed
            self.card_stat_fps['val'].setText(f"{fps:.1f} FPS")
            self.tele_fps_counter = 0
            self.tele_last_fps_time = time.time()

    def update_plots(self):
        """30 FPS 定时刷新波形图 (示波器平滑滚动时间窗)"""
        if not self.history_time or len(self.history_time) < 2:
            return

        t_data = list(self.history_time)
        n = len(t_data)
        if n < 2:
            return

        t_cur = t_data[-1]
        t_min = max(0.0, t_cur - 10.0)
        t_max = max(10.0, t_cur)

        # 1. 刷新 MT6701 实时角度曲线 (0° ~ 360°)
        if len(self.history_h1) >= n:
            h1_data = list(self.history_h1)[:n]
            h2_data = list(self.history_h2)[:n]
            h3_data = list(self.history_h3)[:n]
            self.curve_h1.setData(t_data, h1_data)
            self.curve_h2.setData(t_data, h2_data)
            self.curve_h3.setData(t_data, h3_data)
            self.plot_angle.setXRange(t_min, t_max, padding=0.01)

        # 2. 刷新电机转速曲线 (目标虚线 vs 实测实线)
        if len(self.history_m1) >= n:
            m1_data = list(self.history_m1)[:n]
            m2_data = list(self.history_m2)[:n]
            m3_data = list(self.history_m3)[:n]
            self.curve_m1_tgt.setData(t_data, m1_data)
            self.curve_m2_tgt.setData(t_data, m2_data)
            self.curve_m3_tgt.setData(t_data, m3_data)

            if len(self.history_s1) >= n:
                s1_data = list(self.history_s1)[:n]
                s2_data = list(self.history_s2)[:n]
                s3_data = list(self.history_s3)[:n]
                self.curve_s1_act.setData(t_data, s1_data)
                self.curve_s2_act.setData(t_data, s2_data)
                self.curve_s3_act.setData(t_data, s3_data)
                all_speeds = m1_data + m2_data + m3_data + s1_data + s2_data + s3_data
            else:
                all_speeds = m1_data + m2_data + m3_data

            self.plot_speed.setXRange(t_min, t_max, padding=0.01)
            min_v = min(all_speeds)
            max_v = max(all_speeds)
            bound = max(15.0, abs(min_v) + 5.0, abs(max_v) + 5.0)
            self.plot_speed.setYRange(-bound, bound, padding=0.05)

        # 3. 刷新陀螺仪角速度曲线
        if len(self.history_gx) >= n:
            self.curve_gx.setData(t_data, list(self.history_gx)[:n])
            self.curve_gy.setData(t_data, list(self.history_gy)[:n])
            self.curve_gz.setData(t_data, list(self.history_gz)[:n])
            self.plot_gyro.setXRange(t_min, t_max, padding=0.01)

        # 4. 刷新加速度计曲线
        if len(self.history_ax) >= n:
            self.curve_ax.setData(t_data, list(self.history_ax)[:n])
            self.curve_ay.setData(t_data, list(self.history_ay)[:n])
            self.curve_az.setData(t_data, list(self.history_az)[:n])
            self.plot_accel.setXRange(t_min, t_max, padding=0.01)

    # =====================================================================
    # 日志输出管理
    # =====================================================================
    def on_log_rx(self, line):
        if self.chk_filter_tele.isChecked() and line.startswith("$TELE"):
            return
        self.append_log(f"<span style='color: #27ae60;'>[RX]</span> {line}")

    def on_log_tx(self, line):
        self.append_log(f"<span style='color: #2980b9;'>[TX]</span> {line}")

    def append_log(self, html_text):
        ts = time.strftime("%H:%M:%S")
        entry = f"<span style='color: #6c7a89;'>[{ts}]</span> {html_text}"
        self.txt_log.append(entry)
        if self.chk_autoscroll.isChecked():
            self.txt_log.moveCursor(self.txt_log.textCursor().End)

    def closeEvent(self, event):
        """平稳释放资源"""
        self.plot_timer.stop()
        self.port_scan_timer.stop()
        self.debounce_timer.stop()
        if self.worker.isRunning():
            self.worker.disconnect_port()
            self.worker.wait(500)
        event.accept()


# =========================================================================
# 程序入口
# =========================================================================
def main():
    if hasattr(Qt, 'AA_EnableHighDpiScaling'):
        QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
        QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(20, 22, 26))
    palette.setColor(QPalette.WindowText, QColor(230, 230, 230))
    palette.setColor(QPalette.Base, QColor(24, 27, 32))
    palette.setColor(QPalette.AlternateBase, QColor(30, 34, 41))
    palette.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
    palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
    palette.setColor(QPalette.Text, QColor(230, 230, 230))
    palette.setColor(QPalette.Button, QColor(34, 39, 48))
    palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
    palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
    palette.setColor(QPalette.Highlight, QColor(52, 152, 219))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
