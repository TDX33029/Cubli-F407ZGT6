"""Read-only pose guide for commissioning scripts that own the serial port."""
import argparse
import json
import math
import time
from pathlib import Path

from PyQt5.QtCore import QPointF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QFontDatabase, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget


def gravity(pitch, roll):
    p, r = math.radians(pitch), math.radians(roll)
    return [-math.sin(r)*math.cos(p), math.sin(p), math.cos(r)*math.cos(p)]


class PoseMonitor(QWidget):
    def __init__(self, live_file, capture_file):
        super().__init__()
        font_path=Path('C:/Windows/Fonts/msyh.ttc')
        if font_path.exists():
            QFontDatabase.addApplicationFont(str(font_path))
            QApplication.instance().setFont(QFont('Microsoft YaHei',12))
        self.live_file, self.capture_file = live_file, capture_file
        self.data = {}
        self.captured_until = 0
        self.setWindowTitle('Cubli 实时入位提示（只读）')
        self.setWindowFlag(Qt.WindowStaysOnTopHint,True)
        self.resize(800, 620)
        self.setStyleSheet('QWidget {background:#111827;color:#e5e7eb;} QLabel {font-size:20px;} QPushButton {font-size:18px;padding:12px;background:#374151;}')
        layout = QVBoxLayout(self)
        self.status = QLabel('等待实时姿态')
        self.status.setAlignment(Qt.AlignCenter)
        self.angles = QLabel('Pitch —     Roll —')
        self.angles.setAlignment(Qt.AlignCenter)
        self.target = QLabel('目标尚未设置')
        self.target.setAlignment(Qt.AlignCenter)
        self.error = QLabel('圆心为当前候选目标，圆圈为2°范围')
        self.error.setAlignment(Qt.AlignCenter)
        self.plot = QWidget()
        self.plot.setStyleSheet('background:transparent;')
        self.plot.setMinimumHeight(280)
        self.speeds = QLabel('M1 —    M2 —    M3 —')
        self.speeds.setAlignment(Qt.AlignCenter)
        self.capture = QPushButton('记录当前姿态为候选（不启动电机）')
        self.capture.clicked.connect(self.capture_pose)
        for widget in (self.status, self.angles, self.target, self.error, self.plot, self.speeds, self.capture):
            layout.addWidget(widget)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def refresh(self):
        try:
            self.data = json.loads(self.live_file.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            pass
        data = self.data
        fresh = bool(data) and time.time() - data.get('timestamp', 0) < 1
        self.capture.setEnabled(fresh and data.get('bal') == 0)
        if not fresh:
            self.status.setText('等待实时姿态 · 当前显示已暂停')
            self.update()
            return
        bal = data['bal']
        self.status.setText({0:'电机失能 · 可定位采样', 1:'静止校准中', 2:'等待接管',
                             3:'正在控制 · 请松手', 4:'保护停机',
                             5:f"倒计时 {data.get('count_ms',0)/1000:.1f} 秒 · 请扶稳",
                             6:'等待入位 · 请扶稳'}.get(bal, '短脉冲测试中'))
        if bal == 0 and time.time() < self.captured_until:
            self.status.setText('姿态候选已记录 · 尚未验证为真实平衡点')
        self.angles.setText(f"当前 Pitch {data['pitch']:+.2f}°    Roll {data['roll']:+.2f}°")
        target = data.get('target')
        if target:
            self.target.setText(f"候选目标 Pitch {target[0]:+.2f}°    Roll {target[1]:+.2f}°")
            g, ref = gravity(data['pitch'], data['roll']), gravity(*target)
            angle = math.degrees(math.acos(max(-1,min(1,sum(a*b for a,b in zip(g,ref))))))
            self.error.setText(f'距候选目标 {angle:.2f}° · 圆圈为2°范围')
        self.speeds.setText('    '.join(f'M{i} {v:+.1f} rad/s' for i,v in enumerate(data['speeds'],1)))
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        rectangle = self.plot.geometry()
        center = QPointF(rectangle.center())
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        scale = min(rectangle.width()/10, rectangle.height()/6)
        painter.setPen(QPen(QColor('#475569'), 1))
        painter.drawLine(QPointF(rectangle.left()+20,center.y()), QPointF(rectangle.right()-20,center.y()))
        painter.drawLine(QPointF(center.x(),rectangle.top()+10), QPointF(center.x(),rectangle.bottom()-10))
        painter.setPen(QColor('#94a3b8'))
        painter.drawText(QPointF(center.x()+10,rectangle.top()+25),'Pitch ↑')
        painter.drawText(QPointF(rectangle.right()-90,center.y()-10),'Roll →')
        painter.setPen(QPen(QColor('#22c55e'), 2))
        painter.drawEllipse(center,2*scale,2*scale)
        data = self.data
        if data.get('target'):
            pitch, roll = data['target']
            x = (data['roll']-roll)*math.cos(math.radians(pitch))*scale
            y = -(data['pitch']-pitch)*scale
            x = max(-rectangle.width()/2+20,min(rectangle.width()/2-20,x))
            y = max(-rectangle.height()/2+20,min(rectangle.height()/2-20,y))
            color = QColor('#38bdf8') if time.time()-data.get('timestamp',0)<1 else QColor('#64748b')
            painter.setBrush(color)
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(QPointF(center.x()+x,center.y()+y),9,9)

    def capture_pose(self):
        if not self.capture.isEnabled():
            return
        data = self.data
        candidate = {'pitch_deg':data['pitch'],'roll_deg':data['roll'],
                     'gravity':gravity(data['pitch'],data['roll']),
                     'captured_at':time.time(),'neutral_verified':False,'motor_motion_commanded':False}
        self.capture_file.parent.mkdir(parents=True,exist_ok=True)
        self.capture_file.write_text(json.dumps(candidate,indent=2)+'\n',encoding='utf-8')
        self.captured_until=time.time()+3
        self.status.setText('姿态候选已记录 · 尚未验证为真实平衡点')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--live-file',type=Path,required=True)
    parser.add_argument('--capture-file',type=Path,required=True)
    args=parser.parse_args()
    app=QApplication([])
    window=PoseMonitor(args.live_file,args.capture_file)
    screen=app.primaryScreen().availableGeometry()
    window.resize(min(800,screen.width()-60),min(620,screen.height()-80))
    window.move(screen.center()-window.rect().center())
    window.show()
    window.raise_()
    window.activateWindow()
    app.exec_()
