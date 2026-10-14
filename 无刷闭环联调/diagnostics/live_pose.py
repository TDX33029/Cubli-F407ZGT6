"""Best-effort pose publication; display failures never block serial control."""
import json
import math
import os
import time


class PosePublisher:
    def __init__(self, path, target=None):
        self.path = path
        self.target = target
        self.last_write = -1
        self.pose = {}
        self.pose_time = 0
        self.state = {}

    def update(self, data):
        if not self.path:
            return
        if 'Pit' in data and 'Rol' in data:
            self.pose = dict(data)
            self.pose_time = time.time()
        if 'App' in data:
            self.state = dict(data)
        if not self.pose or time.monotonic() - self.last_write < .1:
            return
        self.last_write = time.monotonic()
        payload = {'timestamp': self.pose_time, 'pitch': float(self.pose['Pit']),
                   'roll': float(self.pose['Rol']), 'bal': int(data.get('Bal', self.state.get('Bal', 0))),
                   'count_ms': int(self.state.get('CountMs', 0)), 'target': self.target,
                   'speeds': [float(self.pose.get(f'V{i}', self.pose.get(f'S{i}', 0))) for i in (1, 2, 3)]}
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary.write_text(json.dumps(payload), encoding='utf-8')
            os.replace(temporary, self.path)
        except OSError:
            pass


def gravity_from_degrees(pitch, roll):
    p, r = math.radians(pitch), math.radians(roll)
    return [-math.sin(r)*math.cos(p), math.sin(p), math.cos(r)*math.cos(p)]
