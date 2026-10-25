"""Dual-only receive provenance. No SDK, ROS, CAN or device initialization."""
import math
import time
import uuid

FRAMES = {'joint12': 0x2A5, 'joint34': 0x2A6, 'joint56': 0x2A7,
          'gripper': 0x2A8, 'status': 0x2A1}


class FeedbackTracker:
    def __init__(self, clock=time.monotonic, wall_clock=time.time):
        self.clock = clock
        self.wall_clock = wall_clock
        self.epoch = uuid.uuid4().hex
        self.frames = {}
        self.consumed = {key: 0 for key in FRAMES}
        self.communication_error = False

    def received(self, frame_id, source_stamp, ros_stamp):
        key = next((key for key, value in FRAMES.items() if value == frame_id), None)
        if key is None:
            return
        # SDK stamps come from python-can's received frame, not publication.
        if not all(math.isfinite(x) and x > 0 for x in (source_stamp, ros_stamp)):
            return
        old = self.frames.get(key)
        if old is not None and source_stamp <= old['source_stamp']:
            return  # Replay/frozen SDK timestamps do not prove new reception.
        delay = self.wall_clock()-source_stamp
        if not math.isfinite(delay) or delay < 0 or ros_stamp-delay <= 0:
            return
        # Account for socket/SDK backlog. A newly parsed OLD CAN frame is old.
        self.frames[key] = dict(count=1 if old is None else old['count']+1,
                                received=self.clock()-delay, source_stamp=source_stamp,
                                ros_stamp=ros_stamp-delay)

    def report(self, timeout):
        now = self.clock()
        frames = {key: dict(count=value['count'], age_s=now-value['received'],
                            source_stamp=value['source_stamp']) for key, value in self.frames.items()}
        healthy = (len(frames) == len(FRAMES) and not self.communication_error
                   and all(0 <= value['age_s'] <= timeout for value in frames.values()))
        return dict(schema=1, epoch=self.epoch, healthy=healthy,
                    communication_error=self.communication_error, frames=frames)

    def take_stamp(self, timeout):
        if not self.report(timeout)['healthy']:
            return None
        if not all(self.frames[key]['count'] > self.consumed[key] for key in FRAMES):
            return None
        self.consumed = {key: value['count'] for key, value in self.frames.items()}
        return min(value['ros_stamp'] for value in self.frames.values())


class FeedbackWatch:
    """Receiver watchdog: heartbeat repetition cannot advance receive progress."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.arms = {}

    def update(self, side, payload, timeout):
        now = self.clock()
        if (payload.get('schema') != 1 or not isinstance(payload.get('epoch'), str)
                or not payload['epoch'] or type(payload.get('healthy')) is not bool
                or set(payload.get('frames', {})) != set(FRAMES)):
            raise ValueError('invalid feedback provenance')
        previous = self.arms.get(side)
        restarted = previous is not None and previous['epoch'] != payload['epoch']
        old = None if restarted else previous
        frames = {}
        for key, value in payload['frames'].items():
            count, age = value['count'], value['age_s']
            stamp = value['source_stamp']
            if (type(count) is not int or count <= 0 or isinstance(age, bool)
                    or not math.isfinite(age) or age < 0
                    or not math.isfinite(stamp) or stamp <= 0):
                raise ValueError('invalid feedback counter/age')
            last = None if old is None else old['frames'][key]
            if last is not None and (count < last['count'] or stamp < last['source_stamp']):
                raise ValueError('feedback counter/timestamp regressed')
            advanced = last is None or (count > last['count'] and stamp > last['source_stamp'])
            progress = now-age if advanced else last['progress']
            frames[key] = dict(count=count, source_stamp=stamp, progress=progress)
        self.arms[side] = dict(epoch=payload['epoch'], frames=frames, received=now,
                               healthy=payload['healthy'] and not payload.get('communication_error', False))
        self.check(side, timeout)
        if restarted:
            raise ValueError('feedback driver restarted; explicit start required')

    def check(self, side, timeout):
        now = self.clock()
        state = self.arms.get(side)
        if state is None or not state['healthy'] or not 0 <= now-state['received'] <= timeout:
            raise ValueError(side + ': missing/unhealthy CAN feedback')
        for key, frame in state['frames'].items():
            if not 0 <= now-frame['progress'] <= timeout:
                raise ValueError(side + ': CAN feedback stalled: ' + key)
