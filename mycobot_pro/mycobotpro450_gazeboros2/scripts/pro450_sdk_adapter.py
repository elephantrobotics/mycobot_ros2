"""Project-local Pro450 feedback fixes; the installed pymycobot is unmodified."""
from contextlib import nullcontext
from dataclasses import dataclass
import hashlib
import os
import socket
import tempfile
import threading
import time
import uuid

try:
    from pymycobot import Pro450Client as _VendorClient
except ImportError:
    _VendorClient = object


ANGLES, MOVING, STOP, TOOL, ARRIVED = 0x20, 0x2B, 0x29, 0xB5, 0x5B


@dataclass(frozen=True)
class AngleSample:
    angles: tuple
    monotonic: float
    wall_time: float
    sequence: int


class RobotConnectionLease:
    """OS-released, host-local ownership of one robot endpoint."""

    def __init__(self, ip, port, timeout=15.0, directory=None):
        key = hashlib.sha256(f'{ip}:{port}'.encode()).hexdigest()[:20]
        self.path = os.path.join(directory or tempfile.gettempdir(), f'pro450-{key}.lock')
        self.handle = open(self.path, 'a+b')
        self.handle.seek(0, os.SEEK_END)
        if not self.handle.tell():
            self.handle.write(b'0')
            self.handle.flush()
        deadline = time.monotonic() + timeout
        while True:
            try:
                if os.name == 'nt':
                    import msvcrt
                    self.handle.seek(0)
                    msvcrt.locking(self.handle.fileno(), msvcrt.LK_NBLCK, 1)
                else:
                    import fcntl
                    fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    self.handle.close()
                    self.handle = None
                    raise RuntimeError(f'Pro450 connection already owned for {ip}:{port}')
                time.sleep(0.05)

    def close(self):
        if self.handle is None:
            return
        if os.name == 'nt':
            import msvcrt
            self.handle.seek(0)
            msvcrt.locking(self.handle.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            import fcntl
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
        self.handle.close()
        self.handle = None


class ResponseList(list):
    """Vendor-compatible list with one frame per response kind and timestamps.

    All access is protected by the vendor's lock / lock_out convention.
    ACK and data for a command remain distinct, as does its arrival event.
    """

    def __init__(self):
        super().__init__()
        self.metadata = {}
        self.consumed = {}
        self.dropped = 0

    @staticmethod
    def register_key(frame):
        payload = frame[4:-2]
        offset = 3 if payload[:2] == b'\xfe\xfe' else 0
        return tuple(payload[offset:offset + 4])

    @staticmethod
    def key(frame):
        ack = len(frame) == 8 and frame[2] == 5 and frame[4] == 0xFF
        register = ResponseList.register_key(frame) if frame[3] == TOOL and not ack else ()
        return frame[3], ack, register

    def offer(self, frame, mono, wall):
        key = self.key(frame)
        retained = []
        for item in self:
            if self.key(item[0]) == key:
                self.metadata.pop(id(item), None)
                self.dropped += 1
            else:
                retained.append(item)
        self[:] = retained
        item = [frame, wall]
        self.metadata[id(item)] = (mono, wall)
        super().append(item)

    def remove(self, item):
        stamp = self.metadata.pop(id(item), None)
        if stamp is not None:
            self.consumed[item[0][3]] = stamp
        super().remove(item)

    def discard(self, genre):
        retained = []
        for item in self:
            if item[0][3] == genre:
                self.metadata.pop(id(item), None)
                self.dropped += 1
            else:
                retained.append(item)
        self[:] = retained
        self.consumed.pop(genre, None)


class Pro450Client(_VendorClient):
    """Single connection, bounded responses and independently arriving feedback."""

    def __init__(self, ip='192.168.0.232', netport=4500, debug=False,
                 feedback_hz=10.0, ownership_timeout=15.0):
        if _VendorClient is object:
            raise RuntimeError('pymycobot is required for real Pro450 mode')
        self._adapter_init(feedback_hz)
        self._lease = RobotConnectionLease(ip, netport, ownership_timeout)
        try:
            super().__init__(ip, netport, debug)
            if not all(hasattr(self, name) for name in ('lock_out', 'lock', 'event', 'write_command')):
                raise RuntimeError('Unsupported Pro450 SDK response interface')
            with self.lock:
                if not isinstance(self.read_command, ResponseList):
                    self.read_command = ResponseList()
        except Exception:
            self.close()
            raise

    def _adapter_init(self, feedback_hz=10.0):
        self._api_lock = threading.RLock()
        self._feedback_lock = threading.RLock()
        self._closed = threading.Event()
        self._pending = {}
        self._last_reply = {}
        self._latest_angle = None
        self._angle_callback = None
        self._sequence = 0
        self._next_angles = 0.0
        self._angle_period = 1.0 / max(1.0, min(20.0, feedback_hz))
        self._lease = None
        self.owner_token = uuid.uuid4().hex
        self.invalid_frames = 0
        self.feedback_error = ''

    def set_angle_callback(self, callback):
        with self._feedback_lock:
            self._angle_callback = callback

    def sample_time(self, kind):
        genre = ANGLES if kind == 'arm' else TOOL
        with self._feedback_lock:
            return self._last_reply.get(genre)

    def _clean_response(self, genre):
        with self.lock_out:
            with self.lock:
                self.read_command.discard(genre)
                self.write_command[:] = [g for g in self.write_command if g != genre]

    def _mesg(self, genre, *args, **kwargs):
        # Existing shutdown paths can issue asynchronous STOP while a query waits.
        guard = nullcontext() if genre == STOP and kwargs.get('_async') else self._api_lock
        with guard:
            if self._closed.is_set():
                raise RuntimeError('Pro450 connection closed')
            if genre == ANGLES:
                delay = self._next_angles - time.monotonic()
                if delay > 0 and self._closed.wait(delay):
                    raise RuntimeError('Pro450 connection closed')
                self._next_angles = time.monotonic() + self._angle_period
            self._clean_response(genre)
            with self._feedback_lock:
                self._pending[genre] = {'sent': None, 'register': None}
                self._last_reply.pop(genre, None)
            try:
                return super()._mesg(genre, *args, **kwargs)
            finally:
                with self.lock_out:
                    with self.lock:
                        stamp = self.read_command.consumed.get(genre)
                        with self._feedback_lock:
                            if stamp is not None:
                                self._last_reply[genre] = stamp
                            self._pending.pop(genre, None)
                        self.read_command.discard(genre)
                        self.write_command[:] = [g for g in self.write_command if g != genre]

    def _send_command(self, genre, real_command):
        raw = self._flatten(real_command)
        with self._feedback_lock:
            pending = self._pending.get(genre)
            if pending is not None:
                pending['sent'] = time.monotonic()
                pending['register'] = ResponseList.register_key(bytearray(raw)) if genre == TOOL else None
        # The caller already holds the vendor lock.
        self.write_command[:] = [g for g in self.write_command if g != genre]
        self.write_command.append(genre)
        self._write(raw, method='socket')

    def _parse_frames(self, buffer):
        frames = []
        index = 0
        while index + 3 <= len(buffer):
            if buffer[index:index + 2] != b'\xfe\xfe':
                index += 1
                continue
            size = buffer[index + 2] + 3
            if size < 6:
                index += 1
                self.invalid_frames += 1
                continue
            if index + size > len(buffer):
                break
            frame = bytearray(buffer[index:index + size])
            if list(frame[-2:]) != self.crc_check(list(frame[:-2])):
                self.invalid_frames += 1
                index += 1
                continue
            frames.append(frame)
            index += size
        return frames, bytearray(buffer[index:])

    def _receive_frames(self, frames, mono, wall):
        callbacks = []
        accepted = False
        for frame in frames:
            genre = frame[3]
            with self._feedback_lock:
                pending = self._pending.get(genre)
                is_ack = ResponseList.key(frame)[1]
                matches = pending is not None and pending['sent'] is not None and mono >= pending['sent']
                if matches and genre == TOOL and not is_ack:
                    matches = ResponseList.register_key(frame) == pending['register']
                if genre == ARRIVED:
                    # Arrival events are only relevant to a currently waiting move.
                    matches = any(g not in (ANGLES, MOVING, TOOL) and p['sent'] is not None
                                  and mono >= p['sent'] for g, p in self._pending.items())
                if genre == ANGLES and not is_ack and len(frame) in (18, 30):
                    width = (len(frame) - 6) // 6
                    values = tuple(self._int2angle(int.from_bytes(frame[i:i + width], 'big', signed=True))
                                   for i in range(4, len(frame) - 2, width))
                    self._sequence += 1
                    sample = AngleSample(values, mono, wall, self._sequence)
                    self._latest_angle = sample
                    if self._angle_callback is not None:
                        callbacks.append((self._angle_callback, sample))
            if matches:
                with self.lock_out, self.lock:
                    # Recheck pending after acquiring the writer lock: it may have expired.
                    with self._feedback_lock:
                        valid = genre == ARRIVED or self._pending.get(genre) is pending
                    if valid:
                        self.read_command.offer(frame, mono, wall)
                        accepted = True
        if accepted:
            self.event.set()
        # Publish only the newest angle in a received batch; never run ROS under SDK locks.
        if callbacks:
            callback, sample = callbacks[-1]
            try:
                callback(sample)
            except Exception as exc:
                self.feedback_error = str(exc)

    def read_thread(self, method=None):
        with self.lock_out, self.lock:
            if not isinstance(self.read_command, ResponseList):
                self.read_command = ResponseList()
        self.buffer = bytearray()
        self.sock.settimeout(0.1)
        while not self._closed.is_set():
            try:
                raw = self.sock.recv(4096)
                if not raw:
                    self.feedback_error = 'Pro450 peer disconnected'
                    self.event.set()
                    break
                mono, wall = time.monotonic(), time.time()
                self.buffer.extend(raw)
                frames, self.buffer = self._parse_frames(self.buffer)
                self._receive_frames(frames, mono, wall)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._closed.is_set():
                    self.feedback_error = str(exc)
                    self.event.set()
                break

    def diagnostics(self):
        with self.lock:
            count = len(self.read_command)
            dropped = self.read_command.dropped
        with self._feedback_lock:
            sample = self._latest_angle
            return {'queued_responses': count, 'discarded_responses': dropped,
                    'invalid_frames': self.invalid_frames, 'feedback_error': self.feedback_error,
                    'angle_age_s': None if sample is None else time.monotonic() - sample.monotonic}

    def close(self):
        self._closed.set()
        self.set_angle_callback(None)
        sock = getattr(self, 'sock', None)
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        thread = getattr(self, 'read_threading', None)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        if self._lease is not None:
            self._lease.close()
            self._lease = None
