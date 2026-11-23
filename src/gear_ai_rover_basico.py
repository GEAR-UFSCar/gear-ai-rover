#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
======================================================================
GEAR AI ROVER - BASICO (reescrita do zero)
======================================================================

Por que reescrever: a v21 (DQN + Visual Odometry + Kalman + A* + mapa
persistente + curiosidade + ONNX) tinha módulos demais competindo entre
si, e o bug real de comportamento (via o alvo e continuava girando)
estava numa fase de busca CONTÍNUA: o servo varria sem parar enquanto
tentava "confirmar" o alvo, então a cada quadro o ângulo mudava e a
confirmação nunca fechava.

Esta versão faz SÓ o essencial, e faz parado/discreto, não contínuo:

  - Liga na ESP32 por TCP e manda comandos de motor/servo (protocolo
    JSON já validado com o firmware LAFVIN).
  - Lê o stream MJPEG da ESP32-CAM.
  - Roda YOLO na imagem, procurando as classes-alvo.
  - Busca: aponta o servo para um ÂNGULO FIXO de cada vez e PARA lá
    para olhar. Se aparecer um candidato, a câmera já está parada -
    não tem "correr atrás" de bearing mudando. Só depois de olhar
    todos os ângulos sem achar nada é que o chassi gira um passo e
    a varredura de ângulos recomeça.
  - Ao confirmar (mesmo objeto visto algumas vezes seguidas, com no
    máximo uma folga pequena de misses): centraliza o chassi, depois
    avança reduzindo a velocidade conforme o alvo cresce na imagem,
    e para quando a caixa fica grande o suficiente (chegou).
  - Trava de identidade: se tem mais de um objeto da mesma classe no
    quadro, o rover continua com o MESMO objeto (por sobreposição de
    caixa), em vez de pular pra qualquer um que tenha a maior
    pontuação naquele instante.
  - Desvio de obstáculo só pelo ultrassônico (mais simples e é o
    sensor que sempre funcionou de verdade nos testes anteriores).

O que foi cortado de propósito (pode voltar depois, um de cada vez,
só se realmente fizer falta):
  - DQN / aprendizado por reforço (não dirigia o robô de verdade na
    v21; só observava e treinava em paralelo - custo sem benefício
    aqui).
  - Visual Odometry, Kalman, A*, mapa persistente, loop closure,
    curiosidade (ICM), ONNX Runtime.
  - Câmera "ativa" seguindo o alvo enquanto o chassi gira (era código
    a mais para uma frescura visual; agora o servo fica centralizado
    durante alinhamento e aproximação, o que também simplifica MUITO
    a conta de para onde apontar o corpo).
  - Desvio de obstáculo por visão (só ultrassônico).

Protocolo com o firmware (mantido, já testado):
  - N:102 D1=direção(1-9) D2=velocidade  -> comando de motor
  - N:5   D1=1 D2=ângulo                 -> servo absoluto
  - N:21                                 -> pede leitura do ultrassônico
  - N:100                                -> STOP explícito (limpa modo/LEDs)
  - "{Heartbeat}" (string crua, não JSON) -> keep-alive
"""

import json
import os
import queue
import re
import socket
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List

import cv2
import numpy as np
import requests
import tkinter as tk
from tkinter import ttk

try:
    from PIL import Image, ImageTk
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# =====================================================================
# CONFIGURAÇÃO (valores calibrados no chassi real - ver comentários)
# =====================================================================

ESP32_IP = "192.168.4.1"
ESP32_TCP_PORT = 100
STREAM_URL = f"http://{ESP32_IP}:81/stream"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolo11n.pt")

DEFAULT_TARGETS = ["bottle"]
DEFAULT_CONFIDENCE = 0.50

# --- PWM / tração ---
# Abaixo de ~38/255 os motoredutores não vencem o atrito estático com o
# chassi no chão (a bobina apita e a roda não gira). Girar no lugar pede
# mais torque que andar reto (as duas rodas raspam em sentidos opostos).
MIN_SPEED = 38          # piso para avançar / dar ré
MIN_TURN_SPEED = 38     # piso para girar no lugar
MAX_SPEED = 60
CRUISE_SPEED = 40        # velocidade "de cruzeiro" longe do alvo
NEAR_SPEED = 38          # velocidade mínima perto do alvo (rampa de chegada)
TURN_SPEED = 40          # giro no lugar (busca / alinhamento grosso)
AVOID_SPEED = 40
RECOVERY_SPEED = 55      # ré ao desviar
MOTOR_BOOST_SPEED = 60   # "chute" de arranque: PWM mais alto só no 1º instante
MOTOR_BOOST_TIME = 0.18  # duração do chute (também é o menor giro possível)
TURN_DEG_PER_SEC = 70.0  # calibração de giro em malha aberta (sem encoder)

# --- Ultrassônico (HC-SR04) ---
SAFE_DISTANCE_CM = 25
CRITICAL_DISTANCE_CM = 15
ULTRA_MIN_VALID_CM = 2
ULTRA_MAX_VALID_CM = 400
ULTRA_CLEAR_CM = 200.0          # firmware devolve 0 quando não há eco = livre
ULTRASONIC_STALE_SECONDS = 1.20
ULTRA_QUERY_INTERVAL = 0.18

# --- Servo / câmera ---
SERVO_MIN = 20
SERVO_MAX = 160
SERVO_CENTER = 90
SERVO_SETTLE_TIME = 0.30
SERVO_SLEW_DEG_PER_SEC = 40.0    # a rampa suave real é feita NO FIRMWARE
CAMERA_HFOV_DEG = 65.0

# Ângulos onde a busca PARA para olhar (varredura discreta, não contínua).
SCAN_ANGLES = [50, 70, 90, 110, 130]
SCAN_HOLD_TIME = 0.35            # quanto tempo olha em cada ângulo antes de ir pro próximo

# --- Detecção / trava de alvo ---
TARGET_MIN_CONFIDENCE_FLOOR = 0.40   # nunca aceita candidato abaixo disso
TARGET_MIN_AREA_RATIO = 0.0035       # caixa menor que isso = ruído
STOP_AREA_RATIO = 0.28               # caixa >= isso = "chegou"
STOP_AREA_RATIO_RESUME = STOP_AREA_RATIO * 0.82   # histerese: só volta a andar se afastar de verdade
MIN_AREA_RATIO_FAR = 0.010           # área "longe" para a rampa de velocidade
TARGET_LOCK_IOU_MIN = 0.12           # sobreposição mínima p/ considerar "o mesmo objeto"

# Confirmação (parado, olhando fixo para o candidato).
CONFIRM_HITS_NEEDED = 3
CONFIRM_MAX_MISSES = 3
CONFIRM_TIMEOUT = 1.30

# Perda de alvo já confirmado (oclusão curta antes de desistir de vez).
TARGET_LOST_TIMEOUT = 1.50

# --- Alinhamento do chassi ---
# O menor giro possível a partir de PARADO é o chute de arranque
# (~0.18s @ TURN_DEG_PER_SEC ~= 11-13 graus). Por isso a zona morta do
# alinhamento GROSSO tem que ser MAIOR que esse giro mínimo, senão o
# rover ultrapassa o alvo a cada correção e fica oscilando para sempre.
# O ajuste FINO (dentro dessa zona) é feito DEPOIS, já em movimento
# (FORWARD_LEFT/FORWARD_RIGHT durante a aproximação), onde não há
# atrito estático a vencer.
ALIGN_DEADBAND_DEG = 15.0        # ~igual ao giro minimo de arranque; ver nota acima
ALIGN_PULSE_TIME = MOTOR_BOOST_TIME
ALIGN_MAX_TIME = 4.0
STEER_DEADBAND_NORM = 0.15       # abaixo disso, segue reto durante a aproximação

# --- Busca / giro entre varreduras ---
SEARCH_TURN_STEP_DEG = 75.0      # quanto o chassi gira quando a varredura de ângulos falha
SEARCH_TURN_TIME = max(0.35, SEARCH_TURN_STEP_DEG / TURN_DEG_PER_SEC)

# --- Desvio de obstáculo (simples: recua e gira para um lado fixo) ---
AVOID_BACK_TIME = 0.55
AVOID_TURN_TIME = 0.55

# --- Rede / timing geral ---
CAMERA_STALE_SECONDS = 1.0
HEARTBEAT_INTERVAL = 0.5
COMMAND_INTERVAL = 0.06
MAX_CAMERA_BUFFER = 300_000
CONTROL_INTERVAL = 0.08          # período do laço de decisão (~12.5 Hz)


# =====================================================================
# ENUMS
# =====================================================================

class Direction(Enum):
    STOP = 9
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4
    FORWARD_LEFT = 5
    BACKWARD_LEFT = 6
    FORWARD_RIGHT = 7
    BACKWARD_RIGHT = 8


class Mode(Enum):
    IDLE = "PARADO"
    MANUAL = "MANUAL"
    SEARCH = "PROCURANDO"
    CONFIRM = "CONFIRMANDO"
    CENTER = "CENTRALIZANDO"
    APPROACH = "APROXIMANDO"
    ARRIVED = "CHEGOU"
    AVOID = "DESVIANDO"
    EMERGENCY_STOP = "EMERGENCIA"


# =====================================================================
# DETECÇÃO
# =====================================================================

@dataclass
class Detection:
    label: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int
    frame_w: int
    frame_h: int
    timestamp: float = field(default_factory=time.time)

    @property
    def width(self) -> int:
        return max(0, self.x2 - self.x1)

    @property
    def height(self) -> int:
        return max(0, self.y2 - self.y1)

    @property
    def area_ratio(self) -> float:
        return (self.width * self.height) / max(1, self.frame_w * self.frame_h)

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def center_norm(self) -> float:
        """Erro horizontal normalizado: -1 (borda esquerda) .. +1 (borda direita)."""
        half = max(1.0, self.frame_w / 2.0)
        return (self.cx - self.frame_w / 2.0) / half

    @property
    def visual_score(self) -> float:
        size_score = min(1.0, self.area_ratio * 8.0)
        center_score = 1.0 - min(1.0, abs(self.center_norm))
        return self.confidence * 0.55 + size_score * 0.25 + center_score * 0.20

    def bbox(self):
        return (self.x1, self.y1, self.x2, self.y2)


def iou(box1, box2) -> float:
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = max(0, box1[2] - box1[0]) * max(0, box1[3] - box1[1])
    area2 = max(0, box2[2] - box2[0]) * max(0, box2[3] - box2[1])
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0.0


# =====================================================================
# ESTADO COMPARTILHADO
# =====================================================================

@dataclass
class RobotState:
    connected: bool = False

    camera_ok: bool = False
    camera_timestamp: float = 0.0
    fps: float = 0.0
    inference_ms: float = 0.0

    ultrasonic_cm: float = 0.0
    ultrasonic_timestamp: float = 0.0
    ultrasonic_ok: bool = False

    servo_angle: int = SERVO_CENTER
    servo_target_angle: int = SERVO_CENTER

    mode: Mode = Mode.IDLE
    last_command: str = "STOP"
    last_speed: int = 0

    target_visible: bool = False
    target_label: str = "-"
    target_confidence: float = 0.0
    target_area: float = 0.0

    obstacle_blocked: bool = False

    tx_count: int = 0
    rx_count: int = 0
    last_error: str = ""

    ai_decision: str = "AGUARDANDO"
    ai_reason: str = "Inicializando"


# =====================================================================
# LOG
# =====================================================================

class EventLog:
    def __init__(self, maxlen: int = 500):
        self.items = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.seq = 0

    def add(self, message: str, level: str = "INFO"):
        with self.lock:
            stamp = time.strftime("%H:%M:%S")
            self.items.append(f"[{stamp}] [{level}] {message}")
            self.seq += 1

    def snapshot(self):
        with self.lock:
            return list(self.items)


# =====================================================================
# CONEXÃO TCP COM A ESP32
# =====================================================================

class RobotConnection:
    """Envia comandos de motor/servo e lê a leitura do ultrassônico.

    Protocolo mantido igual ao já validado em campo: frames JSON
    delimitados por '{' e '}', comando 102 para motor, 5 para servo,
    21 para pedir leitura do ultrassônico, heartbeat cru "{Heartbeat}".
    """

    def __init__(self, state: RobotState, log: EventLog):
        self.state = state
        self.log = log
        self.sock: Optional[socket.socket] = None
        self.lock = threading.RLock()
        self.stop_event = threading.Event()

        self.last_send_at = {}
        self.last_heartbeat = 0.0
        self.last_ultra_query = 0.0
        self.last_servo_send = 0.0
        self.ultra_query_angle = SERVO_CENTER
        self.rx_buffer = ""
        self.reconnect_count = 0

        self.thread = threading.Thread(target=self._connection_loop, daemon=True)
        self.thread.start()

    def _connection_loop(self):
        while not self.stop_event.is_set():
            if not self.state.connected:
                self._try_connect()
                self.stop_event.wait(min(1.0 + self.reconnect_count * 0.5, 5.0))
                continue
            try:
                self._receive()
            except Exception as exc:
                self.state.last_error = str(exc)
                self.log.add(f"Erro RX ESP32: {exc}", "ERROR")
                self._disconnect()
                continue

            now = time.time()
            if now - self.last_heartbeat >= HEARTBEAT_INTERVAL:
                self.send_raw("{Heartbeat}")
                self.last_heartbeat = now

            if now - self.last_ultra_query >= ULTRA_QUERY_INTERVAL:
                self.query_ultrasonic()
                self.last_ultra_query = now

            self.stop_event.wait(0.02)

    def _try_connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.settimeout(3.0)
        try:
            sock.connect((ESP32_IP, ESP32_TCP_PORT))
            sock.settimeout(0.08)
            with self.lock:
                self.sock = sock
                self.state.connected = True
                self.rx_buffer = ""
                self.reconnect_count = 0
            self.log.add(f"TCP conectado: {ESP32_IP}:{ESP32_TCP_PORT}", "INIT")
            self.send_command(Direction.STOP, 0, force=True)
        except Exception as exc:
            self.state.connected = False
            self.state.last_error = str(exc)
            self.reconnect_count += 1
            try:
                sock.close()
            except OSError:
                pass

    def _disconnect(self):
        with self.lock:
            try:
                if self.sock:
                    self.sock.close()
            except OSError:
                pass
            self.sock = None
            self.state.connected = False

    def close(self):
        self.stop_event.set()
        try:
            self.send_command(Direction.STOP, 0, force=True)
        except Exception:
            pass
        self._disconnect()

    def send_raw(self, payload: str) -> bool:
        with self.lock:
            if self.sock is None or not self.state.connected:
                return False
            try:
                self.sock.sendall((payload + "\n").encode("utf-8"))
                self.state.tx_count += 1
                return True
            except Exception as exc:
                self.state.last_error = str(exc)
                self._disconnect()
                return False

    def send_command(self, direction: Direction, speed: int, force: bool = False) -> bool:
        speed = int(max(0, min(MAX_SPEED, speed)))
        if direction == Direction.STOP:
            speed = 0
        elif speed > 0:
            floor = MIN_TURN_SPEED if direction in (Direction.LEFT, Direction.RIGHT) else MIN_SPEED
            speed = max(floor, speed)

        key = (direction.value, speed)
        now = time.time()
        if not force and now - self.last_send_at.get(key, 0.0) < COMMAND_INTERVAL:
            return True
        self.last_send_at[key] = now

        payload = json.dumps({"N": 102, "H": "AI", "D1": direction.value, "D2": speed},
                              separators=(",", ":"))
        ok = self.send_raw(payload)
        if ok:
            self.state.last_command = direction.name
            self.state.last_speed = speed
        return ok

    def set_servo(self, angle: int, force: bool = False) -> bool:
        angle = int(max(SERVO_MIN, min(SERVO_MAX, angle)))
        now = time.time()
        self.state.servo_target_angle = angle
        if not force and now - self.last_servo_send < 0.20:
            return True
        payload = json.dumps({"N": 5, "H": "AI", "D1": 1, "D2": angle}, separators=(",", ":"))
        ok = self.send_raw(payload)
        if ok:
            self.last_servo_send = now
        return ok

    def query_ultrasonic(self) -> bool:
        self.ultra_query_angle = int(self.state.servo_angle)
        return self.send_raw('{"N":21,"H":"AI"}')

    def _receive(self):
        with self.lock:
            sock = self.sock
        if sock is None:
            return
        try:
            data = sock.recv(4096)
        except socket.timeout:
            return
        if not data:
            self._disconnect()
            return

        self.state.rx_count += 1
        self.rx_buffer += data.decode("utf-8", errors="ignore")
        if len(self.rx_buffer) > 32_000:
            self.rx_buffer = self.rx_buffer[-8_000:]

        frames = re.findall(r"\{[^{}]*\}", self.rx_buffer)
        last_close = self.rx_buffer.rfind("}")
        if last_close >= 0:
            self.rx_buffer = self.rx_buffer[last_close + 1:]
        for frame in frames:
            self._parse_response(frame)

    def _parse_response(self, text: str):
        if not text:
            return
        try:
            obj = json.loads(text)
            distance = self._extract_distance(obj)
            if distance is not None:
                self._set_ultrasonic(distance)
                return
        except (ValueError, TypeError):
            pass  # frames como {ok}/{Heartbeat} não são JSON válido - esperado

        lower = text.lower()
        match = re.search(r"(?:ultrasonic|distance|dist|range|us)\s*[:=]\s*(\d+(?:\.\d+)?)", lower)
        if not match:
            match = re.search(r"(\d+(?:\.\d+)?)\s*cm", lower)
        if match:
            self._set_ultrasonic(float(match.group(1)))

    def _extract_distance(self, obj):
        if isinstance(obj, dict):
            for key in ("ultrasonic", "ultrasonic_cm", "distance", "distance_cm", "dist", "cm", "sonar", "range"):
                value = obj.get(key)
                if isinstance(value, (int, float)):
                    return float(value)
                if isinstance(value, str):
                    m = re.search(r"-?\d+(?:\.\d+)?", value)
                    if m:
                        return float(m.group(0))
            for value in obj.values():
                found = self._extract_distance(value)
                if found is not None:
                    return found
        elif isinstance(obj, list):
            for value in obj:
                found = self._extract_distance(value)
                if found is not None:
                    return found
        return None

    def _set_ultrasonic(self, value):
        try:
            distance = float(value)
        except (TypeError, ValueError):
            return
        if distance <= 0.0:
            distance = ULTRA_CLEAR_CM  # sem eco = caminho livre, não leitura ruim
        if not (ULTRA_MIN_VALID_CM <= distance <= ULTRA_MAX_VALID_CM):
            return
        self.state.ultrasonic_cm = distance
        self.state.ultrasonic_timestamp = time.time()
        self.state.ultrasonic_ok = True
        self.state.obstacle_blocked = distance <= SAFE_DISTANCE_CM


# =====================================================================
# CÂMERA (STREAM MJPEG DA ESP32-CAM)
# =====================================================================

class CameraWorker:
    def __init__(self, state: RobotState, log: EventLog):
        self.state = state
        self.log = log
        self.frame = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                self._stream()
            except Exception as exc:
                self.state.camera_ok = False
                self.state.last_error = str(exc)
                self.log.add(f"Erro camera ESP32: {exc}", "ERROR")
                self.stop_event.wait(2.0)

    def _stream(self):
        self.log.add(f"Conectando camera: {STREAM_URL}", "INIT")
        response = requests.get(STREAM_URL, stream=True, timeout=(5, 15))
        response.raise_for_status()
        self.log.add("Camera ESP32 conectada.", "INIT")

        buffer = b""
        frames = 0
        start_time = time.time()

        for chunk in response.iter_content(chunk_size=4096):
            if self.stop_event.is_set():
                break
            if not chunk:
                continue
            buffer += chunk
            if len(buffer) > MAX_CAMERA_BUFFER:
                newest = buffer.rfind(b"\xff\xd8")
                buffer = buffer[newest:] if newest >= 0 else b""

            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    buffer = buffer[-1:]
                    break
                end = buffer.find(b"\xff\xd9", start + 2)
                if end < 0:
                    break
                jpg = buffer[start:end + 2]
                buffer = buffer[end + 2:]
                arr = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                with self.lock:
                    self.frame = frame
                now = time.time()
                self.state.camera_ok = True
                self.state.camera_timestamp = now
                frames += 1
                if now - start_time >= 1.0:
                    self.state.fps = frames / max(0.001, now - start_time)
                    frames = 0
                    start_time = now

        self.state.camera_ok = False

    def get_frame(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):
        self.stop_event.set()


# =====================================================================
# VISÃO / YOLO
# =====================================================================

class VisionWorker:
    def __init__(self, state: RobotState, log: EventLog):
        self.state = state
        self.log = log
        self.model = None
        self.loaded = False
        self.load_error = ""
        self.targets = set(DEFAULT_TARGETS)
        self.confidence = DEFAULT_CONFIDENCE
        self.detections: List[Detection] = []
        self.lock = threading.RLock()
        self.queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set_targets(self, targets):
        clean = {str(t).strip().lower() for t in targets if str(t).strip()}
        with self.lock:
            self.targets = clean or set(DEFAULT_TARGETS)

    def get_targets(self):
        with self.lock:
            return set(self.targets)

    def set_confidence(self, value):
        try:
            value = float(value)
        except Exception:
            value = DEFAULT_CONFIDENCE
        with self.lock:
            self.confidence = max(0.05, min(0.95, value))

    def submit(self, frame):
        try:
            self.queue.put_nowait(frame)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(frame)
            except queue.Full:
                pass

    def _load_model(self):
        if not YOLO_AVAILABLE:
            self.load_error = "Ultralytics nao instalado"
            self.log.add(self.load_error, "ERROR")
            return
        if not os.path.isfile(MODEL_PATH):
            self.load_error = f"Modelo nao encontrado: {MODEL_PATH}"
            self.log.add(self.load_error, "ERROR")
            return
        try:
            self.log.add(f"Carregando YOLO: {MODEL_PATH}", "INIT")
            self.model = YOLO(MODEL_PATH)
            self.loaded = True
            self.log.add("YOLO carregado.", "INIT")
        except Exception as exc:
            self.load_error = f"Erro carregando YOLO: {exc}"
            self.log.add(self.load_error, "ERROR")

    def _run(self):
        self._load_model()
        while not self.stop_event.is_set():
            try:
                frame = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if not self.loaded:
                continue
            started = time.perf_counter()
            try:
                detections = self._infer(frame)
                with self.lock:
                    self.detections = detections
                self.state.inference_ms = (time.perf_counter() - started) * 1000.0
            except Exception as exc:
                self.log.add(f"Erro inferencia: {exc}", "ERROR")

    def _infer(self, frame):
        with self.lock:
            conf = self.confidence
        results = self.model.predict(source=frame, conf=conf, imgsz=640, verbose=False)
        if not results or results[0].boxes is None:
            return []
        result = results[0]
        h, w = frame.shape[:2]
        names = result.names
        out = []
        for box in result.boxes:
            try:
                confidence = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = str(names.get(cls_id, cls_id) if isinstance(names, dict) else names[cls_id]).lower()
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                out.append(Detection(
                    label=label, confidence=confidence,
                    x1=max(0, min(w - 1, x1)), y1=max(0, min(h - 1, y1)),
                    x2=max(0, min(w - 1, x2)), y2=max(0, min(h - 1, y2)),
                    frame_w=w, frame_h=h,
                ))
            except Exception:
                continue
        return out

    def get_all_detections(self):
        with self.lock:
            return list(self.detections)

    def get_target_candidates(self):
        with self.lock:
            targets = set(self.targets)
            conf = max(self.confidence, TARGET_MIN_CONFIDENCE_FLOOR)
            candidates = [
                d for d in self.detections
                if d.label in targets and d.confidence >= conf
                and d.area_ratio >= TARGET_MIN_AREA_RATIO
            ]
        candidates.sort(key=lambda d: d.visual_score, reverse=True)
        return candidates

    def close(self):
        self.stop_event.set()


# =====================================================================
# NAVEGADOR (MÁQUINA DE ESTADOS SIMPLES)
# =====================================================================

class Navigator:
    def __init__(self, state: RobotState, connection: RobotConnection, vision: VisionWorker, log: EventLog):
        self.state = state
        self.connection = connection
        self.vision = vision
        self.log = log

        self.mode = Mode.IDLE
        self.enabled = False

        # Trava de identidade do alvo (a mesma ideia que funcionou na
        # versão anterior, aqui isolada e simples).
        self._locked_bbox = None
        self._locked_frame_dims = None
        self._target_label = ""
        self._target_area = 0.0
        self._target_last_seen = 0.0
        self._target_arrived = False

        # Servo
        self._servo_cmd_angle = SERVO_CENTER
        self._servo_move_from = float(SERVO_CENTER)
        self._servo_move_start = 0.0

        # Busca
        self.scan_index = 0
        self.scan_entered_at = 0.0
        self.scan_angle_sent = False
        self._searching_turn = False
        self._search_turn_until = 0.0

        # Confirmação
        self.confirm_hits = 0
        self.confirm_misses = 0
        self.confirm_started = 0.0
        self._confirm_label = ""

        # Alinhamento
        self.align_started = 0.0
        self.align_pulse_until = 0.0

        # Desvio
        self.avoid_phase = ""
        self.avoid_until = 0.0
        # Direção fixa usada tanto para o giro de busca (setor sem alvo)
        # quanto para o giro de desvio (após recuar do obstáculo). Simples
        # de propósito: sem sonar lateral, não há como saber com certeza
        # qual lado está mais livre, então usamos sempre o mesmo lado e
        # confiamos no ultrassônico para não bater durante o giro.
        self.turn_direction = Direction.RIGHT

        # Movimento (para o "chute" de arranque)
        self.last_motion_command = Direction.STOP
        self.last_motion_time = 0.0
        self.boost_until = 0.0
        self.boost_direction = Direction.STOP

        # Manual
        self.manual_command = Direction.STOP
        self.manual_speed = CRUISE_SPEED

        self.last_decision_key = ""
        self.last_decision_time = 0.0

        self.log.add("Navigator basico iniciado.", "INIT")

    # ---------------------------------------------------------- util --

    def _decide(self, decision, reason, throttle=0.3):
        now = time.time()
        self.state.ai_decision = str(decision)
        self.state.ai_reason = str(reason)
        key = f"{decision}|{reason}"
        if key != self.last_decision_key or now - self.last_decision_time >= throttle:
            self.log.add(f"{decision}: {reason}")
            self.last_decision_key = key
            self.last_decision_time = now

    def _ultra_valid(self, now):
        return (self.state.ultrasonic_ok
                and ULTRA_MIN_VALID_CM <= self.state.ultrasonic_cm <= ULTRA_MAX_VALID_CM
                and now - self.state.ultrasonic_timestamp <= ULTRASONIC_STALE_SECONDS)

    def _front_critical(self, now):
        return self._ultra_valid(now) and self.state.ultrasonic_cm <= CRITICAL_DISTANCE_CM

    def _front_blocked(self, now):
        return not self._ultra_valid(now) or self.state.ultrasonic_cm <= SAFE_DISTANCE_CM

    def _bearing_deg(self, target):
        """Ângulo do alvo em relação ao CHASSI (não à câmera): o offset do
        servo em relação ao centro somado ao ângulo do alvo dentro da
        imagem. Necessário porque a busca detecta o alvo com a câmera
        apontada para 50/70/90/110/130 graus, não só para o centro - sem
        somar o offset do servo, o robô calcularia o giro errado toda vez
        que achasse o alvo fora do centro (ou seja, na maioria das vezes)."""
        image_angle = target.center_norm * (CAMERA_HFOV_DEG / 2.0)
        servo_offset = float(self.state.servo_angle - SERVO_CENTER)
        return servo_offset + image_angle

    def _move_servo(self, angle, now, force=False):
        angle = int(max(SERVO_MIN, min(SERVO_MAX, angle)))
        self.state.servo_target_angle = angle
        if angle != self._servo_cmd_angle or force:
            self._servo_move_from = float(self.state.servo_angle)
            self._servo_move_start = now
            self._servo_cmd_angle = angle
            self.connection.set_servo(angle, force=True)

        span = abs(angle - self._servo_move_from)
        if span <= 0.5:
            self.state.servo_angle = angle
        else:
            traveled = (now - self._servo_move_start) * SERVO_SLEW_DEG_PER_SEC
            if traveled >= span:
                self.state.servo_angle = angle
            else:
                sign = 1.0 if angle >= self._servo_move_from else -1.0
                self.state.servo_angle = int(round(self._servo_move_from + sign * traveled))

    def _servo_settled(self):
        return self.state.servo_angle == self.state.servo_target_angle

    def command(self, direction: Direction, speed: int, allow_when_blocked=False):
        """Manda um comando de motor com o 'chute' de arranque e o
        travamento de segurança do ultrassônico embutidos."""
        now = time.time()
        speed = int(max(0, min(MAX_SPEED, speed)))

        if direction == Direction.STOP:
            self.boost_until = 0.0
            self.last_motion_command = Direction.STOP
            self.connection.send_command(Direction.STOP, 0, force=True)
            return

        forward_like = direction in (Direction.FORWARD, Direction.FORWARD_LEFT, Direction.FORWARD_RIGHT)
        if forward_like and not allow_when_blocked:
            if self._front_critical(now):
                self.connection.send_command(Direction.STOP, 0, force=True)
                self._decide("FREIO_US", f"{self.state.ultrasonic_cm:.0f}cm")
                return
            if not self._ultra_valid(now):
                self.connection.send_command(Direction.STOP, 0, force=True)
                self._decide("US_AGUARDANDO", "sem leitura valida; avanco bloqueado")
                return

        fresh_start = (self.last_motion_command == Direction.STOP
                        or direction != self.last_motion_command
                        or now - self.last_motion_time > 0.65)
        if fresh_start:
            self.connection.send_command(direction, MOTOR_BOOST_SPEED, force=True)
            self.boost_until = now + MOTOR_BOOST_TIME
            self.boost_direction = direction
            self.last_motion_time = now
            self.last_motion_command = direction
        elif now < self.boost_until and self.boost_direction == direction:
            return
        else:
            self.connection.send_command(direction, speed)
            self.last_motion_time = now
            self.last_motion_command = direction

    # ------------------------------------------------------- alvo -----

    def _select_locked(self, candidates):
        """Sem alvo travado: pega o de maior pontuação. Com alvo travado:
        só aceita quem bate (IoU) com a última caixa vista dele - assim
        dois objetos da mesma classe não fazem o rover trocar de alvo."""
        if not candidates:
            return None
        if self._locked_bbox is None or self._locked_frame_dims is None:
            return candidates[0]
        lw, lh = self._locked_frame_dims
        best, best_iou = None, TARGET_LOCK_IOU_MIN
        for det in candidates:
            if (det.frame_w, det.frame_h) != (lw, lh):
                continue
            score = iou(self._locked_bbox, det.bbox())
            if score > best_iou:
                best_iou = score
                best = det
        return best

    def _current_target(self):
        return self._select_locked(self.vision.get_target_candidates())

    def _lock_target(self, det, now):
        self._locked_bbox = det.bbox()
        self._locked_frame_dims = (det.frame_w, det.frame_h)
        self._target_label = det.label
        self._target_area = det.area_ratio
        self._target_last_seen = now
        self.state.target_visible = True
        self.state.target_label = det.label
        self.state.target_confidence = det.confidence
        self.state.target_area = det.area_ratio

    def _clear_target(self):
        self._locked_bbox = None
        self._locked_frame_dims = None
        self._target_arrived = False
        self.state.target_visible = False
        self.state.target_label = "-"

    # ------------------------------------------------------ modos -----

    def set_mode(self, mode: Mode):
        if mode == Mode.EMERGENCY_STOP:
            self.emergency_stop()
            return
        self.connection.send_command(Direction.STOP, 0, force=True)
        self.mode = mode
        self.state.mode = mode
        self.enabled = mode not in (Mode.IDLE, Mode.EMERGENCY_STOP)
        if mode == Mode.SEARCH:
            self._enter_search()
        self._decide("MODO", mode.value)

    def set_targets(self, targets):
        self.vision.set_targets(targets)
        self._clear_target()
        self._decide("ALVOS", ", ".join(sorted(self.vision.get_targets())))

    def set_manual(self, direction, speed):
        self.mode = Mode.MANUAL
        self.state.mode = Mode.MANUAL
        self.enabled = True
        self.manual_command = direction
        self.manual_speed = int(speed)
        self.command(direction, speed, allow_when_blocked=(direction != Direction.FORWARD))

    def set_servo_manual(self, angle):
        if self.mode not in (Mode.IDLE, Mode.MANUAL):
            return
        self._move_servo(angle, time.time(), force=True)

    def stop(self):
        self.connection.send_command(Direction.STOP, 0, force=True)
        self.connection.send_raw('{"N":100}')
        self.mode = Mode.IDLE
        self.state.mode = Mode.IDLE
        self.enabled = False
        self._decide("PARADO", "comando de parada")

    def emergency_stop(self):
        self.connection.send_command(Direction.STOP, 0, force=True)
        self.connection.send_raw('{"N":100}')
        self.mode = Mode.EMERGENCY_STOP
        self.state.mode = Mode.EMERGENCY_STOP
        self.enabled = False
        self._decide("EMERGENCIA", "parada de emergencia")

    # ------------------------------------------------------ busca -----

    def _enter_search(self):
        self._clear_target()
        self.scan_index = 0
        self.scan_entered_at = 0.0
        self.scan_angle_sent = False
        self.confirm_hits = 0
        self.confirm_misses = 0
        self.confirm_started = 0.0
        self.mode = Mode.SEARCH
        self.state.mode = Mode.SEARCH

    def _tick_search(self, now):
        target = self._current_target()

        if target is not None:
            # Achou um candidato enquanto olhava um ângulo fixo: para de
            # varrer e entra em confirmação SEM mover mais nada. É essa
            # imobilidade que garante que a confirmação vai fechar.
            self.command(Direction.STOP, 0)
            self.mode = Mode.CONFIRM
            self.state.mode = Mode.CONFIRM
            self.confirm_hits = 1
            self.confirm_misses = 0
            self.confirm_started = now
            self._confirm_label = target.label
            self._decide("CANDIDATO_AVISTADO", f"{target.label} {target.confidence:.0%} @ servo {self.state.servo_angle}")
            return

        self.command(Direction.STOP, 0)
        angle = SCAN_ANGLES[self.scan_index]
        self._move_servo(angle, now)

        if not self._servo_settled():
            self.scan_angle_sent = False
            return

        if not self.scan_angle_sent:
            self.scan_angle_sent = True
            self.scan_entered_at = now
            self._decide("VARREDURA", f"olhando {angle} graus ({self.scan_index + 1}/{len(SCAN_ANGLES)})")
            return

        if now - self.scan_entered_at < SCAN_HOLD_TIME:
            return

        # Terminou de olhar este ângulo sem achar nada: próximo.
        self.scan_index += 1
        self.scan_angle_sent = False
        if self.scan_index >= len(SCAN_ANGLES):
            self.scan_index = 0
            self._start_search_turn(now)

    def _start_search_turn(self, now):
        self.mode = Mode.SEARCH
        self._search_turn_until = now + SEARCH_TURN_TIME
        self._searching_turn = True
        self._decide("GIRO_BUSCA", f"nenhum candidato no setor; girando {SEARCH_TURN_STEP_DEG:.0f} graus")

    # ---------------------------------------------------- confirmar ---

    def _tick_confirm(self, now):
        target = self._current_target()
        # Câmera parada, olhando fixo para onde o candidato apareceu -
        # não há bearing mudando, então a confirmação é só "viu de novo?".
        if target is not None and target.label == getattr(self, "_confirm_label", target.label):
            self.confirm_hits += 1
            self.confirm_misses = 0
        else:
            self.confirm_misses += 1

        elapsed = now - self.confirm_started
        self._decide("CONFIRMANDO",
                     f"acertos={self.confirm_hits}/{CONFIRM_HITS_NEEDED} falhas={self.confirm_misses}/{CONFIRM_MAX_MISSES}",
                     throttle=0.2)

        if self.confirm_hits >= CONFIRM_HITS_NEEDED:
            self._lock_target(target, now)
            self.mode = Mode.CENTER
            self.state.mode = Mode.CENTER
            self.align_started = now
            self.align_pulse_until = 0.0
            self._decide("ALVO_CONFIRMADO", f"{target.label} {target.confidence:.0%}")
            return

        if self.confirm_misses > CONFIRM_MAX_MISSES or elapsed > CONFIRM_TIMEOUT:
            self._decide("CANDIDATO_DESCARTADO", "nao confirmou; voltando a varrer")
            self._enter_search()

    # ------------------------------------------------- centralizar ----

    def _tick_center(self, now):
        target = self._current_target()
        if target is None:
            if now - self._target_last_seen > TARGET_LOST_TIMEOUT:
                self._decide("ALVO_PERDIDO", "sem deteccao ao centralizar; nova busca")
                self._enter_search()
            else:
                self.command(Direction.STOP, 0)
                self._decide("ALVO_TEMPORARIO_PERDIDO", "aguardando reaparecer")
            return

        self._lock_target(target, now)
        # [Importante] O servo NÃO é mexido aqui - fica exatamente onde a
        # busca o deixou. O bearing (servo + imagem) já diz para onde
        # virar o chassi, então não precisa (e não deve) recentralizar a
        # câmera agora: se recentralizasse de cara, um alvo achado a 40
        # graus do centro sairia do quadro antes do chassi virar para lá.
        bearing = self._bearing_deg(target)

        if abs(bearing) <= ALIGN_DEADBAND_DEG:
            # Alinhado o bastante: SÓ AGORA recentraliza a câmera. Como o
            # erro já é pequeno, o alvo continua dentro do campo de visão
            # depois de centralizar, e a aproximação pode assumir que o
            # servo fica parado no centro (erro de imagem = erro do chassi).
            self._move_servo(SERVO_CENTER, now, force=True)
            self.command(Direction.STOP, 0)
            self.mode = Mode.APPROACH
            self.state.mode = Mode.APPROACH
            self._decide("CENTRALIZADO", f"bearing={bearing:+.1f} graus; iniciando aproximacao")
            return

        if now - self.align_started > ALIGN_MAX_TIME:
            self._decide("ALINHAMENTO_FALHOU", "tempo esgotado; nova busca")
            self._enter_search()
            return

        if now >= self.align_pulse_until:
            direction = Direction.LEFT if bearing < 0 else Direction.RIGHT
            self.command(direction, TURN_SPEED)
            self.align_pulse_until = now + ALIGN_PULSE_TIME
            self._decide("ALINHANDO", f"{direction.name} bearing={bearing:+.1f} graus")
        else:
            self.command(Direction.STOP, 0)

    # -------------------------------------------------- aproximar -----

    def _tick_approach(self, now):
        target = self._current_target()
        if target is None:
            if now - self._target_last_seen > TARGET_LOST_TIMEOUT:
                self._decide("ALVO_PERDIDO", "sem deteccao ao aproximar; nova busca")
                self._enter_search()
            else:
                self.command(Direction.STOP, 0)
                self._decide("ALVO_TEMPORARIO_PERDIDO", "aguardando reaparecer")
            return

        self._lock_target(target, now)

        # [Importante] Checa "já chegou?" ANTES de checar obstáculo. O
        # ultrassônico não distingue o alvo de um obstáculo qualquer -
        # se checasse obstáculo primeiro, o próprio alvo (ficando perto o
        # bastante para o ultrassônico acusar "bloqueado") faria o rover
        # se afastar dele bem na hora de chegar, em vez de parar.
        if target.area_ratio >= STOP_AREA_RATIO:
            self._target_arrived = True
        if self._target_arrived:
            if target.area_ratio >= STOP_AREA_RATIO_RESUME:
                self.command(Direction.STOP, 0)
                self.mode = Mode.ARRIVED
                self.state.mode = Mode.ARRIVED
                self._decide("ALVO_ALCANCADO", f"{target.label} area={target.area_ratio:.1%}")
                return
            self._target_arrived = False

        if self._front_critical(now):
            self.command(Direction.STOP, 0)
            self._enter_avoid(now, "distancia critica")
            return
        if self._front_blocked(now):
            self.command(Direction.STOP, 0)
            self._enter_avoid(now, "obstaculo frontal")
            return

        error = target.center_norm
        p = min(1.0, max(0.0, (target.area_ratio - MIN_AREA_RATIO_FAR) / max(0.001, STOP_AREA_RATIO - MIN_AREA_RATIO_FAR)))
        speed = int(CRUISE_SPEED - p * (CRUISE_SPEED - NEAR_SPEED))
        speed = max(NEAR_SPEED, min(CRUISE_SPEED, speed))

        if abs(error) <= STEER_DEADBAND_NORM:
            direction = Direction.FORWARD
        elif error < 0:
            direction = Direction.FORWARD_LEFT
        else:
            direction = Direction.FORWARD_RIGHT
        self.command(direction, speed)
        self._decide("APROXIMANDO", f"{target.label} area={target.area_ratio:.1%} erro={error:+.2f} PWM={speed}", throttle=0.25)

    # ---------------------------------------------------- chegou ------

    def _tick_arrived(self, now):
        target = self._current_target()
        self.command(Direction.STOP, 0)
        if target is None:
            if now - self._target_last_seen > TARGET_LOST_TIMEOUT:
                self._decide("ALVO_SUMIU", "alvo nao esta mais visivel; nova busca")
                self._enter_search()
            return
        self._lock_target(target, now)
        if target.area_ratio < STOP_AREA_RATIO_RESUME:
            self._target_arrived = False
            self.mode = Mode.APPROACH
            self.state.mode = Mode.APPROACH
            self._decide("ALVO_SE_AFASTOU", "retomando aproximacao")
            return
        self._decide("PARADO_NO_ALVO", f"{target.label} area={target.area_ratio:.1%}", throttle=0.6)

    # ---------------------------------------------------- desvio ------

    def _enter_avoid(self, now, reason):
        self._move_servo(SERVO_CENTER, now, force=True)
        self.mode = Mode.AVOID
        self.state.mode = Mode.AVOID
        self.avoid_phase = "BACK"
        self.avoid_until = now + AVOID_BACK_TIME
        self._decide("DESVIANDO", reason)

    def _tick_avoid(self, now):
        if self.avoid_phase == "BACK":
            self.command(Direction.BACKWARD, RECOVERY_SPEED, allow_when_blocked=True)
            if now >= self.avoid_until:
                self.avoid_phase = "TURN"
                self.avoid_until = now + AVOID_TURN_TIME
            return
        if self.avoid_phase == "TURN":
            self.command(self.turn_direction, AVOID_SPEED, allow_when_blocked=True)
            if now >= self.avoid_until:
                self._decide("DESVIO_CONCLUIDO", "retomando busca")
                self._enter_search()
            return
        # estado inesperado: por segurança, para e recomeça a busca
        self.command(Direction.STOP, 0)
        self._enter_search()

    # ------------------------------------------------------- tick -----

    def tick(self):
        now = time.time()

        if not self.enabled or self.mode in (Mode.IDLE, Mode.EMERGENCY_STOP, Mode.MANUAL):
            return

        if self.mode == Mode.SEARCH:
            if self._searching_turn:
                self._move_servo(SERVO_CENTER, now)
                if now < self._search_turn_until:
                    self.command(self.turn_direction, TURN_SPEED)
                    return
                self.connection.send_command(Direction.STOP, 0, force=True)
                self._searching_turn = False
            self._tick_search(now)
        elif self.mode == Mode.CONFIRM:
            self._tick_confirm(now)
        elif self.mode == Mode.CENTER:
            self._tick_center(now)
        elif self.mode == Mode.APPROACH:
            self._tick_approach(now)
        elif self.mode == Mode.ARRIVED:
            self._tick_arrived(now)
        elif self.mode == Mode.AVOID:
            self._tick_avoid(now)

# =====================================================================
# INTERFACE (TKINTER) - SÓ O ESSENCIAL PARA OPERAR E DEPURAR
# =====================================================================

class RoverApp:
    def __init__(self, root):
        self.root = root
        self.root.title("GEAR AI ROVER - BASICO")
        self.running = True

        self.state = RobotState()
        self.log = EventLog()
        self.connection = RobotConnection(self.state, self.log)
        self.camera = CameraWorker(self.state, self.log)
        self.vision = VisionWorker(self.state, self.log)
        self.navigator = Navigator(self.state, self.connection, self.vision, self.log)

        self._build_ui()
        self._bind_keys()
        self._log_seq_rendered = 0

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(80, self._ui_tick)
        self.root.after(int(CONTROL_INTERVAL * 1000), self._nav_tick)

    # ------------------------------------------------------- UI -------

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)

        left = ttk.Frame(main)
        left.pack(side="left", fill="both", expand=True)
        self.camera_label = ttk.Label(left, text="Aguardando camera...", anchor="center")
        self.camera_label.pack(fill="both", expand=True)

        right = ttk.Frame(main, width=320)
        right.pack(side="right", fill="y")

        self.status_var = tk.StringVar(value="[X] DESCONECTADO")
        ttk.Label(right, textvariable=self.status_var, font=("TkDefaultFont", 11, "bold")).pack(anchor="w")

        self.mode_var = tk.StringVar(value=Mode.IDLE.value)
        ttk.Label(right, textvariable=self.mode_var, font=("TkDefaultFont", 10, "bold")).pack(anchor="w", pady=(4, 8))

        ttk.Label(right, text="Alvo(s) (separados por vírgula):").pack(anchor="w")
        self.targets_var = tk.StringVar(value=", ".join(DEFAULT_TARGETS))
        ttk.Entry(right, textvariable=self.targets_var).pack(fill="x")
        ttk.Button(right, text="Aplicar alvos", command=self._apply_targets).pack(fill="x", pady=(2, 8))

        ttk.Label(right, text="Confiança mínima:").pack(anchor="w")
        self.conf_var = tk.DoubleVar(value=DEFAULT_CONFIDENCE)
        ttk.Scale(right, from_=0.20, to=0.90, variable=self.conf_var,
                  command=self._confidence_changed).pack(fill="x", pady=(0, 8))

        ttk.Button(right, text="INICIAR BUSCA", command=lambda: self.navigator.set_mode(Mode.SEARCH)).pack(fill="x", pady=2)
        ttk.Button(right, text="PARAR", command=self.navigator.stop).pack(fill="x", pady=2)
        ttk.Button(right, text="EMERGENCIA", command=self.navigator.emergency_stop).pack(fill="x", pady=(2, 8))

        manual = ttk.LabelFrame(right, text="Manual (ou use as setas do teclado)")
        manual.pack(fill="x", pady=(0, 8))
        grid = ttk.Frame(manual)
        grid.pack()
        ttk.Button(grid, text="^", width=4,
                   command=lambda: self.navigator.set_manual(Direction.FORWARD, CRUISE_SPEED)).grid(row=0, column=1)
        ttk.Button(grid, text="<", width=4,
                   command=lambda: self.navigator.set_manual(Direction.LEFT, TURN_SPEED)).grid(row=1, column=0)
        ttk.Button(grid, text="v", width=4,
                   command=lambda: self.navigator.set_manual(Direction.BACKWARD, CRUISE_SPEED)).grid(row=1, column=1)
        ttk.Button(grid, text=">", width=4,
                   command=lambda: self.navigator.set_manual(Direction.RIGHT, TURN_SPEED)).grid(row=1, column=2)
        ttk.Button(grid, text="STOP", width=4,
                   command=lambda: self.navigator.set_manual(Direction.STOP, 0)).grid(row=2, column=1)

        self.target_info_var = tk.StringVar(value="--")
        ttk.Label(right, textvariable=self.target_info_var).pack(anchor="w", pady=(4, 0))
        self.us_var = tk.StringVar(value="US: --")
        ttk.Label(right, textvariable=self.us_var).pack(anchor="w")
        self.decision_var = tk.StringVar(value="--")
        ttk.Label(right, textvariable=self.decision_var, wraplength=300).pack(anchor="w", pady=(4, 8))

        ttk.Label(right, text="Log:").pack(anchor="w")
        self.log_text = tk.Text(right, height=14, width=40, state="disabled")
        self.log_text.pack(fill="both", expand=True)

    def _apply_targets(self):
        raw = self.targets_var.get().split(",")
        self.navigator.set_targets(raw)

    def _confidence_changed(self, _value=None):
        self.vision.set_confidence(self.conf_var.get())

    def _bind_keys(self):
        self.root.bind("<KeyPress-Up>", lambda e: self.navigator.set_manual(Direction.FORWARD, CRUISE_SPEED))
        self.root.bind("<KeyPress-Down>", lambda e: self.navigator.set_manual(Direction.BACKWARD, CRUISE_SPEED))
        self.root.bind("<KeyPress-Left>", lambda e: self.navigator.set_manual(Direction.LEFT, TURN_SPEED))
        self.root.bind("<KeyPress-Right>", lambda e: self.navigator.set_manual(Direction.RIGHT, TURN_SPEED))
        self.root.bind("<KeyRelease-Up>", lambda e: self.navigator.set_manual(Direction.STOP, 0))
        self.root.bind("<KeyRelease-Down>", lambda e: self.navigator.set_manual(Direction.STOP, 0))
        self.root.bind("<KeyRelease-Left>", lambda e: self.navigator.set_manual(Direction.STOP, 0))
        self.root.bind("<KeyRelease-Right>", lambda e: self.navigator.set_manual(Direction.STOP, 0))
        self.root.bind("<space>", lambda e: self.navigator.stop())

    def _draw_overlay(self, frame):
        h, w = frame.shape[:2]
        center = w // 2
        cv2.line(frame, (center, 0), (center, h), (255, 255, 255), 1)

        targets = self.vision.get_targets()
        locked = self.navigator._current_target()
        for det in self.vision.get_all_detections():
            is_locked = locked is not None and det.bbox() == locked.bbox() and det.label == locked.label
            if is_locked:
                color, thickness, text = (0, 255, 0), 3, f"ALVO: {det.label} {det.confidence:.2f}"
            elif det.label in targets:
                color, thickness, text = (0, 255, 255), 1, f"outro alvo? {det.label} {det.confidence:.2f}"
            else:
                color, thickness, text = (160, 160, 160), 1, f"{det.label} {det.confidence:.2f}"
            cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), color, thickness)
            cv2.putText(frame, text, (det.x1, max(16, det.y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        cv2.rectangle(frame, (0, 0), (w, 46), (0, 0, 0), -1)
        cv2.putText(frame, f"{self.state.mode.value} | {self.state.ai_decision}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, f"Servo:{self.state.servo_angle} US:{self.state.ultrasonic_cm:.0f}cm FPS:{self.state.fps:.0f}",
                    (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return frame

    def _ui_tick(self):
        if not self.running:
            return
        try:
            frame = self.camera.get_frame()
            if frame is not None:
                self.vision.submit(frame)
                frame = self._draw_overlay(frame)
                if PIL_AVAILABLE:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    image = Image.fromarray(rgb)
                    wa = max(300, self.camera_label.winfo_width() - 10)
                    ha = max(300, self.camera_label.winfo_height() - 10)
                    scale = min(wa / image.width, ha / image.height, 1.0)
                    if scale < 1.0:
                        image = image.resize((max(1, int(image.width * scale)), max(1, int(image.height * scale))))
                    photo = ImageTk.PhotoImage(image)
                    self.camera_label.configure(image=photo, text="")
                    self.camera_label.image = photo

            now = time.time()
            camera_ok = self.state.camera_ok and now - self.state.camera_timestamp <= CAMERA_STALE_SECONDS
            self.status_var.set(
                ("[OK] CONECTADO" if self.state.connected else "[X] RECONECTANDO...")
                + (" | CAMERA OK" if camera_ok else " | CAMERA OFF")
            )
            self.mode_var.set(self.state.mode.value)
            if self.state.target_visible:
                self.target_info_var.set(f"Alvo: {self.state.target_label} {self.state.target_confidence:.0%} area={self.state.target_area:.1%}")
            else:
                self.target_info_var.set("Alvo: --")
            self.us_var.set(f"US: {self.state.ultrasonic_cm:.0f}cm" if self.state.ultrasonic_ok else "US: sem leitura")
            self.decision_var.set(f"{self.state.ai_decision}: {self.state.ai_reason}")
            self._refresh_logs()
        except Exception as exc:
            self.state.last_error = str(exc)
        self.root.after(80, self._ui_tick)

    def _refresh_logs(self):
        seq = self.log.seq
        if seq == self._log_seq_rendered:
            return
        items = self.log.snapshot()
        novos = seq - self._log_seq_rendered
        self._log_seq_rendered = seq
        trecho = items[-novos:] if 0 < novos <= len(items) else items[-200:]
        self.log_text.configure(state="normal")
        for msg in trecho[-200:]:
            self.log_text.insert("end", msg + "\n")
        total = int(self.log_text.index("end-1c").split(".")[0])
        if total > 400:
            self.log_text.delete("1.0", "%d.0" % (total - 300))
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def _nav_tick(self):
        if not self.running:
            return
        try:
            self.navigator.tick()
        except Exception as exc:
            self.state.last_error = str(exc)
            self.log.add(f"Erro no navegador: {exc}", "ERROR")
            try:
                self.connection.send_command(Direction.STOP, 0, force=True)
            except Exception as exc2:
                self.log.add(f"FALHA AO PARAR apos erro: {exc2}", "ERROR")
        self.root.after(max(40, int(CONTROL_INTERVAL * 1000)), self._nav_tick)

    def close(self):
        if not self.running:
            return
        self.running = False

        def _try(desc, fn):
            try:
                fn()
            except Exception as exc:
                print(f"[encerramento] falhou em {desc}: {exc}")

        _try("parada de emergencia", self.navigator.emergency_stop)
        _try("comando STOP final", lambda: self.connection.send_command(Direction.STOP, 0, force=True))
        _try("fechar camera", self.camera.close)
        _try("fechar visao", self.vision.close)
        _try("fechar conexao", self.connection.close)
        _try("destruir janela", self.root.destroy)


def main():
    root = tk.Tk()
    app = RoverApp(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.close()


if __name__ == "__main__":
    main()
