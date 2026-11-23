#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
======================================================================
GEAR AI ROVER v21 ULTIMATE - NAVEGAÇÃO INTELIGENTE + DQN + VO + A*
======================================================================

Melhorias v21:
- DQN (Deep Q-Network) com Experience Replay e Target Network
- Visual Odometry com Lucas-Kanade Optical Flow
- A* Path Planning no mapa persistente
- Object Tracking (SORT simplificado)
- Kalman Filter para fusão de sensores
- Behavior Tree para navegação hierárquica
- Curiosity-Driven Exploration (ICM)
- ONNX Runtime para inferência acelerada
- Frame Skipping Adaptativo
- Correções de bugs críticos (v20)
- Reward Shaping avançado
- Detecção de loop closure
- Sistema de metas múltiplas
- Logging estruturado com métricas
"""

import json
import math
import os
import queue
import re
import socket
import threading
import time
import heapq
import random
from collections import deque, defaultdict
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Tuple

import cv2
import numpy as np
import requests
import tkinter as tk
from tkinter import ttk, messagebox

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

try:
    import onnxruntime as ort
    ONNX_AVAILABLE = True
except ImportError:
    ONNX_AVAILABLE = False

try:
    from sklearn.ensemble import IsolationForest
    SKLEARN_AVAILABLE = True
except ImportError:
    IsolationForest = None
    SKLEARN_AVAILABLE = False

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    import torch.optim as optim
    TORCH_AVAILABLE = True
except ImportError:
    torch = None
    nn = None
    F = None
    optim = None
    TORCH_AVAILABLE = False

# =====================================================================
# CONFIGURAÇÃO
# =====================================================================

ESP32_IP = "192.168.4.1"
ESP32_TCP_PORT = 100
STREAM_URL = f"http://{ESP32_IP}:81/stream"
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolo11n.pt")
ONNX_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "yolo11n.onnx")

DEFAULT_TARGETS = ["bottle"]
DEFAULT_CONFIDENCE = 0.50

# PWM / tração
# [Fase 1.3] ATENÇÃO: PWM não é proporcional à vontade. Abaixo de ~90/255 estes
# motoredutores não vencem o atrito estático com o chassi no chão: a bobina
# apita (PWM audível ~976 Hz do analogWrite) e a roda NÃO gira. Foi o que
# aconteceu com o valor 35 que eu tinha colocado aqui.
# Pisos de arranque, aplicados em send_command().
# GIRAR NO LUGAR exige bem mais torque que andar reto: as duas rodas raspam no
# chão em sentidos opostos. Por isso o piso do giro é maior que o do avanço.
MIN_SPEED = 38        # avanço / ré
MIN_TURN_SPEED = 38   # giro no lugar (LEFT/RIGHT)
MAX_SPEED = 60
# Cruzeiro calibrado no chão: ~40 é bom, 70 já é forte demais. Só funciona
# porque TODO arranque recebe o chute MOTOR_BOOST_SPEED antes de cair pra cá.
DEFAULT_MANUAL_SPEED = 40
SEARCH_SPEED = 38     # piso da rampa de aproximação (perto do alvo)
FOLLOW_SPEED = 38     # cruzeiro de aproximação (longe do alvo)
TURN_SPEED = 38      # giro no lugar: acima do piso de 95, mas menos brusco
ALIGN_SPEED = 38
AVOID_SPEED = 38
RECOVERY_SPEED = 60   # desatolar pede um pouco mais
MOTOR_BOOST_SPEED = 60
MOTOR_BOOST_TIME = 0.18

# Ultrassônico
SAFE_DISTANCE_CM = 25
CRITICAL_DISTANCE_CM = 15
ULTRA_MIN_VALID_CM = 2
ULTRA_MAX_VALID_CM = 400
# [Fase 1.1] O HC-SR04 / firmware devolve 0 quando não há eco: nada dentro do
# alcance = caminho livre. Tratar 0 como "longe/livre", não como leitura
# inválida -- senão o avanço (FORWARD) e a varredura de BUSCA ficam presos.
ULTRA_CLEAR_CM = 200.0
ULTRASONIC_STALE_SECONDS = 1.20
ULTRA_QUERY_INTERVAL = 0.18

# Servo / câmera
SERVO_MIN = 20
SERVO_MAX = 160
SERVO_CENTER = 90
SERVO_STEP = 5
SERVO_MOVE_INTERVAL = 0.22
SERVO_SETTLE_TIME = 0.30
# [Fase 1] A rampa suave do servo é feita NO FIRMWARE (1°/25 ms ≈ 40°/s).
# O Python apenas envia o ângulo-alvo uma vez e estima quando o servo chegou.
SERVO_SLEW_DEG_PER_SEC = 40.0
# [Fase 2] VISÃO ATIVA (a câmera aponta para o alvo, o chassi segue a câmera).
# É o esquema VOR/OKR usado em cabeças pan-tilt de robôs: realimentação visual
# aponta a câmera, e o corpo acompanha para trazer a câmera de volta ao centro.
# O limite existe porque o ultrassônico está no MESMO servo: virar a câmera
# demais deixaria o sonar medindo para o lado, não para a frente.
SERVO_TRACK_MAX_OFFSET = 25      # graus máx. fora do centro durante a perseguição
SERVO_TRACK_MIN_STEP = 4         # ignora correções menores (evita ficar tremendo)
FOLLOW_STEER_BEARING_DEG = 4.0   # abaixo disso segue reto
ALIGN_BEARING_DEG = 30.0         # acima disso vale parar e girar o chassi
SERVO_SCAN_ANGLES = [45, 65, 90, 115, 135]
# [Fase 2.1] Confirmacao M-de-N (iniciacao de track classica): exigimos
# SEARCH_CONFIRM_FRAMES deteccoes DENTRO da janela, tolerando ate
# SEARCH_CONFIRM_MAX_MISSES quadros sem deteccao no meio. Antes eram 3 quadros
# CONSECUTIVOS e um unico quadro perdido zerava a contagem - com o YOLO piscando
# (limiar 0.50 + borrao de movimento + JPEG corrompido do ESP32) ela quase nunca
# fechava, e o rover via o alvo por um instante e voltava a girar. O NUMERO de
# deteccoes exigido nao mudou: a protecao contra falso positivo e a mesma.
SEARCH_CONFIRM_FRAMES = 3
SEARCH_CONFIRM_WINDOW = 1.60        # era 0.90: curto demais para ~6 FPS reais
SEARCH_CONFIRM_MAX_MISSES = 5       # ~0.5 s de buraco a 10 Hz de navegacao
SEARCH_CONFIRM_BEARING_TOL = 18.0   # graus de variacao de rumo ainda aceitos
SEARCH_MIN_CONFIDENCE = 0.45
SEARCH_REPOSITION_INTERVAL = 0.35
SEARCH_FORWARD_HOP_TIME = 0.65
SEARCH_FORWARD_HOP_SPEED = 45
# Legado: substituídos pela varredura por setores (SEARCH_SECTOR_DEG abaixo).
SEARCH_TURN_BASE_TIME = 0.34
SEARCH_TURN_MAX_TIME = 0.80
SEARCH_MAX_STATIONARY_CYCLES = 1
CAMERA_HFOV_DEG = 65.0
SERVO_RIGHT_IS_POSITIVE = True

# [Fase 2] Busca sistemática em 360 graus.
# Cobertura visual com o chassi parado = varredura do servo (45..135 = 90 graus)
# + campo da câmera (65) ~= 155 graus. Girando 110 graus por setor sobram ~45 de
# sobreposição, que absorvem o erro do giro em malha aberta (não há encoders).
# 360/110 -> 4 setores fecham a volta.
SEARCH_SECTOR_DEG = 110.0
SEARCH_FULL_CIRCLE_DEG = 360.0
SEARCH_SECTOR_MIN_TIME = 0.35   # piso de segurança para o giro de setor
SEARCH_SECTOR_MAX_TIME = 3.00   # teto de segurança para o giro de setor

# Alinhamento do chassi
# [Fase 2] O menor giro que este chassi consegue dar a partir de PARADO é o
# chute de arranque (~0.18 s) => ~11 graus. Com a zona morta antiga de 0.075
# (= +-2.4 graus) o alinhamento era GEOMETRICAMENTE impossível: qualquer pulso
# passava 4x do alvo e ele oscilava até estourar ALIGN_MAX_TIME.
# Agora ALIGN faz só o giro GROSSO (parado) e o ajuste fino é feito dirigindo,
# com FORWARD_LEFT/FORWARD_RIGHT, onde não há atrito estático a vencer.
# Regra: o pulso mínimo (= o chute de arranque, ~11.4 graus) TEM de caber
# dentro da zona morta, senão o rover passa do ponto a cada correção e oscila.
ALIGN_DEADBAND = 0.45        # ~+-14.6 graus > pulso máximo (~12.9) -> assenta
ALIGN_HARD_DEADBAND = 0.60   # acima disso, pulso 25% mais longo
ALIGN_PULSE_TIME = 0.18      # = MOTOR_BOOST_TIME: o pulso é o chute, e só
# (FOLLOW_STEER_DEADBAND removido: a decisao do chassi passou a ser em GRAUS
#  de bearing -> FOLLOW_STEER_BEARING_DEG / ALIGN_BEARING_DEG.)
ALIGN_MAX_TIME = 4.0
TURN_DEG_PER_SEC = 70.0

# Rastreamento do alvo: "coasting" através de oclusões curtas.
# A literatura de multi-object tracking chama de coasting manter a track viva
# com a última posição durante uma oclusão. É eficaz, mas precisa ser LIMITADO:
# coasting longo demais passa a gerar falsos positivos. ~12 quadros @10 FPS ~ 1.2 s.
TRACK_MAX_AGE_FRAMES = 12
TARGET_COAST_MAX_AGE = 10     # até aqui ainda seguimos o alvo "de memória"

# Alvo
# [Fase 2] Anti-alucinação: com a câmera do ESP32 o YOLO nano gera falsos
# positivos pequenos e de baixa confiança. Além dos 3 quadros de confirmação,
# exigimos confiança mínima e um tamanho mínimo de caixa.
TARGET_DETECTION_MIN_CONF = 0.40
TARGET_MIN_AREA_RATIO = 0.0035   # caixa < 0.35% do quadro = ruído
TARGET_MEMORY_SECONDS = 1.20
TARGET_LOST_TIMEOUT = 1.80
STOP_AREA_RATIO = 0.28
MIN_AREA_RATIO = 0.010
# [Revisao] Histerese na chegada: sem isso, com a área da caixa oscilando por
# ruído do detector bem em cima de STOP_AREA_RATIO, o rover ficava "ir - parar
# - ir - parar" no último meio metro. Uma vez que ele CHEGOU, só volta a
# avançar se o alvo realmente se afastar (área cair bem abaixo do limite), não
# só oscilar 1-2% em torno dele.
TARGET_STOP_RESUME_RATIO = STOP_AREA_RATIO * 0.82

# [Revisao] TRAVA DE IDENTIDADE (evita "zapear" entre objetos da mesma classe).
# Antes, TODA leitura de alvo (em _update_target_state, _acquire_target e
# _align) chamava vision.get_target_detection(), que devolve o candidato de
# MAIOR visual_score NO QUADRO ATUAL - sem nenhuma noção de "é o mesmo objeto
# de antes". Com duas garrafas no campo de visão, bastava a pontuação oscilar
# um pouco (a maior mudando de tamanho/posição a cada quadro) para o rover
# trocar de alvo a cada tick: o bearing pulava, o alinhamento nunca fechava e
# o robô ficava "confuso" entre os dois objetos. Agora o rover mantém a
# identidade pela CAIXA (IoU) do último alvo travado, e só aceita um candidato
# novo como "o mesmo alvo" se ele se sobrepuser o suficiente à última posição
# vista. Se nenhum candidato bater, o alvo travado é tratado como ausente
# neste quadro (cai no coasting/timeout já existentes) em vez de pular para
# outro objeto da mesma classe.
TARGET_LOCK_IOU_MIN = 0.12

# Obstáculos visuais
VISUAL_OBSTACLE_MIN_CONF = 0.42
OBSTACLE_BOTTOM_Y_RATIO = 0.50
OBSTACLE_CENTER_X_RATIO = 0.30
OBSTACLE_AREA_RATIO = 0.08
OBSTACLE_MEMORY_SECONDS = 0.70

VISUAL_OBSTACLE_LABELS = {
    "person", "bicycle", "car", "motorcycle", "bus", "train", "truck",
    "boat", "chair", "couch", "bed", "dining table", "laptop", "backpack",
    "suitcase", "potted plant", "bench", "dog", "cat", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "traffic light",
    "stop sign", "fire hydrant", "wall", "door", "window", "stairs",
    "railing", "fence", "pole", "box", "crate", "barrel", "plant"
}

# Navegação
CONTROL_INTERVAL = 0.10
COMMAND_INTERVAL = 0.10
HEARTBEAT_INTERVAL = 0.50
MANUAL_WATCHDOG_SECONDS = 0.75
CAMERA_STALE_SECONDS = 1.50

# Desvio
AVOID_BACK_TIME = 0.30
AVOID_TURN_TIME = 0.48
AVOID_VERIFY_TIME = 0.40
AVOID_SERVO_WAIT = 0.55
AVOID_MAX_CYCLES = 3      # depois disso o desvio vira recuperação escalonada

# Travamento
STUCK_COMMAND_SECONDS = 3.0
STUCK_SIGNATURE_SECONDS = 2.6
STUCK_COOLDOWN_SECONDS = 4.0
# [Fase 2] Limiar do "progress checker": fluxo optico medio abaixo disso,
# COMANDANDO movimento, significa que o mundo nao esta mudando => travado.
STUCK_FLOW_THRESHOLD = 0.8
STUCK_BACK_TIME = 0.55
STUCK_TURN_TIME = 0.65
STUCK_FORWARD_TIME = 0.40

# Caminho ruim
BAD_PATH_WINDOW = 20
BAD_PATH_THRESHOLD = 0.65
BAD_PATH_COOLDOWN_SECONDS = 4.0

# Interface
CENTER_DEADZONE = 65
MAX_CAMERA_BUFFER = 500_000

# DQN Config
DQN_STATE_DIM = 8
DQN_ACTION_DIM = 5
DQN_HIDDEN_DIM = 128
DQN_REPLAY_BUFFER_SIZE = 10000
DQN_BATCH_SIZE = 64
DQN_GAMMA = 0.95
DQN_LEARNING_RATE = 1e-3
DQN_EPSILON_START = 1.0
DQN_EPSILON_END = 0.05
DQN_EPSILON_DECAY = 0.995
DQN_TARGET_UPDATE = 100
DQN_SAVE_INTERVAL = 50

# Visual Odometry
VO_GRID_SIZE = 20
VO_MIN_FEATURES = 15
VO_MAX_FEATURES = 100
VO_FEATURE_QUALITY = 0.01
VO_FEATURE_DISTANCE = 10

# A* Path Planning
ASTAR_GRID_SIZE = 25.0
ASTAR_MAX_CELLS = 2000

# Kalman Filter
KALMAN_PROCESS_NOISE = 0.01
KALMAN_MEASUREMENT_NOISE = 0.1

# Frame Skip Adaptativo
TARGET_FPS = 15
FPS_ADJUST_INTERVAL = 2.0

# =====================================================================
# ENUMS
# =====================================================================

class NavMode(Enum):
    IDLE = "PARADO"
    MANUAL = "MANUAL"
    SEARCH = "BUSCA"
    ALIGN = "ALINHAR"
    FOLLOW = "SEGUIR"
    AVOID = "DESVIO"
    STUCK = "TRAVADO"
    EMERGENCY_STOP = "EMERGENCIA"
    PATH_FOLLOW = "SEGUINDO_CAMINHO"


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


class PathQuality(Enum):
    EXCELLENT = "EXCELENTE"
    GOOD = "BOM"
    WARNING = "AVISO"
    DANGER = "PERIGO"


class BehaviorStatus(Enum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"
    RUNNING = "RUNNING"

# =====================================================================
# DQN - DEEP Q-NETWORK
# =====================================================================

# [Revisao] Sem torch instalado `nn` e None, e `class DQNNetwork(nn.Module)`
# estourava AttributeError JA NO IMPORT: o programa inteiro nao abria por causa
# de uma dependencia OPCIONAL. Com torch ausente a base vira `object`; estas
# classes so sao instanciadas sob `if TORCH_AVAILABLE`, entao nada mais muda.
_TorchModule = nn.Module if TORCH_AVAILABLE else object


class DQNNetwork(_TorchModule):
    """Rede neural para aproximação da função Q."""
    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int = 128):
        super(DQNNetwork, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim * 2),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_dim * 2, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim)
        )

    def forward(self, x):
        return self.net(x)


class ReplayBuffer:
    """Buffer de experiência para treinamento off-policy."""
    def __init__(self, capacity: int = DQN_REPLAY_BUFFER_SIZE):
        self.buffer = deque(maxlen=capacity)
        self.lock = threading.Lock()

    def push(self, state, action, reward, next_state, done):
        with self.lock:
            self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size: int):
        with self.lock:
            if len(self.buffer) < batch_size:
                batch_size = len(self.buffer)
            batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return (
            torch.FloatTensor(states),
            torch.LongTensor(actions),
            torch.FloatTensor(rewards),
            torch.FloatTensor(next_states),
            torch.FloatTensor(dones)
        )

    def __len__(self):
        with self.lock:
            return len(self.buffer)


class DQNAgent:
    """Agente DQN com Target Network e Experience Replay."""
    def __init__(self, state_dim: int = DQN_STATE_DIM, 
                 action_dim: int = DQN_ACTION_DIM,
                 hidden_dim: int = DQN_HIDDEN_DIM,
                 load_path: Optional[str] = None):
        if not TORCH_AVAILABLE:
            raise ImportError("PyTorch não disponível. Instale: pip install torch")

        self.state_dim = state_dim
        self.action_dim = action_dim
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Redes
        self.policy_net = DQNNetwork(state_dim, action_dim, hidden_dim).to(self.device)
        self.target_net = DQNNetwork(state_dim, action_dim, hidden_dim).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()

        # Otimizador
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=DQN_LEARNING_RATE)
        self.memory = ReplayBuffer(DQN_REPLAY_BUFFER_SIZE)

        # Hiperparâmetros
        self.gamma = DQN_GAMMA
        self.epsilon = DQN_EPSILON_START
        self.epsilon_end = DQN_EPSILON_END
        self.epsilon_decay = DQN_EPSILON_DECAY
        self.batch_size = DQN_BATCH_SIZE
        self.target_update = DQN_TARGET_UPDATE
        self.update_count = 0

        # Estatísticas
        self.loss_history = deque(maxlen=100)
        self.q_value_history = deque(maxlen=100)
        self.reward_history = deque(maxlen=100)

        # Carregar modelo
        self.load_path = load_path
        if load_path and os.path.exists(load_path):
            self.load()

    def select_action(self, state, greedy=False):
        """Seleciona ação usando epsilon-greedy."""
        if not greedy and random.random() < self.epsilon:
            return random.randint(0, self.action_dim - 1)

        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            q_values = self.policy_net(state_tensor)
            self.q_value_history.append(float(q_values.max().cpu()))
            return q_values.argmax().item()

    def learn(self):
        """Atualiza a rede usando uma amostra do replay buffer."""
        if len(self.memory) < self.batch_size:
            return 0.0

        states, actions, rewards, next_states, dones = self.memory.sample(self.batch_size)
        states = states.to(self.device)
        actions = actions.to(self.device)
        rewards = rewards.to(self.device)
        next_states = next_states.to(self.device)
        dones = dones.to(self.device)

        # Q(s, a)
        current_q = self.policy_net(states).gather(1, actions.unsqueeze(1))

        # max Q'(s', a')
        with torch.no_grad():
            next_q = self.target_net(next_states).max(1)[0]
            target_q = rewards + (1 - dones) * self.gamma * next_q

        # Loss
        loss = F.smooth_l1_loss(current_q.squeeze(), target_q)

        # Otimização
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), max_norm=10)
        self.optimizer.step()

        # [Fase 1.3] detach() antes de virar escalar: sem isso o PyTorch emite
        # "UserWarning: Converting a tensor with requires_grad=True to a scalar".
        self.loss_history.append(float(loss.detach().cpu()))
        self.update_count += 1

        # Atualizar target network
        if self.update_count % self.target_update == 0:
            self.target_net.load_state_dict(self.policy_net.state_dict())

        # Decay epsilon
        self.epsilon = max(self.epsilon_end, self.epsilon * self.epsilon_decay)

        # [Revisao] Faltava o detach() AQUI (so a linha do loss_history tinha).
        # Era o que ainda emitia "UserWarning: Converting a tensor with
        # requires_grad=True to a scalar".
        return float(loss.detach().cpu())

    def get_stats(self):
        """Retorna estatísticas de treinamento."""
        return {
            "epsilon": round(self.epsilon, 4),
            "updates": self.update_count,
            "avg_loss": round(np.mean(self.loss_history), 4) if self.loss_history else 0,
            "avg_q": round(np.mean(self.q_value_history), 2) if self.q_value_history else 0,
            "avg_reward": round(np.mean(self.reward_history), 2) if self.reward_history else 0,
            "buffer_size": len(self.memory)
        }

    def save(self, path: Optional[str] = None):
        """Salva o modelo."""
        path = path or self.load_path
        if not path:
            return
        try:
            torch.save({
                'policy_state_dict': self.policy_net.state_dict(),
                'target_state_dict': self.target_net.state_dict(),
                'optimizer_state_dict': self.optimizer.state_dict(),
                'epsilon': self.epsilon,
                'update_count': self.update_count,
            }, path)
        except Exception as exc:
            print("[DQN] falha ao salvar em %s: %s" % (path, exc))

    def load(self):
        """Carrega o modelo."""
        try:
            checkpoint = torch.load(self.load_path, map_location=self.device)
            self.policy_net.load_state_dict(checkpoint['policy_state_dict'])
            self.target_net.load_state_dict(checkpoint['target_state_dict'])
            self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
            self.epsilon = checkpoint.get('epsilon', DQN_EPSILON_START)
            self.update_count = checkpoint.get('update_count', 0)
        except FileNotFoundError:
            pass   # primeira execucao: nao ha modelo salvo ainda, normal
        except Exception as exc:
            print("[DQN] modelo existe mas nao pode ser carregado (%s); comecando do zero" % exc)

# =====================================================================
# VISUAL ODOMETRY - LUCAS-KANADE
# =====================================================================

class VisualOdometry:
    """Estima deslocamento relativo usando optical flow esparsa."""
    def __init__(self):
        self.prev_gray = None
        self.prev_kp = None
        self.accumulated_dx = 0.0
        self.accumulated_dy = 0.0
        self.accumulated_rotation = 0.0
        self.frame_count = 0
        self.valid = False

        self.feature_params = dict(
            maxCorners=VO_MAX_FEATURES,
            qualityLevel=VO_FEATURE_QUALITY,
            minDistance=VO_FEATURE_DISTANCE,
            blockSize=7
        )

        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
        )

        self.dx_history = deque(maxlen=10)
        self.dy_history = deque(maxlen=10)
        self.rot_history = deque(maxlen=10)

    def update(self, frame, servo_angle=SERVO_CENTER):
        if frame is None:
            return 0.0, 0.0, 0.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (320, 240))

        if self.prev_gray is None or self.prev_kp is None or len(self.prev_kp) < VO_MIN_FEATURES:
            self.prev_gray = gray
            self.prev_kp = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.valid = False
            return 0.0, 0.0, 0.0

        next_kp, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_kp, None, **self.lk_params
        )

        if next_kp is None or status is None:
            self.prev_gray = gray
            self.prev_kp = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.valid = False
            return 0.0, 0.0, 0.0

        good_prev = self.prev_kp[status == 1]
        good_next = next_kp[status == 1]

        if len(good_prev) < VO_MIN_FEATURES:
            self.prev_gray = gray
            self.prev_kp = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            self.valid = False
            return 0.0, 0.0, 0.0

        dx_raw = np.mean(good_next[:, 0] - good_prev[:, 0])
        dy_raw = np.mean(good_next[:, 1] - good_prev[:, 1])

        h, w = gray.shape
        center_x, center_y = w / 2, h / 2
        prev_rel = good_prev - np.array([center_x, center_y])
        next_rel = good_next - np.array([center_x, center_y])
        prev_angles = np.arctan2(prev_rel[:, 1], prev_rel[:, 0])
        next_angles = np.arctan2(next_rel[:, 1], next_rel[:, 0])
        angle_diffs = np.unwrap(next_angles - prev_angles)
        rotation_raw = np.mean(angle_diffs) * 180 / np.pi

        self.dx_history.append(dx_raw)
        self.dy_history.append(dy_raw)
        self.rot_history.append(rotation_raw)

        dx = float(np.median(self.dx_history))
        dy = float(np.median(self.dy_history))
        rotation = float(np.median(self.rot_history))

        servo_offset = (servo_angle - SERVO_CENTER) * 0.1
        dx -= servo_offset

        self.accumulated_dx += dx
        self.accumulated_dy += dy
        self.accumulated_rotation += rotation
        self.frame_count += 1
        self.valid = True

        self.prev_gray = gray
        self.prev_kp = good_next.reshape(-1, 1, 2)

        if self.frame_count % 10 == 0:
            new_kp = cv2.goodFeaturesToTrack(gray, mask=None, **self.feature_params)
            if new_kp is not None and len(new_kp) > VO_MIN_FEATURES:
                self.prev_kp = new_kp

        return dx, dy, rotation

    def get_pose(self):
        return {
            "dx_total": round(self.accumulated_dx, 2),
            "dy_total": round(self.accumulated_dy, 2),
            "rotation_total": round(self.accumulated_rotation, 2),
            "frames": self.frame_count,
            "valid": self.valid
        }

    def reset(self):
        self.prev_gray = None
        self.prev_kp = None
        self.accumulated_dx = 0.0
        self.accumulated_dy = 0.0
        self.accumulated_rotation = 0.0
        self.frame_count = 0
        self.valid = False
        self.dx_history.clear()
        self.dy_history.clear()
        self.rot_history.clear()


# =====================================================================
# KALMAN FILTER - FUSÃO DE SENSORES
# =====================================================================

class KalmanFilter2D:
    """Filtro de Kalman para estimar posição e velocidade 2D."""
    def __init__(self):
        self.state = np.zeros(4)
        self.covariance = np.eye(4) * 100

        self.F = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=float)

        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)

        self.Q = np.eye(4) * KALMAN_PROCESS_NOISE
        self.R = np.eye(2) * KALMAN_MEASUREMENT_NOISE

        self.initialized = False

    def predict(self, dt=1.0):
        F = self.F.copy()
        F[0, 2] = dt
        F[1, 3] = dt

        self.state = F @ self.state
        self.covariance = F @ self.covariance @ F.T + self.Q

    def update(self, measurement):
        z = np.array(measurement, dtype=float)

        if not self.initialized:
            self.state[:2] = z
            self.initialized = True
            return

        y = z - self.H @ self.state
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        self.covariance = (np.eye(4) - K @ self.H) @ self.covariance

    def get_position(self):
        return float(self.state[0]), float(self.state[1])

    def get_velocity(self):
        return float(self.state[2]), float(self.state[3])

    def reset(self):
        self.state = np.zeros(4)
        self.covariance = np.eye(4) * 100
        self.initialized = False


# =====================================================================
# A* PATH PLANNING
# =====================================================================

class AStarPlanner:
    """Planejador de caminho A* para o mapa do rover."""
    def __init__(self, cell_size=ASTAR_GRID_SIZE):
        self.cell_size = cell_size
        self.grid = {}
        self.path = []

    def _world_to_grid(self, x, y):
        return int(x / self.cell_size), int(y / self.cell_size)

    def _grid_to_world(self, gx, gy):
        return (gx + 0.5) * self.cell_size, (gy + 0.5) * self.cell_size

    def _heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def _neighbors(self, node):
        x, y = node
        return [
            (x+1, y), (x-1, y), (x, y+1), (x, y-1),
            (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)
        ]

    def update_grid(self, cells):
        self.grid = {}
        for (gx, gy), cell in cells.items():
            if cell.blocked_score > 0.5 and cell.blocked_hits >= 2:
                self.grid[(gx, gy)] = 100
            elif cell.visits > 0:
                self.grid[(gx, gy)] = 1
            else:
                self.grid[(gx, gy)] = 5

    def plan(self, start_x, start_y, goal_x, goal_y, max_iterations=1000):
        start = self._world_to_grid(start_x, start_y)
        goal = self._world_to_grid(goal_x, goal_y)

        if start == goal:
            return []

        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        iterations = 0
        while open_set and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct_path(came_from, current, start)

            for neighbor in self._neighbors(current):
                if self.grid.get(neighbor, 0) >= 100:
                    continue

                tentative_g = g_score[current] + self._heuristic(current, neighbor)
                tentative_g += self.grid.get(neighbor, 5)

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def _reconstruct_path(self, came_from, current, start):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()

        world_path = []
        for node in path:
            wx, wy = self._grid_to_world(node[0], node[1])
            world_path.append((wx, wy))
        return world_path


# =====================================================================
# OBJECT TRACKING - SORT SIMPLIFICADO
# =====================================================================

class ObjectTracker:
    """Tracker simples baseado em IoU para manter IDs de objetos."""
    def __init__(self, max_age=TRACK_MAX_AGE_FRAMES, min_hits=2, iou_threshold=0.3):
        self.tracks = {}
        self.next_id = 0
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.track_history = {}

    def _iou(self, box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union = area1 + area2 - inter

        return inter / union if union > 0 else 0

    def update(self, detections):
        for track_id in list(self.tracks.keys()):
            self.tracks[track_id]['age'] += 1

        matched = set()
        used_tracks = set()

        for det in detections:
            det_bbox = (det.x1, det.y1, det.x2, det.y2)
            best_iou = self.iou_threshold
            best_track = None

            for track_id, track in self.tracks.items():
                if track_id in used_tracks:
                    continue
                iou = self._iou(det_bbox, track['bbox'])
                if iou > best_iou:
                    best_iou = iou
                    best_track = track_id

            if best_track is not None:
                self.tracks[best_track]['bbox'] = det_bbox
                self.tracks[best_track]['age'] = 0
                self.tracks[best_track]['hits'] += 1
                self.tracks[best_track]['label'] = det.label
                self.tracks[best_track]['confidence'] = det.confidence
                used_tracks.add(best_track)
                matched.add(id(det))

                if best_track not in self.track_history:
                    self.track_history[best_track] = []
                self.track_history[best_track].append({
                    'cx': det.cx, 'cy': det.cy, 'time': time.time()
                })
                if len(self.track_history[best_track]) > 20:
                    del self.track_history[best_track][:-20]

        for det in detections:
            if id(det) not in matched:
                self.tracks[self.next_id] = {
                    'bbox': (det.x1, det.y1, det.x2, det.y2),
                    'age': 0,
                    'hits': 1,
                    'label': det.label,
                    'confidence': det.confidence
                }
                self.track_history[self.next_id] = [{
                    'cx': det.cx, 'cy': det.cy, 'time': time.time()
                }]
                self.next_id += 1

        for track_id in list(self.tracks.keys()):
            if self.tracks[track_id]['age'] > self.max_age:
                del self.tracks[track_id]
                # BUG: o histórico do track morto nunca era removido -> vazamento
                # de memória numa sessão longa.
                self.track_history.pop(track_id, None)

        active_tracks = []
        for track_id, track in self.tracks.items():
            if track['hits'] >= self.min_hits:
                active_tracks.append({
                    'id': track_id,
                    'bbox': track['bbox'],
                    'label': track['label'],
                    'confidence': track.get('confidence', 0),
                    'age': track['age']
                })

        return active_tracks

    def get_active_tracks(self):
        """Lê as tracks vivas SEM avançar o tracker.

        BUG corrigido: o overlay chamava update() de novo a cada quadro, então o
        tracker era avançado DUAS vezes por ciclo - as tracks envelheciam no
        dobro da velocidade e os hits contavam em dobro."""
        return [
            {
                'id': tid, 'bbox': tr['bbox'], 'label': tr['label'],
                'confidence': tr.get('confidence', 0), 'age': tr['age'],
            }
            for tid, tr in self.tracks.items() if tr['hits'] >= self.min_hits
        ]

    def find_track_by_label(self, label, max_age=None, preferred_bbox=None,
                             min_iou=0.05):
        """Track viva com este rótulo (para 'coasting' de oclusão).

        [Revisao] Antes escolhia sempre a track de MENOR idade com o rótulo -
        ou seja, com dois objetos da mesma classe (ex. duas garrafas), se o
        alvo real ficasse oclusão e o OUTRO objeto continuasse visível, o
        coasting "seguia" o objeto errado, porque a track dele tinha idade 0
        e a do alvo real, oclusão, tinha idade > 0. Agora, quando o chamador
        informa a última caixa vista do alvo (preferred_bbox), a escolha
        prioriza a track cuja última posição mais se sobrepõe a ela; só cai
        de volta para "menor idade" quando não há informação de posição."""
        limit = self.max_age if max_age is None else max_age
        candidates = [
            (tid, tr) for tid, tr in self.tracks.items()
            if tr['label'] == label and tr['hits'] >= self.min_hits and tr['age'] <= limit
        ]
        if not candidates:
            return None

        if preferred_bbox is not None:
            best = None
            best_iou = min_iou
            for tid, tr in candidates:
                iou = self._iou(preferred_bbox, tr['bbox'])
                if iou > best_iou:
                    best_iou = iou
                    best = (tid, tr)
            if best is not None:
                return best
            # Nenhuma track bate com a posição esperada: não adivinha - cai no
            # fallback de idade abaixo só entre as MESMAS não descartadas,
            # mantendo o comportamento antigo como último recurso.

        return min(candidates, key=lambda item: item[1]['age'])

    def get_track_velocity(self, track_id):
        if track_id not in self.track_history or len(self.track_history[track_id]) < 2:
            return 0, 0

        history = self.track_history[track_id][-5:]
        if len(history) < 2:
            return 0, 0

        dx = history[-1]['cx'] - history[0]['cx']
        dy = history[-1]['cy'] - history[0]['cy']
        dt = history[-1]['time'] - history[0]['time']

        if dt > 0:
            return dx / dt, dy / dt
        return 0, 0

    def reset(self):
        self.tracks.clear()
        self.track_history.clear()
        self.next_id = 0


# =====================================================================
# CURIOSITY-DRIVEN EXPLORATION (ICM)
# =====================================================================

class CuriosityModule:
    """Módulo de curiosidade para exploração intrínseca."""
    def __init__(self, state_dim=5, action_dim=5, log=None):
        self.log = log
        if not TORCH_AVAILABLE:
            self.enabled = False
            return

        self.enabled = True
        self.state_dim = state_dim
        self.action_dim = action_dim

        self.feature = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 32)
        )

        self.forward_model = nn.Sequential(
            nn.Linear(32 + action_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 32)
        )

        self.inverse_model = nn.Sequential(
            nn.Linear(32 * 2, 64),
            nn.ReLU(),
            nn.Linear(64, action_dim)
        )

        self.optimizer = optim.Adam(
            list(self.feature.parameters()) +
            list(self.forward_model.parameters()) +
            list(self.inverse_model.parameters()),
            lr=1e-3
        )

        self.curiosity_history = deque(maxlen=100)

    def compute_intrinsic_reward(self, state, action, next_state):
        if not self.enabled:
            return 0.0

        try:
            s = torch.FloatTensor(state).unsqueeze(0)
            ns = torch.FloatTensor(next_state).unsqueeze(0)
            a = torch.zeros(1, self.action_dim)
            a[0, action] = 1.0

            phi_s = self.feature(s)
            phi_ns = self.feature(ns)

            pred_phi_ns = self.forward_model(torch.cat([phi_s, a], dim=1))
            forward_loss = F.mse_loss(pred_phi_ns, phi_ns)

            pred_a = self.inverse_model(torch.cat([phi_s, phi_ns], dim=1))
            inverse_loss = F.cross_entropy(pred_a, torch.LongTensor([action]))

            # [Revisao] `.item()` era chamado ANTES do backward: total_loss virava
            # um float puro, `isinstance(total_loss, torch.Tensor)` dava False e
            # backward() NUNCA rodava. O modulo nunca aprendeu nada - e mesmo
            # assim o erro dele (ruido de uma rede aleatoria) entrava na
            # recompensa do DQN a cada ciclo.
            self.optimizer.zero_grad()
            total_loss = forward_loss + 0.1 * inverse_loss
            total_loss.backward()
            self.optimizer.step()

            curiosity = float(forward_loss.detach()) * 10.0
            self.curiosity_history.append(curiosity)
            return curiosity
        except Exception as exc:
            # CLAUDE.md REGRA 10: nao esconder erro. Caminho quente -> throttle.
            if self.log is not None:
                self.log.add_throttled("icm", "Curiosidade (ICM) falhou: %s" % exc)
            return 0.0

    def get_avg_curiosity(self):
        return np.mean(self.curiosity_history) if self.curiosity_history else 0.0


# =====================================================================
# FRAME SKIP ADAPTATIVO
# =====================================================================

class AdaptiveFrameSkip:
    """Ajusta dinamicamente quantos frames pular baseado no FPS."""
    def __init__(self, target_fps=TARGET_FPS):
        self.target_fps = target_fps
        self.skip_count = 0
        self.frame_counter = 0
        self.last_adjust = time.time()
        self.fps_history = deque(maxlen=5)

    def should_process(self, current_fps):
        self.frame_counter += 1
        now = time.time()

        if now - self.last_adjust >= FPS_ADJUST_INTERVAL:
            self.fps_history.append(current_fps)
            avg_fps = np.mean(self.fps_history) if self.fps_history else current_fps

            if avg_fps < self.target_fps * 0.7:
                self.skip_count = min(self.skip_count + 1, 3)
            elif avg_fps > self.target_fps * 1.3 and self.skip_count > 0:
                self.skip_count = max(self.skip_count - 1, 0)

            self.last_adjust = now

        return self.frame_counter % (self.skip_count + 1) == 0

    def get_stats(self):
        return {
            "skip": self.skip_count,
            "target_fps": self.target_fps,
            "avg_fps": round(np.mean(self.fps_history), 1) if self.fps_history else 0
        }


# =====================================================================
# LOOP CLOSURE DETECTION
# =====================================================================

class LoopClosureDetector:
    """Detecta quando o rover retorna a um local já visitado."""
    def __init__(self):
        self.orb = cv2.ORB_create(nfeatures=200)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.keyframes = []
        self.threshold = 25
        self.min_distance = 100.0
        self.last_check = 0
        self.check_interval = 30

    def check(self, frame, current_pose):
        self.last_check += 1
        if self.last_check < self.check_interval:
            return False, None
        self.last_check = 0

        if frame is None:
            return False, None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.resize(gray, (160, 120))
        kp, des = self.orb.detectAndCompute(gray, None)

        if des is None or len(des) < 10:
            return False, None

        cx, cy = current_pose

        for i, (old_des, old_pose, _) in enumerate(self.keyframes):
            dist = math.hypot(cx - old_pose[0], cy - old_pose[1])
            if dist < self.min_distance:
                continue

            try:
                matches = self.bf.knnMatch(des, old_des, k=2)
                good = []
                for match_pair in matches:
                    if len(match_pair) == 2:
                        m, n = match_pair
                        if m.distance < 0.75 * n.distance:
                            good.append(m)

                if len(good) > self.threshold:
                    return True, old_pose
            except Exception:
                continue

        self.keyframes.append((des, current_pose, time.time()))
        if len(self.keyframes) > 30:
            self.keyframes.pop(0)

        return False, None

# =====================================================================
# DADOS
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
    servo_angle: int = SERVO_CENTER
    track_id: int = -1

    @property
    def cx(self) -> float:
        return (self.x1 + self.x2) / 2.0

    @property
    def cy(self) -> float:
        return (self.y1 + self.y2) / 2.0

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
    def center_error(self) -> float:
        return self.cx - self.frame_w / 2.0

    @property
    def center_norm(self) -> float:
        return self.center_error / max(1.0, self.frame_w / 2.0)

    @property
    def bottom_ratio(self) -> float:
        return self.y2 / max(1, self.frame_h)

    @property
    def visual_score(self) -> float:
        size_score = min(1.0, self.area_ratio * 8.0)
        center_score = 1.0 - min(1.0, abs(self.center_norm))
        return self.confidence * 0.55 + size_score * 0.25 + center_score * 0.20


@dataclass
class RobotState:
    connected: bool = False
    camera_ok: bool = False
    camera_timestamp: float = 0.0

    ultrasonic_cm: float = 0.0
    ultrasonic_timestamp: float = 0.0
    ultrasonic_ok: bool = False
    ultrasonic_blocked: bool = False
    ultrasonic_angle: int = SERVO_CENTER
    ultrasonic_pending_angle: int = SERVO_CENTER

    servo_angle: int = SERVO_CENTER
    servo_target_angle: int = SERVO_CENTER
    servo_moving: bool = False
    servo_settled_since: float = 0.0

    mode: NavMode = NavMode.IDLE
    last_command: str = "STOP"
    last_speed: int = 0

    target_visible: bool = False
    target_coasting: bool = False   # seguindo pela track, sem detecção fresca
    target_label: str = "-"
    target_confidence: float = 0.0
    target_center_error: float = 0.0
    target_area: float = 0.0
    target_bearing_deg: float = 0.0

    alignment_error_deg: float = 0.0
    alignment_progress: float = 0.0

    obstacle: bool = False
    obstacle_label: str = "-"
    obstacle_confidence: float = 0.0
    obstacle_area: float = 0.0

    stuck: bool = False
    bad_path: bool = False
    path_quality: PathQuality = PathQuality.GOOD

    fps: float = 0.0
    inference_ms: float = 0.0
    tx_count: int = 0
    rx_count: int = 0

    ai_decision: str = "AGUARDANDO"
    ai_reason: str = "Inicializando"
    search_heading: str = "CENTRO"
    exploration_percent: float = 0.0
    total_detections: int = 0
    last_error: str = ""

    # Novos campos v21
    dqn_epsilon: float = DQN_EPSILON_START
    dqn_loss: float = 0.0
    vo_dx: float = 0.0
    vo_dy: float = 0.0
    vo_rotation: float = 0.0
    kalman_x: float = 0.0
    kalman_y: float = 0.0
    kalman_vx: float = 0.0
    kalman_vy: float = 0.0
    path_plan: List[Tuple[float, float]] = field(default_factory=list)
    curiosity_reward: float = 0.0
    frame_skip: int = 0
    loop_closure_detected: bool = False
    tracked_objects: int = 0

    # Memória de sonar por ângulo
    ultrasonic_map: dict = field(default_factory=dict)


# =====================================================================
# MAPA ESPACIAL PERSISTENTE (OTIMIZADO)
# =====================================================================

@dataclass
class MapCell:
    visits: int = 0
    free_hits: int = 0
    blocked_hits: int = 0
    target_hits: int = 0
    last_seen: float = 0.0
    label: str = ""

    @property
    def blocked_score(self):
        total = self.free_hits + self.blocked_hits
        return self.blocked_hits / total if total else 0.0


class PersistentMap:
    """Mapa 2D estimado com A* integration e loop closure correction."""
    CELL_CM = 25.0
    MAX_CELLS = 2500
    PWM_TO_CM_S = 0.13
    TURN_SIGN = 1.0

    def __init__(self, path):
        self.path = path
        self.lock = threading.RLock()
        self.cells = {}
        self.x_cm = 0.0
        self.y_cm = 0.0
        self.heading_deg = 0.0
        self.last_motion_time = time.time()
        self.last_command = Direction.STOP
        self.last_save = 0.0
        self.dirty = False
        self._last_speed = 0
        self._summary_cache = None
        self._summary_time = 0.0
        self.planner = AStarPlanner(self.CELL_CM)
        self.load()

    def _key(self, x, y):
        return (int(math.floor(x / self.CELL_CM)), int(math.floor(y / self.CELL_CM)))

    def _cell(self, key):
        if key not in self.cells:
            self.cells[key] = MapCell()
        return self.cells[key]

    def reset(self):
        with self.lock:
            self.cells.clear()
            self.x_cm = 0.0
            self.y_cm = 0.0
            self.heading_deg = 0.0
            self.last_motion_time = time.time()
            self.last_command = Direction.STOP
            self.dirty = True
            self.save(force=True)

    def update_motion(self, direction, speed, now=None):
        now = now or time.time()
        with self.lock:
            dt = max(0.0, min(0.35, now - self.last_motion_time))

            if self.last_command in (Direction.FORWARD, Direction.FORWARD_LEFT, 
                                     Direction.FORWARD_RIGHT, Direction.BACKWARD, 
                                     Direction.BACKWARD_LEFT, Direction.BACKWARD_RIGHT):
                sign = -1.0 if self.last_command in (Direction.BACKWARD, Direction.BACKWARD_LEFT, 
                                                      Direction.BACKWARD_RIGHT) else 1.0
                diag = self.last_command in (Direction.FORWARD_LEFT, Direction.BACKWARD_LEFT, 
                                              Direction.FORWARD_RIGHT, Direction.BACKWARD_RIGHT)
                rel = -28.0 if self.last_command in (Direction.FORWARD_LEFT, Direction.BACKWARD_LEFT) else                       28.0 if diag else 0.0
                theta = math.radians(self.heading_deg + rel)
                dist = max(0.0, min(8.0, self._speed_to_cm_s(self._last_speed) * dt))
                self.x_cm += math.cos(theta) * dist * sign
                self.y_cm += math.sin(theta) * dist * sign
            elif self.last_command == Direction.LEFT:
                self.heading_deg -= self._turn_deg(self._last_speed, dt)
            elif self.last_command == Direction.RIGHT:
                self.heading_deg += self._turn_deg(self._last_speed, dt)

            self.heading_deg = (self.heading_deg + 540.0) % 360.0 - 180.0
            self.last_command = direction
            self._last_speed = int(speed)
            self.last_motion_time = now

            key = self._key(self.x_cm, self.y_cm)
            c = self._cell(key)
            c.visits += 1
            c.last_seen = now
            self.dirty = True

    def _speed_to_cm_s(self, pwm):
        return max(0.0, float(pwm) * self.PWM_TO_CM_S)

    def _turn_deg(self, pwm, dt):
        return min(20.0, float(pwm) / 255.0 * 85.0 * dt)

    def _mark_ray(self, distance, angle_deg, now):
        if distance <= 0:
            return
        theta = math.radians(self.heading_deg + angle_deg)
        steps = max(1, int(distance / self.CELL_CM))
        for i in range(1, steps + 1):
            d = min(distance, i * self.CELL_CM)
            x = self.x_cm + math.cos(theta) * d
            y = self.y_cm + math.sin(theta) * d
            c = self._cell(self._key(x, y))
            c.free_hits += 1
            c.last_seen = now
        if distance < ULTRA_MAX_VALID_CM - 5:
            d = max(0.0, distance - 2.0)
            x = self.x_cm + math.cos(theta) * d
            y = self.y_cm + math.sin(theta) * d
            c = self._cell(self._key(x, y))
            c.blocked_hits += 1
            c.last_seen = now
        self._trim()
        self.dirty = True

    def observe_ultrasonic(self, distance, servo_angle, now=None):
        with self.lock:
            now = now or time.time()
            rel = float(servo_angle - SERVO_CENTER)
            if not SERVO_RIGHT_IS_POSITIVE:
                rel *= -1.0
            self._mark_ray(float(distance), rel, now)

    def record_target(self, label, bearing_deg=0.0, now=None):
        with self.lock:
            now = now or time.time()
            theta = math.radians(self.heading_deg + bearing_deg)
            x = self.x_cm + math.cos(theta) * 50.0
            y = self.y_cm + math.sin(theta) * 50.0
            c = self._cell(self._key(x, y))
            c.target_hits += 1
            c.label = label
            c.last_seen = now
            self.dirty = True

    def best_exploration_heading(self):
        with self.lock:
            candidates = []
            for (gx, gy), cell in self.cells.items():
                if cell.free_hits == 0:
                    continue
                if cell.blocked_score > 0.65 and cell.blocked_hits >= 2:
                    continue
                unknown = 0
                for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                    n = (gx+dx, gy+dy)
                    if n not in self.cells:
                        unknown += 1
                if unknown == 0:
                    continue
                cx = (gx + 0.5) * self.CELL_CM
                cy = (gy + 0.5) * self.CELL_CM
                dist = math.hypot(cx - self.x_cm, cy - self.y_cm)
                angle = math.degrees(math.atan2(cy - self.y_cm, cx - self.x_cm))
                rel = (angle - self.heading_deg + 540.0) % 360.0 - 180.0
                score = unknown * 2.0 + min(2.0, cell.visits / 5.0) - dist / 120.0 - cell.blocked_score * 3.0
                candidates.append((score, rel, dist))
            if not candidates:
                return None
            candidates.sort(reverse=True)
            return candidates[0][1]

    def plan_path_to(self, goal_x, goal_y):
        """Planeja caminho A* até o objetivo."""
        with self.lock:
            self.planner.update_grid(self.cells)
            return self.planner.plan(self.x_cm, self.y_cm, goal_x, goal_y)

    def correct_loop_closure(self, detected_pose):
        """Corrige pose quando loop closure é detectado."""
        with self.lock:
            dx = detected_pose[0] - self.x_cm
            dy = detected_pose[1] - self.y_cm
            self.x_cm += dx * 0.5
            self.y_cm += dy * 0.5
            self.dirty = True

    def mark_heading_blocked(self, bearing_deg):
        with self.lock:
            theta = math.radians(self.heading_deg + bearing_deg)
            x = self.x_cm + math.cos(theta) * self.CELL_CM
            y = self.y_cm + math.sin(theta) * self.CELL_CM
            c = self._cell(self._key(x, y))
            c.blocked_hits += 1
            c.last_seen = time.time()
            self.dirty = True

    def record_scan_sector(self, left, right, center, now=None):
        now = now or time.time()
        with self.lock:
            for angle, distance in ((-35, left), (0, center), (35, right)):
                if distance <= 0:
                    continue
                theta = math.radians(self.heading_deg + angle)
                d = min(float(distance), 100.0)
                x = self.x_cm + math.cos(theta) * d
                y = self.y_cm + math.sin(theta) * d
                c = self._cell(self._key(x, y))
                if distance >= SAFE_DISTANCE_CM:
                    c.free_hits += 2
                else:
                    c.blocked_hits += 2
                c.last_seen = now
            self.dirty = True

    def summary(self):
        # [Revisao] Era TRES varreduras completas de ate 2500 celulas (7500
        # iteracoes), chamadas pela interface a 12 Hz E pela recompensa do DQN a
        # 10 Hz - as duas na thread do Tk. Agora: UMA varredura, com cache de
        # meio segundo. A pose (que muda todo tique) continua sendo lida ao vivo.
        with self.lock:
            now = time.time()
            if self._summary_cache is None or now - self._summary_time >= 0.5:
                blocked = visited = targets = 0
                for c in self.cells.values():
                    if c.blocked_score > 0.65 and c.blocked_hits >= 2:
                        blocked += 1
                    if c.visits > 0:
                        visited += 1
                    targets += c.target_hits
                self._summary_cache = (len(self.cells), visited, blocked, targets)
                self._summary_time = now
            n_cells, visited, blocked, targets = self._summary_cache
            return {
                "cells": n_cells,
                "visited": visited,
                "blocked": blocked,
                "targets": targets,
                "x_cm": round(self.x_cm, 1),
                "y_cm": round(self.y_cm, 1),
                "heading": round(self.heading_deg, 1),
            }

    def export_cells(self):
        with self.lock:
            return {f"{k[0]},{k[1]}": {
                "visits": v.visits, "free": v.free_hits, "blocked": v.blocked_hits,
                "target_hits": v.target_hits, "label": v.label, "last_seen": v.last_seen
            } for k, v in self.cells.items()}

    def save(self, force=False):
        with self.lock:
            now = time.time()
            # [Revisao] O projeto vive dentro do OneDrive: gravar um JSON
            # INDENTADO de ate 2500 celulas a cada 2 s mantinha o sincronismo do
            # OneDrive trabalhando sem parar. Compacto (~40% menor) e a cada 5 s.
            if not force and (not self.dirty or now - self.last_save < 5.0):
                return
            payload = {
                "version": 2,
                "cell_cm": self.CELL_CM,
                "pose": {"x_cm": self.x_cm, "y_cm": self.y_cm, "heading_deg": self.heading_deg},
                "cells": self.export_cells(),
            }
            tmp = self.path + ".tmp"
            try:
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(payload, f, ensure_ascii=False, separators=(",", ":"))
                os.replace(tmp, self.path)
                self.last_save = now
                self.dirty = False
            except OSError:
                pass

    def load(self):
        with self.lock:
            try:
                with open(self.path, "r", encoding="utf-8") as f:
                    payload = json.load(f)
                pose = payload.get("pose", {})
                self.x_cm = float(pose.get("x_cm", 0.0))
                self.y_cm = float(pose.get("y_cm", 0.0))
                self.heading_deg = float(pose.get("heading_deg", 0.0))
                self.cells.clear()
                for raw_key, raw in payload.get("cells", {}).items():
                    a, b = raw_key.split(",", 1)
                    self.cells[(int(a), int(b))] = MapCell(
                        visits=int(raw.get("visits", 0)),
                        free_hits=int(raw.get("free", 0)),
                        blocked_hits=int(raw.get("blocked", 0)),
                        target_hits=int(raw.get("target_hits", 0)),
                        last_seen=float(raw.get("last_seen", 0.0)),
                        label=str(raw.get("label", "")),
                    )
                self.dirty = False
            except (OSError, ValueError, TypeError, json.JSONDecodeError):
                self.cells.clear()

    def _trim(self):
        if len(self.cells) <= self.MAX_CELLS:
            return
        ranked = sorted(self.cells.items(), key=lambda kv: kv[1].last_seen)
        for key, _ in ranked[:max(1, len(self.cells) - self.MAX_CELLS)]:
            self.cells.pop(key, None)


# =====================================================================
# EVENT LOG
# =====================================================================

class EventLog:
    def __init__(self, maxlen: int = 2000):
        self.seq = 0   # contador p/ a UI saber se ha algo novo sem varrer a lista
        self._throttle = {}
        self.items = deque(maxlen=maxlen)
        self.lock = threading.Lock()
        self.metrics = defaultdict(lambda: deque(maxlen=100))

    def add(self, message: str, level: str = "INFO"):
        self.seq += 1
        prefixes = {
            "INIT": "[R]", "INFO": "[i]", "WARN": "[!]", "ERROR": "[X]",
            "TARGET": "[T]", "NAV": "[N]", "OBSTACLE": "[O]", "RECOVER": "[REC]",
            "AI": "[AI]", "DQN": "[DQN]", "VO": "[VO]", "KALMAN": "[K]",
            "PATH": "[P]", "TRACK": "[TR]", "LOOP": "[L]",
        }
        with self.lock:
            stamp = time.strftime("%H:%M:%S")
            self.items.append(f"[{stamp}] {prefixes.get(level, '-')} [{level}] {message}")

    def add_throttled(self, key: str, message: str, level: str = "WARN", interval: float = 5.0):
        """Registra um erro que acontece em caminho quente sem inundar o log.

        CLAUDE.md REGRA 10: nunca esconder erro com try/except genérico. Mas
        logar a cada tick inundaria a tela; então limitamos por chave."""
        now = time.time()
        if now - self._throttle.get(key, 0.0) < interval:
            return
        self._throttle[key] = now
        self.add(message, level)

    def add_metric(self, name: str, value: float):
        with self.lock:
            self.metrics[name].append((time.time(), value))

    def get_metric_avg(self, name: str, window_seconds: float = 60.0):
        with self.lock:
            data = self.metrics.get(name, deque())
            now = time.time()
            values = [v for t, v in data if now - t <= window_seconds]
            return np.mean(values) if values else 0.0

    def snapshot(self):
        with self.lock:
            return list(self.items)

# =====================================================================
# TCP / ESP32 (OTIMIZADO)
# =====================================================================

class RobotConnection:
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
                self.send_heartbeat()
                self.last_heartbeat = now

            # [Fase 2] Com visão ativa o servo raramente fica exatamente no
            # centro. Aceitamos consultar enquanto ele estiver dentro do cone
            # frontal - a leitura continua servindo para "tem algo à frente?".
            if (now - self.last_ultra_query >= ULTRA_QUERY_INTERVAL
                    and not self.state.servo_moving
                    and abs(self.state.servo_angle - SERVO_CENTER) <= SERVO_TRACK_MAX_OFFSET):
                self.query_ultrasonic(angle_hint=self.state.servo_angle)
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
                # Silencio INTENCIONAL e estreito: o socket ja falhou em
                # conectar; fechar so libera o descritor. Nao ha erro util aqui.
                pass

    def _disconnect(self):
        with self.lock:
            try:
                if self.sock:
                    self.sock.close()
            except OSError:
                # Silencio INTENCIONAL e estreito: fechar socket ja quebrado.
                pass
            self.sock = None
            self.state.connected = False

    def close(self):
        self.stop_event.set()
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
            # [Fase 1.3] Nunca comandar movimento abaixo do piso de arranque:
            # o motor só apitaria (PWM audível) sem girar a roda com carga.
            # [Fase 2] Giro no lugar tem piso próprio, maior.
            floor = MIN_TURN_SPEED if direction in (Direction.LEFT, Direction.RIGHT) else MIN_SPEED
            speed = max(floor, speed)

        key = (direction.value, speed)
        now = time.time()
        if not force and now - self.last_send_at.get(key, 0.0) < COMMAND_INTERVAL:
            return True
        self.last_send_at[key] = now

        payload = json.dumps({
            "N": 102,
            "H": "AI",
            "D1": direction.value,
            "D2": speed,
        }, separators=(",", ":"))
        ok = self.send_raw(payload)
        if ok:
            self.state.last_command = direction.name
            self.state.last_speed = speed
        return ok

    def set_servo(self, angle: int, force: bool = False) -> bool:
        # [Fase 1] Envia apenas o ANGULO-ALVO. A rampa suave e feita no firmware
        # (updateServo, ~40 graus/s). Este metodo NAO estima mais a posicao real
        # do servo - isso agora e responsabilidade do SmartNavigator, que conhece
        # a velocidade da rampa. Aqui so registramos o envio.
        angle = int(max(SERVO_MIN, min(SERVO_MAX, angle)))
        now = time.time()
        self.state.servo_target_angle = angle
        if not force and now - self.last_servo_send < SERVO_MOVE_INTERVAL:
            return True

        payload = json.dumps({
            "N": 5,
            "H": "AI",
            "D1": 1,
            "D2": angle,
        }, separators=(",", ":"))
        ok = self.send_raw(payload)
        if ok:
            self.last_servo_send = now
        return ok

    def query_ultrasonic(self, angle_hint: Optional[int] = None) -> bool:
        if angle_hint is None:
            angle_hint = self.state.servo_angle
        try:
            self.ultra_query_angle = int(angle_hint)
        except Exception:
            self.ultra_query_angle = SERVO_CENTER
        self.state.ultrasonic_pending_angle = self.ultra_query_angle
        return self.send_raw('{"N":21,"H":"AI"}')

    def send_heartbeat(self) -> bool:
        # [Fase 1 - R1] O ESP32 espera EXATAMENTE "{Heartbeat}" e o consome
        # internamente (nao repassa ao Arduino). Enviar "{\"N\":100}" aqui fazia
        # o Arduino parar os motores a cada 0,5 s (stutter) e ainda derrubava o
        # TCP a cada ~4 s. "{\"N\":100}" agora e reservado para STOP explicito.
        return self.send_raw("{Heartbeat}")

    def _receive(self):
        # [Revisao] Antes o recv() rodava DENTRO do `with self.lock`. Com timeout
        # de 0,08 s e sem dados chegando, o lock ficava tomado ~80% do tempo - e
        # send_raw() precisa do MESMO lock. Ou seja: todo comando de motor (STOP
        # de seguranca inclusive) podia esperar ate 80 ms, na thread do Tk, o que
        # tambem travava a interface. Agora so a leitura da referencia do socket
        # e protegida; recv e send em direcoes opostas do mesmo socket TCP sao
        # seguros em paralelo.
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

        # [Fase 1.2] O ESP32 descarrega a serial do Arduino quando vê '}', e nao
        # no '\n'. Fragmentar por linha deixava a leitura do ultrassonico presa
        # no buffer ate a mensagem seguinte (~180 ms de atraso, as vezes mais).
        # Extraimos os frames {...} completos assim que chegam.
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
            # ESPERADO: a maioria dos frames do firmware NÃO é JSON válido
            # ({ok}, {Heartbeat}, {ultrasonic:N}). Cai no regex abaixo de
            # propósito. Exceção estreita para não engolir outros erros.
            pass

        lower = text.lower()
        patterns = (
            r"(?:ultrasonic|ultrassonic|ultrassonico|distance|dist|range|us)\s*[:=]\s*(\d+(?:\.\d+)?)",
            r"(\d+(?:\.\d+)?)\s*cm",
        )
        for pattern in patterns:
            match = re.search(pattern, lower)
            if match:
                self._set_ultrasonic(float(match.group(1)))
                return

    def _extract_distance(self, obj):
        if isinstance(obj, dict):
            for key in ("ultrasonic", "ultrasonic_cm", "distance", "distance_cm", "dist", "cm", "sonar", "range"):
                value = obj.get(key)
                if isinstance(value, (int, float)):
                    return float(value)
                if isinstance(value, str):
                    match = re.search(r"-?\d+(?:\.\d+)?", value)
                    if match:
                        return float(match.group(0))
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
        # [Fase 1.1] 0 (ou negativo) = sem eco -> caminho livre, não leitura ruim.
        if distance <= 0.0:
            distance = ULTRA_CLEAR_CM
        if not (ULTRA_MIN_VALID_CM <= distance <= ULTRA_MAX_VALID_CM):
            return

        now = time.time()
        angle = int(self.ultra_query_angle)
        self.state.ultrasonic_cm = distance
        self.state.ultrasonic_timestamp = now
        self.state.ultrasonic_ok = True
        self.state.ultrasonic_blocked = distance <= SAFE_DISTANCE_CM
        self.state.ultrasonic_angle = angle
        self.state.ultrasonic_map[angle] = distance


# =====================================================================
# CÂMERA ESP32
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
                # [Fase 1.3] Descarta o atraso acumulado SEM cortar no meio de um
                # quadro: recomeça no início do JPEG mais recente. O corte antigo
                # num byte arbitrário (buffer[-200_000:]) deixava um JPEG partido
                # e era o que gerava "Corrupt JPEG data: premature end of data".
                newest = buffer.rfind(b"\xff\xd8")
                buffer = buffer[newest:] if newest >= 0 else b""

            while True:
                start = buffer.find(b"\xff\xd8")
                if start < 0:
                    # Só cabeçalhos/lixo do multipart: não deixa o buffer crescer.
                    # Mantém 1 byte porque o 0xFF do marcador pode estar partido
                    # no limite do chunk.
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
# VISÃO / YOLO (COM ONNX SUPPORT)
# =====================================================================

class VisionWorker:
    def __init__(self, state: RobotState, log: EventLog):
        self.state = state
        self.log = log
        self.model = None
        self.onnx_session = None
        self.loaded = False
        self.load_error = ""
        self.targets = set(DEFAULT_TARGETS)
        self.confidence = DEFAULT_CONFIDENCE
        self.detections = []
        self.history = deque(maxlen=100)
        self.lock = threading.RLock()
        self.queue = queue.Queue(maxsize=1)
        self.stop_event = threading.Event()
        self.servo_angle = SERVO_CENTER
        self.frame_servo_angle = SERVO_CENTER
        self.use_onnx = False
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

    def submit(self, frame, servo_angle=SERVO_CENTER):
        self.servo_angle = int(servo_angle)
        item = (frame, self.servo_angle)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.queue.put_nowait(item)
            except queue.Full:
                pass

    def _load_model(self):
        # [Revisao] ARMADILHA REMOVIDA. Se o arquivo yolo11n.onnx existisse, este
        # bloco marcava use_onnx=True, loaded=True e dava RETURN - sem nunca
        # carregar o modelo YOLO. Dai _infer_onnx rodava a sessao ONNX,
        # DESCARTAVA o resultado e chamava _infer_yolo, que fazia
        # self.model.predict(...) com self.model = None: AttributeError em TODO
        # quadro, engolido pelo except do laco de inferencia. Na pratica, bastava
        # exportar um .onnx para a deteccao de objetos parar de funcionar por
        # completo, sem nenhum aviso visivel.
        # O pos-processamento ONNX nunca foi implementado, entao ativar esse
        # caminho nao acelera nada - so duplicaria a inferencia.
        if ONNX_AVAILABLE and os.path.isfile(ONNX_PATH):
            self.log.add(
                "yolo11n.onnx encontrado, mas o pos-processamento ONNX ainda "
                "nao foi implementado; seguindo com o YOLO nativo.", "WARN")

        # YOLO nativo
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
                item = self.queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if not self.loaded:
                continue

            if isinstance(item, tuple):
                frame, servo_angle = item
            else:
                frame, servo_angle = item, SERVO_CENTER
            self.frame_servo_angle = int(servo_angle)

            started = time.perf_counter()
            try:
                if self.use_onnx and self.onnx_session:
                    detections = self._infer_onnx(frame)
                else:
                    detections = self._infer_yolo(frame)
                elapsed = (time.perf_counter() - started) * 1000.0
                with self.lock:
                    self.detections = detections
                    self.history.append((time.time(), detections))
                self.state.inference_ms = elapsed
            except Exception as exc:
                self.log.add(f"Erro inferencia: {exc}", "ERROR")

    def _infer_onnx(self, frame):
        """[NAO ATIVO] Esqueleto da inferência via ONNX Runtime.

        [Revisao] Antes esta função rodava a sessão ONNX de verdade, JOGAVA O
        RESULTADO FORA e chamava _infer_yolo - ou seja, inferência DUPLICADA por
        quadro. O que falta para valer é o pós-processamento (decodificar as
        caixas da saída bruta do modelo, aplicar NMS e mapear as classes).
        Enquanto isso não existir, _load_model não ativa este caminho.

        Pré-processamento de referência, para quem for terminar:
            img = cv2.resize(frame, (640, 640)).transpose(2, 0, 1)
            img = np.expand_dims(img, 0).astype(np.float32) / 255.0
            saida = self.onnx_session.run(
                None, {self.onnx_session.get_inputs()[0].name: img})
        """
        return self._infer_yolo(frame)

    def _infer_yolo(self, frame):
        with self.lock:
            conf = self.confidence

        results = self.model.predict(
            source=frame,
            conf=conf,
            imgsz=640,
            verbose=False,
            device=None,
        )
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
                if isinstance(names, dict):
                    label = str(names.get(cls_id, cls_id)).lower()
                else:
                    label = str(names[cls_id]).lower()
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                out.append(Detection(
                    label=label,
                    confidence=confidence,
                    x1=max(0, min(w - 1, x1)),
                    y1=max(0, min(h - 1, y1)),
                    x2=max(0, min(w - 1, x2)),
                    y2=max(0, min(h - 1, y2)),
                    frame_w=w,
                    frame_h=h,
                    servo_angle=self.frame_servo_angle,
                ))
            except Exception:
                continue
        return out

    def get_all_detections(self):
        with self.lock:
            return list(self.detections)

    def get_target_candidates(self):
        """Todos os candidatos da(s) classe(s)-alvo no quadro atual, do
        maior para o menor visual_score. [Revisao] Antes só existia
        get_target_detection() (devolve 1), o que obrigava o navegador a
        confiar cegamente no "melhor do quadro" sem poder comparar contra o
        alvo já travado. Ver TARGET_LOCK_IOU_MIN."""
        with self.lock:
            targets = set(self.targets)
            conf = self.confidence
            candidates = [
                d for d in self.detections
                if d.label in targets
                and d.confidence >= max(conf, TARGET_DETECTION_MIN_CONF)
                and d.area_ratio >= TARGET_MIN_AREA_RATIO
            ]
        candidates.sort(key=lambda d: d.visual_score, reverse=True)
        return candidates

    def get_target_detection(self):
        candidates = self.get_target_candidates()
        return candidates[0] if candidates else None

    def get_obstacles(self):
        with self.lock:
            targets = set(self.targets)
            detections = list(self.detections)
        return [
            d for d in detections
            if d.label not in targets
            and d.label in VISUAL_OBSTACLE_LABELS
            and d.confidence >= VISUAL_OBSTACLE_MIN_CONF
        ]

    def close(self):
        self.stop_event.set()

# =====================================================================
# NAVEGADOR INTELIGENTE v21
# =====================================================================

class SmartNavigator:
    def __init__(self, state: RobotState, connection: RobotConnection, vision: VisionWorker, log: EventLog):
        self.state = state
        self.connection = connection
        self.vision = vision
        self.log = log

        base = os.path.dirname(os.path.abspath(__file__))
        self.spatial_map = PersistentMap(os.path.join(base, "rover_map.json"))
        self.last_mapped_ultra_ts = 0.0

        # Novos módulos v21
        self.dqn = None
        if TORCH_AVAILABLE:
            try:
                self.dqn = DQNAgent(load_path=os.path.join(base, "rover_dqn.pt"))
                self.log.add("DQN inicializado.", "INIT")
            except Exception as exc:
                self.log.add(f"DQN não disponível: {exc}", "WARN")

        self.visual_odometry = VisualOdometry()
        self.kalman = KalmanFilter2D()
        self.tracker = ObjectTracker()
        self.curiosity = CuriosityModule(log=log)
        self.frame_skipper = AdaptiveFrameSkip()
        self.loop_detector = LoopClosureDetector()

        # Estado interno
        self.mode = NavMode.IDLE
        self.enabled = False
        self.strategy = "ADAPTATIVA"
        self.search_direction = Direction.LEFT

        # Alvo
        self.primary_target: Optional[Detection] = None
        self.primary_target_time = 0.0
        self.target_lost_since: Optional[float] = None
        self.target_bearing_deg = 0.0
        self._last_frame_dims = None           # p/ reconstruir alvo em coasting
        # [Revisao] Trava de identidade por caixa (ver TARGET_LOCK_IOU_MIN):
        # última bbox do alvo confirmado, usada para reconhecer "o mesmo
        # objeto" entre vários candidatos da mesma classe.
        self._locked_bbox = None
        self._locked_frame_dims = None
        # [Revisao] Histerese de chegada (ver TARGET_STOP_RESUME_RATIO):
        # True assim que a caixa do alvo passa de STOP_AREA_RATIO; só volta a
        # False quando a área cai bem abaixo disso (alvo se afastou de
        # verdade), evitando o "ir-parar-ir-parar" por ruído na borda.
        self._target_arrived = False
        self._vo_flow = deque(maxlen=40)       # progresso visual recente
        self._motion_since = 0.0               # quando o movimento atual começou
        self._recovery_attempts = 0            # escalonamento da recuperação
        self._dqn_last_saved = 0               # último update_count salvo em disco
        self._last_visited_count = 0           # p/ premiar célula NOVA, não o total
        self.avoid_cycles = 0                  # tentativas do desvio atual

        # Servo (Fase 1: firmware faz a rampa; aqui so alvo + estimativa)
        self._servo_cmd_angle = SERVO_CENTER   # ultimo alvo enviado ao firmware
        self._servo_cmd_time = 0.0
        self._servo_move_from = float(SERVO_CENTER)
        self._servo_move_start = 0.0

        # Servo / busca
        self.scan_angles = list(SERVO_SCAN_ANGLES)
        self.scan_index = 0
        self.scan_direction = 1
        self.scan_waiting_measurement = False
        self.scan_query_sent = False
        self.scan_query_time = 0.0
        self.scan_last_angle = SERVO_CENTER
        self.scan_cycle = 0
        self.scan_distances = {}
        self.last_search_reposition = 0.0
        self.search_phase = "SCAN"
        self.search_turn_direction = Direction.LEFT
        self.search_turn_until = 0.0
        self.search_hop_until = 0.0
        self.search_heading_score = 0.0
        self.search_confirm_label = ""
        self.search_confirm_count = 0
        self.search_confirm_started = 0.0
        self.search_confirm_last_bearing = 0.0
        self.search_confirm_misses = 0
        self.search_candidate_bearing = 0.0
        self.search_total_cycles = 0
        self.search_last_target_seen = 0.0
        self.search_last_action = ""
        # [Fase 2] varredura sistemática de 360 graus
        self.sweep_stage = 0                   # etapa da varredura visual do setor
        self.sweep_center_since = 0.0
        self.sweep_probe_until = 0.0
        self.search_swept_deg = 0.0            # graus já cobertos nesta posição
        self.search_sector_dir = Direction.LEFT  # sentido fixo durante uma volta
        self.search_confirm_hold_until = 0.0   # janela parada p/ confirmar candidato

        # Alinhamento
        self.align_started = 0.0
        self.align_last_pulse = 0.0
        self.align_until = 0.0
        self.align_initial_done = False

        # Desvio
        self.avoid_phase = "NONE"
        self.avoid_direction = Direction.LEFT
        self.avoid_started = 0.0
        self.avoid_until = 0.0
        self.avoid_wait_until = 0.0
        self.avoid_query_sent = False
        self.avoid_left = 0.0
        self.avoid_right = 0.0

        # Movimento / tração
        self.last_motion_command = Direction.STOP
        self.last_motion_time = 0.0
        self.last_speed = SEARCH_SPEED
        self.boost_until = 0.0
        self.boost_direction = Direction.STOP

        # Manual
        self.manual_last_input = 0.0
        self.manual_command = Direction.STOP
        self._manual_speed = DEFAULT_MANUAL_SPEED  # [Fase 1] p/ reenvio periodico

        # Travamento
        self.last_signature = ""
        self.last_signature_time = 0.0
        self.last_stuck_time = 0.0
        self.stuck_started = 0.0

        # Caminho
        self.path_history = deque(maxlen=BAD_PATH_WINDOW)
        self.last_bad_path_time = 0.0

        # DQN state
        self.last_dqn_state = None
        self.last_dqn_action = 0
        self.last_dqn_reward = 0.0

        self.last_decision_key = ""
        self.last_decision_time = 0.0

        self.log.add("SmartNavigator v21 iniciado - DQN + VO + Kalman + A*.", "INIT")

    def _update_spatial_memory(self, now):
        """Integra movimento estimado, VO, Kalman e novas leituras do sonar."""
        self.spatial_map.update_motion(self.last_motion_command, self.last_speed, now)

        # Atualizar Kalman com pose do mapa
        self.kalman.predict(dt=0.1)
        self.kalman.update([self.spatial_map.x_cm, self.spatial_map.y_cm])
        kx, ky = self.kalman.get_position()
        self.state.kalman_x = kx
        self.state.kalman_y = ky
        self.state.kalman_vx, self.state.kalman_vy = self.kalman.get_velocity()

        ts = self.state.ultrasonic_timestamp
        if self.state.ultrasonic_ok and ts > self.last_mapped_ultra_ts:
            self.spatial_map.observe_ultrasonic(self.state.ultrasonic_cm, self.state.ultrasonic_angle, now)
            self.last_mapped_ultra_ts = ts

        target = self.vision.get_target_detection()
        if target:
            self.spatial_map.record_target(target.label, self._bearing_from_detection(target), now)

        self.spatial_map.save()

        # Loop closure detection
        if self.visual_odometry.valid:
            loop_detected, old_pose = self.loop_detector.check(
                None, (self.spatial_map.x_cm, self.spatial_map.y_cm)
            )
            if loop_detected:
                self.state.loop_closure_detected = True
                self.spatial_map.correct_loop_closure(old_pose)
                self.log.add("Loop closure detectado! Correção aplicada.", "LOOP")
            else:
                self.state.loop_closure_detected = False

    def _build_dqn_state(self):
        """Constrói vetor de estado para o DQN."""
        front = self.state.ultrasonic_map.get(90, 0.0)
        left = max(self.state.ultrasonic_map.get(55, 0.0), self.state.ultrasonic_map.get(45, 0.0))
        right = max(self.state.ultrasonic_map.get(125, 0.0), self.state.ultrasonic_map.get(135, 0.0))

        target = self.vision.get_target_detection()
        target_dist = 1.0 - min(1.0, self.state.target_area * 3.0) if target else 1.0
        target_angle = self.target_bearing_deg / 90.0 if self.target_bearing_deg else 0.0

        return [
            front / 400.0,
            left / 400.0,
            right / 400.0,
            target_dist,
            target_angle,
            self.state.servo_angle / 180.0,
            self.state.alignment_error_deg / 90.0,
            1.0 if self.state.stuck else 0.0
        ]

    def _dqn_action_to_direction(self, action):
        """Converte ação do DQN para Direction."""
        mapping = {
            0: Direction.STOP,
            1: Direction.FORWARD,
            2: Direction.LEFT,
            3: Direction.RIGHT,
            4: Direction.BACKWARD
        }
        return mapping.get(action, Direction.STOP)

    def _compute_reward(self, target_visible, obstacle, stuck, progress):
        """Reward shaping avançado."""
        reward = 0.0

        if target_visible:
            reward += 1.0
            # Bônus por progresso em direção ao alvo
            reward += float(max(-0.5, min(0.5, progress)))

        if obstacle:
            reward -= 2.0

        if stuck:
            reward -= 3.0

        # Penalidade por ficar parado
        if self.last_motion_command == Direction.STOP:
            reward -= 0.1

        # [Revisao] Bonus por exploracao = celulas NOVAS desde o passo anterior.
        # Antes somava 0.01 * TOTAL de celulas visitadas: um termo que so cresce
        # (chegava a +25) e engolia todos os outros (que valem +-3), corrompendo
        # o alvo do TD. Recompensa tem de premiar a DESCOBERTA, nao o acumulado.
        visited = self.spatial_map.summary()['visited']
        novas = max(0, visited - self._last_visited_count)
        self._last_visited_count = visited
        reward += 0.20 * min(5, novas)

        # Curiosidade
        if self.curiosity.enabled:
            next_state = self._build_dqn_state()
            if self.last_dqn_state is not None:
                curiosity_reward = self.curiosity.compute_intrinsic_reward(
                    self.last_dqn_state, self.last_dqn_action, next_state
                )
                reward += curiosity_reward * 0.1
                self.state.curiosity_reward = curiosity_reward

        return reward

    def _decide(self, decision, reason, level="NAV", throttle=0.35):
        now = time.time()
        self.state.ai_decision = str(decision)
        self.state.ai_reason = str(reason)
        key = f"{decision}|{reason}"
        if key != self.last_decision_key or now - self.last_decision_time >= throttle:
            self.log.add(f"{decision}: {reason}", level)
            self.last_decision_key = key
            self.last_decision_time = now

    def _set_mode(self, mode):
        self.mode = mode
        self.state.mode = mode

    def _camera_valid(self, now):
        return self.state.camera_ok and now - self.state.camera_timestamp <= CAMERA_STALE_SECONDS

    def _ultra_valid(self, now):
        return (
            self.state.ultrasonic_ok
            and ULTRA_MIN_VALID_CM <= self.state.ultrasonic_cm <= ULTRA_MAX_VALID_CM
            and now - self.state.ultrasonic_timestamp <= ULTRASONIC_STALE_SECONDS
        )

    def _front_blocked(self, now):
        return not self._ultra_valid(now) or self.state.ultrasonic_cm <= SAFE_DISTANCE_CM

    def _front_critical(self, now):
        return self._ultra_valid(now) and self.state.ultrasonic_cm <= CRITICAL_DISTANCE_CM

    def _acquire_target(self, now):
        """[Fase 2] Devolve o alvo atual, com 'coasting' através de oclusões.

        1) detecção fresca da visão;
        2) senão, se o tracker ainda mantém uma track viva com o mesmo rótulo
           (alguém passou na frente por um instante), seguimos por ela;
        3) senão, None.

        Sem isso, qualquer pessoa cruzando a frente do alvo fazia o rover
        desistir e recomeçar uma volta inteira de 360 graus."""
        target = self._current_target_detection()
        if target is not None:
            self._last_frame_dims = (target.frame_w, target.frame_h)
            self.state.target_coasting = False
            return target, False

        if self.primary_target is None or not self._last_frame_dims:
            return None, False

        found = self.tracker.find_track_by_label(
            self.primary_target.label, max_age=TARGET_COAST_MAX_AGE,
            preferred_bbox=self._locked_bbox,
        )
        if found is None:
            self.state.target_coasting = False
            return None, False

        _tid, tr = found
        x1, y1, x2, y2 = tr['bbox']
        w, h = self._last_frame_dims
        coasted = Detection(
            label=tr['label'],
            confidence=float(tr.get('confidence', 0.0)),
            x1=int(x1), y1=int(y1), x2=int(x2), y2=int(y2),
            frame_w=w, frame_h=h,
            servo_angle=int(self.state.servo_angle),
        )
        self.state.target_coasting = True
        return coasted, True

    def _target_fresh(self, now):
        return self.primary_target is not None and now - self.primary_target_time <= TARGET_MEMORY_SECONDS

    def _bearing_from_detection(self, det):
        image_angle = det.center_norm * (CAMERA_HFOV_DEG / 2.0)
        image_angle = max(-CAMERA_HFOV_DEG / 2.0, min(CAMERA_HFOV_DEG / 2.0, image_angle))
        servo_offset = float(det.servo_angle - SERVO_CENTER)
        if not SERVO_RIGHT_IS_POSITIVE:
            servo_offset *= -1.0
        return servo_offset + image_angle

    @staticmethod
    def _iou_boxes(box1, box2):
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])
        inter = max(0, x2 - x1) * max(0, y2 - y1)
        area1 = max(0, box1[2] - box1[0]) * max(0, box1[3] - box1[1])
        area2 = max(0, box2[2] - box2[0]) * max(0, box2[3] - box2[1])
        union = area1 + area2 - inter
        return inter / union if union > 0 else 0.0

    def _select_locked_target(self, candidates):
        """[Revisao] Escolhe QUAL candidato desta lista é "o alvo", em vez de
        simplesmente pegar o de maior visual_score todo quadro.

        - Sem alvo travado ainda (primeira aquisição): usa o melhor score,
          como antes.
        - Com alvo travado: procura, entre os candidatos da MESMA classe,
          aquele cuja caixa mais se sobrepõe (IoU) à última caixa vista do
          alvo travado. Só é aceito acima de TARGET_LOCK_IOU_MIN.
        - Se o alvo travado não bate com nenhum candidato deste quadro,
          devolve None (o objeto travado não está "aqui" agora) em vez de
          silenciosamente adotar outro objeto da mesma classe. Quem chama já
          sabe lidar com None (coasting via tracker, timeout de alvo, etc.).
        """
        if not candidates:
            return None
        if self._locked_bbox is None or self._locked_frame_dims is None:
            return candidates[0]

        lw, lh = self._locked_frame_dims
        best = None
        best_iou = TARGET_LOCK_IOU_MIN
        for det in candidates:
            if det.frame_w != lw or det.frame_h != lh:
                continue  # resolução mudou: IoU não é comparável
            iou = self._iou_boxes(self._locked_bbox, (det.x1, det.y1, det.x2, det.y2))
            if iou > best_iou:
                best_iou = iou
                best = det
        return best

    def _current_target_detection(self):
        """Ponto único de leitura do alvo visual no quadro atual, já
        aplicando a trava de identidade. Substitui as chamadas diretas a
        vision.get_target_detection() espalhadas pelo navegador."""
        return self._select_locked_target(self.vision.get_target_candidates())

    def _clear_target_lock(self):
        self._locked_bbox = None
        self._locked_frame_dims = None
        self._target_arrived = False

    def _set_target(self, target, now):
        self.primary_target = target
        self.primary_target_time = now
        self.target_bearing_deg = self._bearing_from_detection(target)
        self.state.target_visible = True
        self.state.target_label = target.label
        self.state.target_confidence = target.confidence
        self.state.target_center_error = target.center_error
        self.state.target_area = target.area_ratio
        self.state.target_bearing_deg = self.target_bearing_deg
        # [Revisao] Atualiza a trava de identidade com a caixa deste quadro,
        # para o próximo tick reconhecer o MESMO objeto por sobreposição.
        self._locked_bbox = (target.x1, target.y1, target.x2, target.y2)
        self._locked_frame_dims = (target.frame_w, target.frame_h)

    def _move_servo_slow(self, target, now):
        """[Fase 1] Nao pica mais o movimento em passos de 5 graus.

        Envia o ANGULO-ALVO ao firmware apenas quando ele muda; o firmware faz a
        rampa suave (~40 graus/s, nao bloqueante). Aqui so estimamos a posicao
        atual do servo em funcao do tempo decorrido, para a logica de navegacao
        (_servo_settled, gate do ultrassonico). O servo fica parado quando ja
        esta no alvo (nenhum comando e enviado)."""
        target = int(max(SERVO_MIN, min(SERVO_MAX, target)))
        self.state.servo_target_angle = target

        # Envia o alvo ao firmware SOMENTE quando ele muda. Enquanto o alvo nao
        # muda, nenhum comando de servo e transmitido (servo fica parado).
        if target != self._servo_cmd_angle:
            self._servo_move_from = float(self.state.servo_angle)
            self._servo_move_start = now
            self._servo_cmd_angle = target
            self._servo_cmd_time = now
            self.connection.set_servo(target, force=True)
            self.state.servo_moving = True
            self.state.servo_settled_since = 0.0

        # Estimativa da posicao real do servo (rampa linear do firmware).
        span = abs(target - self._servo_move_from)
        if span <= 0.5:
            self.state.servo_angle = target
        else:
            traveled = (now - self._servo_move_start) * SERVO_SLEW_DEG_PER_SEC
            if traveled >= span:
                self.state.servo_angle = target
            else:
                sign = 1.0 if target >= self._servo_move_from else -1.0
                self.state.servo_angle = int(round(self._servo_move_from + sign * traveled))

        if self.state.servo_angle == target:
            self.state.servo_moving = False
            if self.state.servo_settled_since == 0.0:
                self.state.servo_settled_since = now
        else:
            self.state.servo_moving = True
            self.state.servo_settled_since = 0.0

    def _servo_settled(self, now):
        return (
            self.state.servo_angle == self.state.servo_target_angle
            and self.state.servo_settled_since > 0.0
            and now - self.state.servo_settled_since >= SERVO_SETTLE_TIME
        )

    def _reset_search(self):
        # [Revisao] Toda busca nova começa sem trava de identidade: o objeto
        # que estava travado pode não ser mais o alvo relevante (perdido,
        # trocado pelo operador, etc.) e não deve "vazar" para o próximo
        # candidato confirmado.
        self._clear_target_lock()
        self.scan_index = 0
        self.scan_direction = 1
        self.scan_waiting_measurement = False
        self.scan_query_sent = False
        self.scan_query_time = 0.0
        self.scan_last_angle = SERVO_CENTER
        self.scan_distances = {}
        self.scan_cycle = 0
        self.search_phase = "SCAN"
        self.search_turn_until = 0.0
        self.search_hop_until = 0.0
        self.search_heading_score = 0.0
        self.search_total_cycles = 0
        self.search_confirm_label = ""
        self.search_confirm_count = 0
        self.search_confirm_started = 0.0
        self.search_confirm_last_bearing = 0.0
        self.search_confirm_misses = 0
        self.search_candidate_bearing = 0.0
        self.search_last_action = "RESET"
        self.last_search_reposition = time.time()
        # [Fase 2] Toda busca nova recomeça o círculo do zero: é isso que faz o
        # "perdeu o alvo -> procura de novo" olhar também para trás.
        self.search_swept_deg = 0.0
        self.search_confirm_hold_until = 0.0
        self.sweep_stage = 0
        self.sweep_center_since = 0.0
        self.sweep_probe_until = 0.0
        # Honra a Estratégia escolhida na interface (BUSCA_ESQUERDA/DIREITA).
        self.search_sector_dir = self.search_direction
        self.state.servo_target_angle = self.scan_angles[0]
        self.state.search_heading = f"SCAN {self.scan_angles[0]} graus"

    def _advance_scan(self):
        """[Legado] Passo a passo do scan antigo (parava em 5 ângulos). Não é
        mais chamado: a busca virou varredura visual contínua na fase SCAN."""
        self.scan_index += self.scan_direction
        if self.scan_index >= len(self.scan_angles):
            self.scan_index = len(self.scan_angles) - 2
            self.scan_direction = -1
            self.scan_cycle += 1
        elif self.scan_index < 0:
            self.scan_index = 1
            self.scan_direction = 1
            self.scan_cycle += 1

        self.state.servo_target_angle = self.scan_angles[self.scan_index]
        self.scan_query_sent = False
        self.scan_query_time = 0.0
        self.scan_waiting_measurement = True

    def _scan_sonar_step(self, now):
        """[Legado] Media o sonar parando em cada ângulo. Não é mais chamado -
        o sonar agora só é consultado no centro, antes de avançar."""
        if not self._servo_settled(now):
            return False

        angle = int(self.state.servo_angle)
        if not self.scan_query_sent:
            self.connection.query_ultrasonic(angle_hint=angle)
            self.scan_query_sent = True
            self.scan_query_time = now
            return False

        if (
            self.state.ultrasonic_ok
            and self.state.ultrasonic_angle == angle
            and self.state.ultrasonic_timestamp >= self.scan_query_time
        ):
            self.scan_distances[angle] = self.state.ultrasonic_cm
            self.scan_query_sent = False
            self.scan_last_angle = angle
            return True

        return False

    def _sector_turn_time(self):
        """Tempo de giro para cobrir SEARCH_SECTOR_DEG (malha aberta, sem encoder)."""
        estimate = SEARCH_SECTOR_DEG / max(1.0, TURN_DEG_PER_SEC)
        return max(SEARCH_SECTOR_MIN_TIME, min(SEARCH_SECTOR_MAX_TIME, estimate))

    def _start_sector_turn(self, now, direction, reason):
        """Gira o chassi um setor e contabiliza na volta de 360 graus."""
        turn_time = self._sector_turn_time()
        self.search_turn_direction = direction
        self.state.servo_target_angle = SERVO_CENTER
        self.search_phase = "TURN"
        self.search_turn_until = now + turn_time
        self.search_last_action = f"SETOR_{direction.name}"
        self.search_swept_deg += SEARCH_SECTOR_DEG
        self.command(direction, TURN_SPEED)
        self._decide(
            "VARRER_SETOR",
            f"{direction.name} ~{SEARCH_SECTOR_DEG:.0f}deg em {turn_time:.2f}s | "
            f"volta {min(self.search_swept_deg, SEARCH_FULL_CIRCLE_DEG):.0f}/360 | {reason}",
            throttle=0.6,
        )

    def _finish_scan_cycle(self, now):
        """[Fase 2] Fecha a varredura do setor atual e decide o próximo passo.

        Regra nova: o rover só sai do lugar DEPOIS de cobrir 360 graus na
        posição atual. Antes disso ele gira de setor em setor, sempre no mesmo
        sentido. É isso que permite encontrar um alvo que está atrás dele, e que
        elimina a oscilação esquerda/direita (CLAUDE.md §17)."""
        self.search_total_cycles += 1
        self.scan_cycle += 1

        # Durante uma volta a direção do giro é FIXA, então não é preciso sonar
        # nenhum aqui - só quando o círculo fecha e é hora de escolher para onde
        # ir. Aí o servo já está centralizado e a leitura viva serve.
        center_score = self.state.ultrasonic_cm if self._ultra_valid(now) else 0.0

        # --- Círculo ainda não fechou: girar para o próximo setor ---
        if self.search_swept_deg < SEARCH_FULL_CIRCLE_DEG:
            self._start_sector_turn(now, self.search_sector_dir, "setor varrido sem alvo")
            return

        # --- Volta completa sem encontrar o alvo: mudar de posição ---
        self.search_swept_deg = 0.0

        # A memória do mapa pode redirecionar para uma fronteira promissora.
        remembered = self.spatial_map.best_exploration_heading()
        if remembered is not None and abs(remembered) > 18.0:
            self.search_sector_dir = Direction.RIGHT if remembered > 0 else Direction.LEFT
            self._decide("MAPA_PRIORIZA", f"frontier={remembered:+.1f}deg", throttle=1.0)

        if center_score >= SAFE_DISTANCE_CM:
            self.state.servo_target_angle = SERVO_CENTER
            self.search_phase = "HOP"
            self.search_hop_until = now + SEARCH_FORWARD_HOP_TIME
            self.search_last_action = "HOP"
            self._decide(
                "VOLTA_COMPLETA",
                f"360deg sem alvo; avançando para novo ponto (frente={center_score:.0f}cm)",
                throttle=0.7,
            )
            return

        # Volta completa E frente bloqueada: gira mais um setor procurando saída.
        self._start_sector_turn(now, self.search_sector_dir, f"frente bloqueada {center_score:.0f}cm")

    def _reset_confirmacao(self):
        """[Fase 2.1] Zera o acumulador M-de-N da confirmacao de candidato."""
        self.search_confirm_label = ""
        self.search_confirm_count = 0
        self.search_confirm_misses = 0
        self.search_confirm_started = 0.0

    def _search_target_confirmation(self, target, now):
        """[Fase 2.1] Confirmacao M-de-N em vez de N quadros consecutivos.

        Exige as MESMAS SEARCH_CONFIRM_FRAMES deteccoes, mas dentro de uma
        janela e tolerando ate SEARCH_CONFIRM_MAX_MISSES quadros sem deteccao.
        Deve ser chamada TODO tique, inclusive com target=None - e ela quem
        decide se um quadro perdido derruba ou nao a confirmacao."""
        janela_aberta = (
            self.search_confirm_started > 0
            and now - self.search_confirm_started <= SEARCH_CONFIRM_WINDOW
        )
        conf_min = max(
            SEARCH_MIN_CONFIDENCE,
            self.vision.confidence if hasattr(self.vision, "confidence") else 0.0,
        )

        if target is None or target.confidence < conf_min:
            if janela_aberta:
                self.search_confirm_misses += 1
                if self.search_confirm_misses > SEARCH_CONFIRM_MAX_MISSES:
                    self._reset_confirmacao()
            elif self.search_confirm_started > 0:
                self._reset_confirmacao()      # janela expirou sem fechar
            return False

        bearing = self._bearing_from_detection(target)
        same_label = target.label == self.search_confirm_label
        stable_bearing = (abs(bearing - self.search_confirm_last_bearing)
                          <= SEARCH_CONFIRM_BEARING_TOL)

        if same_label and stable_bearing and janela_aberta:
            self.search_confirm_count += 1
        else:
            self.search_confirm_label = target.label
            self.search_confirm_count = 1
            self.search_confirm_misses = 0
            self.search_confirm_started = now

        self.search_confirm_last_bearing = bearing
        # Rumo do ultimo candidato VALIDO: e para ca que a camera olha durante o
        # CONFIRM, em vez de ficar travada no centro.
        self.search_candidate_bearing = bearing

        if self.search_confirm_count >= SEARCH_CONFIRM_FRAMES:
            self.search_last_target_seen = now
            return True
        return False

    def _search(self, now):
        target = self._current_target_detection()

        # [Revisao] Antes SÓ o giro de setor (TURN) freava para confirmar. Mas a
        # maior parte do tempo de busca é a varredura do SERVO (fase SCAN), e um
        # candidato avistado ali NÃO interrompia a varredura: o servo continuava
        # se afastando dele a cada quadro, o bearing mudava, "bearing estável"
        # nunca fechava, e o rover via o objeto por um instante e seguia
        # varrendo/girando - era esse o bug real por trás de "identifica e
        # continua procurando". Agora qualquer candidato visto FORA da fase
        # CONFIRM/HOP (TURN *ou* SCAN) freia a câmera na hora.
        if target and self.search_phase in ("TURN", "SCAN"):
            self.connection.send_command(Direction.STOP, 0, force=True)
            if self.search_phase == "TURN":
                # [Fase 2.1] Devolve ao contador da volta a fracao do setor que
                # NAO chegou a ser girada. Sem isso o angulo abortado ficava
                # marcado como coberto e a volta de 360 deixava um ponto cego.
                restante = max(0.0, self.search_turn_until - now)
                if restante > 0.0:
                    fracao = min(1.0, restante / max(0.01, self._sector_turn_time()))
                    self.search_swept_deg = max(
                        0.0, self.search_swept_deg - SEARCH_SECTOR_DEG * fracao)
                self.search_turn_until = 0.0
            self.search_phase = "CONFIRM"
            self.search_confirm_hold_until = now + SEARCH_CONFIRM_WINDOW + 0.60
            self._reset_confirmacao()
            self.search_confirm_last_bearing = self._bearing_from_detection(target)
            self.search_candidate_bearing = self.search_confirm_last_bearing
            self._decide("FREANDO_P_CONFIRMAR",
                         f"candidato '{target.label}' durante {self.search_phase}", "TARGET")
            return

        # Roda a confirmacao TODO tique (mesmo sem deteccao), para que o
        # orcamento de quadros perdidos seja contabilizado la dentro.
        confirmado = self._search_target_confirmation(target, now)
        if target is not None and confirmado:
            self._set_target(target, now)
            self.state.servo_target_angle = SERVO_CENTER
            self._set_mode(NavMode.ALIGN)
            self.align_started = now
            self.align_last_pulse = 0.0
            self.align_until = now
            self.align_initial_done = False
            self.command(Direction.STOP, 0)
            self._decide(
                "ALVO_CONFIRMADO",
                f"{target.label} {target.confidence:.0%} | bearing={self.target_bearing_deg:+.1f} graus | frames={self.search_confirm_count}",
                "TARGET",
            )
            return

        if target:
            self.state.target_visible = True
            self.state.target_label = target.label
            self.state.target_confidence = target.confidence
            self.state.target_center_error = target.center_error
            self.state.target_area = target.area_ratio
            self._decide(
                "CONFIRMANDO_ALVO",
                f"{target.label} {target.confidence:.0%} | quadro {self.search_confirm_count}/{SEARCH_CONFIRM_FRAMES}"
                f" | falhas {self.search_confirm_misses}/{SEARCH_CONFIRM_MAX_MISSES}",
                "TARGET",
                throttle=0.25,
            )
        else:
            self.state.target_visible = False
            # [Fase 2.1] NAO zera mais a contagem aqui. Um unico quadro em que o
            # YOLO piscava obrigava o rover a recomecar do zero - era a causa
            # principal do "ve o alvo por um instante e continua girando".

        if self.search_phase == "CONFIRM":
            # [Fase 2.1] VISAO ATIVA durante a confirmacao (CLAUDE.md secao 36):
            # a camera aponta para o rumo do candidato em vez de ficar travada no
            # centro. Depois do STOP o chassi ainda desliza (nao ha freio), e com
            # a camera centrada o candidato saia do quadro antes de fechar os
            # quadros de confirmacao.
            self.command(Direction.STOP, 0)
            alvo_servo = SERVO_CENTER + max(
                -SERVO_TRACK_MAX_OFFSET,
                min(SERVO_TRACK_MAX_OFFSET, self.search_candidate_bearing),
            )
            self.state.servo_target_angle = int(round(alvo_servo))
            self._move_servo_slow(self.state.servo_target_angle, now)
            self.state.search_heading = "CONFIRMANDO CANDIDATO"
            # Timeout por TEMPO, nao por "alvo sumiu": se a deteccao ficar
            # piscando sem nunca fechar os quadros, esta fase travaria para
            # sempre.
            if now >= self.search_confirm_hold_until:
                self._reset_confirmacao()
                self.search_phase = "SCAN"
                self.sweep_stage = 0
                self.scan_query_sent = False
                # [Fase 2.1] Reinicia a varredura no INICIO do setor. Antes o
                # servo ficava onde estava (centro, vindo do giro) e a
                # "revarredura" cobria so a metade direita: um candidato visto a
                # esquerda nunca era reexaminado e o rover girava para longe
                # dele.
                self.state.servo_target_angle = self.scan_angles[0]
                self._decide(
                    "CANDIDATO_NAO_CONFIRMADO",
                    f"revarrendo o setor (rumo visto {self.search_candidate_bearing:+.0f} graus)",
                    throttle=0.7,
                )
            return

        if self.search_phase == "SCAN":
            # [Fase 2 - inc.2] Varredura VISUAL contínua. O servo desliza sem
            # parar e o YOLO analisa cada quadro; NÃO há consulta ao sonar aqui.
            # Quem acha o alvo é a câmera - o ultrassônico responde "tem algo
            # ali?", não "o que é" (CLAUDE.md §24). O sonar só é consultado
            # antes de avançar. Isso derruba o setor de ~9 s para ~3.4 s.
            self.command(Direction.STOP, 0)
            self._move_servo_slow(self.state.servo_target_angle, now)

            if self.state.servo_angle == self.state.servo_target_angle:
                if self.sweep_stage == 0:
                    self.state.servo_target_angle = self.scan_angles[-1]
                    self.sweep_stage = 1
                elif self.sweep_stage == 1:
                    # setor inteiro já foi visto na perna anterior; volta ao
                    # centro para medir a frente antes de decidir.
                    self.state.servo_target_angle = SERVO_CENTER
                    self.sweep_stage = 2
                    self.sweep_center_since = now
                    self.sweep_probe_until = now + 1.20
                else:
                    # no centro: espera UMA leitura fresca do sonar (a consulta
                    # de fundo só dispara com o servo parado no centro) antes de
                    # decidir. Sem isso decidiríamos com um valor de 4 s atrás.
                    fresh = (self._ultra_valid(now)
                             and self.state.ultrasonic_timestamp >= self.sweep_center_since)
                    if fresh or now >= self.sweep_probe_until:
                        self.sweep_stage = 0
                        self._finish_scan_cycle(now)

        elif self.search_phase == "TURN":
            self.state.servo_target_angle = SERVO_CENTER
            self._move_servo_slow(SERVO_CENTER, now)
            if now < self.search_turn_until:
                self.command(self.search_turn_direction, TURN_SPEED)
                self.state.search_heading = f"GIRO {self.search_turn_direction.name}"
                return
            self.connection.send_command(Direction.STOP, 0, force=True)
            self.search_phase = "SCAN"
            self.sweep_stage = 0
            self.scan_distances = {}
            self.scan_index = 0
            self.scan_direction = 1
            self.scan_query_sent = False
            self.state.servo_target_angle = SERVO_SCAN_ANGLES[0]
            self._decide("NOVO_SETOR", f"iniciando busca após {self.search_turn_direction.name}", throttle=0.7)

        elif self.search_phase == "HOP":
            self.state.servo_target_angle = SERVO_CENTER
            self._move_servo_slow(SERVO_CENTER, now)
            if not self._servo_settled(now):
                self.command(Direction.STOP, 0)
                return
            if now < self.search_hop_until:
                if self._front_blocked(now):
                    self.connection.send_command(Direction.STOP, 0, force=True)
                    self.search_phase = "SCAN"
                    self.scan_distances = {}
                    self.scan_index = 0
                    self.scan_direction = 1
                    self._decide("HOP_ABORTADO", "HC-SR04 bloqueou avanço", "OBSTACLE")
                    return
                self.command(Direction.FORWARD, SEARCH_FORWARD_HOP_SPEED)
                self.state.search_heading = "AVANCO EXPLORATORIO"
                return
            self.connection.send_command(Direction.STOP, 0, force=True)
            self.search_phase = "SCAN"
            self.sweep_stage = 0
            self.scan_distances = {}
            self.scan_index = 0
            self.scan_direction = 1
            self.scan_query_sent = False
            self.state.servo_target_angle = SERVO_SCAN_ANGLES[0]
            self._decide("NOVO_PONTO", "nova posição alcançada; retomando busca", throttle=0.7)

        best = max(self.scan_distances.values(), default=0.0)
        swept = min(self.search_swept_deg, SEARCH_FULL_CIRCLE_DEG)
        # [Fase 2] O progresso agora é a cobertura angular da volta, não quantos
        # ângulos de servo foram medidos - é o número que interessa na demo.
        self.state.exploration_percent = min(100.0, 100.0 * swept / SEARCH_FULL_CIRCLE_DEG)
        self.state.search_heading = f"{self.search_phase} | volta {swept:.0f}/360 | servo {self.state.servo_angle}"
        self._decide(
            "BUSCA_360",
            f"fase={self.search_phase} | volta={swept:.0f}/360deg | servo={self.state.servo_angle} | melhor_US={best:.0f}cm",
            throttle=0.5,
        )

    def _align(self, now):
        self.state.servo_target_angle = SERVO_CENTER
        self._move_servo_slow(SERVO_CENTER, now)

        if not self._servo_settled(now):
            self.command(Direction.STOP, 0)
            self._decide("CENTRALIZANDO_CAMERA", f"servo {self.state.servo_angle} -> {SERVO_CENTER}")
            return

        target = self._current_target_detection()
        if target:
            self._set_target(target, now)
            error = target.center_norm
            self.align_initial_done = True
        else:
            error = max(-1.0, min(1.0, self.target_bearing_deg / (CAMERA_HFOV_DEG / 2.0)))

        self.state.alignment_error_deg = error * CAMERA_HFOV_DEG / 2.0
        self.state.alignment_progress = max(0.0, min(100.0, 100.0 * (1.0 - abs(error))))

        if abs(error) <= ALIGN_DEADBAND:
            self.command(Direction.STOP, 0)
            self._set_mode(NavMode.FOLLOW)
            self._decide("ALINHADO", f"erro={self.state.alignment_error_deg:+.1f} graus", "TARGET")
            return

        if now - self.align_started > ALIGN_MAX_TIME:
            self.command(Direction.STOP, 0)
            self._set_mode(NavMode.SEARCH)
            self._reset_search()
            self._decide("REALINHAMENTO_FALHOU", "Sem confirmação visual suficiente", "WARN")
            return

        if target is None and not self.align_initial_done:
            direction = Direction.LEFT if self.target_bearing_deg < 0 else Direction.RIGHT
            duration = min(0.85, max(0.18, abs(self.target_bearing_deg) / TURN_DEG_PER_SEC))
            self.command(direction, ALIGN_SPEED)
            self.align_initial_done = True
            self.align_until = now + duration
            self._decide("ALINHANDO_MEMORIA", f"giro inicial {direction.name} por {duration:.2f}s")
            return

        if target is None and self.align_initial_done and now >= self.align_until:
            self.connection.send_command(Direction.STOP, 0, force=True)
            self._set_mode(NavMode.SEARCH)
            self._reset_search()
            self._decide("REBUSCA", "Alvo não reencontrado após alinhamento inicial")
            return

        if now < self.align_until:
            self.command(Direction.LEFT if error < 0 else Direction.RIGHT, ALIGN_SPEED)
            return

        if now >= self.align_last_pulse:
            direction = Direction.LEFT if error < 0 else Direction.RIGHT
            pulse = ALIGN_PULSE_TIME if abs(error) < ALIGN_HARD_DEADBAND else ALIGN_PULSE_TIME * 1.25
            self.command(direction, ALIGN_SPEED)
            self.align_last_pulse = now + pulse
            self._decide("ALINHANDO", f"{direction.name} | erro={self.state.alignment_error_deg:+.1f} graus")
        else:
            self.command(Direction.STOP, 0)

    def _follow(self, now):
        target, coasted = self._acquire_target(now)
        if target is not None and not coasted:
            self._set_target(target, now)
            self.target_lost_since = None
        elif target is not None and coasted:
            # Seguindo "de memória" pela track: mantém o rumo, mas NÃO renova o
            # carimbo de detecção fresca, para o timeout continuar correndo.
            self.state.target_visible = True
            self.target_bearing_deg = self._bearing_from_detection(target)
            self._decide("ALVO_OCLUSO", "seguindo pela track enquanto o alvo some", "TARGET", throttle=0.6)
        elif self._target_fresh(now):
            target = self.primary_target
        else:
            if self.target_lost_since is None:
                self.target_lost_since = now
            self.command(Direction.STOP, 0)
            self.state.target_visible = False
            if now - self.target_lost_since >= TARGET_LOST_TIMEOUT:
                self.primary_target = None
                self.primary_target_time = 0.0
                self._clear_target_lock()
                self._set_mode(NavMode.SEARCH)
                self._reset_search()
                self._decide("ALVO_PERDIDO", "Iniciando nova busca com servo", "TARGET")
            else:
                self._decide("MEMORIA_ALVO", "Alvo temporariamente perdido")
            return

        # [Fase 2] VISÃO ATIVA: a câmera aponta para o alvo (realimentação
        # visual), e o chassi segue a câmera. Antes o servo era forçado ao
        # centro e o rover PARAVA até ele chegar lá - perdia o alvo e o tempo.
        # O desvio é limitado porque o ultrassônico divide o mesmo servo.
        bearing = self._bearing_from_detection(target)
        desired_servo = SERVO_CENTER + max(-SERVO_TRACK_MAX_OFFSET,
                                           min(SERVO_TRACK_MAX_OFFSET, bearing))
        if abs(desired_servo - self.state.servo_target_angle) >= SERVO_TRACK_MIN_STEP:
            self.state.servo_target_angle = int(round(desired_servo))

        if not self._ultra_valid(now):
            self.command(Direction.STOP, 0)
            self._decide("US_AGUARDANDO", "HC-SR04 sem leitura válida", "WARN")
            return

        if self._front_critical(now):
            self.command(Direction.STOP, 0)
            self._prepare_avoid(now, "distância crítica")
            return

        if self._front_blocked(now):
            self._prepare_avoid(now, "obstáculo frontal")
            return

        obstacles = self.vision.get_obstacles()
        visual_block = False
        visual_det = None
        for det in obstacles:
            if det.bottom_ratio >= OBSTACLE_BOTTOM_Y_RATIO and abs(det.center_norm) <= OBSTACLE_CENTER_X_RATIO:
                visual_block = True
                visual_det = det
                break
            if det.area_ratio >= OBSTACLE_AREA_RATIO and abs(det.center_norm) <= 0.36:
                visual_block = True
                visual_det = det
                break
        if visual_block:
            self._prepare_avoid(now, f"obstáculo visual: {visual_det.label}",
                                label=visual_det.label,
                                confidence=visual_det.confidence,
                                area=visual_det.area_ratio)
            return
        self.state.obstacle = False

        # [Fase 2] O chassi decide pelo BEARING TOTAL (servo + erro na imagem),
        # não só pelo erro na imagem: com a câmera virada, o alvo pode estar
        # centralizado no quadro e ainda assim bem lateral em relação às rodas.
        if abs(bearing) > ALIGN_BEARING_DEG:
            self._set_mode(NavMode.ALIGN)
            self.align_started = now
            self.align_last_pulse = 0.0
            self.align_until = now
            self.align_initial_done = True
            self.target_bearing_deg = bearing
            self._decide("REALINHAR", f"alvo a {bearing:+.0f} graus do eixo do chassi")
            return

        if target.area_ratio >= STOP_AREA_RATIO:
            self._target_arrived = True

        if self._target_arrived:
            if target.area_ratio >= TARGET_STOP_RESUME_RATIO:
                self.command(Direction.STOP, 0)
                self._decide("ALVO_ALCANCADO", f"{target.label} area={target.area_ratio:.1%}", "TARGET")
                return
            # [Revisao] Área caiu bem abaixo do limiar de parada: o alvo se
            # afastou de verdade (ou foi trocado pelo re-lock), não é ruído.
            # Libera a aproximação de novo em vez de continuar "chegado".
            self._target_arrived = False

        # [Revisao] A rampa de aproximacao agora vale SEMPRE.
        # Antes ela so era aplicada quando o argmax do DQN calhava de dar 1
        # (FORWARD). Mas so a acao 1 chega ao replay buffer (last_dqn_action era
        # 1 fixo), entao as outras 4 saidas da rede NUNCA sao treinadas e o
        # argmax entre elas e ruido. Na pratica, em boa parte dos ciclos o rover
        # nao desacelerava ao chegar perto e batia no corte seco do
        # STOP_AREA_RATIO. A rampa e classica e deterministica (CLAUDE.md 34/45);
        # o DQN continua observando e treinando, mas nao dirige - que e
        # exatamente o que a interface ja declara.
        p = min(1.0, max(0.0, (target.area_ratio - MIN_AREA_RATIO) / max(0.001, STOP_AREA_RATIO - MIN_AREA_RATIO)))
        speed = int(FOLLOW_SPEED - p * (FOLLOW_SPEED - SEARCH_SPEED))
        speed = max(SEARCH_SPEED, min(FOLLOW_SPEED, speed))

        dqn_action = 1
        if self.dqn is not None:
            try:
                dqn_action = self.dqn.select_action(self._build_dqn_state(), greedy=True)
            except Exception as exc:
                self.log.add_throttled("dqn_speed", "DQN (acao) falhou: %s" % exc)

        # [Fase 2] Correção fina DIRIGINDO: já em movimento não há atrito
        # estático a vencer, então a curva suave funciona (CLAUDE.md §34).
        if abs(bearing) <= FOLLOW_STEER_BEARING_DEG:
            drive = Direction.FORWARD
        elif bearing < 0:
            drive = Direction.FORWARD_LEFT
        else:
            drive = Direction.FORWARD_RIGHT
        self.command(drive, speed)

        # Treinar DQN
        if self.dqn is not None and self.last_dqn_state is not None:
            try:
                progress = 0.25 if target.area_ratio > self.state.target_area else -0.05
                reward = self._compute_reward(True, False, False, progress)
                next_state = self._build_dqn_state()
                done = False
                self.dqn.memory.push(self.last_dqn_state, self.last_dqn_action, reward, next_state, done)
                loss = self.dqn.learn()
                self.state.dqn_loss = loss
                self.state.dqn_epsilon = self.dqn.epsilon
                self.last_dqn_reward = reward
                self.dqn.reward_history.append(reward)
            except Exception as exc:
                self.log.add_throttled("dqn_learn", "DQN (treino) falhou: %s" % exc)

        self.last_dqn_state = self._build_dqn_state()
        # [Revisao] Era 1 fixo, entao o replay buffer so continha a acao FORWARD
        # e as outras 4 saidas da rede nunca recebiam gradiente nenhum.
        self.last_dqn_action = dqn_action

        self._decide(
            "SEGUIR_ALVO",
            f"{target.label} {target.confidence:.0%} | area={target.area_ratio:.1%} | US={self.state.ultrasonic_cm:.0f}cm | PWM={speed}",
            throttle=0.25,
        )

    def _prepare_avoid(self, now, reason="obstáculo", label="HC-SR04",
                       confidence=0.0, area=0.0):
        if self.mode == NavMode.AVOID and now - self.avoid_started < 0.25:
            return
        # [Revisao] `state.obstacle` NUNCA era posto em True em lugar nenhum do
        # programa: o aviso "OBSTACULO - DESVIO" no video, o campo "Obstaculo"
        # do painel e o rotulo/confianca do obstaculo eram telemetria morta. Na
        # feira isso importa - e o que explica ao publico por que ele desviou.
        self.state.obstacle = True
        self.state.obstacle_label = label
        self.state.obstacle_confidence = confidence
        self.state.obstacle_area = area
        self._set_mode(NavMode.AVOID)
        self.avoid_started = now
        self.avoid_cycles = 0
        self.avoid_phase = "BACK"
        self.avoid_until = now + AVOID_BACK_TIME
        self.avoid_left = 0.0
        self.avoid_right = 0.0
        self.avoid_query_sent = False
        self.state.servo_target_angle = SERVO_CENTER
        self.spatial_map.mark_heading_blocked(0.0)
        self.command(Direction.BACKWARD, AVOID_SPEED, allow_forward_safety_override=True)
        self._decide("DESVIO_PREPARANDO", reason, "OBSTACLE")

    def _avoid(self, now):
        if self.avoid_phase == "BACK":
            if now < self.avoid_until:
                self.command(Direction.BACKWARD, AVOID_SPEED)
                return
            self.connection.send_command(Direction.STOP, 0, force=True)
            self.state.servo_target_angle = 55
            self.avoid_phase = "WAIT_LEFT_SERVO"
            self.avoid_wait_until = now + AVOID_SERVO_WAIT
            return

        if self.avoid_phase == "WAIT_LEFT_SERVO":
            self._move_servo_slow(55, now)
            if self._servo_settled(now) and now >= self.avoid_wait_until:
                self.connection.query_ultrasonic(angle_hint=55)
                self.avoid_phase = "READ_LEFT"
                self.avoid_wait_until = now + AVOID_SERVO_WAIT
            return

        if self.avoid_phase == "READ_LEFT":
            if self.state.ultrasonic_ok and self.state.ultrasonic_angle == self.state.servo_angle and self.state.ultrasonic_timestamp >= now - AVOID_SERVO_WAIT:
                self.avoid_left = self.state.ultrasonic_cm
                self.state.servo_target_angle = 125
                self.avoid_phase = "WAIT_RIGHT_SERVO"
                self.avoid_wait_until = now + AVOID_SERVO_WAIT
            elif now >= self.avoid_wait_until:
                self.avoid_left = 0.0
                self.state.servo_target_angle = 125
                self.avoid_phase = "WAIT_RIGHT_SERVO"
                self.avoid_wait_until = now + AVOID_SERVO_WAIT
            return

        if self.avoid_phase == "WAIT_RIGHT_SERVO":
            self._move_servo_slow(125, now)
            if self._servo_settled(now) and now >= self.avoid_wait_until:
                self.connection.query_ultrasonic(angle_hint=125)
                self.avoid_phase = "READ_RIGHT"
                self.avoid_wait_until = now + AVOID_SERVO_WAIT
            return

        if self.avoid_phase == "READ_RIGHT":
            if self.state.ultrasonic_ok and self.state.ultrasonic_angle == self.state.servo_angle and self.state.ultrasonic_timestamp >= now - AVOID_SERVO_WAIT:
                self.avoid_right = self.state.ultrasonic_cm
                self._choose_avoid_side(now)
                return
            if now >= self.avoid_wait_until:
                self.avoid_right = 0.0
                self._choose_avoid_side(now)
            return

        if self.avoid_phase == "TURN":
            self.state.servo_target_angle = SERVO_CENTER
            self._move_servo_slow(SERVO_CENTER, now)
            if now < self.avoid_until:
                self.command(self.avoid_direction, TURN_SPEED)
                return
            self.connection.send_command(Direction.STOP, 0, force=True)
            self.avoid_phase = "VERIFY"
            self.avoid_wait_until = now + AVOID_VERIFY_TIME
            return

        if self.avoid_phase == "VERIFY":
            self.state.servo_target_angle = SERVO_CENTER
            self._move_servo_slow(SERVO_CENTER, now)
            if not self._servo_settled(now):
                return

            if not self.avoid_query_sent:
                self.connection.query_ultrasonic(angle_hint=SERVO_CENTER)
                self.avoid_query_sent = True
                self.avoid_wait_until = now + AVOID_VERIFY_TIME
                return

            if self.state.ultrasonic_angle == SERVO_CENTER and self.state.ultrasonic_timestamp >= now - AVOID_VERIFY_TIME - 0.05:
                if self._ultra_valid(now) and not self._front_blocked(now):
                    next_mode = NavMode.FOLLOW if self._target_fresh(now) else NavMode.SEARCH
                    self._set_mode(next_mode)
                    if next_mode == NavMode.SEARCH:
                        self._reset_search()
                    self.avoid_query_sent = False
                    self.state.obstacle = False
                    self._decide("DESVIO_OK", f"L={self.avoid_left:.0f}cm R={self.avoid_right:.0f}cm")
                    return

            if now >= self.avoid_wait_until:
                # BUG corrigido: aqui o desvio voltava para BACK sem NENHUM
                # limite. Se o sonar nunca confirmasse a frente livre (canto,
                # superfície que reflete mal, sensor ruim), o rover ficava
                # recuando e girando para sempre - e o progress checker não
                # pegava, porque ele ESTAVA se movendo. Agora escalona para a
                # recuperação, que alterna estratégia a cada tentativa.
                self.avoid_query_sent = False
                self.avoid_cycles += 1
                if self.avoid_cycles >= AVOID_MAX_CYCLES:
                    self._enter_stuck(
                        now,
                        "desvio não resolveu em %d tentativas; escalando para recuperação"
                        % self.avoid_cycles,
                    )
                    return
                self.avoid_phase = "BACK"
                self.avoid_until = now + AVOID_BACK_TIME
                self._decide(
                    "DESVIO_REPETIR",
                    "tentativa %d/%d - frente ainda bloqueada" % (self.avoid_cycles, AVOID_MAX_CYCLES),
                    "OBSTACLE",
                )

    def _choose_avoid_side(self, now):
        if self.avoid_left >= SAFE_DISTANCE_CM and self.avoid_left >= self.avoid_right:
            self.avoid_direction = Direction.LEFT
        elif self.avoid_right >= SAFE_DISTANCE_CM:
            self.avoid_direction = Direction.RIGHT
        elif self.avoid_left > self.avoid_right:
            self.avoid_direction = Direction.LEFT
        else:
            self.avoid_direction = Direction.RIGHT

        self.state.servo_target_angle = SERVO_CENTER
        self.avoid_phase = "TURN"
        self.avoid_until = now + AVOID_TURN_TIME
        self.command(self.avoid_direction, TURN_SPEED)
        self._decide(
            "DESVIO_GIRANDO",
            f"{self.avoid_direction.name} | L={self.avoid_left:.0f}cm R={self.avoid_right:.0f}cm",
            "OBSTACLE",
        )

    def command(self, direction, speed, allow_forward_safety_override=False):
        """CORREÇÃO v21: Registra last_motion_time e last_motion_command corretamente."""
        now = time.time()
        speed = int(max(0, min(MAX_SPEED, speed)))

        if direction == Direction.STOP:
            self.boost_until = 0.0
            self.last_motion_command = Direction.STOP
            self.connection.send_command(Direction.STOP, 0, force=True)
            return True

        forward = direction in (Direction.FORWARD, Direction.FORWARD_LEFT, Direction.FORWARD_RIGHT)
        if forward and not allow_forward_safety_override:
            # Freio real: obstáculo CONFIRMADO muito perto -> sempre bloqueia.
            if self._front_critical(now):
                self.connection.send_command(Direction.STOP, 0, force=True)
                self._decide("FREIO_US", f"{self.state.ultrasonic_cm:.0f}cm", "OBSTACLE")
                return False
            # [Fase 1.1] Sem telemetria do HC-SR04: só o AUTÔNOMO fica bloqueado
            # (fail-safe). No MANUAL o operador é a camada de segurança, então o
            # rover continua dirigível mesmo se o ultrassônico estiver mudo.
            if not self._ultra_valid(now) and self.mode != NavMode.MANUAL:
                self.connection.send_command(Direction.STOP, 0, force=True)
                self._decide("US_AGUARDANDO", "Sem leitura válida; avanço autônomo bloqueado", "WARN")
                return False

        fresh_start = (
            self.last_motion_command == Direction.STOP
            or direction != self.last_motion_command
            or now - self.last_motion_time > 0.65
        )
        if fresh_start:
            self.connection.send_command(direction, MOTOR_BOOST_SPEED, force=True)
            self.boost_until = now + MOTOR_BOOST_TIME
            self.boost_direction = direction
            self.last_motion_time = now
            self.last_motion_command = direction
        elif now < self.boost_until and self.boost_direction == direction:
            return True
        else:
            self.connection.send_command(direction, speed)
            self.last_motion_time = now
            self.last_motion_command = direction

        self.last_speed = speed
        return True

    def _target_signature(self, target):
        return f"{target.label}:{int(target.cx/28)}:{int(target.cy/28)}:{int(target.area_ratio*20)}"

    def _stuck_detection(self, now):
        """[Fase 2] Progress checker no espírito do Nav2.

        Antes isto só rodava no modo SEGUIR (`if self.mode != FOLLOW: return`),
        então encalhar durante a BUSCA ou o DESVIO - justamente onde o rover
        manobra perto de móveis - passava despercebido para sempre.

        O sinal universal é o fluxo óptico da câmera: comandando movimento, se
        a imagem não muda, o rover não está indo a lugar nenhum (vale inclusive
        para giro, que não desloca mas muda muito a imagem)."""
        if self.mode not in (NavMode.FOLLOW, NavMode.SEARCH, NavMode.AVOID):
            self._motion_since = 0.0
            return
        if now - self.last_stuck_time < STUCK_COOLDOWN_SECONDS:
            return

        if self.last_motion_command == Direction.STOP:
            self._motion_since = 0.0
            return
        if self._motion_since == 0.0:
            self._motion_since = now
            return
        if now - self._motion_since < STUCK_COMMAND_SECONDS:
            return

        if len(self._vo_flow) >= 12 and self.visual_odometry.valid:
            flow = sum(self._vo_flow) / len(self._vo_flow)
            if flow < STUCK_FLOW_THRESHOLD:
                self._enter_stuck(
                    now,
                    "sem progresso visual (%.2f px/quadro) comandando %s ha %.1fs"
                    % (flow, self.last_motion_command.name, now - self._motion_since),
                )
                return
            self._recovery_attempts = 0   # progresso saudável zera o escalonamento

        if self.mode == NavMode.FOLLOW:
            target = self._current_target_detection()
            if not target:
                return
            signature = self._target_signature(target)
            if signature != self.last_signature:
                self.last_signature = signature
                self.last_signature_time = now
                return
            if self.last_signature_time and now - self.last_signature_time >= STUCK_SIGNATURE_SECONDS:
                self._enter_stuck(now, "Alvo visual praticamente imóvel durante avanço")

    def _enter_stuck(self, now, reason):
        self.last_stuck_time = now
        self.stuck_started = now
        self._motion_since = 0.0
        self._recovery_attempts += 1
        self.state.stuck = True
        self._set_mode(NavMode.STUCK)
        self.command(Direction.BACKWARD, RECOVERY_SPEED)
        self._decide("TRAVADO", reason, "RECOVER")

    def _stuck_recovery(self, now):
        # [Fase 2] Recuperação ESCALONADA. CLAUDE.md §19: nunca repetir
        # indefinidamente a ação que falhou. A cada tentativa seguida o rover
        # recua mais, gira mais e troca o lado do giro - mesmo princípio das
        # "recovery behaviors" encadeadas do Nav2.
        n = min(max(self._recovery_attempts - 1, 0), 3)
        back_t = STUCK_BACK_TIME * (1.0 + 0.6 * n)
        turn_t = STUCK_TURN_TIME * (1.0 + 0.5 * n)
        turn_dir = Direction.RIGHT if (self._recovery_attempts % 2) else Direction.LEFT

        elapsed = now - self.stuck_started
        if elapsed < back_t:
            self.command(Direction.BACKWARD, RECOVERY_SPEED)
            return
        if elapsed < back_t + turn_t:
            self.command(turn_dir, TURN_SPEED)
            return
        if elapsed < back_t + turn_t + STUCK_FORWARD_TIME:
            if self._ultra_valid(now) and not self._front_critical(now):
                self.command(Direction.FORWARD, SEARCH_SPEED)
            else:
                self.connection.send_command(Direction.STOP, 0, force=True)
            return

        # [Revisao] Este STOP estava DEPOIS de um `return`, dentro do `if`:
        # codigo morto. O rover saia da recuperacao com o ultimo comando de
        # motor ainda valendo, em vez de parar antes de recomecar a busca.
        self.connection.send_command(Direction.STOP, 0, force=True)
        self.state.stuck = False
        self.last_signature = ""
        self._vo_flow.clear()
        self._motion_since = 0.0
        self._set_mode(NavMode.SEARCH)
        self._reset_search()
        self._decide(
            "RECUPERADO",
            "tentativa %d; nova busca iniciada" % self._recovery_attempts,
            "RECOVER",
        )

    def _bad_path(self):
        blocked = 0
        for det in self.vision.get_obstacles():
            if det.confidence >= VISUAL_OBSTACLE_MIN_CONF and det.bottom_ratio >= OBSTACLE_BOTTOM_Y_RATIO and abs(det.center_norm) <= 0.34:
                blocked = 1
                break
        self.path_history.append(blocked)
        if len(self.path_history) < BAD_PATH_WINDOW:
            return
        ratio = sum(self.path_history) / len(self.path_history)
        self.state.bad_path = ratio >= BAD_PATH_THRESHOLD
        if ratio < 0.30:
            self.state.path_quality = PathQuality.EXCELLENT
        elif ratio < 0.50:
            self.state.path_quality = PathQuality.GOOD
        elif ratio < 0.70:
            self.state.path_quality = PathQuality.WARNING
        else:
            self.state.path_quality = PathQuality.DANGER

        if self.state.bad_path and self.mode not in (NavMode.AVOID, NavMode.STUCK, NavMode.EMERGENCY_STOP):
            now = time.time()
            if now - self.last_bad_path_time >= BAD_PATH_COOLDOWN_SECONDS:
                self.last_bad_path_time = now
                self.connection.send_command(Direction.STOP, 0, force=True)
                self._set_mode(NavMode.SEARCH)
                self._reset_search()
                self._decide("CAMINHO_RUIM", f"{ratio:.0%} leituras visuais frontais bloqueadas", "WARN")
                self.path_history.clear()

    def _update_target_state(self, now):
        target = self._current_target_detection()
        if target:
            self._set_target(target, now)
        elif not self._target_fresh(now):
            self.state.target_visible = False

    def _update_ultra_timeout(self, now):
        if self.state.ultrasonic_ok and now - self.state.ultrasonic_timestamp > ULTRASONIC_STALE_SECONDS:
            self.state.ultrasonic_ok = False
            self.state.ultrasonic_blocked = False

    def _update_telemetry(self):
        self.state.total_detections = len(self.vision.get_all_detections())
        self.state.tracked_objects = len(self.tracker.tracks)

    def set_strategy(self, strategy):
        self.strategy = str(strategy)
        if self.strategy == "BUSCA_ESQUERDA":
            self.search_direction = Direction.LEFT
        elif self.strategy == "BUSCA_DIREITA":
            self.search_direction = Direction.RIGHT
        self._decide("ESTRATEGIA", self.strategy)

    def set_targets(self, targets):
        self.vision.set_targets(targets)
        self.primary_target = None
        self.primary_target_time = 0.0
        self._clear_target_lock()
        self._decide("ALVOS", ", ".join(sorted(self.vision.get_targets())), "TARGET")

    def set_mode(self, mode):
        if mode == NavMode.EMERGENCY_STOP:
            self.emergency_stop()
            return
        self.connection.send_command(Direction.STOP, 0, force=True)
        self._set_mode(mode)
        self.enabled = mode not in (NavMode.IDLE, NavMode.EMERGENCY_STOP)
        self.state.stuck = False
        self.target_lost_since = None
        self.boost_until = 0.0

        if mode == NavMode.SEARCH:
            self._reset_search()
        elif mode == NavMode.FOLLOW:
            self.state.servo_target_angle = SERVO_CENTER
            self.align_initial_done = True
        self._decide("MODO", mode.value)

    def set_manual(self, direction, speed):
        if self.mode == NavMode.EMERGENCY_STOP:
            self.log.add("Manual ignorado: emergência ativa.", "WARN")
            return
        self._set_mode(NavMode.MANUAL)
        self.enabled = True
        self.manual_command = direction
        self._manual_speed = int(speed)
        self.manual_last_input = time.time()
        self.command(direction, speed)

    def set_servo_manual(self, angle):
        if self.mode not in (NavMode.IDLE, NavMode.MANUAL):
            self._decide("SERVO_BLOQUEADO", "Modo automático controla o servo", "WARN")
            return
        angle = int(max(SERVO_MIN, min(SERVO_MAX, angle)))
        self.state.servo_target_angle = angle
        self._move_servo_slow(angle, time.time())

    def stop(self):
        self.connection.send_command(Direction.STOP, 0, force=True)
        # [Fase 1 - R1] N:100 = STOP explicito (limpa modo/LEDs no firmware).
        # Enviado APENAS em paradas deliberadas do operador, nunca como heartbeat.
        self.connection.send_raw('{"N":100}')
        self.manual_command = Direction.STOP
        self.boost_until = 0.0
        self._set_mode(NavMode.IDLE)
        self.enabled = False
        self.state.stuck = False
        self.state.obstacle = False
        self._decide("PARADO", "Comando de parada")

    def emergency_stop(self):
        self.connection.send_command(Direction.STOP, 0, force=True)
        # [Fase 1 - R1] STOP explicito de emergencia.
        self.connection.send_raw('{"N":100}')
        self.manual_command = Direction.STOP
        self.boost_until = 0.0
        self._set_mode(NavMode.EMERGENCY_STOP)
        self.enabled = False
        self.state.stuck = False
        self.state.obstacle = False
        self._decide("EMERGENCIA", "Parada de emergência ativada", "ERROR")

    def tick(self):
        now = time.time()

        # [Fase 2] Amostra o fluxo óptico: é o sinal de progresso do rover.
        self._vo_flow.append(abs(self.state.vo_dx) + abs(self.state.vo_dy))

        self._update_spatial_memory(now)
        self._update_ultra_timeout(now)
        self._update_target_state(now)

        # Atualizar tracking
        all_dets = self.vision.get_all_detections()
        self.tracker.update(all_dets)

        # [Fase 1] Mantem a estimativa da posicao do servo convergindo para o
        # alvo em todos os modos (menos emergencia). Nao envia comando se o alvo
        # nao mudou; so acompanha a rampa que o firmware executa.
        if self.mode != NavMode.EMERGENCY_STOP:
            self._move_servo_slow(self.state.servo_target_angle, now)

        if self.mode == NavMode.EMERGENCY_STOP:
            return

        if self.mode == NavMode.IDLE:
            self._update_telemetry()
            return

        if self.mode == NavMode.MANUAL:
            if now - self.manual_last_input > MANUAL_WATCHDOG_SECONDS:
                self.connection.send_command(Direction.STOP, 0, force=True)
                self.manual_command = Direction.STOP
                self.last_motion_command = Direction.STOP
                self._decide("MANUAL_TIMEOUT", "Sem comando recente; rover parado", "WARN")
            elif self.manual_command != Direction.STOP:
                # [Fase 1 - R4] Reenvia o comando manual corrente a cada tick
                # enquanto a tecla estiver ativa, para alimentar o watchdog do
                # firmware (600 ms). Sem isso, com key-repeat lento do SO o rover
                # pararia sozinho no meio de uma tecla pressionada.
                self.command(self.manual_command, self._manual_speed)
            self._update_telemetry()
            return

        if not self._camera_valid(now):
            self.connection.send_command(Direction.STOP, 0, force=True)
            self._decide("CAMERA_INDISPONIVEL", "Parada de segurança: stream ESP32 ausente", "ERROR")
            return

        if self.mode == NavMode.SEARCH:
            self._search(now)
        elif self.mode == NavMode.ALIGN:
            self._align(now)
        elif self.mode == NavMode.FOLLOW:
            self._follow(now)
        elif self.mode == NavMode.AVOID:
            self._avoid(now)
        elif self.mode == NavMode.STUCK:
            self._stuck_recovery(now)

        self._stuck_detection(now)
        self._bad_path()
        self._update_telemetry()

        # Salvar DQN periodicamente
        # BUG corrigido: a condição era `update_count % 50 == 0`, mas o
        # update_count só muda quando o DQN treina. Parado num múltiplo de 50,
        # a condição ficava VERDADEIRA em todo tick e o modelo era gravado em
        # disco ~10x por segundo, travando o laço de navegação.
        if (self.dqn is not None
                and self.dqn.update_count > 0
                and self.dqn.update_count - self._dqn_last_saved >= DQN_SAVE_INTERVAL):
            self._dqn_last_saved = self.dqn.update_count
            self.dqn.save()

# =====================================================================
# GUI
# =====================================================================

class GEARAIRoverV21:
    def __init__(self, root):
        self.root = root
        self.root.title("GEAR AI Rover v21 ULTIMATE - DQN + VO + Kalman + A*")
        self.root.geometry("1600x1000")
        self.root.minsize(1300, 800)
        self.running = True

        self.state = RobotState()
        self.logbook = EventLog()
        self.connection = RobotConnection(self.state, self.logbook)
        self.camera = CameraWorker(self.state, self.logbook)
        self.vision = VisionWorker(self.state, self.logbook)
        self.navigator = SmartNavigator(self.state, self.connection, self.vision, self.logbook)

        self._init_variables()
        self._build_ui()
        self._bind_keys()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(80, self._ui_tick)
        self.root.after(int(CONTROL_INTERVAL * 1000), self._nav_tick)

        self.logbook.add("=" * 72, "INIT")
        self.logbook.add("GEAR AI Rover v21 ULTIMATE", "INIT")
        self.logbook.add("DQN + Visual Odometry + Kalman + A* + Tracking", "INIT")
        self.logbook.add(f"ESP32 TCP: {ESP32_IP}:{ESP32_TCP_PORT}", "INIT")
        self.logbook.add(f"Camera: {STREAM_URL}", "INIT")
        self.logbook.add("=" * 72, "INIT")

    def _init_variables(self):
        self.status_var = tk.StringVar(value="[X] DESCONECTADO")
        self.footer_var = tk.StringVar(value="Inicializando...")
        self.camera_info_var = tk.StringVar(value="Camera: --")
        self.target_info_var = tk.StringVar(value="Alvo: --")
        self.obstacle_info_var = tk.StringVar(value="Obstaculo: --")
        self.ultrasonic_info_var = tk.StringVar(value="US: --")
        self.servo_var = tk.StringVar(value="Servo: 90 graus")
        self.alignment_var = tk.StringVar(value="Alinhamento: --")
        self.ai_decision_var = tk.StringVar(value="AGUARDANDO")
        self.ai_reason_var = tk.StringVar(value="Sistema inicializando")
        self.telemetry_var = tk.StringVar(value="")
        self.speed_var = tk.IntVar(value=DEFAULT_MANUAL_SPEED)
        self.speed_label_var = tk.StringVar(value=str(DEFAULT_MANUAL_SPEED))
        self.confidence_var = tk.DoubleVar(value=DEFAULT_CONFIDENCE)
        self.confidence_text_var = tk.StringVar(value=f"{DEFAULT_CONFIDENCE:.2f}")
        self.targets_var = tk.StringVar(value=", ".join(DEFAULT_TARGETS))
        self.path_quality_var = tk.StringVar(value="Qualidade: --")
        self.strategy_var = tk.StringVar(value="ADAPTATIVA")
        self.mode_var = tk.StringVar(value="PARADO")
        self.dqn_var = tk.StringVar(value="DQN: --")
        self.vo_var = tk.StringVar(value="VO: --")
        self.kalman_var = tk.StringVar(value="Kalman: --")
        self.curiosity_var = tk.StringVar(value="Curiosidade: --")
        self._log_seq_rendered = 0

    def _build_ui(self):
        main = ttk.Frame(self.root, padding=8)
        main.pack(fill="both", expand=True)

        header = ttk.Frame(main)
        header.pack(fill="x", pady=(0, 8))
        ttk.Label(header, text="GEAR AI ROVER v21", font=("TkDefaultFont", 20, "bold")).pack(side="left")
        ttk.Label(header, text="ULTIMATE - DQN + VO + KALMAN + A*", font=("TkDefaultFont", 12, "bold")).pack(side="left", padx=15)
        ttk.Label(header, textvariable=self.status_var, font=("TkDefaultFont", 12, "bold")).pack(side="right")

        pane = ttk.PanedWindow(main, orient=tk.HORIZONTAL)
        pane.pack(fill="both", expand=True)
        left = ttk.Frame(pane)
        right = ttk.Frame(pane)
        pane.add(left, weight=4)
        pane.add(right, weight=2)

        self._build_camera_frame(left)
        self._build_control_panel(right)

        footer = ttk.Frame(main)
        footer.pack(fill="x", pady=(8, 0))
        ttk.Label(footer, textvariable=self.footer_var).pack(side="left")
        ttk.Button(footer, text="FECHAR", command=self.close).pack(side="right")

    def _build_camera_frame(self, parent):
        frame = ttk.LabelFrame(parent, text="Camera ESP32-S3-CAM + YOLO + Overlay v21")
        frame.pack(fill="both", expand=True, padx=(0, 5))
        self.camera_label = ttk.Label(frame, text="Aguardando camera ESP32...")
        self.camera_label.pack(fill="both", expand=True, padx=5, pady=5)
        # [Fase 1.1] Clicar no vídeo tira o foco de qualquer campo de texto, para
        # que as setas / W-A-S-D voltem a controlar o rover (ver R2).
        self.camera_label.bind("<Button-1>", lambda _e: self.root.focus_set())

        info = ttk.Frame(frame)
        info.pack(fill="x", padx=5, pady=5)
        ttk.Label(info, text="Alvo:").pack(side="left", padx=5)
        ttk.Label(info, textvariable=self.target_info_var, font=("TkDefaultFont", 10, "bold")).pack(side="left")
        ttk.Label(info, text="Obstaculo:").pack(side="left", padx=18)
        ttk.Label(info, textvariable=self.obstacle_info_var, font=("TkDefaultFont", 10, "bold")).pack(side="left")
        ttk.Label(info, textvariable=self.ultrasonic_info_var, font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=18)
        ttk.Label(info, textvariable=self.servo_var, font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=18)
        ttk.Label(info, textvariable=self.camera_info_var).pack(side="right", padx=5)

    def _build_control_panel(self, parent):
        notebook = ttk.Notebook(parent)
        notebook.pack(fill="both", expand=True)

        control = ttk.Frame(notebook, padding=10)
        notebook.add(control, text="Controle")
        ttk.Label(control, text="Controle Manual", font=("TkDefaultFont", 12, "bold")).pack(pady=(0, 10))

        direction_frame = ttk.Frame(control)
        direction_frame.pack()
        buttons = [
            ("Q", Direction.FORWARD_LEFT), ("W", Direction.FORWARD), ("E", Direction.FORWARD_RIGHT),
            ("A", Direction.LEFT), ("S", Direction.STOP), ("D", Direction.RIGHT),
            ("Z", Direction.BACKWARD_LEFT), ("X", Direction.BACKWARD), ("C", Direction.BACKWARD_RIGHT),
        ]
        for i, (text, direction) in enumerate(buttons):
            ttk.Button(direction_frame, text=text, width=6, command=lambda d=direction: self.manual(d)).grid(
                row=i // 3, column=i % 3, padx=3, pady=3
            )

        ttk.Label(control, text="Velocidade Manual").pack(pady=(15, 2))
        self.speed_scale = ttk.Scale(control, from_=MIN_SPEED, to=MAX_SPEED, orient="horizontal", command=self._speed_changed)
        self.speed_scale.set(DEFAULT_MANUAL_SPEED)
        self.speed_scale.pack(fill="x", padx=10)
        ttk.Label(control, textvariable=self.speed_label_var).pack()

        servo_frame = ttk.LabelFrame(control, text="Camera / Servo")
        servo_frame.pack(fill="x", pady=(12, 6), padx=5)
        self.servo_scale = ttk.Scale(servo_frame, from_=SERVO_MIN, to=SERVO_MAX, orient="horizontal", command=self._servo_changed)
        self.servo_scale.set(SERVO_CENTER)
        self.servo_scale.pack(fill="x", padx=8, pady=4)
        ttk.Label(servo_frame, textvariable=self.servo_var).pack()
        ttk.Label(servo_frame, text="Lento de propósito. No automático, as rodas só avançam após alinhar com a câmera.", wraplength=340, justify="center").pack(pady=3)

        ttk.Label(control, text="W/A/S/D ou Setas | Q/E/Z/C = diagonais | Espaço = emergência", justify="center").pack(pady=10)
        ttk.Button(control, text="PARAR", command=self.stop).pack(fill="x", pady=3)
        ttk.Button(control, text="PARADA DE EMERGENCIA", command=self.emergency_stop).pack(fill="x", pady=10)

        ai = ttk.Frame(notebook, padding=10)
        notebook.add(ai, text="IA / Alvos")
        ttk.Label(ai, text="Modo", font=("TkDefaultFont", 10, "bold")).pack()
        ttk.Label(ai, textvariable=self.mode_var, font=("TkDefaultFont", 12, "bold")).pack()
        ttk.Label(ai, text="Decisao da IA", font=("TkDefaultFont", 10, "bold")).pack(pady=(8, 0))
        ttk.Label(ai, textvariable=self.ai_decision_var, font=("TkDefaultFont", 12, "bold")).pack()
        ttk.Label(ai, text="Motivo", font=("TkDefaultFont", 9, "bold")).pack(pady=(8, 0))
        ttk.Label(ai, textvariable=self.ai_reason_var, wraplength=360, justify="left").pack()
        ttk.Label(ai, textvariable=self.alignment_var, font=("TkDefaultFont", 10, "bold")).pack(pady=6)

        # Novos displays v21
        ttk.Separator(ai).pack(fill="x", pady=5)
        ttk.Label(ai, textvariable=self.dqn_var, font=("TkDefaultFont", 9)).pack()
        ttk.Label(ai, textvariable=self.vo_var, font=("TkDefaultFont", 9)).pack()
        ttk.Label(ai, textvariable=self.kalman_var, font=("TkDefaultFont", 9)).pack()
        ttk.Label(ai, textvariable=self.curiosity_var, font=("TkDefaultFont", 9)).pack()

        ttk.Separator(ai).pack(fill="x", pady=7)

        ttk.Label(ai, text="Objetos-alvo (virgula)", font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        ttk.Entry(ai, textvariable=self.targets_var).pack(fill="x", pady=5)
        ttk.Button(ai, text="Aplicar Alvos", command=self._apply_targets).pack(fill="x")

        ttk.Label(ai, text="Confianca YOLO", font=("TkDefaultFont", 10, "bold")).pack(pady=(10, 0))
        scale = ttk.Scale(ai, from_=0.10, to=0.90, variable=self.confidence_var, orient="horizontal", command=self._confidence_changed)
        scale.pack(fill="x")
        ttk.Label(ai, textvariable=self.confidence_text_var).pack()

        ttk.Label(ai, text="Estrategia", font=("TkDefaultFont", 10, "bold")).pack(pady=(10, 0))
        combo = ttk.Combobox(ai, textvariable=self.strategy_var, values=["ADAPTATIVA", "BUSCA_ESQUERDA", "BUSCA_DIREITA"], state="readonly")
        combo.pack(fill="x", pady=4)
        combo.bind("<<ComboboxSelected>>", self._strategy_changed)

        ttk.Separator(ai).pack(fill="x", pady=10)
        ttk.Button(ai, text="INICIAR BUSCA", command=lambda: self.set_mode(NavMode.SEARCH)).pack(fill="x", pady=3)
        ttk.Button(ai, text="SEGUIR ALVO", command=lambda: self.set_mode(NavMode.FOLLOW)).pack(fill="x", pady=3)
        ttk.Button(ai, text="PARAR IA", command=self.stop).pack(fill="x", pady=3)
        ttk.Button(ai, text="RESETAR MAPA", command=self.reset_map).pack(fill="x", pady=3)
        ttk.Separator(ai).pack(fill="x", pady=8)
        ttk.Label(ai, text="IA de navegação v21", font=("TkDefaultFont", 10, "bold")).pack()
        ttk.Label(ai, text=(
            "ATIVO: visao (YOLO) + tracking com coasting + busca 360 por setores"
            " + visao ativa (camera segue o alvo) + progress checker/recuperacao. "
            "EXPERIMENTAL, nao dirige o rover: DQN, Kalman, A*, curiosidade."
        ), wraplength=360, justify="center").pack(pady=3)
        ttk.Button(ai, text="SALVAR APRENDIZADO", command=self.save_ai).pack(fill="x", pady=3)

        telemetry = ttk.Frame(notebook, padding=10)
        notebook.add(telemetry, text="Telemetria")
        ttk.Label(telemetry, textvariable=self.path_quality_var, font=("TkDefaultFont", 11, "bold")).pack(pady=5)
        ttk.Label(telemetry, textvariable=self.telemetry_var, justify="left", font=("Courier", 9)).pack(anchor="w")
        ttk.Separator(telemetry).pack(fill="x", pady=10)
        ttk.Label(telemetry, text="Logica de navegacao v21", font=("TkDefaultFont", 10, "bold")).pack(anchor="w")
        ttk.Label(telemetry, text=(
            "O QUE REALMENTE DIRIGE O ROVER:\n"
            "  Visao YOLO: confirmacao em 3 quadros + filtro de area minima\n"
            "  Tracking IoU: segura o alvo durante oclusao curta (coasting)\n"
            "  Busca 360 por setores: acha alvo atras do rover\n"
            "  Visao ativa: camera aponta o alvo, chassi segue a camera\n"
            "  Progress checker (fluxo optico): travamento + recuperacao\n"
            "  Ultrassonico: freio de seguranca e liberacao do avanco\n"
            "\n"
            "EXPERIMENTAL, sem efeito na navegacao:\n"
            "  DQN (so modula velocidade), Kalman, A*, curiosidade (ICM)\n"
            "  Odometria = PWM x constante, sem encoders: pose aproximada"
        ), justify="left", wraplength=370).pack(anchor="w", pady=5)

        logs = ttk.Frame(notebook, padding=10)
        notebook.add(logs, text="Logs")
        self.log_text = tk.Text(logs, height=25, width=50, state="disabled", wrap="word", font=("Courier", 9))
        self.log_text.pack(fill="both", expand=True)

    def _speed_changed(self, value):
        try:
            speed = int(float(value))
        except Exception:
            speed = DEFAULT_MANUAL_SPEED
        self.speed_var.set(speed)
        self.speed_label_var.set(str(speed))

    def _confidence_changed(self, value):
        try:
            conf = float(value)
        except Exception:
            conf = DEFAULT_CONFIDENCE
        self.confidence_var.set(conf)
        self.confidence_text_var.set(f"{conf:.2f}")
        self.vision.set_confidence(conf)

    def _servo_changed(self, value):
        try:
            angle = int(float(value))
        except Exception:
            angle = SERVO_CENTER
        self.servo_var.set(f"Servo: {angle} graus")
        self.navigator.set_servo_manual(angle)

    def _strategy_changed(self, _event=None):
        self.navigator.set_strategy(self.strategy_var.get())

    def _apply_targets(self):
        targets = [x.strip().lower() for x in self.targets_var.get().split(",") if x.strip()]
        if not targets:
            messagebox.showwarning("Alvos", "Digite pelo menos um objeto.")
            return
        self.navigator.set_targets(targets)
        # [Fase 1.1] devolve o foco à janela para as teclas de direção voltarem.
        self.root.focus_set()

    def manual(self, direction):
        if direction == Direction.STOP:
            self.stop()
        else:
            self.navigator.set_manual(direction, self.speed_var.get())

    def set_mode(self, mode):
        if not self.vision.get_targets():
            messagebox.showwarning("IA", "Defina pelo menos um alvo.")
            return
        self.navigator.set_mode(mode)

    def reset_map(self):
        if messagebox.askyesno("Mapa", "Apagar o mapa persistente e começar a exploração do zero?"):
            self.navigator.spatial_map.reset()
            self.navigator.visual_odometry.reset()
            self.navigator.kalman.reset()
            self.navigator.tracker.reset()
            self.logbook.add("Mapa espacial + VO + Kalman + Tracker resetados.", "NAV")

    def save_ai(self):
        try:
            self.navigator.spatial_map.save(force=True)
            if self.navigator.dqn:
                self.navigator.dqn.save()
            self.logbook.add("Mapa + DQN salvos.", "NAV")
        except Exception as exc:
            self.logbook.add(f"Falha ao salvar IA: {exc}", "ERROR")

    def stop(self):
        self.navigator.stop()

    def emergency_stop(self):
        self.navigator.emergency_stop()
        self.logbook.add("PARADA DE EMERGENCIA", "ERROR")

    def _bind_keys(self):
        self.root.bind("<KeyPress>", self._key_down)
        self.root.bind("<KeyRelease>", self._key_up)

    def _typing_in_text_field(self):
        """[Fase 1 - R2] True quando o foco esta num campo de texto (ex.: o campo
        de objetos-alvo). Evita que digitar 'dog', 'cup', 'sandwich' etc. seja
        interpretado como comando de movimento (w/a/s/d/...) ou emergencia."""
        try:
            widget = self.root.focus_get()
        except Exception:
            return False
        return isinstance(widget, (tk.Entry, ttk.Entry, tk.Text, ttk.Combobox))

    def _key_down(self, event):
        if self._typing_in_text_field():
            return
        key = event.keysym.lower()
        mapping = {
            "w": Direction.FORWARD, "up": Direction.FORWARD,
            "s": Direction.BACKWARD, "down": Direction.BACKWARD,
            "a": Direction.LEFT, "left": Direction.LEFT,
            "d": Direction.RIGHT, "right": Direction.RIGHT,
            "q": Direction.FORWARD_LEFT, "e": Direction.FORWARD_RIGHT,
            "z": Direction.BACKWARD_LEFT, "c": Direction.BACKWARD_RIGHT,
        }
        if key == "space":
            self.emergency_stop()
        elif key in mapping:
            self.manual(mapping[key])

    def _key_up(self, _event):
        # [Fase 1 - R2] Nao renovar o watchdog manual enquanto se digita num campo.
        if self._typing_in_text_field():
            return
        # CORREÇÃO v21: Atualiza timestamp corretamente (não subtrai 0.20)
        self.navigator.manual_last_input = time.time()

    def _draw_overlay(self, frame):
        h, w = frame.shape[:2]
        center = w // 2

        cv2.line(frame, (center, 0), (center, h), (255, 255, 255), 1)
        cv2.rectangle(
            frame,
            (max(0, center - CENTER_DEADZONE), 0),
            (min(w - 1, center + CENTER_DEADZONE), h - 1),
            (255, 255, 255), 1,
        )

        servo_x = int(center + (self.state.servo_angle - SERVO_CENTER) / max(1, SERVO_MAX - SERVO_MIN) * (w * 0.40))
        servo_x = max(0, min(w - 1, servo_x))
        cv2.line(frame, (center, 18), (servo_x, 18), (0, 255, 255), 2)
        cv2.circle(frame, (servo_x, 18), 5, (0, 255, 255), -1)

        targets = self.vision.get_targets()
        obstacle_ids = {id(d) for d in self.vision.get_obstacles()}

        # Caixas do TRACKER (roxo). Antes eram desenhadas para TODA track viva,
        # inclusive as que ja tinham caixa de deteccao no mesmo quadro - ou seja,
        # dois contornos em volta do mesmo objeto, e um contorno para cada pessoa
        # ou cadeira em volta. Poluia a tela sem informar nada.
        # Agora so aparece quando a track NAO tem deteccao fresca neste quadro
        # (age > 0), que e exatamente o caso em que ela informa algo: o objeto
        # sumiu e o rover ainda sabe onde ele esta.
        for track in self.navigator.tracker.get_active_tracks():
            if track['age'] == 0:
                continue
            x1, y1, x2, y2 = track['bbox']
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 1)
            cv2.putText(frame, f"MEMORIA: {track['label']}", (x1, max(16, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)

        # [Revisao] Antes TODO detection da classe-alvo era desenhado em verde
        # grosso, do mesmo jeito - com duas garrafas em quadro, o operador via
        # duas caixas "igualmente o alvo" e não tinha como saber qual delas o
        # rover realmente estava seguindo. Agora só a caixa TRAVADA (a mesma
        # que o navegador usa para dirigir) fica em verde grosso com rótulo
        # "ALVO"; candidatos da mesma classe que NÃO são o alvo travado ficam
        # em amarelo fino com "outro alvo?", deixando visível quando há
        # ambiguidade em cena.
        locked = self.navigator._current_target_detection()
        for det in self.vision.get_all_detections():
            is_locked = locked is not None and (
                det.x1, det.y1, det.x2, det.y2, det.label
            ) == (locked.x1, locked.y1, locked.x2, locked.y2, locked.label)
            if is_locked:
                color = (0, 255, 0)
                thickness = 3
                label_text = f"ALVO: {det.label} {det.confidence:.2f}"
            elif det.label in targets:
                color = (0, 255, 255)
                thickness = 1
                label_text = f"outro alvo? {det.label} {det.confidence:.2f}"
            elif id(det) in obstacle_ids:
                color = (0, 0, 255)
                thickness = 2
                label_text = f"{det.label} {det.confidence:.2f}"
            else:
                color = (160, 160, 160)
                thickness = 1
                label_text = f"{det.label} {det.confidence:.2f}"

            cv2.rectangle(frame, (det.x1, det.y1), (det.x2, det.y2), color, thickness)
            cv2.putText(frame, label_text, (det.x1, max(16, det.y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        # Overlay v21 com mais informações
        cv2.rectangle(frame, (0, 0), (w, 110), (0, 0, 0), -1)
        cv2.putText(frame,
                    f"{self.state.mode.value} | {self.state.ai_decision} | FPS:{self.state.fps:.0f} | INF:{self.state.inference_ms:.0f}ms",
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame,
                    f"Alvo:{self.state.target_label if self.state.target_visible else '-'} | Servo:{self.state.servo_angle} | US:{self.state.ultrasonic_cm:.0f}cm",
                    (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame,
                    f"Alinhamento:{self.state.alignment_error_deg:+.1f}deg | Comando:{self.state.last_command} PWM:{self.state.last_speed}",
                    (8, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame,
                    f"DQN:ε={self.state.dqn_epsilon:.3f} Loss={self.state.dqn_loss:.4f} | VO:dx={self.state.vo_dx:.1f} | Kalman:({self.state.kalman_x:.0f},{self.state.kalman_y:.0f})",
                    (8, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1, cv2.LINE_AA)

        if self.state.obstacle:
            cv2.putText(frame, "OBSTACULO - DESVIO", (10, h - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
        if self.state.stuck:
            cv2.putText(frame, "TRAVAMENTO - RECUPERANDO", (10, h - 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)
        if self.state.loop_closure_detected:
            cv2.putText(frame, "LOOP CLOSURE DETECTADO", (w - 320, h - 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 0, 255), 2, cv2.LINE_AA)
        if self.state.ultrasonic_ok and self.state.ultrasonic_cm <= CRITICAL_DISTANCE_CM:
            cv2.putText(frame, "ULTRASSONICO CRITICO", (w - 260, h - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        return frame

    def _ui_tick(self):
        if not self.running:
            return
        try:
            frame = self.camera.get_frame()
            if frame is not None:
                # Frame skip adaptativo
                # [Revisao] state.frame_skip nunca era atualizado: a interface
                # mostrava "Skip:0" para sempre, mesmo com o pulo ativo.
                self.state.frame_skip = self.navigator.frame_skipper.skip_count
                if self.navigator.frame_skipper.should_process(self.state.fps):
                    self.vision.submit(frame, self.state.servo_angle)

                    # Atualizar Visual Odometry
                    try:
                        dx, dy, rot = self.navigator.visual_odometry.update(frame, self.state.servo_angle)
                        self.state.vo_dx = dx
                        self.state.vo_dy = dy
                        self.state.vo_rotation = rot
                    except Exception as exc:
                        # Importa: a deteccao de travamento usa o fluxo optico.
                        # Se a VO morrer em silencio, o rover perde o sensor de
                        # progresso e ninguem fica sabendo.
                        self.logbook.add_throttled(
                            "vo_fail", "Odometria visual falhou (detecção de travamento degradada): %s" % exc)

                display = self._draw_overlay(frame.copy())
                if PIL_AVAILABLE:
                    rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
                    image = Image.fromarray(rgb)
                    wa = max(400, self.camera_label.winfo_width() - 10)
                    ha = max(300, self.camera_label.winfo_height() - 10)
                    scale = min(wa / image.width, ha / image.height, 1.0)
                    if scale < 1.0:
                        image = image.resize((max(1, int(image.width * scale)), max(1, int(image.height * scale))))
                    photo = ImageTk.PhotoImage(image)
                    self.camera_label.configure(image=photo, text="")
                    self.camera_label.image = photo
                else:
                    self.camera_label.configure(text="Pillow nao instalado; camera recebida")

            now = time.time()
            camera_ok = self.state.camera_ok and now - self.state.camera_timestamp <= CAMERA_STALE_SECONDS
            self.status_var.set("[OK] CONECTADO" if self.state.connected else "[X] RECONECTANDO...")
            self.camera_info_var.set(f"Camera: {'OK' if camera_ok else 'OFF'} | FPS:{self.state.fps:.0f} | INF:{self.state.inference_ms:.0f}ms | Skip:{self.state.frame_skip}")
            self.mode_var.set(self.state.mode.value)
            self.servo_var.set(f"Servo: {self.state.servo_angle} -> {self.state.servo_target_angle} graus")
            self.alignment_var.set(f"Alinhamento: {self.state.alignment_error_deg:+.1f} graus | progresso={self.state.alignment_progress:.0f}%")

            # [Revisao] Mostra o mesmo alvo que o navegador está de fato
            # usando (com a trava de identidade), não só "o melhor do
            # quadro" - senão a tela mostraria um objeto enquanto o rover
            # segue outro.
            target = self.navigator._current_target_detection()
            if target:
                self.target_info_var.set(f"{target.label} {target.confidence:.0%} | area={target.area_ratio:.1%}")
            elif self.state.target_coasting:
                # [Fase 2] alvo ocluso: seguindo pela track, sem deteccao fresca
                self.target_info_var.set(f"{self.state.target_label} (OCLUSO - seguindo pela track)")
            else:
                self.target_info_var.set("--")

            if self.state.obstacle:
                self.obstacle_info_var.set(f"{self.state.obstacle_label} {self.state.obstacle_confidence:.0%}")
            else:
                self.obstacle_info_var.set("--")

            if self.state.ultrasonic_ok:
                if self.state.ultrasonic_cm <= CRITICAL_DISTANCE_CM:
                    self.ultrasonic_info_var.set(f"US CRITICO {self.state.ultrasonic_cm:.0f}cm")
                elif self.state.ultrasonic_cm <= SAFE_DISTANCE_CM:
                    self.ultrasonic_info_var.set(f"US BLOQUEADO {self.state.ultrasonic_cm:.0f}cm")
                else:
                    self.ultrasonic_info_var.set(f"US LIVRE {self.state.ultrasonic_cm:.0f}cm")
            else:
                self.ultrasonic_info_var.set("US SEM LEITURA")

            self.ai_decision_var.set(self.state.ai_decision)
            self.ai_reason_var.set(self.state.ai_reason)
            self.path_quality_var.set(f"Qualidade visual: {self.state.path_quality.value}")

            # Novos displays v21
            if self.navigator.dqn:
                stats = self.navigator.dqn.get_stats()
                self.dqn_var.set(f"DQN: ε={stats['epsilon']:.3f} | Updates:{stats['updates']} | Loss:{stats['avg_loss']:.4f} | Q:{stats['avg_q']:.1f}")
            else:
                self.dqn_var.set("DQN: Não disponível")

            vo_pose = self.navigator.visual_odometry.get_pose()
            self.vo_var.set(f"VO: dx={vo_pose['dx_total']:.0f} dy={vo_pose['dy_total']:.0f} rot={vo_pose['rotation_total']:.0f}° frames={vo_pose['frames']}")

            self.kalman_var.set(f"Kalman: ({self.state.kalman_x:.0f}cm, {self.state.kalman_y:.0f}cm) v=({self.state.kalman_vx:.1f}, {self.state.kalman_vy:.1f})")
            self.curiosity_var.set(f"Curiosidade: {self.state.curiosity_reward:.3f} | Tracked: {self.state.tracked_objects}")

            map_values = ", ".join(f"{k}:{v:.0f}cm" for k, v in sorted(self.state.ultrasonic_map.items()))
            map_summary = self.navigator.spatial_map.summary()
            self.telemetry_var.set(
                f"Modo:             {self.state.mode.value}\n"
                f"Comando:          {self.state.last_command}\n"
                f"PWM:              {self.state.last_speed}\n"
                f"Camera:           {'OK' if camera_ok else 'OFF'}\n"
                f"FPS camera:       {self.state.fps:.1f}\n"
                f"Inferencia:       {self.state.inference_ms:.1f} ms\n"
                f"Ultrassonico:     {self.state.ultrasonic_cm:.0f} cm\n"
                f"Angulo US:        {self.state.ultrasonic_angle} graus\n"
                f"US status:        {'OK' if self.state.ultrasonic_ok else 'SEM LEITURA'}\n"
                f"Servo:            {self.state.servo_angle} -> {self.state.servo_target_angle}\n"
                f"Alinhamento:      {self.state.alignment_error_deg:+.1f} graus\n"
                f"Alvo:             {'SIM' if self.state.target_visible else 'NAO'}\n"
                f"Obstaculo:        {'SIM' if self.state.obstacle else 'NAO'}\n"
                f"Travado:          {'SIM' if self.state.stuck else 'NAO'}\n"
                f"Caminho ruim:     {'SIM' if self.state.bad_path else 'NAO'}\n"
                f"Loop Closure:     {'SIM' if self.state.loop_closure_detected else 'NAO'}\n"
                f"Mapa sonar:       {map_values or '--'}\n"
                f"Mapa espacial:    {map_summary['cells']} cel | bloqueadas={map_summary['blocked']}\n"
                f"Pose estimada:    ({map_summary['x_cm']}cm, {map_summary['y_cm']}cm) {map_summary['heading']}deg\n"
                f"Pose Kalman:      ({self.state.kalman_x:.0f}cm, {self.state.kalman_y:.0f}cm)\n"
                f"TX/RX:            {self.state.tx_count}/{self.state.rx_count}\n"
                f"Ultimo erro:      {self.state.last_error[:80] if self.state.last_error else '--'}\n"
                f"DQN epsilon:      {self.state.dqn_epsilon:.4f}\n"
                f"DQN loss:         {self.state.dqn_loss:.4f}\n"
                f"Curiosidade:      {self.state.curiosity_reward:.3f}\n"
                f"Objetos tracked:  {self.state.tracked_objects}"
            )

            self.footer_var.set(
                f"ESP32 {ESP32_IP}:{ESP32_TCP_PORT} | CAMERA {STREAM_URL} | "
                f"YOLO + DQN + VO + Kalman + A* | targets: {', '.join(sorted(self.vision.get_targets()))}"
            )
            self._refresh_logs()
        except Exception as exc:
            self.state.last_error = str(exc)
        self.root.after(80, self._ui_tick)

    def _refresh_logs(self):
        # BUG de desempenho corrigido: antes o widget inteiro era apagado e
        # reescrito com 180 linhas a CADA 80 ms (~2250 inserts/s no Tk). Como
        # o laco de navegacao roda na mesma thread da interface, isso roubava
        # tempo das decisoes. Agora so acrescenta o que e novo, e so quando ha
        # algo novo de verdade.
        seq = self.logbook.seq
        if seq == self._log_seq_rendered:
            return
        items = self.logbook.snapshot()
        novos = seq - self._log_seq_rendered
        self._log_seq_rendered = seq
        trecho = items[-novos:] if 0 < novos <= len(items) else items[-180:]

        self.log_text.configure(state="normal")
        for msg in trecho[-180:]:
            self.log_text.insert("end", msg + "\n")
        # numa feira o programa fica horas ligado: limita o widget
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
            self.logbook.add(f"Erro Navigator: {exc}", "ERROR")
            try:
                self.connection.send_command(Direction.STOP, 0, force=True)
            except Exception as exc2:
                self.logbook.add("FALHA AO PARAR apos erro do navegador: %s" % exc2, "ERROR")
        self.root.after(max(40, int(CONTROL_INTERVAL * 1000)), self._nav_tick)

    def close(self):
        if not self.running:
            return
        self.running = False

        # Encerramento: cada etapa e best-effort, mas NAO em silencio. Se algo
        # falhar aqui (ex.: o rover nao recebeu o STOP), voce precisa saber -
        # o log da interface ja esta sendo destruido, entao vai para o terminal.
        def _try(desc, fn):
            try:
                fn()
            except Exception as exc:
                print("[encerramento] falhou em %s: %s" % (desc, exc))

        _try("salvar mapa/DQN", self._save_state_on_close)
        _try("parada de emergencia", self.navigator.emergency_stop)
        _try("comando STOP final", lambda: self.connection.send_command(Direction.STOP, 0, force=True))
        _try("fechar camera", self.camera.close)
        _try("fechar visao", self.vision.close)
        _try("fechar conexao", self.connection.close)
        _try("destruir janela", self.root.destroy)

    def _save_state_on_close(self):
        self.navigator.spatial_map.save(force=True)
        if self.navigator.dqn:
            self.navigator.dqn.save()


# =====================================================================
# MAIN
# =====================================================================

def main():
    root = tk.Tk()
    app = GEARAIRoverV21(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.close()


if __name__ == "__main__":
    main()
