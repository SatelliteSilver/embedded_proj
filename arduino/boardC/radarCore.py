import pygame
import serial
import math
import sys
import time
import struct
import random # (Pygame 기본 포함이므로 유틸리티로 남겨둡니다)

# --- 시리얼 설정 (비블로킹) ---
try:
    ser_radar = serial.Serial('COM14', 115200, timeout=0)  # ← 0: non-blocking
    ser_bt = serial.Serial('COM9', 115200, timeout=0, write_timeout=0.2)
except Exception as e:
    print(f"시리얼 포트 연결 실패: {e}")
    print("가상 시리얼 모드로 실행합니다 (데이터 수신 안 됨).")
    # 폴백(Fallback)을 위해 None으로 설정
    class MockSerial:
        def in_waiting(self): return 0
        def read(self, n): return b''
        def write(self, data): return len(data)
        def close(self): pass
    ser_radar = MockSerial()
    ser_bt = MockSerial()


# --- Pygame 초기화 ---
pygame.init()
WIDTH, HEIGHT = 960, 540
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Ultrasonic Radar - One-Sweep Cluster View")
clock = pygame.time.Clock()
font = pygame.font.SysFont("consolas", 18)
font_small = pygame.font.SysFont("consolas", 14)

# --- 색상 ---
GREEN = (98, 245, 31)
RED = (255, 50, 50)
BG_FADE = (0, 4, 0)
WHITE = (255, 255, 255)
ORANGE = (255, 170, 0)
CAR_COLOR = (0, 210, 255)
TARGET_COLOR = (255, 240, 0)
UI_BG = (20, 20, 20)
UI_ACTIVE = (60, 60, 60)
WARN = (255, 120, 120)
# AI_COLOR 삭제

# --- 중심 좌표 ---
center_x, center_y = WIDTH // 2, HEIGHT

# --- 변수 ---
angle = 0
distance = 0
prev_angle = 0
sweep_dir = 1  # 1 = 정방향(0→180), -1 = 역방향(180→0)

# --- 거리 스케일 (최대 160 cm) ---
MAX_DIST_CM = 160
MAX_RADIUS_PX = 330
scale = MAX_RADIUS_PX / MAX_DIST_CM  # px/cm

# 각도별 거리 기억 (0~180도)
distance_map = [0 for _ in range(181)]

# --- 클러스터링 파라미터 ---
DIST_TOL = 12
ANGLE_GAP_MAX = 4
MIN_CLUSTER_SIZE = 3
BIG_DOT_RADIUS = 10
HIT_RADIUS = 14

# --- 데이터 저장 ---
sweep_points = []
latest_clusters = []

# --- 상태 ---
active_sweep = False
show_clusters_now = False
last_space_ts = 0
ANGLE_STABLE_TOL = 1
STABLE_FRAMES = 12
angle_stable_count = 0
IDLE_END_SEC = 0.40
MIN_SWEEP_POINTS = 12
last_rx_ts = 0.0

END_NEAR_DEG = 15
def near_end(a): return (a <= END_NEAR_DEG) or (a >= 180 - END_NEAR_DEG)

# --- UI 버튼 ---
BTN_W, BTN_H = 110, 34
BTN_PAD = 10
btn_target = pygame.Rect(WIDTH - BTN_PAD - BTN_W, BTN_PAD, BTN_W, BTN_H)
btn_car = pygame.Rect(btn_target.left - BTN_PAD - BTN_W, BTN_PAD, BTN_W, BTN_H)
btn_detect = pygame.Rect(btn_car.left, btn_car.bottom + BTN_PAD, BTN_W, BTN_H)
btn_attack = pygame.Rect(btn_target.left, btn_target.bottom + BTN_PAD, BTN_W, BTN_H)
btn_block = pygame.Rect(btn_car.left - BTN_PAD - BTN_W, btn_car.top, BTN_W, BTN_H)
# btn_rl_episode 삭제

select_mode = None
car_idx = None
target_idx = None
block_idx = None
attack_view = False
waiting_ok = False
last_info_msg = ""
last_info_time = 0.0

# (AI 관련 설정 및 변수 모두 삭제)

# ---------- 유틸 ----------
def pol_to_xy(a_deg, d_cm):
    rad = math.radians(a_deg)
    x = center_x + (d_cm * scale) * math.cos(rad)
    y = center_y - (d_cm * scale) * math.sin(rad)
    return int(x), int(y)

def polar_to_xy_cm(a_deg, d_cm):
    rad = math.radians(a_deg)
    x_cm = d_cm * math.cos(rad)
    y_cm = d_cm * math.sin(rad)
    return x_cm, y_cm

def process_clusters(points):
    if not points:
        return []
    pts = sorted(points, key=lambda x: x[0])
    clusters = []
    cur = [pts[0]]
    for i in range(1, len(pts)):
        a_prev, d_prev = pts[i - 1]
        a, d = pts[i]
        if abs(d - d_prev) <= DIST_TOL and (a - a_prev) <= ANGLE_GAP_MAX:
            cur.append((a, d))
        else:
            clusters.append(cur)
            cur = [(a, d)]
    clusters.append(cur)

    result = []
    for group in clusters:
        if len(group) < MIN_CLUSTER_SIZE:
            continue
        angles = [g[0] for g in group]
        dists  = [g[1] for g in group]
        mean_a = int(round(sum(angles) / len(angles)))
        mean_d = int(round(sum(dists) / len(dists)))
        span   = max(angles) - min(angles)
        if 0 < mean_d <= MAX_DIST_CM:
            result.append({"angle": mean_a, "dist": mean_d, "span": span})
    return result

def hit_test_cluster(mx, my):
    if not show_clusters_now or active_sweep:
        return None, None
    best_idx, best_d = None, 1e9
    for i, obj in enumerate(latest_clusters):
        x, y = pol_to_xy(obj["angle"], obj["dist"])
        d = math.hypot(mx - x, my - y)
        if d < best_d:
            best_idx, best_d = i, d
    if best_d <= HIT_RADIUS:
        return best_idx, best_d
    return None, None

def info(msg):
    global last_info_msg, last_info_time
    last_info_msg = msg
    last_info_time = time.time()

# --- 시리얼 라인 파서 ---
buf_radar = bytearray()
buf_bt    = bytearray()
def read_nonblocking_lines(ser, buf, max_lines=32):
    lines = []
    try:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            if chunk:
                buf.extend(chunk)
                parts = buf.split(b'\n')
                buf[:] = parts[-1]
                for raw in parts[:-1]:
                    lines.append(raw.rstrip(b'\r').decode('utf-8', errors='ignore'))
                    if len(lines) >= max_lines:
                        break
    except Exception:
        pass
    return lines

# (AI 관련 함수 모두 삭제)

# ---------- 동작 함수 ----------
def start_detection_sweep():
    global distance_map, sweep_points, latest_clusters
    global show_clusters_now, active_sweep, angle_stable_count
    global last_rx_ts, attack_view, waiting_ok
    
    # (RL 관련 방어 코드 삭제)
        
    distance_map = [0 for _ in range(181)]
    sweep_points = []
    latest_clusters = []
    show_clusters_now = False
    active_sweep = True
    angle_stable_count = 0
    last_rx_ts = time.time()
    attack_view = False
    waiting_ok = False
    ser_radar.write(b'S')
    print("[TX RADAR] start sweep")

def build_packet(speed, angle, status):
    if abs(angle) == 0:
        angle_val = 0x00
    elif angle > 0:
        angle_val = angle & 0x7F
    else:
        angle_val = (abs(angle) & 0x7F) | 0x80
    pkt = 0
    pkt |= (0b01 << 30)
    pkt |= (speed & 0xFF) << 22
    pkt |= (0b10 << 20)
    pkt |= (angle_val << 12)
    pkt |= (0b11 << 10)
    pkt |= (status & 0x1) << 9
    return pkt

def read_bt_binary():
    if ser_bt.in_waiting >= 4:
        data = ser_bt.read(4)
        val = struct.unpack('>I', data)[0]
        status = (val >> 9) & 0x1
        return status
    return None

def do_attack():
    global waiting_ok, attack_view
    
    # (RL 관련 방어 코드 삭제)
        
    if car_idx is None or target_idx is None:
        info("CAR/TARGET not selected.")
        return

    car_obj = latest_clusters[car_idx]
    tgt_obj = latest_clusters[target_idx]
    x1, y1 = polar_to_xy_cm(car_obj["angle"], car_obj["dist"])
    x2, y2 = polar_to_xy_cm(tgt_obj["angle"], tgt_obj["dist"])
    path = [(x1, y1)]

    # --- BLOCK 피해서 우회 ---
    if block_idx is not None:
        blk_obj = latest_clusters[block_idx]
        bx, by = polar_to_xy_cm(blk_obj["angle"], blk_obj["dist"])

        # 차 → 블록, 블록 → 타겟 각도 계산
        angle_car_blk = math.atan2(by - y1, bx - x1)
        angle_blk_tgt = math.atan2(y2 - by, x2 - bx)

        # 두 각도 차이를 보고 어느 쪽으로 피할지 결정 (왼쪽 or 오른쪽)
        diff = (angle_blk_tgt - angle_car_blk + math.pi) % (2 * math.pi) - math.pi
        avoid_dir = -1 if diff > 0 else 1  # +는 왼쪽, -는 오른쪽
        AVOID_RADIUS = 25  # cm 단위 (회피 거리)

        # 블록 중심을 기준으로 90도 회피점 생성
        avoid_ang = math.atan2(y1 - by, x1 - bx) + avoid_dir * math.pi / 2
        ax = bx + AVOID_RADIUS * math.cos(avoid_ang)
        ay = by + AVOID_RADIUS * math.sin(avoid_ang)
        path.append((ax, ay))
        print(f"[PATH] AVOID via ({ax:.1f}, {ay:.1f}) around BLOCK ({bx:.1f},{by:.1f})")

    # --- 타겟 추가 ---
    path.append((x2, y2))

    # --- 세그먼트 계산 ---
    segments = []
    for i in range(1, len(path)):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        dx, dy = (x1 - x0), (y1 - y0)
        dist = math.hypot(dx, dy)
        angle_abs = math.degrees(math.atan2(dy, dx))
        segments.append({"angle_abs": angle_abs, "dist": dist})

    print(f"[PATH] {len(segments)} segments generated")

    base_speed = 20
    k = 0.5
    MAX_V = 255
    for i, seg in enumerate(segments):
        prev_angle = segments[i - 1]["angle_abs"] if i > 0 else 0.0
        delta_angle = seg["angle_abs"] - prev_angle
        dist = seg["dist"]
        speed = min(int(base_speed + k * dist), MAX_V)
        seg["delta_angle"] = delta_angle
        seg["speed"] = speed

    # --- 즉시 경로 시각화 ---
    attack_view = True
    info(f"ATTACK path ready ({len(segments)} segments)")
    waiting_ok = True
    pygame.display.flip()    # 화면 즉시 갱신
    time.sleep(0.3)          # 경로 표시 살짝 보이게

    # --- 첫 번째 세그먼트 송신 ---
    if len(segments) > 0:
        first = segments[0]
        pkt = build_packet(first["speed"], int(round(first["delta_angle"])), 0)
        pkt_bytes = struct.pack('>I', pkt)
        try:
            ser_bt.write(pkt_bytes)
            print(f"[BT TX] SEG0: speed={first['speed']}, angle={int(round(first['delta_angle']))}, dist={first['dist']:.1f}cm")
        except Exception as e:
            info(f"Transmission failed: {e}")
            return

    # --- 이후 나머지 세그먼트 전송 루프 ---
    for i, seg in enumerate(segments[1:], start=1):
        t0 = time.time()
        next_ok = False
        while time.time() - t0 < 10.0:
            status_reply = read_bt_binary()
            if status_reply == 1:
                next_ok = True
                print("[BT RX] status=1 → next segment")
                break
            time.sleep(0.05)

        if not next_ok:
            info("No status=1 response (segment aborted)")
            return

        # 다음 세그먼트 전송
        pkt = build_packet(seg["speed"], int(round(seg["delta_angle"])), 0)
        pkt_bytes = struct.pack('>I', pkt)
        try:
            ser_bt.write(pkt_bytes)
            print(f"[BT TX] SEG{i}: speed={seg['speed']}, angle={int(round(seg['delta_angle']))}, dist={seg['dist']:.1f}cm")
        except Exception as e:
            info(f"Transmission failed: {e}")
            return

    try:
        ser_bt.write(b"END\n")
        print("[BT TX] END")
        info("ATTACK complete (avoid path)")
    except Exception as e:
        info(f"END send failed: {e}")

