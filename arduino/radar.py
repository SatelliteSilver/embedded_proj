import pygame
import serial
import math
import sys
import time

# --- 시리얼 설정 (비블로킹) ---
ser_radar = serial.Serial('COM14', 115200, timeout=0)  # ← 0: non-blocking
ser_bt    = serial.Serial('COM9',  115200, timeout=0, write_timeout=0.2)

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
DIST_TOL = 12         # 같은 물체로 묶을 거리 허용(cm)
ANGLE_GAP_MAX = 4     # 연속으로 인정할 최대 각도 간격(도)
MIN_CLUSTER_SIZE = 3  # 최소 포인트 수
BIG_DOT_RADIUS = 10   # 큰 원 반지름(픽셀)
HIT_RADIUS = 14       # 마우스 클릭 판정 반경(픽셀)

# 스윕 동안 수집한 포인트(각도, 거리)
sweep_points = []
# 가장 최근 스윕에서 검출된 클러스터(큰 원으로 그림)
latest_clusters = []   # list of dicts: {"angle":..., "dist":..., "span":...}

# --- 상태 ---
active_sweep = False         # 스윕 진행 중
show_clusters_now = False    # 스윕 종료 후 클러스터 표시
last_space_ts = 0            # SPACE 디바운스

# 스윕 종료 판정
ANGLE_STABLE_TOL = 1
STABLE_FRAMES = 12
angle_stable_count = 0

IDLE_END_SEC = 0.40
MIN_SWEEP_POINTS = 12
last_rx_ts = 0.0

# 끝각(사용자 요청: 15°/165°)
END_NEAR_DEG = 15
def near_end(a): return (a <= END_NEAR_DEG) or (a >= 180 - END_NEAR_DEG)

# --- UI: 버튼들 ---
BTN_W, BTN_H = 110, 34
BTN_PAD = 10

# 1행: CAR, TARGET
btn_target = pygame.Rect(WIDTH - BTN_PAD - BTN_W, BTN_PAD, BTN_W, BTN_H)
btn_car    = pygame.Rect(btn_target.left - BTN_PAD - BTN_W, BTN_PAD, BTN_W, BTN_H)
# 2행: DETECTION, ATTACK
btn_detect = pygame.Rect(btn_car.left, btn_car.bottom + BTN_PAD, BTN_W, BTN_H)
btn_attack = pygame.Rect(btn_target.left, btn_target.bottom + BTN_PAD, BTN_W, BTN_H)

# UI 상태
select_mode = None       # None / "CAR" / "TARGET"
car_idx = None           # latest_clusters에서 CAR로 선택된 인덱스
target_idx = None        # latest_clusters에서 TARGET로 선택된 인덱스
attack_view = False      # ATTACK 후 CAR/TARGET만 보이게

# 5단계: BT 전송 후 OK 대기 상태
waiting_ok = False
last_info_msg = ""
last_info_time = 0.0

# ---------- 유틸 ----------
def pol_to_xy(a_deg, d_cm):
    """화면 픽셀 좌표(아크 그릴 때 사용)"""
    rad = math.radians(a_deg)
    x = center_x + (d_cm * scale) * math.cos(rad)
    y = center_y - (d_cm * scale) * math.sin(rad)
    return int(x), int(y)

def polar_to_xy_cm(a_deg, d_cm):
    """로봇용 좌표(cm). 원점: 레이더 기준점, +x: 오른쪽, +y: 위쪽."""
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
        a_prev, d_prev = pts[i-1]
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
        mean_d = int(round(sum(dists)  / len(dists)))
        span   = max(angles) - min(angles)
        if 0 < mean_d <= MAX_DIST_CM:
            result.append({"angle": mean_a, "dist": mean_d, "span": span})
    return result

def hit_test_cluster(mx, my):
    """마우스 좌표(mx,my)와 가장 가까운 클러스터를 찾아 HIT_RADIUS 내면 (idx, dist) 반환, 아니면 (None, None)"""
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

# --- 시리얼 라인 파서 (누적 버퍼 기반 비블로킹) ---
buf_radar = bytearray()
buf_bt    = bytearray()

def read_nonblocking_lines(ser, buf, max_lines=32):
    """in_waiting만큼 즉시 읽고, 개행 기준으로 잘라 최대 max_lines개 반환"""
    lines = []
    try:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n)
            if chunk:
                buf.extend(chunk)
                parts = buf.split(b'\n')      # \r\n, \n 모두 지원
                buf[:] = parts[-1]            # 마지막 조각은 미완성 → 버퍼에 남김
                for raw in parts[:-1]:
                    lines.append(raw.rstrip(b'\r').decode('utf-8', errors='ignore'))
                    if len(lines) >= max_lines:
                        break
    except Exception:
        pass
    return lines

# ---------- 그리기 ----------
def draw_radar():
    for cm in range(20, MAX_DIST_CM, 20):
        r = int(cm * scale)
        pygame.draw.arc(screen, GREEN, (center_x - r, center_y - r, 2*r, 2*r),
                        math.pi, 2*math.pi, 1)
        screen.blit(font.render(f"{cm} cm", True, GREEN),
                    (center_x + 10, center_y - r - 10))

    pygame.draw.arc(screen, GREEN,
                    (center_x - MAX_RADIUS_PX, center_y - MAX_RADIUS_PX,
                     2*MAX_RADIUS_PX, 2*MAX_RADIUS_PX),
                    math.pi, 2*math.pi, 2)
    screen.blit(font.render(f"{MAX_DIST_CM} cm", True, GREEN),
                (center_x + 10, center_y - MAX_RADIUS_PX - 10))

    for deg in range(0, 181, 30):
        rad = math.radians(deg)
        x = center_x + MAX_RADIUS_PX * math.cos(rad)
        y = center_y - MAX_RADIUS_PX * math.sin(rad)
        pygame.draw.line(screen, GREEN, (center_x, center_y), (x, y), 1)
        lx = center_x + (MAX_RADIUS_PX + 40) * math.cos(rad)
        ly = center_y - (MAX_RADIUS_PX + 20) * math.sin(rad)
        screen.blit(font.render(f"{deg}°", True, GREEN), (lx - 15, ly - 10))

def draw_line(a):
    rad = math.radians(a)
    x = center_x + MAX_RADIUS_PX * math.cos(rad)
    y = center_y - MAX_RADIUS_PX * math.sin(rad)
    pygame.draw.line(screen, GREEN, (center_x, center_y), (x, y), 2)

def draw_memory_objects():
    # 스윕 중일 때만 빨간 점
    if not active_sweep:
        return
    for a in range(181):
        d = distance_map[a]
        if 0 < d <= MAX_DIST_CM:
            x, y = pol_to_xy(a, d)
            pygame.draw.circle(screen, RED, (x, y), 4)

def draw_clusters():
    if not show_clusters_now:
        return

    def should_draw(idx):
        if not attack_view:
            return True
        # ATTACK 이후엔 CAR/TARGET만
        return (idx == car_idx) or (idx == target_idx)

    for idx, obj in enumerate(latest_clusters):
        if not should_draw(idx):
            continue
        x, y = pol_to_xy(obj["angle"], obj["dist"])
        # 기본(주황)
        pygame.draw.circle(screen, ORANGE, (x, y), BIG_DOT_RADIUS, width=2)
        pygame.draw.circle(screen, ORANGE, (x, y), 3)
        screen.blit(font.render(f'{obj["dist"]}cm', True, ORANGE), (x + 8, y - 8))
        # 선택 강조
        labels = []
        if idx == car_idx:
            pygame.draw.circle(screen, CAR_COLOR, (x, y), BIG_DOT_RADIUS + 3, width=2)
            labels.append("CAR")
        if idx == target_idx:
            pygame.draw.circle(screen, TARGET_COLOR, (x, y), BIG_DOT_RADIUS + 6, width=2)
            labels.append("TARGET")
        if labels:
            tag = "/".join(labels)
            screen.blit(font_small.render(tag, True, WHITE), (x + 8, y + 8))

def draw_text(a, d):
    d_clamped = d if 0 < d <= MAX_DIST_CM else 0
    state = ("SWEEP" if active_sweep else
             ("WAIT_OK" if waiting_ok else "HOLD"))
    msg = f"[{state}] Angle: {a:3d}°   Distance: {d_clamped:3d} cm (max {MAX_DIST_CM} cm)"
    screen.blit(font.render(msg, True, WHITE), (20, 20))

    # 하단 알림(3초 표시)
    if last_info_msg and (time.time() - last_info_time < 3.0):
        screen.blit(font.render(last_info_msg, True, WARN), (20, HEIGHT - 30))

def draw_ui():
    # 1행
    pygame.draw.rect(screen, UI_BG, btn_car, border_radius=8)
    pygame.draw.rect(screen, UI_BG, btn_target, border_radius=8)
    if select_mode == "CAR":
        pygame.draw.rect(screen, UI_ACTIVE, btn_car, border_radius=8)
    elif select_mode == "TARGET":
        pygame.draw.rect(screen, UI_ACTIVE, btn_target, border_radius=8)
    pygame.draw.rect(screen, CAR_COLOR, btn_car, width=2, border_radius=8)
    pygame.draw.rect(screen, TARGET_COLOR, btn_target, width=2, border_radius=8)
    screen.blit(font.render("CAR", True, CAR_COLOR),
                (btn_car.centerx - font.size("CAR")[0]//2, btn_car.centery - font.get_height()//2))
    screen.blit(font.render("TARGET", True, TARGET_COLOR),
                (btn_target.centerx - font.size("TARGET")[0]//2, btn_target.centery - font.get_height()//2))

    # 2행
    pygame.draw.rect(screen, UI_BG, btn_detect, border_radius=8)
    pygame.draw.rect(screen, UI_BG, btn_attack, border_radius=8)
    pygame.draw.rect(screen, WHITE, btn_detect, width=2, border_radius=8)
    pygame.draw.rect(screen, WHITE, btn_attack, width=2, border_radius=8)
    screen.blit(font.render("DETECTION", True, WHITE),
                (btn_detect.centerx - font.size("DETECTION")[0]//2, btn_detect.centery - font.get_height()//2))
    screen.blit(font.render("ATTACK", True, WHITE),
                (btn_attack.centerx - font.size("ATTACK")[0]//2, btn_attack.centery - font.get_height()//2))

    # 현재 선택 요약
    info_y = btn_detect.bottom + 6
    car_info = f"CAR: {'-' if car_idx is None else car_idx}"
    tgt_info = f"TGT: {'-' if target_idx is None else target_idx}"
    screen.blit(font_small.render(car_info, True, CAR_COLOR), (btn_car.left, info_y))
    screen.blit(font_small.render(tgt_info, True, TARGET_COLOR), (btn_target.left, info_y))

# ---------- 동작 함수들 ----------
def start_detection_sweep():
    """DETECTION: 스윕 1회 시작"""
    global distance_map, sweep_points, latest_clusters
    global show_clusters_now, active_sweep, angle_stable_count
    global last_rx_ts, attack_view, waiting_ok
    distance_map = [0 for _ in range(181)]
    sweep_points = []
    latest_clusters = []
    show_clusters_now = False
    active_sweep = True
    angle_stable_count = 0
    last_rx_ts = time.time()
    attack_view = False         # 새 스윕에서는 일반 표시
    waiting_ok = False          # OK 대기 해제
    ser_radar.write(b'S')             # 아두이노 트리거
    print("[TX RADAR] start sweep")

def do_attack():
    """ATTACK 버튼 누르면 테스트용 명령 전송 (쉼표 없이 문자+숫자 바이트)"""
    global waiting_ok

    # 보낼 값 설정
    v_val = 50   # 속도
    a_val = 50   # 각도
    s_val = 1    # 상태 or 플래그

    # 메시지 구성: v <50> a <50> s <1>
    msg = bytearray([
        ord('v'), v_val & 0xFF,
        ord('a'), a_val & 0xFF,
        ord('s'), s_val & 0xFF
    ])

    try:
        ser_bt.write(msg)
        time.sleep(0.05)  # 블루투스 안정화
        print("[BT TX - RAW BYTES]", list(msg))  # 사람이 보기 쉽게 출력
        info("테스트 명령 전송 완료. OK 신호 대기중...")
        waiting_ok = True
    except Exception as e:
        info(f"전송 실패: {e}")

# ---------- 메인 루프 ----------
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            ser_radar.close()
            ser_bt.close()
            pygame.quit(); sys.exit()
        # --- 마우스 클릭 처리 ---
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            # 버튼 클릭
            if btn_car.collidepoint(mx, my):
                select_mode = "CAR"
            elif btn_target.collidepoint(mx, my):
                select_mode = "TARGET"
            elif btn_detect.collidepoint(mx, my):
                start_detection_sweep()
            elif btn_attack.collidepoint(mx, my):
                do_attack()
            else:
                # 스윕 종료 후(=HOLD) + 클러스터 클릭 시 지정
                if not active_sweep and show_clusters_now and select_mode is not None:
                    idx, _ = hit_test_cluster(mx, my)
                    if idx is not None:
                        if select_mode == "CAR":
                            car_idx = idx
                        elif select_mode == "TARGET":
                            target_idx = idx
                        # 모드 유지(연속 지정 가능)
                        # select_mode = None

    # --- 스페이스바(대체 트리거): 한 번의 스윕 시작(디바운스 포함) ---
    keys = pygame.key.get_pressed()
    now = time.time()
    if keys[pygame.K_SPACE] and (now - last_space_ts) > 0.25 and not active_sweep:
        last_space_ts = now
        start_detection_sweep()

    # --- 레이더(각도,거리) 수신: COM14 (비블로킹 폴링) ---
    for line in read_nonblocking_lines(ser_radar, buf_radar, max_lines=50):
        if ',' in line:
            try:
                a_str, d_str = line.split(',', 1)
                a = int(a_str); d = int(d_str)
                if 0 <= a <= 180:
                    angle = a
                distance = d
                if 0 < distance <= MAX_DIST_CM:
                    last_rx_ts = time.time()
            except ValueError:
                pass  # 파싱 실패 무시

    # --- 블루투스 제어 수신: COM20 (비블로킹 폴링) ---
    for btline in read_nonblocking_lines(ser_bt, buf_bt, max_lines=50):
        up = btline.upper()
        if up == "OK" and waiting_ok:
            info("OK 수신! (6단계 동작은 다음 단계에서 구현)")
            print("[BT RX] OK")
            # 6단계 구현 전이므로 waiting_ok 유지/해제는 여기서 보류
        elif up == "CLEAR":
            print("[BT RX] CLEAR")  # 7단계에서 처리 예정

    # --- 각도 '정지' 카운트 (지터 허용) ---
    if abs(angle - prev_angle) <= ANGLE_STABLE_TOL:
        angle_stable_count = min(angle_stable_count + 1, STABLE_FRAMES + 5)
    else:
        angle_stable_count = 0

    # --- 방향 추정(정보용) ---
    if angle > prev_angle: sweep_dir = 1
    elif angle < prev_angle: sweep_dir = -1
    prev_angle = angle

    # --- 거리 갱신 & 스윕 포인트 수집 (스윕 중에만 수집) ---
    if active_sweep:
        if 0 < distance <= MAX_DIST_CM:
            distance_map[angle] = distance
            if not sweep_points or sweep_points[-1][0] != angle:
                sweep_points.append((angle, distance))
        else:
            distance_map[angle] = 0

    # --- 스윕 종료 (끝각 도달 + idle/각도안정) ---
    if active_sweep:
        idle_elapsed = time.time() - last_rx_ts
        end_ready = near_end(angle)

        finalize = False
        reason = ""
        if (len(sweep_points) >= MIN_SWEEP_POINTS) and end_ready and (idle_elapsed >= IDLE_END_SEC):
            finalize = True; reason = "rx-idle@end"
        elif end_ready and (angle_stable_count >= STABLE_FRAMES):
            finalize = True; reason = "angle-stable@end"
        elif end_ready and (idle_elapsed >= IDLE_END_SEC * 3):
            finalize = True; reason = "failsafe@end"

        if finalize:
            n_pts = len(sweep_points)
            latest_clusters = process_clusters(sweep_points)
            show_clusters_now = True
            active_sweep = False
            sweep_points = []
            distance_map = [0 for _ in range(181)]
            print(f"[CLUSTER] pts={n_pts} obj={len(latest_clusters)} ({reason})")
            # 스윕 직후에는 ATTACK 이전이므로 전체 클러스터 보이기
            attack_view = False

    # --- 화면 갱신 ---
    screen.fill(BG_FADE)
    draw_radar()
    draw_memory_objects()   # 스윕 중엔 빨간 점
    draw_clusters()         # 스윕 종료 후엔 클러스터(ATTACK 후엔 CAR/TARGET만)
    draw_line(angle)
    draw_text(angle, distance)
    draw_ui()

    pygame.display.flip()
    clock.tick(30) 
