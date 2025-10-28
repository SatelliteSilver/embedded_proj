# main_app.py
import pygame
import math
import sys
import time
# 'serial'과 'struct'는 여기서 import하지 않습니다.
from radar import HardwareManager

# port variables Init
radar_port = 'COM10'
bluetooth_port = 'COM9'

# --- Pygame 초기화 ---
pygame.init()
WIDTH, HEIGHT = 960, 540
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Ultrasonic Radar - One-Sweep Cluster View")
clock = pygame.time.Clock()
font = pygame.font.SysFont("consolas", 18)
font_small = pygame.font.SysFont("consolas", 14)

# --- 통신 관리자 초기화 ---
# 이 객체가 모든 시리얼 통신을 담당합니다.
hw = HardwareManager(radar_port, bluetooth_port)
if not hw.is_ready():
    print("치명적 오류: 통신 포트를 열 수 없습니다. 프로그램을 종료합니다.")
    pygame.quit()
    sys.exit()

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

select_mode = None
car_idx = None
target_idx = None
block_idx = None
attack_view = False
waiting_ok = False
last_info_msg = ""
last_info_time = 0.0

# ---------- 유틸 ----------
# (시리얼 통신과 무관한 유틸리티 함수들은 그대로 유지)
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
        dists = [g[1] for g in group]
        mean_a = int(round(sum(angles) / len(angles)))
        mean_d = int(round(sum(dists) / len(dists)))
        span = max(angles) - min(angles)
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

# --- 시리얼 파서 관련 함수 (read_nonblocking_lines, build_packet 등) 제거 ---
# --- -> HardwareManager로 이동됨 ---

# ---------- 그리기 ----------
# (모든 draw_ 함수는 Pygame에만 의존하므로 그대로 유지)
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
        return (idx == car_idx) or (idx == target_idx) or (idx == block_idx)

    for idx, obj in enumerate(latest_clusters):
        if not should_draw(idx):
            continue
        x, y = pol_to_xy(obj["angle"], obj["dist"])
        pygame.draw.circle(screen, ORANGE, (x, y), BIG_DOT_RADIUS, width=2)
        pygame.draw.circle(screen, ORANGE, (x, y), 3)
        screen.blit(font.render(f'{obj["dist"]}cm', True, ORANGE), (x + 8, y - 8))
        labels = []
        if idx == car_idx:
            pygame.draw.circle(screen, CAR_COLOR, (x, y), BIG_DOT_RADIUS + 3, width=2)
            labels.append("CAR")
        if idx == target_idx:
            pygame.draw.circle(screen, TARGET_COLOR, (x, y), BIG_DOT_RADIUS + 6, width=2)
            labels.append("TARGET")
        if idx == block_idx:  # ✅ BLOCK 표시 추가
            pygame.draw.circle(screen, (180, 180, 255), (x, y), BIG_DOT_RADIUS + 4, width=2)
            labels.append("BLOCK")
        if labels:
            tag = "/".join(labels)
            screen.blit(font_small.render(tag, True, WHITE), (x + 8, y + 8))

def draw_text(a, d):
    d_clamped = d if 0 < d <= MAX_DIST_CM else 0
    state = ("SWEEP" if active_sweep else ("WAIT_OK" if waiting_ok else "HOLD"))
    msg = f"[{state}] Angle: {a:3d}°   Distance: {d_clamped:3d} cm (max {MAX_DIST_CM} cm)"
    screen.blit(font.render(msg, True, WHITE), (20, 20))
    if last_info_msg and (time.time() - last_info_time < 3.0):
        screen.blit(font.render(last_info_msg, True, WARN), (20, HEIGHT - 30))

def draw_ui():
    pygame.draw.rect(screen, UI_BG, btn_car, border_radius=8)
    pygame.draw.rect(screen, UI_BG, btn_target, border_radius=8)
    pygame.draw.rect(screen, UI_BG, btn_block, border_radius=8)
    if select_mode == "CAR":
        pygame.draw.rect(screen, UI_ACTIVE, btn_car, border_radius=8)
    elif select_mode == "TARGET":
        pygame.draw.rect(screen, UI_ACTIVE, btn_target, border_radius=8)
    elif select_mode == "BLOCK":
        pygame.draw.rect(screen, UI_ACTIVE, btn_block, border_radius=8)
    pygame.draw.rect(screen, CAR_COLOR, btn_car, width=2, border_radius=8)
    pygame.draw.rect(screen, TARGET_COLOR, btn_target, width=2, border_radius=8)
    pygame.draw.rect(screen, WHITE, btn_block, width=2, border_radius=8)
    screen.blit(font.render("CAR", True, CAR_COLOR), (btn_car.centerx - 24, btn_car.centery - 10))
    screen.blit(font.render("TARGET", True, TARGET_COLOR), (btn_target.centerx - 38, btn_target.centery - 10))
    screen.blit(font.render("BLOCK", True, WHITE), (btn_block.centerx - 30, btn_block.centery - 10))
    pygame.draw.rect(screen, UI_BG, btn_detect, border_radius=8)
    pygame.draw.rect(screen, UI_BG, btn_attack, border_radius=8)
    pygame.draw.rect(screen, WHITE, btn_detect, width=2, border_radius=8)
    pygame.draw.rect(screen, WHITE, btn_attack, width=2, border_radius=8)
    screen.blit(font.render("DETECTION", True, WHITE), (btn_detect.centerx - 50, btn_detect.centery - 10))
    screen.blit(font.render("ATTACK", True, WHITE), (btn_attack.centerx - 40, btn_attack.centery - 10))
    info_y = btn_detect.bottom + 6
    car_info = f"CAR: {'-' if car_idx is None else car_idx}"
    tgt_info = f"TGT: {'-' if target_idx is None else target_idx}"
    blk_info = f"BLK: {'-' if block_idx is None else block_idx}"
    screen.blit(font_small.render(car_info, True, CAR_COLOR), (btn_car.left, info_y))
    screen.blit(font_small.render(tgt_info, True, TARGET_COLOR), (btn_target.left, info_y))
    screen.blit(font_small.render(blk_info, True, WHITE), (btn_block.left, info_y))

# ---------- 동작 함수 ----------
def start_detection_sweep():
    global distance_map, sweep_points, latest_clusters
    global show_clusters_now, active_sweep, angle_stable_count
    global last_rx_ts, attack_view, waiting_ok
    
    # 1. 애플리케이션 상태 초기화 (여기서 수행)
    distance_map = [0 for _ in range(181)]
    sweep_points = []
    latest_clusters = []
    show_clusters_now = False
    active_sweep = True
    angle_stable_count = 0
    last_rx_ts = time.time()
    attack_view = False
    waiting_ok = False
    
    # 2. 하드웨어에 명령 전송 (HardwareManager 객체 사용)
    hw.start_detection_sweep()

def draw_attack_path():
    if not attack_view:
        return
    if car_idx is None or target_idx is None:
        return

    pts = []
    car = latest_clusters[car_idx]
    pts.append(pol_to_xy(car["angle"], car["dist"]))

    if block_idx is not None:
        blk = latest_clusters[block_idx]
        pts.append(pol_to_xy(blk["angle"], blk["dist"]))

    tgt = latest_clusters[target_idx]
    pts.append(pol_to_xy(tgt["angle"], tgt["dist"]))

    # 선 + 화살표 그리기
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        pygame.draw.line(screen, (255, 200, 50), (x0, y0), (x1, y1), 3)

        # --- 화살표 머리 ---
        dx, dy = (x1 - x0), (y1 - y0)
        ang = math.atan2(dy, dx)
        arrow_len = 12
        left = (x1 - arrow_len * math.cos(ang - 0.4),
                y1 - arrow_len * math.sin(ang - 0.4))
        right = (x1 - arrow_len * math.cos(ang + 0.4),
                 y1 - arrow_len * math.sin(ang + 0.4))
        pygame.draw.polygon(screen, (255, 200, 50), [(x1, y1), left, right])

def do_attack():
    global waiting_ok, attack_view
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
        
        angle_car_blk = math.atan2(by - y1, bx - x1)
        angle_blk_tgt = math.atan2(y2 - by, x2 - bx)
        
        diff = (angle_blk_tgt - angle_car_blk + math.pi) % (2 * math.pi) - math.pi
        avoid_dir = -1 if diff > 0 else 1  # +는 왼쪽, -는 오른쪽
        AVOID_RADIUS = 25  # cm 단위 (회피 거리)
        
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
    pygame.display.flip()  # 화면 즉시 갱신
    time.sleep(0.3)        # 경로 표시 살짝 보이게

    # --- 첫 번째 세그먼트 송신 (hw 객체 사용) ---
    if len(segments) > 0:
        first = segments[0]
        success = hw.send_attack_segment(first["speed"], int(round(first["delta_angle"])), 0)
        if not success:
            info("First Transmission failed")
            return

    # --- 이후 나머지 세그먼트 전송 루프 (hw 객체 사용) ---
    for i, seg in enumerate(segments[1:], start=1):
        
        # 블로킹으로 status=1 응답 대기 (hw 객체 사용)
        status_reply = hw.read_bt_binary_status(timeout=10.0)

        if status_reply != 1:
            info("No status=1 response (segment aborted)")
            return
        
        print("[APP] status=1 OK → next segment")

        # 다음 세그먼트 전송 (hw 객체 사용)
        success = hw.send_attack_segment(seg["speed"], int(round(seg["delta_angle"])), 0)
        if not success:
            info(f"Next Transmission failed (SEG{i})")
            return

    hw.send_attack_end() # (필요시)
    info("ATTACK complete (avoid path)")

# ---------- 메인 루프 ----------
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False # 루프 종료
            
        # --- 마우스 클릭 처리 ---
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            # 버튼 클릭
            if btn_car.collidepoint(mx, my):
                select_mode = "CAR"
            elif btn_target.collidepoint(mx, my):
                select_mode = "TARGET"
            elif btn_block.collidepoint(mx, my):  # ✅
                select_mode = "BLOCK"
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
                        elif select_mode == "BLOCK":  # ✅
                            block_idx = idx
                        # 모드 유지(연속 지정 가능)

    # --- 스페이스바(대체 트리거): 한 번의 스윕 시작(디바운스 포함) ---
    keys = pygame.key.get_pressed()
    now = time.time()
    if keys[pygame.K_SPACE] and (now - last_space_ts) > 0.25 and not active_sweep:
        last_space_ts = now
        start_detection_sweep()

    # --- [변경] 레이더(각도,거리) 수신: hw.poll_radar_data() 사용 ---
    new_radar_data = hw.poll_radar_data()
    for a, d in new_radar_data:
        angle = a
        distance = d
        if 0 < distance <= MAX_DIST_CM:
            last_rx_ts = time.time()

    # --- [변경] 블루투스 제어 수신: hw.poll_bt_text() 사용 ---
    bt_messages = hw.poll_bt_text()
    for up_msg in bt_messages:
        if up_msg == "OK" and waiting_ok:
            info("OK 수신! (6단계 동작은 다음 단계에서 구현)")
            print("[APP RX] OK")
            # 6단계 구현 전이므로 waiting_ok 유지/해제는 여기서 보류
        elif up_msg == "CLEAR":
            print("[APP RX] CLEAR")  # 7단계에서 처리 예정

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
    draw_memory_objects()  # 스윕 중엔 빨간 점
    draw_clusters()        # 스윕 종료 후엔 클러스터(ATTACK 후엔 CAR/TARGET만)
    draw_attack_path()
    draw_line(angle)
    draw_text(angle, distance)
    draw_ui()

    pygame.display.flip()
    clock.tick(30)

# --- 루프 종료 후 정리 ---
hw.close()
pygame.quit()
sys.exit()
