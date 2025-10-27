import pygame
import sys
import time
import math
# ✅ radar_core에서 모든 변수와 함수를 가져옴 (* 사용)
from radar_core import *

# ---------- 그리기 함수들 ----------
# (이 파일은 이제 그리기(draw)와 이벤트 루프만 담당)

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
        if idx == block_idx:
            pygame.draw.circle(screen, (180, 180, 255), (x, y), BIG_DOT_RADIUS + 4, width=2)
            labels.append("BLOCK")
        if labels:
            tag = "/".join(labels)
            screen.blit(font_small.render(tag, True, WHITE), (x + 8, y + 8))

def draw_text(a, d):
    d_clamped = d if 0 < d <= MAX_DIST_CM else 0
    # state에 AI 관련 내용 삭제
    state = ("SWEEP" if active_sweep else ("WAIT_OK" if waiting_ok else "HOLD"))
        
    msg = f"[{state}] Angle: {a:3d}°   Distance: {d_clamped:3d} cm (max {MAX_DIST_CM} cm)"
    screen.blit(font.render(msg, True, WHITE), (20, 20))
    
    # AI 상태 표시 삭제
        
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
    
    # RL 버튼 그리기 삭제
    
    info_y = btn_detect.bottom + 6
    car_info = f"CAR: {'-' if car_idx is None else car_idx}"
    tgt_info = f"TGT: {'-' if target_idx is None else target_idx}"
    blk_info = f"BLK: {'-' if block_idx is None else block_idx}"
    screen.blit(font_small.render(car_info, True, CAR_COLOR), (btn_car.left, info_y))
    screen.blit(font_small.render(tgt_info, True, TARGET_COLOR), (btn_target.left, info_y))
    screen.blit(font_small.render(blk_info, True, WHITE), (btn_block.left, info_y))

def draw_attack_path():
    if not attack_view:
        return
    if car_idx is None or target_idx is None:
        return

    # car_idx, target_idx 등은 radar_core에서 임포트되어 접근 가능
    car = latest_clusters[car_idx]
    pts = [pol_to_xy(car["angle"], car["dist"])]

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

# ---------- 메인 루프 ----------
while True:
    # --- 1. 이벤트 처리 ---
    for event in pygame.event.get():    
        if event.type == pygame.QUIT:
            ser_radar.close()
            ser_bt.close()
            pygame.quit(); sys.exit()
            
        # --- 마우스 클릭 처리 ---
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mx, my = event.pos
            # 버튼 클릭 (btn_car 등은 radar_core에서 임포트됨)
            if btn_car.collidepoint(mx, my):
                select_mode = "CAR"
            elif btn_target.collidepoint(mx, my):
                select_mode = "TARGET"
            elif btn_block.collidepoint(mx, my):
                select_mode = "BLOCK"
            elif btn_detect.collidepoint(mx, my):
                start_detection_sweep() # radar_core의 함수
            elif btn_attack.collidepoint(mx, my):
                do_attack() # radar_core의 함수
            # RL 버튼 클릭 이벤트 삭제
            else:
                # 스윕 종료 후(=HOLD) + 클러스터 클릭 시 지정
                if not active_sweep and show_clusters_now and select_mode is not None:
                    idx, _ = hit_test_cluster(mx, my) # radar_core의 함수
                    if idx is not None:
                        if select_mode == "CAR":
                            car_idx = idx
                        elif select_mode == "TARGET":
                            target_idx = idx
                        elif select_mode == "BLOCK":
                            block_idx = idx

    # --- 스페이스바(대체 트리거) ---
    keys = pygame.key.get_pressed()
    now = time.time()
    if keys[pygame.K_SPACE] and (now - last_space_ts) > 0.25 and not active_sweep:
        last_space_ts = now
        start_detection_sweep() # radar_core의 함수

    # --- 2. 시리얼 수신 (레이더) ---
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

    # --- 3. 시리얼 수신 (블루투스) ---
    for btline in read_nonblocking_lines(ser_bt, buf_bt, max_lines=50):
        up = btline.upper()
        
        # (AI 관련 수신 로직 모두 삭제)
            
        if up == "OK" and waiting_ok:
            info("OK 수신! (6단계 동작은 다음 단계에서 구현)")
            print("[BT RX] OK")
        elif up == "CLEAR":
            print("[BT RX] CLEAR")

    # --- 4. 상태 업데이트 (스윕 및 각도 안정화) ---
    if abs(angle - prev_angle) <= ANGLE_STABLE_TOL:
        angle_stable_count = min(angle_stable_count + 1, STABLE_FRAMES + 5)
    else:
        angle_stable_count = 0

    if angle > prev_angle: sweep_dir = 1
    elif angle < prev_angle: sweep_dir = -1
    prev_angle = angle

    if active_sweep:
        if 0 < distance <= MAX_DIST_CM:
            distance_map[angle] = distance
            if not sweep_points or sweep_points[-1][0] != angle:
                sweep_points.append((angle, distance))
        else:
            distance_map[angle] = 0

    # --- 5. 스윕 종료 처리 ---
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
            print(f"[CLUSTER] pts={n_pts} obj={len(latest_clusters)} ({reason})")

            # (RL 스캔 완료 처리 로직 삭제)
            
            sweep_points = []
            distance_map = [0 for _ in range(181)]
            attack_view = False

    # --- 6. 화면 갱신 (그리기) ---
    screen.fill(BG_FADE)
    draw_radar()
    draw_memory_objects()
    draw_clusters()
    draw_attack_path()  
    draw_line(angle)
    draw_text(angle, distance)
    draw_ui()

    pygame.display.flip()
    clock.tick(30)

