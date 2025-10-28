# hardware_manager.py
import serial
import struct
import time

class HardwareManager:
    """
    모든 COM 포트 통신을 전담하는 클래스.
    이 클래스만 'serial'과 'struct' 라이브러리를 사용합니다.
    """
    def __init__(self, radar_port='COM10', bt_port='COM9', baudrate=115200):
        self.ser_radar = None
        self.ser_bt = None
        self.buf_radar = bytearray()
        self.buf_bt = bytearray()

        try:
            # 레이더 포트 (비블로킹 읽기)
            self.ser_radar = serial.Serial(radar_port, baudrate, timeout=0)
            print(f"성공: 레이더 포트({radar_port}) 연결됨")
        except serial.SerialException as e:
            print(f"오류: 레이더 포트({radar_port}) 연결 실패: {e}")

        try:
            # 블루투스 포트 (비블로킹 읽기, 쓰기 타임아웃)
            self.ser_bt = serial.Serial(bt_port, baudrate, timeout=0, write_timeout=0.2)
            print(f"성공: BT 포트({bt_port}) 연결됨")
        except serial.SerialException as e:
            print(f"오류: BT 포트({bt_port}) 연결 실패: {e}")

    def is_ready(self):
        """두 포트가 모두 열렸는지 확인"""
        return self.ser_radar is not None and self.ser_bt is not None

    def _read_nonblocking_lines(self, ser, buf, max_lines=32):
        """
        비블로킹으로 시리얼 포트에서 라인을 읽어 리스트로 반환 (내부 헬퍼)
        """
        lines = []
        if not ser:
            return lines
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
            pass # 포트 오류 무시
        return lines

    def _build_packet(self, speed, angle, status):
        """
        블루투스로 전송할 4바이트 바이너리 패킷 생성 (내부 헬퍼)
        """
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

    # --- Public API 메서드 ---

    def poll_radar_data(self):
        """
        레이더 포트를 폴링하여 (각도, 거리) 튜플 리스트를 반환합니다.
        """
        parsed_data = []
        lines = self._read_nonblocking_lines(self.ser_radar, self.buf_radar, max_lines=50)
        for line in lines:
            if ',' in line:
                try:
                    a_str, d_str = line.split(',', 1)
                    a = int(a_str)
                    d = int(d_str)
                    if 0 <= a <= 180:
                        parsed_data.append((a, d))
                except ValueError:
                    pass # 파싱 실패 무시
        return parsed_data

    def poll_bt_text(self):
        """
        블루투스 포트를 폴링하여 'OK', 'CLEAR' 같은 텍스트 메시지 리스트를 반환합니다.
        """
        text_messages = []
        lines = self._read_nonblocking_lines(self.ser_bt, self.buf_bt, max_lines=50)
        for line in lines:
            up_msg = line.upper().strip()
            if up_msg:
                text_messages.append(up_msg)
        return text_messages

    def read_bt_binary_status(self, timeout=10.0):
        """
        'do_attack' 중 사용되는 *블로킹* 바이너리 수신 메서드.
        status=1 응답을 기다립니다.
        """
        if not self.ser_bt:
            return None
            
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                if self.ser_bt.in_waiting >= 4:
                    data = self.ser_bt.read(4)
                    val = struct.unpack('>I', data)[0]
                    status = (val >> 9) & 0x1
                    return status
            except Exception as e:
                print(f"BT 바이너리 읽기 오류: {e}")
                return None # 오류 발생 시 즉시 종료
            time.sleep(0.05) # CPU 사용량 조절
            
        print("BT 바이너리 응답 시간 초과")
        return None # 타임아웃

    def start_detection_sweep(self):
        """레이더에 스윕 시작 명령('S') 전송"""
        if self.ser_radar:
            try:
                self.ser_radar.write(b'S')
                print("[HW TX RADAR] start sweep (S)")
                return True
            except Exception as e:
                print(f"레이더 쓰기 오류: {e}")
        return False

    def send_attack_segment(self, speed, angle, status):
        """'do_attack'에서 계산된 세그먼트(패킷) 전송"""
        if not self.ser_bt:
            return False
            
        pkt_val = self._build_packet(speed, angle, status)
        pkt_bytes = struct.pack('>I', pkt_val)
        
        try:
            self.ser_bt.write(pkt_bytes)
            print(f"[HW BT TX] SEG: speed={speed}, angle={angle}, status={status}")
            return True
        except Exception as e:
            print(f"BT 세그먼트 전송 실패: {e}")
            return False

    def send_attack_end(self):
        """(주석 처리된 원본 코드 기준) 공격 종료 명령 전송"""
        if not self.ser_bt:
            return False
        try:
            # 원본 코드에서는 주석처리 되어 있었으나, 필요시 활성화
            # self.ser_bt.write(b"END\n")
            print("[HW BT TX] END (명령 전송 시뮬레이션)")
            return True
        except Exception as e:
            print(f"BT END 전송 실패: {e}")
            return False
            
    def close(self):
        """모든 시리얼 포트를 닫습니다."""
        if self.ser_radar:
            self.ser_radar.close()
            print("레이더 포트 닫힘")
        if self.ser_bt:
            self.ser_bt.close()
            print("BT 포트 닫힘")
