import serial
import time

# --- 설정 ---
BLUETOOTH_PORT = "COM5"  # 2단계에서 확인한 COM 포트 번호
BAUD_RATE = 9600         # 아두이노의 블루투스 보드레이트와 동일하게
# -------------

try:
    # 1. 블루투스 포트에 연결
    ser = serial.Serial(BLUETOOTH_PORT, BAUD_RATE, timeout=1.0)
    print(f"성공: {BLUETOOTH_PORT} 포트에 연결되었습니다. (대기 중...)")
    time.sleep(2) # 연결 안정화를 위한 잠시 대기
    ser.flushInput() # 이전에 쌓인 데이터 비우기

except serial.SerialException as e:
    print(f"오류: {BLUETOOTH_PORT} 포트에 연결할 수 없습니다.")
    print(f"  -> {e}")
    print("  -> 1. 블루투스 페어링을 확인하세요.")
    print("  -> 2. COM 포트 번호가 맞는지 확인하세요.")
    exit()


# 2. "항시 지켜보기" (무한 루프)
try:
    while True:
        # 아두이노로부터 데이터가 왔는지 확인
        if ser.in_waiting > 0:
            
            # 3. 정보 받기 (라인 단위로 읽기)
            # 아두이노에서 .println()으로 보낸 데이터를 \n 기준으로 읽음
            raw_data = ser.readline()
            
            # 'bytes' 타입을 'string'으로 변환 (utf-8 디코딩)
            try:
                data_str = raw_data.decode('utf-8').strip() # .strip()으로 양쪽 공백/개행문자 제거
                print(f"▶ 수신: '{data_str}'")

                # 4. 연산 처리 (예: "v500a203s10" 파싱)
                #    어제 만든 아두이노 코드를 Python 버전으로 구현
                try:
                    param_v = int(data_str[data_str.find('v')+1 : data_str.find('a')])
                    param_a = int(data_str[data_str.find('a')+1 : data_str.find('s')])
                    param_s = int(data_str[data_str.find('s')+1 :])

                    print(f"  ...파싱됨: V={param_v}, A={param_a}, S={param_s}")

                    # --- 여기가 노트북의 연산 처리 부분 ---
                    # 예: V 값은 절반으로, A 값은 10을 더함
                    processed_v = param_v / 2
                    processed_a = param_a + 10
                    # -------------------------------------

                    # 5. 아두이노로 결과 넘겨주기
                    result_str = f"proc_v{processed_v}_a{processed_a}\n" # \n 필수
                    
                    # 'string'을 'bytes'로 변환(인코딩)하여 전송
                    ser.write(result_str.encode('utf-8'))
                    print(f"◀ 전송: '{result_str.strip()}'")
                
                except Exception as e_parse:
                    print(f"  ...파싱 오류: {e_parse} (데이터: {data_str})")
                    ser.write(b"Error: Cannot parse data\n") # 에러 피드백

            except UnicodeDecodeError:
                print(f"▶ 수신 오류: UTF-8 디코딩 실패 (데이터: {raw_data})")
                
        # CPU 자원을 너무 많이 쓰지 않도록 잠시 대기
        time.sleep(0.01) 

except KeyboardInterrupt:
    print("\n프로그램 종료 (Ctrl+C)")

except serial.SerialException as e:
    print(f"\n시리얼 오류: {e}")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("블루투스 연결을 종료했습니다.")
