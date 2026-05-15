import cv2
import numpy as np
import math
import serial
import time

# ==========================================
#           1. ตั้งค่าเริ่มต้น (CONFIGURATION)
# ==========================================
WS, HS = 600, 500              # ขนาดหน้าจอที่แสดงผล
PORT = '/dev/serial0'          # พอร์ตเชื่อมต่อ Serial กับ Pico
BAUDRATE = 115200
STEPS_PER_DEG = 126.67         # อัตราส่วน Step มอเตอร์ต่อ 1 องศา

CAMERA_CONSTANT = 5800.0       # ค่าคงที่สำหรับคำนวณระยะห่าง (Depth)
colorLower = (25, 50, 70)      # ช่วงสีเหลือง (HSV) ขอบเขตล่าง
colorUpper = (50, 255, 255)    # ช่วงสีเหลือง (HSV) ขอบเขตบน

# ข้อมูลทางกายภาพของหุ่นยนต์ (หน่วยเป็น มม.) สำหรับคำนวณ Kinematics
a, b, c, d, e, f, g, h = 68.50, 40.98, 63.49, 68.42, 63.80, 107.15, -2.62, 60.68
LASER_Y_OFFSET = -a + c - e    # ระยะเยื้องของแกนยิงในแนวแกน Y
LASER_Z_OFFSET = b + g + d + h  # ระยะเยื้องของแกนยิงในแนวแกน Z
TENNIS_BALL_DIA_MM = 67.0      # ขนาดจริงของลูกเทนนิส

# ค่าชดเชยสำหรับการเล็ง (Tuning)
TUNE_PAN, TUNE_TILT = -0.04, -0.5  

# ==========================================
#           2. ฟังก์ชันหลัก (CORE FUNCTIONS)
# ==========================================

def calculate_target_angles(Y_c, Z_c, depth_cm):
    """
    ฟังก์ชันคำนวณหาองศาของแกน Pan และ Tilt จากพิกัดภาพและระยะห่าง
    """
    X_c = depth_cm * 10.0      # แปลงหน่วยจาก ซม. เป็น มม.
    P_cam = np.array([X_c, Y_c, Z_c, 1.0])
    
    T_PC = np.array([[1, 0, 0, f], [0, 1, 0, LASER_Y_OFFSET], [0, 0, 1, b + g + d], [0, 0, 0, 1]])
    P_base = T_PC @ P_cam
    XT, YT, ZT = P_base[0], P_base[1], P_base[2]
    
    # คำนวณองศา Pan (แนวราบ)
    R_xy = math.hypot(XT, YT)
    if R_xy < abs(LASER_Y_OFFSET): return None, None
    theta1_rad = math.atan2(-XT, YT) + math.acos(LASER_Y_OFFSET / R_xy)
    
    # คำนวณองศา Tilt (แนวดิ่ง)
    X_prime = XT * math.cos(theta1_rad) + YT * math.sin(theta1_rad)
    B_val, C_val = ZT - b, LASER_Z_OFFSET - b
    R_xz = math.hypot(X_prime, B_val)
    if R_xz < abs(C_val): return math.degrees(theta1_rad), None
    theta2_rad = math.atan2(X_prime, B_val) - math.acos(C_val / R_xz)
    
    pan_comp = math.degrees(math.atan2(TUNE_PAN * TENNIS_BALL_DIA_MM, X_c))
    tilt_comp = math.degrees(math.atan2(TUNE_TILT * TENNIS_BALL_DIA_MM, X_c))
    
    return math.degrees(theta1_rad) - pan_comp, math.degrees(theta2_rad) - tilt_comp

def wait_for_done(ser):
    """ฟังก์ชันรอการตอบกลับ 'DONE' จาก Pico เพื่อยืนยันว่าทำงานเสร็จแล้ว"""
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "DONE" in line:
                return True
        time.sleep(0.01)

# ==========================================
#           3. ลูปการทำงานหลัก (MAIN LOOP)
# ==========================================

def run_sentry_system():
    cap = None
    ser = None

    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        ser.flush()
        print("[CONNECTED] Motion 2350 Online.")

        print("[SYSTEM] Waiting for Homing...")
        wait_for_done(ser)
        print("[READY] Homing Complete. Control Mode Active.")

        cap = cv2.VideoCapture(0)
        cap.set(3, WS)
        cap.set(4, HS)
        cx, cy = WS / 2.0, HS / 2.0
        
        is_aimed = False

        while True:
            ret, frame = cap.read()
            if not ret: break
            frame = cv2.flip(frame, 1)
            
            # --- ส่วนการประมวลผลภาพ (Detection) ---
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, colorLower, colorUpper)
            mask = cv2.dilate(cv2.erode(mask, None, iterations=2), None, iterations=2)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            ball_detected = None
            if len(cnts) > 0:
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                if radius > 15:
                    ball_detected = (x, y, radius)
                    cv2.circle(frame, (int(x), int(y)), int(radius), (255, 255, 255), 2)

            # --- การแสดงผล UI บนหน้าจอ ---
            status_color = (0, 255, 0) if is_aimed else (0, 255, 255)
            status_text = "AIMED - F: FIRE | B: BURST" if is_aimed else "IDLE - PRESS 'L' TO LOCK"
            cv2.putText(frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            cv2.putText(frame, "Q: Quit | L: Lock | F: Fire | B: Burst", (20, HS-50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            cv2.imshow('Gunner Station', frame)
            key = cv2.waitKey(1) & 0xFF

            # --- คำสั่ง 1: ล็อคเป้าหมาย (กดปุ่ม 'L') ---
            if key == ord('l') and ball_detected:
                print("[ACTION] Locking Target (0.5s Rapid Sample)...")
                pan_samples, tilt_samples = [], []
                
                for _ in range(15):
                    success, f = cap.read()
                    if not success: continue
                    f = cv2.flip(f, 1)
                    l_hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
                    l_mask = cv2.inRange(l_hsv, colorLower, colorUpper)
                    l_cnts, _ = cv2.findContours(l_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    
                    if l_cnts:
                        lc = max(l_cnts, key=cv2.contourArea)
                        ((lx, ly), lr) = cv2.minEnclosingCircle(lc)
                        if lr > 10:
                            depth = CAMERA_CONSTANT / (lr * 2)
                            mm_px = TENNIS_BALL_DIA_MM / (lr * 2)
                            p, t = calculate_target_angles((lx-cx)*mm_px, -(ly-cy)*mm_px, depth)
                            if p is not None:
                                pan_samples.append(p); tilt_samples.append(t)
                
                if pan_samples:
                    avg_p, avg_t = sum(pan_samples)/len(pan_samples), sum(tilt_samples)/len(tilt_samples)
                    sx, sy = int(avg_p * STEPS_PER_DEG), int(-avg_t * STEPS_PER_DEG)
                    ser.write(f"X{sx}Y{sy}\n".encode())
                    print(f"[MOVE] Sent X:{sx} Y:{sy}.")
                    wait_for_done(ser)
                    is_aimed = True
                    print("[READY] Target Locked.")

            # --- คำสั่ง 2: ยิงทีละนัด (กดปุ่ม 'F') ---
            elif key == ord('f'):
                if is_aimed:
                    print("[FIRE] Dispatching Single Shot...")
                    ser.write(b"F\n") 
                    wait_for_done(ser) 
                    print("[SUCCESS] Shot Finished.")
                    is_aimed = False
                else:
                    print("[WARNING] Lock target first!")

            # --- คำสั่ง 3: ยิงรัว (กดปุ่ม 'B') ---
            elif key == ord('b'):
                if is_aimed:
                    print("[FIRE] Dispatching 3-Round Burst...")
                    ser.write(b"B\n") 
                    wait_for_done(ser)
                    print("[SUCCESS] Burst Finished.")
                    is_aimed = False
                else:
                    print("[WARNING] Lock target first!")

            elif key == ord('q'):
                break

    except Exception as e:
        print(f"[CRASH] {e}")

    finally:
        print("quiting...")
        if cap: cap.release()
        if ser: ser.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    run_sentry_system()
