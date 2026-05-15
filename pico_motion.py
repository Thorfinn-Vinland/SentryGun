from machine import Pin, UART
import time
import math

# --- 1. การตั้งค่าขาเชื่อมต่อ (SETUP) ---
PAN_STEP, PAN_DIR = Pin(0, Pin.OUT), Pin(1, Pin.OUT)
TILT_STEP, TILT_DIR = Pin(16, Pin.OUT), Pin(17, Pin.OUT)
PAN_LIM, TILT_LIM = Pin(26, Pin.IN, Pin.PULL_DOWN), Pin(27, Pin.IN, Pin.PULL_DOWN)
FIRE_PIN = Pin(8, Pin.OUT)  # ขาควบคุมการยิง
uart = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5), timeout=100)

# ค่าคงที่สำหรับการเคลื่อนที่
STEPS_PER_DEG = 126.67
MIN_DELAY, MAX_DELAY = 250, 500  # หน่วงเวลา Step (น้อย=เร็ว, มาก=ช้า)
curr_p, curr_t = 0, 0            # ตำแหน่งปัจจุบัน
LIMITS = {'P_POS': 11400, 'P_NEG': -11400, 'T_POS': 3800, 'T_NEG': -5320}  # ขีดจำกัดองศา

@micropython.native  # สั่งให้ทำงานด้วยความเร็วสูงสุด
def pulse(p, t, delay):
    """ส่งสัญญาณ Pulse ให้มอเตอร์หมุน 1 Step"""
    if p: PAN_STEP.value(1)
    if t: TILT_STEP.value(1)
    time.sleep_us(20)  # ยืนยันสัญญาณคงที่
    PAN_STEP.value(0); TILT_STEP.value(0)
    time.sleep_us(int(delay))

def move_s_curve(rel_x, rel_y):
    """
    ควบคุมมอเตอร์ให้ขยับแบบ S-Curve: เริ่มต้นช้า -> เร็วตรงกลาง -> ช้าตอนจบ
    ช่วยลดการสั่นสะเทือนของโครงสร้างหุ่นยนต์
    """
    global curr_p, curr_t
    
    # ตรวจสอบไม่ให้ขยับเกินขีดจำกัด (Clamp)
    tp = max(min(curr_p + rel_x, LIMITS['P_POS']), LIMITS['P_NEG'])
    tt = max(min(curr_t + rel_y, LIMITS['T_POS']), LIMITS['T_NEG'])
    ax, ay = int(tp - curr_p), int(tt - curr_t)
    
    if ax == 0 and ay == 0: return
    print(f"[MOVE] X:{ax} Y:{ay}")

    # กำหนดทิศทางการหมุน (Direction)
    PAN_DIR.value(0 if ax > 0 else 1)
    TILT_DIR.value(0 if ay > 0 else 1)
    
    total_steps = max(abs(ax), abs(ay))
    
    # ลูปคำนวณการหน่วงเวลาแบบ S-Curve
    for i in range(total_steps):
        progress = i / total_steps
        # ใช้ฟังก์ชัน cos เพื่อสร้างกราฟความเร็วที่นุ่มนวล
        s_multiplier = (math.cos(progress * math.pi * 2) + 1) / 2
        dynamic_delay = MIN_DELAY + (s_multiplier * (MAX_DELAY - MIN_DELAY))
        pulse(i < abs(ax), i < abs(ay), dynamic_delay)

    curr_p, curr_t = tp, tt

def fire(count):
    """
    ควบคุมการยิง โดยส่งสัญญาณไปที่ FIRE_PIN
    count: จำนวนนัดที่ต้องการยิง
    """
    for i in range(count):
        FIRE_PIN.value(1); time.sleep(0.16)  # ปล่อยสัญญาณกระตุ้นการยิง
        FIRE_PIN.value(0); time.sleep(0.05 if i < count-1 else 0)

def run_homing():
    """
    โหมดหาจุดเริ่มต้น (Homing)
    หุ่นยนต์จะหมุนไปจนชน Limit Switch เพื่อเซ็ตพิกัด (0, 0)
    """
    print("Homing..."); PAN_DIR.value(1); TILT_DIR.value(1)
    p_l = t_l = False
    while not (p_l and t_l):
        if PAN_LIM.value(): p_l = True
        if TILT_LIM.value(): t_l = True
        pulse(not p_l, not t_l, 400)
    
    time.sleep(0.5); PAN_DIR.value(0); TILT_DIR.value(0)
    # หมุนกลับมาที่จุดศูนย์กลางหลังชนสวิตช์
    for i in range(13933): pulse(i < 13933, i < 5550, 300)
    global curr_p, curr_t; curr_p, curr_t = 0, 0

# --- 2. ลูปรับคำสั่ง (MAIN LOOP) ---
try:
    run_homing()
    # แจ้งเตือน Pi 4 ว่า Homing เสร็จแล้ว
    [uart.write(b"HOMING_DONE\n") or time.sleep(0.2) for _ in range(3)]
    
    while True:
        if uart.any():
            time.sleep_ms(10)
            raw = uart.readline()
            if not raw: continue
            try:
                cmd = raw.decode().strip()
                if 'X' in cmd:  # คำสั่งเคลื่อนที่ เช่น "X100Y50"
                    parts = cmd.split('Y')
                    move_s_curve(int(parts[0][1:]), int(parts[1]))
                elif cmd == 'F': fire(1)  # คำสั่งยิงนัดเดียว
                elif cmd == 'B': fire(3)  # คำสั่งยิง 3 นัด
                uart.write(b"DONE\n")     # แจ้งกลับว่าทำงานเสร็จแล้ว
            except: uart.write(b"ERROR\n")
except:
    FIRE_PIN.value(0)  # กรณีเกิด Error ให้หยุดยิงทันทีเพื่อความปลอดภัย
