from machine import Pin, PWM
import time
import sys
import select

# ===== LED 指示灯 =====
led = Pin(25, Pin.OUT)

# ===== 舵机初始化 =====
servo0 = PWM(Pin(0))   # 红
servo1 = PWM(Pin(1))   # 蓝
servo2 = PWM(Pin(2))   # 绿
servo0.freq(50)
servo1.freq(50)
servo2.freq(50)

# ===== 激光笔 =====
laser = Pin(3, Pin.OUT)

# ===== 舵机参数 =====
MIN_DUTY = 1638   # 0°
MAX_DUTY = 7864   # 180°

def set_servo_angle(pwm, angle):
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    duty = int(MIN_DUTY + (angle / 180.0) * (MAX_DUTY - MIN_DUTY))
    pwm.duty_u16(duty)

# ===== 串口轮询 =====
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

# ===== 初始化 =====
laser.off()
led.on()
time.sleep(0.2)
led.off()

print("Pico 串口控制已就绪（行模式协议 \\n）")
print("命令: 0=舵机0红, 1=舵机1蓝, 2=舵机2绿, 3=开激光, 4=关激光")

# ===== 主循环 =====
while True:
    if poll.poll(10):
        try:
            line = sys.stdin.readline()
        except Exception:
            line = None
        if not line:
            continue
        line = line.strip()
        if not line:
            continue

        cmd = line[0]

        if cmd == '0':
            led.on()
            set_servo_angle(servo0, 150)
            print("舵机0 -> 150°")
            time.sleep(5)
            set_servo_angle(servo0, 0)
            print("舵机0 -> 0°")
            led.off()

        elif cmd == '1':
            led.on()
            set_servo_angle(servo1, 150)
            print("舵机1 -> 150°")
            time.sleep(5)
            set_servo_angle(servo1, 0)
            print("舵机1 -> 0°")
            led.off()

        elif cmd == '2':
            led.on()
            set_servo_angle(servo2, 150)
            print("舵机2 -> 150°")
            time.sleep(5)
            set_servo_angle(servo2, 0)
            print("舵机2 -> 0°")
            led.off()

        elif cmd == '3':
            laser.on()
            print("激光笔已开启")

        elif cmd == '4':
            laser.off()
            print("激光笔已关闭")

        else:
            print("未知命令:", line)

    time.sleep(0.01)