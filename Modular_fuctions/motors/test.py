#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""
Pico 舵机 & 激光笔 简单测试脚本
用法：
    python test_pico.py             # 进入交互菜单
    python test_pico.py 0           # 直接发命令 0
"""
import sys
import time
import serial

# ===== Pico 命令字节定义（与主程序完全一致，必须带 \n）=====
CMD_SERVO_RED   = b'0\n'   # 红色 -> 舵机0
CMD_SERVO_BLUE  = b'1\n'   # 蓝色 -> 舵机1
CMD_SERVO_GREEN = b'2\n'   # 绿色 -> 舵机2
CMD_LASER_ON    = b'3\n'   # 开启激光笔
CMD_LASER_OFF   = b'4\n'   # 关闭激光笔

# ===== 串口配置 =====
# 请根据实际情况修改，Pico 一般是 /dev/ttyACM0 或 /dev/ttyACM1
PORT = '/dev/ttyACM0'
BAUD = 115200


def open_serial(port=PORT, baud=BAUD):
    for p in [port, '/dev/ttyACM0', '/dev/ttyACM1',
              '/dev/ttyUSB0', '/dev/ttyUSB1']:
        try:
            s = serial.Serial(p, baud, timeout=1)
            time.sleep(0.5)
            print(f"[OK] 已连接 Pico: {p}")
            return s
        except Exception as e:
            print(f"[WARN] 尝试 {p} 失败: {e}")
    print("[ERR] 无法打开任何串口！")
    return None


def send_cmd(ser, cmd):
    if ser is None or not ser.is_open:
        print("[ERR] 串口未打开")
        return
    ser.write(cmd)
    ser.flush()
    print(f"[TX] -> {cmd!r}")
    # 接收 Pico 回显
    time.sleep(0.1)
    while ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                print(f"[RX] <- {line}")
        except Exception:
            break


def main():
    ser = open_serial()
    if ser is None:
        sys.exit(1)

    # 如果命令行直接给了参数，则只发一次
    if len(sys.argv) == 2:
        cmd_map = {
            '0': CMD_SERVO_RED,
            '1': CMD_SERVO_BLUE,
            '2': CMD_SERVO_GREEN,
            '3': CMD_LASER_ON,
            '4': CMD_LASER_OFF,
        }
        c = sys.argv[1]
        if c in cmd_map:
            send_cmd(ser, cmd_map[c])
        else:
            print(f"未知命令: {c}")
        ser.close()
        return

    # 否则进入交互菜单
    menu = """
================ Pico 测试菜单 ================
  0 : 舵机0 转150° 停5s 回0° (RED)
  1 : 舵机1 转150° 停5s 回0° (BLUE)
  2 : 舵机2 转150° 停5s 回0° (GREEN)
  3 : 开启激光笔
  4 : 关闭激光笔
  q : 退出
================================================
"""
    while True:
        print(menu)
        c = input("请输入命令 > ").strip()
        if c == 'q' or c == 'Q':
            break
        elif c == '0':
            send_cmd(ser, CMD_SERVO_RED)
        elif c == '1':
            send_cmd(ser, CMD_SERVO_BLUE)
        elif c == '2':
            send_cmd(ser, CMD_SERVO_GREEN)
        elif c == '3':
            send_cmd(ser, CMD_LASER_ON)
        elif c == '4':
            send_cmd(ser, CMD_LASER_OFF)
        else:
            print("未知命令")

    ser.close()
    print("Bye~")


if __name__ == '__main__':
    main()