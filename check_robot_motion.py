#!/usr/bin/env python3
"""診斷腳本：檢查機器人TCP位置變化"""
import time
import math
from rtde_receive import RTDEReceiveInterface

robot_ip = "192.168.0.200"
rtde_r = RTDEReceiveInterface(robot_ip, 500)

print("正在監測機器人TCP位置變化...")
print("請確保機器人已停止移動\n")

history = []
window_size = 200

while True:
    tcp = rtde_r.getActualTCPPose()
    tcp6 = (float(tcp[0]), float(tcp[1]), float(tcp[2]),
            float(tcp[3]), float(tcp[4]), float(tcp[5]))

    history.append(tcp6)
    if len(history) > window_size:
        history.pop(0)

    if len(history) >= window_size:
        xs = [p[0] for p in history]
        ys = [p[1] for p in history]
        zs = [p[2] for p in history]
        rxs = [p[3] for p in history]
        rys = [p[4] for p in history]
        rzs = [p[5] for p in history]

        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        dz = max(zs) - min(zs)
        dpos = math.sqrt(dx*dx + dy*dy + dz*dz)

        drx = max(rxs) - min(rxs)
        dry = max(rys) - min(rys)
        drz = max(rzs) - min(rzs)
        drot = math.sqrt(drx*drx + dry*dry + drz*drz)

        stationary = (dpos <= 0.0001) and (drot <= 0.003)
        status = "✓ 靜止" if stationary else "✗ 移動中"

        print(f"\r{status} | 位置變化: {dpos*1000:.4f} mm | 旋轉變化: {drot:.5f} rad ({math.degrees(drot):.3f}°)", end="", flush=True)

    time.sleep(0.002)  # 500 Hz
