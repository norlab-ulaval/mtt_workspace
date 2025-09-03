#!/usr/bin/env python3
"""Inject fake 0x2FF tachometer frames onto (v)can for testing the ROS wrapper.

Frame layout (8 bytes):
  Byte0: temp A (int8)
  Byte1: temp B (int8)
  Byte2-3: tachometer instant (uint16 BE)  - raw RPS * scaling (firmware specific)
  Byte4-7: tachometer cumulative (uint32 BE)

Run:
  sudo modprobe vcan
  sudo ip link add dev vcan0 type vcan || true
  sudo ip link set up vcan0
  ros2 launch mtt_driver mtt_teleop.launch.py test_mode:=true &
  python3 mtt_fake_tachometer.py

Stop with Ctrl-C.
"""
import time
import can
import struct
import math

ARB_ID = 0x2FF

def main():
    bus = can.interface.Bus('vcan0', bustype='socketcan')
    cumulative = 0
    start = time.time()
    period = 0.05  # 20 Hz like real device
    rps_base = 40  # arbitrary for testing
    try:
        while True:
            t = time.time() - start
            # Vary speed a bit with a sine wave
            rps = int(max(0, rps_base + 10 * math.sin(t)))
            cumulative += rps  # simplistic accumulation
            temp_a = 30 + int(5 * math.sin(t / 10))
            temp_b = 31 + int(5 * math.cos(t / 12))
            frame = struct.pack('>bbH I', temp_a, temp_b, rps, cumulative)
            msg = can.Message(arbitration_id=ARB_ID, data=frame, is_extended_id=False)
            try:
                bus.send(msg)
            except can.CanError as e:
                print(f"Send failed: {e}")
            time.sleep(period)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
