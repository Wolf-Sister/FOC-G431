"""Quick diagnostic: connect, dump telemetry, test commands.

Usage: python tools/debug_serial.py COM5
"""

import sys
import time
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tools.serial_iface import SerialInterface


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    if port is None:
        ports = SerialInterface.list_ports()
        if not ports:
            print("No serial ports found.")
            return
        port = ports[0]
        print(f"Auto-detected: {port}")

    print(f"Opening {port}...")
    iface = SerialInterface(port)

    print("Connecting (waiting for telemetry)...")
    if not iface.connect(timeout_s=5.0):
        print("FAILED: No telemetry received. Check:")
        print("  1. MCU is powered and in closed-loop mode (Phase 2)")
        print("  2. USB-TTL TX -> PA15, RX -> PB3, GND -> GND")
        print("  3. Baud rate is 115200")
        return

    print("Connected! Dumping 10 telemetry frames...\n")
    for i in range(10):
        frame = iface.read_telemetry(timeout_s=1.0)
        if frame:
            print(f"[{i:2d}] mode={frame['mode']:.0f}  "
                  f"iq_target={frame['iq_target']:+.3f}  "
                  f"iq_meas={frame['iq_meas']:+.3f}  "
                  f"vq_cmd={frame['vq_cmd']:+.3f}  "
                  f"status={frame['status_flag']:.0f}")
        else:
            print(f"[{i:2d}] TIMEOUT — no frame received")

    print("\nSending T=0.5 command...")
    iface.send_cmd("T=0.5")
    if iface.wait_for_flag(timeout_s=2.0):
        print("Flag received! Command acknowledged by MCU.")
    else:
        print("WARNING: No flag — MCU may not have received command.")

    time.sleep(0.2)
    print("\nPost-command frames:")
    for i in range(5):
        frame = iface.read_telemetry(timeout_s=0.5)
        if frame:
            print(f"  iq_target={frame['iq_target']:+.3f}  "
                  f"iq_meas={frame['iq_meas']:+.3f}  "
                  f"status={frame['status_flag']:.0f}")
        else:
            print(f"  TIMEOUT")

    iface.send_cmd("T=0.0")
    iface.disconnect()
    print("\nDone.")


if __name__ == "__main__":
    main()
