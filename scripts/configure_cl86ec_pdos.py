#!/usr/bin/env python3
"""
Configure CL86EC EtherCAT stepper drive PDOs to the 39B full layout.

Run this ONCE after every drive power-cycle before starting the C++ motor driver.
The configuration is stored in volatile RAM — it resets on power-cycle.

Usage (as root on the robot, using the pysoem venv):
    sudo /home/robot/ethercat_venv/bin/python3 \
        /home/mohamed/Project/mtt_ws/scripts/configure_cl86ec_pdos.py

PDO layout configured:
    RxPDO (SM2, 0x1600, 16B): CW + TargetPos + TargetVel + TouchProbe + DigOut
    TxPDO (SM3, 0x1A00, 39B): ErrorCode + SW + Pos + DigIn + Vel + FollowErr +
                               TP1+ + TP1- + TP2+ + TP2- + ModeDisplay

Without this script (drive in EEPROM default):
    RxPDO = 10B  [CW + TargetPos + TargetVel]
    TxPDO = 10B  [SW + Pos + Vel]
The C++ driver auto-detects and handles the 10B layout too (limited: no error code).
"""

import struct
import sys

try:
    import pysoem
except ImportError:
    print("pysoem not found. Activate the venv:")
    print("  sudo /home/robot/ethercat_venv/bin/python3 <this_script>")
    sys.exit(1)

IFACE = "enp6s0"


def sdo_write(slave, idx, sub, data, label=""):
    try:
        slave.sdo_write(idx, sub, data)
        return True
    except Exception as e:
        print(f"  [WARN] {label or f'0x{idx:04X}:{sub}'}: {e}")
        return False


def configure_pdos(slave):
    # ── RxPDO: SM2 ← 0x1600 (5 entries = 16B) ──────────────────────────────
    sdo_write(slave, 0x1C12, 0, bytes([0x00]),           "1C12:0 clear")
    sdo_write(slave, 0x1600, 0, bytes([0x00]),           "1600:0 clear")

    rxmap = [
        (0x6040, 0, 16),  # ControlWord
        (0x607A, 0, 32),  # TargetPosition
        (0x60FF, 0, 32),  # TargetVelocity
        (0x60B8, 0, 16),  # TouchProbeFunction
        (0x60FE, 1, 32),  # DigitalOutputs
    ]
    for i, (idx, sub, bits) in enumerate(rxmap):
        sdo_write(slave, 0x1600, i + 1,
                  struct.pack("<I", (idx << 16) | (sub << 8) | bits),
                  f"1600:{i+1} = 0x{idx:04X}:{sub:02X} {bits}b")

    sdo_write(slave, 0x1600, 0, bytes([5]),              "1600:0 count=5")
    sdo_write(slave, 0x1C12, 1, struct.pack("<H", 0x1600), "1C12:1 → 0x1600")
    sdo_write(slave, 0x1C12, 0, bytes([1]),              "1C12:0 enable")

    # ── TxPDO: SM3 ← 0x1A00 (12 entries = 39B) ─────────────────────────────
    sdo_write(slave, 0x1C13, 0, bytes([0x00]),           "1C13:0 clear")
    sdo_write(slave, 0x1A00, 0, bytes([0x00]),           "1A00:0 clear")

    txmap = [
        (0x603F, 0, 16),  # ErrorCode
        (0x6041, 0, 16),  # StatusWord
        (0x6064, 0, 32),  # PositionActual
        (0x60FD, 0, 32),  # DigitalInputs
        (0x606C, 0, 32),  # VelocityActual
        (0x60F4, 0, 32),  # FollowingError
        (0x60B9, 0, 16),  # TouchProbeStatus
        (0x60BA, 0, 32),  # TouchProbe1Positive
        (0x60BB, 0, 32),  # TouchProbe1Negative
        (0x60BC, 0, 32),  # TouchProbe2Positive
        (0x60BD, 0, 32),  # TouchProbe2Negative
        (0x6061, 0,  8),  # ModesOfOperationDisplay
    ]
    for i, (idx, sub, bits) in enumerate(txmap):
        sdo_write(slave, 0x1A00, i + 1,
                  struct.pack("<I", (idx << 16) | (sub << 8) | bits),
                  f"1A00:{i+1} = 0x{idx:04X}:{sub:02X} {bits}b")

    sdo_write(slave, 0x1A00, 0, bytes([12]),              "1A00:0 count=12")
    sdo_write(slave, 0x1C13, 1, struct.pack("<H", 0x1A00), "1C13:1 → 0x1A00")
    sdo_write(slave, 0x1C13, 0, bytes([1]),               "1C13:0 enable")


def verify(slave):
    rx_cnt = struct.unpack("B", slave.sdo_read(0x1C12, 0, 1))[0]
    tx_cnt = struct.unpack("B", slave.sdo_read(0x1C13, 0, 1))[0]
    rx_pdo = struct.unpack("<H", slave.sdo_read(0x1C12, 1, 2))[0] if rx_cnt else 0
    tx_pdo = struct.unpack("<H", slave.sdo_read(0x1C13, 1, 2))[0] if tx_cnt else 0
    rx_entries = struct.unpack("B", slave.sdo_read(rx_pdo, 0, 1))[0] if rx_pdo else 0
    tx_entries = struct.unpack("B", slave.sdo_read(tx_pdo, 0, 1))[0] if tx_pdo else 0

    rx_bits = 0
    for j in range(1, rx_entries + 1):
        e = struct.unpack("<I", slave.sdo_read(rx_pdo, j, 4))[0]
        rx_bits += e & 0xFF
    tx_bits = 0
    for j in range(1, tx_entries + 1):
        e = struct.unpack("<I", slave.sdo_read(tx_pdo, j, 4))[0]
        tx_bits += e & 0xFF

    print(f"  SM2 (RxPDO): {rx_cnt} assignment, {rx_entries} entries, {rx_bits}b = {rx_bits//8}B")
    print(f"  SM3 (TxPDO): {tx_cnt} assignment, {tx_entries} entries, {tx_bits}b = {tx_bits//8}B")
    ok = (rx_bits == 128 and tx_bits == 312)
    print(f"  {'✓ OK (16B RxPDO, 39B TxPDO)' if ok else '✗ UNEXPECTED SIZE — check warnings above'}")
    return ok


def main():
    print(f"\n=== CL86EC PDO Configurator ===")
    print(f"Interface: {IFACE}")

    master = pysoem.Master()
    try:
        master.open(IFACE)
    except Exception as e:
        print(f"Cannot open {IFACE}: {e}")
        print("Are you root?")
        sys.exit(1)

    n = master.config_init()
    if n == 0:
        master.close()
        print("No EtherCAT slaves found. Drive powered?")
        sys.exit(1)

    slave = master.slaves[0]
    print(f"Slave: {slave.name}  Vendor=0x{slave.man:08X}  Product=0x{slave.id:08X}\n")

    print("Configuring PDOs...")
    configure_pdos(slave)

    print("\nVerifying:")
    ok = verify(slave)

    master.close()
    if ok:
        print("\nDone. You can now start the C++ motor driver (com_motor container).")
    else:
        print("\nConfiguration may be incomplete. Check warnings above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
