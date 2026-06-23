#!/usr/bin/env python3
"""
Minimal standalone CL86EC test — no ROS, direct pysoem.
Prints EC/SW every step, tries fault reset, then enables and moves.

Usage (as root in Docker):
    python3 /home/mohamed/Project/mtt_ws/scripts/test_cl86ec.py

If the motor moves: drive + wiring are OK, issue is in the ROS C++ driver.
If it still shows EC=0x0231: hardware fault (wiring/current/motor issue).
"""
import struct, time, sys

try:
    import pysoem
except ImportError:
    print("pysoem not found. Install with: pip3 install pysoem")
    sys.exit(1)

IFACE = "enp6s0"   # same NIC as the C++ driver

CW_FAULT_RESET = 0x0080
CW_READY       = 0x0006   # Shutdown → Ready to switch on
CW_SWITCHED    = 0x0007   # Switch on
CW_ENABLED     = 0x000F   # Enable operation

STATES = {
    0x0000: "Not ready to switch on",
    0x0040: "Switch on disabled",
    0x0021: "Ready to switch on",
    0x0023: "Switched on",
    0x0027: "Operation enabled",
    0x0007: "Quick stop active",
    0x000F: "Fault reaction active",
    0x0008: "Fault",
}

def state_name(sw):
    return STATES.get(sw & 0x006F, f"Unknown (0x{sw & 0x006F:04X})")

def cycle(slave, master, cw, pos=0, vel=0):
    slave.output = struct.pack("<HiiHI", cw & 0xFFFF, pos, vel, 0, 0)
    master.send_processdata()
    master.receive_processdata(2000)
    d = slave.input
    if d and len(d) >= 8:
        ec  = struct.unpack_from("<H", d, 0)[0]
        sw  = struct.unpack_from("<H", d, 2)[0]
        pos = struct.unpack_from("<i", d, 4)[0]
        return ec, sw, pos
    return 0xFFFF, 0x0000, 0

def run_cycles(slave, master, cw, n, pos=0, vel=0):
    for _ in range(n):
        ec, sw, p = cycle(slave, master, cw, pos, vel)
        time.sleep(0.001)
    return ec, sw, p

# ── Connect ──
print(f"\n=== CL86EC Direct Test ===")
print(f"Opening {IFACE}...")
master = pysoem.Master()
try:
    master.open(IFACE)
except Exception as e:
    print(f"Cannot open {IFACE}: {e}")
    print("Are you root? Is the EtherCAT cable connected?")
    sys.exit(1)

n = master.config_init()
if n == 0:
    master.close()
    print("No EtherCAT slaves found. Drive powered?")
    sys.exit(1)

s = master.slaves[0]
print(f"Slave: {s.name}  Vendor=0x{s.man:08X}  Product=0x{s.id:08X}")

try:
    master.config_map()
    master.config_dc()
except Exception as e:
    print(f"config_map/dc failed: {e}")

st = master.state_check(pysoem.SAFEOP_STATE, 5_000_000)
print(f"SAFE-OP: {'OK' if st == pysoem.SAFEOP_STATE else 'TIMEOUT'}")

master.state = pysoem.OP_STATE
master.write_state()
st = master.state_check(pysoem.OP_STATE, 5_000_000)
print(f"OP:      {'OK' if st == pysoem.OP_STATE else 'TIMEOUT (continuing anyway)'}")

# ── Initial status ──
ec, sw, pos = run_cycles(s, master, 0, 20)
print(f"\n--- Initial state ---")
print(f"  EC=0x{ec:04X}  SW=0x{sw:04X}  state='{state_name(sw)}'  pos={pos}")
if ec != 0:
    print(f"  ⚠  ERROR CODE 0x{ec:04X} active at startup!")
    print(f"     0x0231 = Short circuit / phase current fault → check motor wiring")
    print(f"     0x6010 = EtherCAT watchdog → communication issue")
    print(f"     0x3120 = Under voltage")

# ── Fault reset ──
print(f"\n--- Fault reset ---")
run_cycles(s, master, 0x0000, 100)
run_cycles(s, master, CW_FAULT_RESET, 300)
run_cycles(s, master, 0x0000, 200)
ec, sw, pos = run_cycles(s, master, 0, 10)
print(f"  EC=0x{ec:04X}  SW=0x{sw:04X}  state='{state_name(sw)}'")

if ec != 0:
    print(f"  ✗ Fault 0x{ec:04X} NOT cleared by software reset → hardware condition persists")
else:
    print(f"  ✓ No error code after reset")

# ── CiA 402 enable (aggressive — send CW_ENABLED regardless of intermediate states) ──
print(f"\n--- Enable sequence ---")
seed = pos
print(f"  Seed position: {seed}")

# Send Shutdown then wait up to 5s for Ready to switch on
run_cycles(s, master, CW_READY, 50, seed)
for i in range(5000):
    ec, sw, p = cycle(s, master, CW_READY, seed)
    if state_name(sw) in ("Ready to switch on", "Switched on", "Operation enabled"):
        print(f"  t={i}ms → '{state_name(sw)}'  EC=0x{ec:04X}")
        break
    if i % 1000 == 0:
        print(f"  t={i}ms: SW=0x{sw:04X} '{state_name(sw)}'  EC=0x{ec:04X}")
    time.sleep(0.001)
else:
    print(f"  ✗ Did not reach 'Ready to switch on' in 5s. SW=0x{sw:04X} EC=0x{ec:04X}")

# Switch on
run_cycles(s, master, CW_SWITCHED, 100, seed)
# Enable operation
run_cycles(s, master, CW_ENABLED,  100, seed)
ec, sw, pos = run_cycles(s, master, CW_ENABLED, 50, seed)
print(f"  After CW_ENABLED: SW=0x{sw:04X} '{state_name(sw)}'  EC=0x{ec:04X}")

enabled = bool(sw & 0x0004)  # SW_OPERATION_ENABLED
print(f"  ENABLED: {'✓ YES' if enabled else '✗ NO'}")

# ── Move test ──
if enabled:
    target = seed + 5000
    print(f"\n--- Move test: {seed} → {target} (+5000 counts) ---")
    for i in range(3000):
        ec, sw, p = cycle(s, master, CW_ENABLED, target)
        if i % 500 == 0:
            print(f"  t={i}ms: pos={p}  vel={struct.unpack_from('<i', s.input, 12)[0]}  SW=0x{sw:04X}")
        time.sleep(0.001)
    print(f"  Final pos={p}  (expected ~{target})")
    if abs(p - target) < 100:
        print("  ✓ Motor moved correctly!")
    else:
        print(f"  ✗ Motor did not reach target (error={p - target} counts)")
else:
    print("\n⚠  Drive not enabled — cannot test movement.")
    print("   Fix EC=0x0231 first:")
    print("   1. Power OFF the CL86EC drive")
    print("   2. Check motor wiring: A+/A- and B+/B- (multimeter: ~few ohms each coil)")
    print("   3. Power ON drive and wait for LED to stop flashing")
    print("   4. Run this script again")

# ── Cleanup ──
print("\n--- Cleanup ---")
run_cycles(s, master, 0, 100)
master.state = pysoem.INIT_STATE
master.write_state()
master.close()
print("Done.")
