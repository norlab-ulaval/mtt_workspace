#!/usr/bin/env python3
"""
Script de test d'articulation autonome avancé (Stress Test Thermique & Calibration).
Intègre la recherche 100% automatique des butées et corrige le bug majeur du mode CloseLoop.
"""

import time
import math
import struct
import threading
import argparse
import csv
import datetime
import can

try:
    import serial
except ImportError:
    print("Erreur : la librairie 'pyserial' est requise pour lire le STM32.")
    exit(1)

# ─── LUT DU STM32 ────────────────────────────────────────────────────────────
yaw_bit_coords = [
    82, 214, 313, 430, 510, 600, 723, 831, 929, 1028, 1128, 1201, 1306, 1410,
    1529, 1628, 1737, 1826, 1906, 2016, 2119, 2226, 2337, 2443, 2520, 2617,
    2716, 2815, 2908, 3052, 3106, 3258, 3323, 3408
]
yaw_angle_coords_deg = [
    48, 43, 40, 36, 35, 32, 29, 25, 23, 20, 18, 15, 12, 9, 5, 2, 0, -2, -4,
    -7, -8, -10, -14, -18, -20, -21, -28, -26, -30, -35, -40, -43, -45, -49
]

def interpolate_lut(x, x_coords, y_coords):
    if x <= x_coords[0]: return y_coords[0]
    if x >= x_coords[-1]: return y_coords[-1]
    for i in range(len(x_coords) - 1):
        if x_coords[i] <= x <= x_coords[i+1]:
            t = (x - x_coords[i]) / (x_coords[i+1] - x_coords[i])
            return y_coords[i] + t * (y_coords[i+1] - y_coords[i])
    return 0.0

# ─── LECTEUR STM32 ───────────────────────────────────────────────────────────
class STM32Reader:
    def __init__(self, port='/dev/ttyACM0', baud=921600):
        self.port = port
        self.baud = baud
        self.angle = 0.0
        self.raw_adc = 0
        self.dt = 0.0
        self.velocity = 0.0
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception:
            return
            
        parse_state = 0
        adc1_low = adc1_high = adc2_low = adc2_high = 0
        last_time = time.time()
        last_angle = 0.0
        
        while self.running:
            try:
                buf = ser.read(64)
                for byte in buf:
                    if parse_state == 0:
                        if byte == 0xAA: parse_state = 1
                    elif parse_state == 1:
                        adc1_low = byte; parse_state = 2
                    elif parse_state == 2:
                        adc1_high = byte; parse_state = 3
                    elif parse_state == 3:
                        adc2_low = byte; parse_state = 4
                    elif parse_state == 4:
                        adc2_high = byte; parse_state = 5
                    elif parse_state == 5:
                        expected = adc1_low ^ adc1_high ^ adc2_low ^ adc2_high
                        if byte == expected:
                            yaw_bits = adc2_low | ((adc2_high & 0x0F) << 8)
                            self.raw_adc = yaw_bits
                            new_angle = interpolate_lut(yaw_bits, yaw_bit_coords, yaw_angle_coords_deg)
                            
                            t = time.time()
                            dt = t - last_time
                            if dt > 0.001:
                                self.velocity = (new_angle - last_angle) / dt
                                self.dt = dt
                            
                            self.angle = new_angle
                            last_angle = new_angle
                            last_time = t
                        parse_state = 0
            except Exception:
                pass

# ─── LECTEUR CAN ─────────────────────────────────────────────────────────────
class CANReader:
    def __init__(self, iface='can0'):
        self.bus = can.interface.Bus(iface, bustype='socketcan')
        self.temp_a = 0; self.temp_b = 0
        self.soc = 0; self.i_batt = 0.0; self.v_batt_raw = 0
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        
    def _run(self):
        while self.running:
            try:
                msg = self.bus.recv(1.0)
                if not msg: continue
                if msg.arbitration_id == 0x2FF and len(msg.data) >= 8:
                    self.temp_a = struct.unpack('b', bytes([msg.data[0]]))[0]
                    self.temp_b = struct.unpack('b', bytes([msg.data[1]]))[0]
                elif msg.arbitration_id == 0x602 and len(msg.data) >= 8:
                    self.soc = msg.data[0]
                    raw_i = struct.unpack('>h', msg.data[1:3])[0]
                    self.i_batt = raw_i * 0.0103 - 0.72
            except Exception:
                pass

# ─── LOGIQUE DE CALIBRATION AUTOMATIQUE ──────────────────────────────────────
class AutoCalibrator:
    def __init__(self):
        self.state = "INIT"
        self.target = 0.0
        self.stuck_time = 0.0
        self.limit_right = None
        self.limit_left = None
        self.t_state = 0.0

    def update(self, current_angle, dt):
        self.t_state += dt
        
        if self.state == "INIT":
            self.target = current_angle
            if self.t_state > 1.0:
                self.state = "SEARCH_RIGHT"
                self.t_state = 0.0
                self.stuck_time = 0.0
                
        elif self.state == "SEARCH_RIGHT":
            self.target -= 2.0 * dt  # Négatif = Droite
            # Détection intelligente : si l'erreur PID devient énorme (> 5°), c'est qu'on est bloqué !
            # Ça ignore complètement les vibrations locales du capteur.
            if (self.target - current_angle) < -5.0:
                self.stuck_time += dt
            else:
                self.stuck_time = 0.0
                
            if self.stuck_time > 1.5:
                self.limit_right = current_angle
                self.state = "GO_CENTER_1"
                self.t_state = 0.0
                
        elif self.state == "GO_CENTER_1":
            self.target = 0.0
            if abs(current_angle) < 2.0 or self.t_state > 4.0:
                self.state = "SEARCH_LEFT"
                self.t_state = 0.0
                self.stuck_time = 0.0
                
        elif self.state == "SEARCH_LEFT":
            self.target += 2.0 * dt # Positif = Gauche
            if (self.target - current_angle) > 5.0:
                self.stuck_time += dt
            else:
                self.stuck_time = 0.0
                
            if self.stuck_time > 1.5:
                self.limit_left = current_angle
                self.state = "TEST_MAX_SPEED"
                self.t_state = 0.0
                
        elif self.state == "TEST_MAX_SPEED":
            # On demande d'aller à la butée droite instantanément (saut violent)
            # Le PID va envoyer 100% de puissance. Cela teste la vitesse max mécanique.
            if self.limit_right is not None:
                self.target = self.limit_right + 2.0 
            if self.t_state > 3.0:
                self.state = "TEST_MIN_SPEED"
                self.t_state = 0.0
                self.target = current_angle
                
        elif self.state == "TEST_MIN_SPEED":
            # On demande un mouvement extrêmement lent (0.5 deg/s) vers la gauche.
            # Permet d'analyser la sensibilité et la friction (stick-slip) dans le CSV.
            self.target += 0.5 * dt
            if self.t_state > 8.0:
                self.state = "DONE"
                self.t_state = 0.0
                
        elif self.state == "DONE":
            self.target = 0.0

        return self.target

# ─── LOGIQUE DU STRESS TEST ULTIME ───────────────────────────────────────────
class StressTester:
    def __init__(self, limit=45.0, speed_multiplier=1.0):
        self.state = "INIT"
        self.target = 0.0
        self.limit = limit
        self.speed = speed_multiplier
        self.t_state = 0.0
        self.failure_detected = False
        self.stuck_time = 0.0
        self.history_adc = []

    def update(self, current_angle, dt, steer_norm, raw_adc):
        self.t_state += dt
        
        # 1. Détection Ultime de Panne (Unresponsiveness)
        # Si on envoie une forte commande (> 0.5) mais que le raw_adc ne bouge pas de plus de 2 bits
        self.history_adc.append(raw_adc)
        if len(self.history_adc) > 50: # Garde 1 seconde de données (à 50Hz)
            self.history_adc.pop(0)
            
        if len(self.history_adc) == 50 and abs(steer_norm) > 0.5:
            if max(self.history_adc) - min(self.history_adc) <= 2:
                self.stuck_time += dt
            else:
                self.stuck_time = 0.0
        else:
            self.stuck_time = 0.0

        if self.stuck_time > 1.5:
            self.failure_detected = True
            return current_angle # Coupe l'effort immédiatement (erreur = 0)
            
        # 2. Séquenceur de stress
        if self.state == "INIT":
            self.target = 0.0
            if self.t_state > 2.0:
                self.state = "SWEEP"
                self.t_state = 0.0
                
        elif self.state == "SWEEP":
            # Balayage doux et large (Sine wave)
            self.target = math.sin(self.t_state * 0.5 * self.speed) * self.limit
            if self.t_state > 15.0:
                self.state = "STEP"
                self.t_state = 0.0
                
        elif self.state == "STEP":
            # Sauts ultra violents de butée à butée
            phase = int(self.t_state * 0.5 * self.speed) % 2
            self.target = self.limit if phase == 0 else -self.limit
            if self.t_state > 10.0:
                self.state = "VIBRATE"
                self.t_state = 0.0
                
        elif self.state == "VIBRATE":
            # Haute fréquence, petite amplitude (stress thermique des MOSFETs)
            self.target = math.sin(self.t_state * 15.0 * self.speed) * 5.0
            if self.t_state > 10.0:
                self.state = "CRAWL"
                self.t_state = 0.0
                self.target = -self.limit
                
        elif self.state == "CRAWL":
            # Vitesse très petite pour forcer le contrôleur à hacher le courant à basse tension
            self.target += 2.0 * dt * self.speed
            if self.t_state > 20.0 or self.target > self.limit:
                self.state = "SWEEP" # ON BOUCLE À L'INFINI JUSQU'À CE QUE ÇA CRASH !
                self.t_state = 0.0
                self.target = 0.0
                
        elif self.state == "DONE":
            self.target = 0.0

        return self.target

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, choices=['open', 'closed'], default='closed')
    parser.add_argument('--profile', type=str, choices=['sine', 'sweep', 'vibrate', 'step', 'find_limits', 'stress_test'], default='sine')
    parser.add_argument('--limit', type=float, default=30.0)
    parser.add_argument('--speed', type=float, default=1.0)
    parser.add_argument('--kp', type=float, default=0.08)
    parser.add_argument('--can', type=str, default='can0')
    parser.add_argument('--stm_port', type=str, default='/dev/ttyACM0')
    args = parser.parse_args()

    stm = STM32Reader(port=args.stm_port)
    can_reader = CANReader(iface=args.can)
    auto_calibrator = AutoCalibrator()
    stress_tester = StressTester(limit=args.limit, speed_multiplier=args.speed)
    
    import os
    if not os.path.exists("mtt_logs"): os.makedirs("mtt_logs")
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_filename = f"mtt_logs/mtt_stress_test_{args.mode}_{args.profile}_{timestamp}.csv"
    
    with open(csv_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time_s', 'TargetAngle_deg', 'ActualAngle_deg', 'Velocity_deg_s', 'SteerCmd_Norm', 'Temp_B_C', 'I_Batt_A', 'RawADC_bits', 'StmDt_s'])
        
        t0 = time.time()
        last_t = t0
        last_steer_norm_for_stress = 0.0
        
        try:
            while True:
                t = time.time()
                now = t - t0
                dt = t - last_t
                last_t = t
                
                target_angle = 0.0
                actual_angle = stm.angle
                steer_norm = 0.0
                
                if args.mode == 'open':
                    if args.profile == 'sweep': steer_norm = math.sin(now * 0.2 * args.speed) * 0.4
                    elif args.profile == 'vibrate': steer_norm = math.sin(now * 15.0 * args.speed) * 0.5
                    elif args.profile == 'step': steer_norm = 0.0 if (int(now * 0.5 * args.speed) % 2 == 0) else 0.8
                    else: steer_norm = math.sin(now * 1.5 * args.speed) * 0.7
                else:
                    if args.profile == 'find_limits':
                        target_angle = auto_calibrator.update(actual_angle, dt)
                    elif args.profile == 'stress_test':
                        target_angle = stress_tester.update(actual_angle, dt, last_steer_norm_for_stress, stm.raw_adc)
                        if stress_tester.failure_detected:
                            print(f"\n\n [PANNE DÉTECTÉE] Le contrôleur ne répond plus (Time: {now:.1f}s) !")
                            print(f"Angle figé à {actual_angle:.1f}°, RawADC={stm.raw_adc}, Commande ignorée={last_steer_norm_for_stress:.2f}")
                            print("COUPURE DE SÉCURITÉ IMMÉDIATE. Arrêt du script.")
                            break
                    elif args.profile == 'sweep': target_angle = math.sin(now * 0.2 * args.speed) * 50.0 
                    elif args.profile == 'vibrate': target_angle = math.sin(now * 15.0 * args.speed) * 3.0
                    elif args.profile == 'step': target_angle = 0.0 if (int(now * 0.5 * args.speed) % 2 == 0) else args.limit
                    else: target_angle = math.sin(now * 1.5 * args.speed) * args.limit
                        
                    steer_norm = args.kp * (target_angle - actual_angle)
                
                steer_norm = max(-1.0, min(1.0, steer_norm))
                last_steer_norm_for_stress = steer_norm
                
                # --- LE BUG ETAIT ICI ! ---
                # On met IMPÉRATIVEMENT SteeringMode = OpenLoop (0x00) pour le contrôleur moteur.
                # C'est NOTRE script Python qui ferme la boucle (PID). Le contrôleur moteur n'a 
                # plus de capteur branché dessus, si on lui dit "ClosedLoop", il va se baser sur 
                # son capteur fantôme et forcer à 100% dans le vide !
                steer_byte = int((steer_norm + 1.0) * 0.5 * 255)
                steer_byte = max(0, min(255, steer_byte))
                
                data = bytearray(8)
                data[0] = 0x00
                data[1] = 0xE8 
                data[2] = 0x00 # Throttle
                data[3] = 0x7F # WINCH NEUTRAL IMPORTANT
                data[4] = 0x00 # Brake
                data[5] = steer_byte
                data[6] = 0x00  # <--- TOUJOURS 0x00 (OpenLoop) !
                data[7] = 0x00
                
                msg = can.Message(arbitration_id=0x001, data=data, is_extended_id=False)
                can_reader.bus.send(msg)
                
                writer.writerow([f"{now:.3f}", f"{target_angle:.2f}", f"{actual_angle:.2f}", f"{stm.velocity:.1f}", f"{steer_norm:.2f}", can_reader.temp_b, f"{can_reader.i_batt:.1f}", stm.raw_adc, f"{stm.dt:.4f}"])
                f.flush()
                
                msg_console = f"[{now:5.1f}s] Angle: {actual_angle:+5.1f}° | Cmd: {steer_norm:+.2f} | Vitesse: {stm.velocity:+.1f}°/s | Temp: {can_reader.temp_b}°C"
                if args.profile == 'find_limits':
                    msg_console += f" | {auto_calibrator.state}"
                    if auto_calibrator.limit_right: msg_console += f" | R:{auto_calibrator.limit_right:.1f}°"
                    if auto_calibrator.limit_left: msg_console += f" | L:{auto_calibrator.limit_left:.1f}°"
                elif args.profile == 'stress_test':
                    msg_console += f" | STRESS: {stress_tester.state}"
                
                print(msg_console, end='\r')
                
                # Quitter automatiquement si l'étalonnage ou le stress test est fini
                if args.profile == 'find_limits' and auto_calibrator.state == 'DONE':
                    print("\n\nÉtalonnage terminé avec succès ! Arrêt automatique...")
                    break
                if args.profile == 'stress_test' and stress_tester.state == 'DONE':
                    print("\n\nStress Test terminé sans détecter de panne ! Arrêt automatique...")
                    break
                    
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            data = bytearray(8)
            data[0] = 0x00
            data[1] = 0x60
            data[2] = 0x00
            data[3] = 0x7F # WINCH NEUTRAL
            data[4] = 0x00
            data[5] = 127
            data[6] = 0x00
            data[7] = 0x00
            can_reader.bus.send(can.Message(arbitration_id=0x001, data=data, is_extended_id=False))
        finally:
            stm.running = False; can_reader.running = False

if __name__ == '__main__': main()
