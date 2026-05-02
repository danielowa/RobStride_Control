#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride MIT Mode Position Control (minimal reliable version)
Mode: Mode 0 (MIT Mode)
Communication: repeatedly calls write_operation_frame

Usage: python3 position_control_mit.py <motor_id>
"""

import sys
import os
import time
import math
import threading
import signal
from typing import Optional

# Try to import SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType
except ImportError:
    # Fallback to current directory structure
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType
    except ImportError as e:
        print(f"❌ Failed to import SDK: {e}")
        sys.exit(1)

class PositionControllerMIT:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel
        
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()  # Mutex to prevent socket conflicts

        self.running = True
        self.connected = False
        self.target_position = 0.0  # Target position (rad)

        # Default parameters (MIT mode)
        self.kp = 30.0  # Stiffness (Nm/rad)
        self.kd = 0.5   # Damping (Nm/rad/s)

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    def connect(self):
        print(f"🔍 Connecting to CAN channel {self.channel}...")

        # Define motors
        motors = {
            self.motor_name: Motor(id=self.motor_id, model="rs-03") # Update model string to match your actual motor
        }

        # Basic calibration parameters
        calibration = {
            self.motor_name: {"direction": 1, "homing_offset": 0.0}
        }

        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)
            
            with self.lock:
                # Set mode BEFORE enable (motor rejects mode changes while enabled)
                print("⚙️ Setting MIT mode (Mode 0)...")
                self.bus.write(self.motor_name, ParameterType.MODE, 0)

                # Enable motor
                print(f"⚡ Enabling motor ID: {self.motor_id} ...")
                self.bus.enable(self.motor_name)
                time.sleep(0.5)

                # 2. Set a known, safe initial target
                print("🏠 Setting initial target to 0.0 ...")
                self.target_position = 0.0 # Set to 0 radians

                # 3. Send first MIT command frame to hold position
                self.bus.write_operation_frame(
                    self.motor_name,
                    self.target_position,
                    self.kp,
                    self.kd,
                    0.0, # velocity_ff
                    0.0  # torque_ff
                )
                print(f"🏠 初始目标已设为: 0.0°")
            
            self.connected = True

            # Start background control thread
            self.control_thread = threading.Thread(target=self.loop, daemon=True)
            self.control_thread.start()

            print("✅ Initialization complete (Mode 0)!")
            return True

        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.connected = False
            return False

    def loop(self):
        """Control thread: continuously sends MIT frames to hold position"""
        print("🔄 Control loop started (Mode 0 @ 50Hz)")

        while self.running and self.connected:
            try:
                with self.lock:
                    # 1. Send MIT frame (write only)
                    self.bus.write_operation_frame(
                        self.motor_name,
                        self.target_position,
                        self.kp,
                        self.kd,
                        0.0, # velocity_ff
                        0.0  # torque_ff
                    )

                    # 2. Read status frame (read only)
                    # This is important to drain the CAN receive buffer and prevent overflow.
                    # The return value can be ignored — we only care about clearing the buffer.
                    self.bus.read_operation_frame(self.motor_name)

                time.sleep(0.02) # 50Hz control rate

            except Exception as e:
                # Ignore timeouts — they are common in read_operation_frame
                if "No response from the motor" not in str(e):
                    print(f"⚠️ Communication error: {e}")
                time.sleep(0.5)

    def set_angle(self, angle_degrees: float):
        """Set target angle (in degrees)"""
        # Clamp range, e.g. +/- 2 full rotations
        angle_degrees = max(-720.0, min(720.0, angle_degrees))
        # target_position is thread-safe (atomic assignment)
        self.target_position = math.radians(angle_degrees)
        print(f" -> Target set: {angle_degrees:.1f}°")

    def set_kp(self, kp: float):
        """Set stiffness"""
        if 0 <= kp <= 500:
            self.kp = kp
            print(f" -> Stiffness (Kp) set: {self.kp:.1f}")
        else:
            print("❌ Kp must be in range 0-500")

    def set_kd(self, kd: float):
        """Set damping"""
        if 0 <= kd <= 5:
            self.kd = kd
            print(f" -> Damping (Kd) set: {self.kd:.1f}")
        else:
            print("❌ Kd must be in range 0-5")

    def stop_and_exit(self):
        print("\n🛑 Stopping...")
        self.running = False

        if self.control_thread:
            self.control_thread.join(timeout=0.5) # Wait for thread to exit

        if self.bus and self.connected:
            try:
                with self.lock:
                    # Return to zero position
                    print("🏠 Returning to zero position...")
                    self.bus.write_operation_frame(self.motor_name, 0.0, self.kp, self.kd, 0.0, 0.0)
                    time.sleep(1.0) # Wait for motor to move
                    # Disable
                    print("🚫 Disabling motor...")
                    self.bus.disable(self.motor_name)
            except Exception as e:
                print(f"⚠️ Error while stopping: {e}")
            finally:
                self.bus.disconnect()

        print("👋 Goodbye")
        sys.exit(0)

    def run_interactive(self):
        print("\n" + "="*40)
        print(f"🎮 MIT Position Console (ID: {self.motor_id})")
        print("="*40)
        print("👉 Enter a number (degrees) and press Enter to change position")
        print("👉 'kp <value>' (e.g. kp 20) to adjust stiffness (reduce oscillation)")
        print("👉 'kd <value>' (e.g. kd 0.8) to adjust damping (reduce oscillation)")
        print("👉 '0' or 'home' to return to zero")
        print("👉 'q' to quit")
        print(f"⚠️  Current Kp={self.kp} | Kd={self.kd}")
        print("-" * 40)

        while True:
            try:
                cmd = input(f"[{math.degrees(self.target_position):.1f}°] >> ").strip().lower()
                
                if not cmd:
                    continue
                    
                if cmd in ['q', 'quit', 'exit']:
                    break
                
                if cmd in ['0', 'home']:
                    self.set_angle(0.0)
                    continue

                if cmd.startswith("kp "):
                    try:
                        new_kp = float(cmd.split()[1])
                        self.set_kp(new_kp)
                    except Exception:
                        print("❌ Invalid Kp. Example: kp 20.0")
                    continue

                if cmd.startswith("kd "):
                    try:
                        new_kd = float(cmd.split()[1])
                        self.set_kd(new_kd)
                    except Exception:
                        print("❌ Invalid Kd. Example: kd 0.5")
                    continue

                try:
                    angle = float(cmd)
                    self.set_angle(angle)
                except ValueError:
                    print("❌ Invalid input — enter a number (degrees) or 'kp', 'kd'")

            except KeyboardInterrupt:
                break
        
        self.stop_and_exit()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 position_control_mit.py <motor_id>")
        sys.exit(1)
        
    motor_id = int(sys.argv[1])
    
    controller = PositionControllerMIT(motor_id)
    signal.signal(signal.SIGINT, controller._signal_handler)
    signal.signal(signal.SIGTERM, controller._signal_handler)
    
    if controller.connect():
        controller.run_interactive()

if __name__ == "__main__":
    main()