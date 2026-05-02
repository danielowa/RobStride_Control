#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride Speed Mode Control Script (Fixed Version)
Mode: Mode 2 (Speed Control Mode)
Communication: Uses WRITE_PARAMETER to update VELOCITY_TARGET (spd_ref)

Usage: python3 speed_control.py <motor_id>
"""

import sys
import os
import time
import threading
import signal
from typing import Optional

# Try to import SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType
except ImportError:
    # Assume current directory structure
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType
    except ImportError as e:
        print(f"❌ Failed to import SDK: {e}")
        sys.exit(1)

class SpeedController:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel
        
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()  # Mutex lock to prevent socket conflicts
        
        self.running = True
        self.connected = False
        self.target_velocity = 0.0
        self.current_status = None
        
        # Default parameters
        self.max_velocity = 20.0  # rad/s safety limit
        self.kp = 10.0
        self.ki = 1

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    def connect(self):
        print(f"🔍 Connecting to CAN channel {self.channel}...")

        # Define motor
        motors = {
            self.motor_name: Motor(id=self.motor_id, model="rs-02") # Modify model name according to actual hardware
        }
        
        # Simple calibration parameters
        calibration = {
            self.motor_name: {"direction": 1, "homing_offset": 0.0}
        }

        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)

            # Switch to speed mode BEFORE enable (motor rejects mode changes while enabled)
            print("⚙️ Setting speed mode (Mode 2)...")
            self.bus.write(self.motor_name, ParameterType.MODE, 2)

            # Verify mode
            actual_mode = self.bus.read(self.motor_name, ParameterType.MODE)
            print(f"📋 Mode readback: {actual_mode} (expected 2)")

            # Now enable in Mode 2
            print(f"⚡ Enabling motor ID: {self.motor_id}...")
            self.bus.enable(self.motor_name)
            time.sleep(0.3)

            # Disable CAN timeout — in speed mode, only WRITE_PARAMETER frames
            # are sent (not OPERATION_CONTROL), and the default CAN timeout may
            # silently cut torque output when it doesn't see type-1 frames.
            print("⚙️ Disabling CAN timeout...")
            self.bus.write(self.motor_name, ParameterType.CAN_TIMEOUT, 0)

            # Initialize PID and limits
            print("⚙️ Writing control parameters...")
            self.bus.write(self.motor_name, ParameterType.TORQUE_LIMIT, 10.0)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_LIMIT, self.max_velocity)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_KP, self.kp)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_KI, self.ki)

            # Zero the target
            self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
            
            self.connected = True
            print("✅ Initialization complete!")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False

    def loop(self):
        """Control thread: continuously sends heartbeat/velocity commands and reads status"""
        print("🔄 Control loop started")
        
        while self.running and self.connected:
            try:
                with self.lock:
                    # In Mode 2, we need to write VELOCITY_TARGET
                    # bus.write waits for a response packet (receive_status_frame), so this itself is a form of status reading
                    # Protocol 0x700A = VELOCITY_TARGET

                    self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, self.target_velocity)

                    # To read more detailed status (e.g. current torque), use read_operation_frame
                    # but the write response already contains status data; the SDK's write internally calls receive_status_frame
                    # We skip extra reads here to maintain high frequency
                    
                time.sleep(0.05) # 20Hz refresh rate to prevent bus congestion
                
            except Exception as e:
                print(f"⚠️ Communication error: {e}")
                time.sleep(0.5)

    def set_velocity(self, vel: float):
        """Set target velocity (with clamping)"""
        vel = max(-self.max_velocity, min(self.max_velocity, vel))
        self.target_velocity = vel
        print(f" -> Target set: {self.target_velocity:.2f} rad/s")

    def stop_and_exit(self):
        print("\n🛑 Stopping...")
        self.running = False
        self.target_velocity = 0.0
        
        if self.bus and self.connected:
            try:
                with self.lock:
                    # Stop first
                    self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
                    time.sleep(0.2)
                    # Disable
                    self.bus.disable(self.motor_name)
            except Exception:
                pass
            self.bus.disconnect()
        sys.exit(0)

    def run_interactive(self):
        # Start background send thread
        t = threading.Thread(target=self.loop, daemon=True)
        t.start()

        print("\n" + "="*40)
        print(f"🎮 Speed console (ID: {self.motor_id})")
        print("="*40)
        print("👉 Enter a number (rad/s) and press Enter to change speed")
        print("👉 Enter '0' to stop")
        print("👉 Enter 'q' to quit")
        print(f"⚠️  Current safety speed limit: ±{self.max_velocity} rad/s")
        print("-" * 40)

        while True:
            try:
                cmd = input(f"[{self.target_velocity:.1f} rad/s] >> ").strip().lower()
                
                if not cmd:
                    continue
                    
                if cmd in ['q', 'quit', 'exit']:
                    break
                
                try:
                    vel = float(cmd)
                    self.set_velocity(vel)
                except ValueError:
                    print("❌ Invalid input, please enter a number")

            except KeyboardInterrupt:
                break
        
        self.stop_and_exit()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 speed_control.py <motor_id>")
        sys.exit(1)
        
    motor_id = int(sys.argv[1])
    
    controller = SpeedController(motor_id)
    signal.signal(signal.SIGINT, controller._signal_handler)
    
    if controller.connect():
        controller.run_interactive()

if __name__ == "__main__":
    main()