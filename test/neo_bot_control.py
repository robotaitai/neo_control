#!/usr/bin/env python3
"""
neo_bot_control.py

Interactive control and real‑time odometry visualization for the Neo‑Bot over USB‑CDC.

Usage:
  pip install pyserial matplotlib
  python neo_bot_control.py --port /dev/cu.usbserial-0001 --baud 115200

Controls:
  Enter linear and angular velocities at the prompt (e.g. "0.2 0.0") to drive.
  Ctrl+C to exit.

"""

import serial
import threading
import matplotlib.pyplot as plt
import argparse
import time

class NeoBotController:
    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.lock = threading.Lock()
        self.xs, self.ys = [], []
        self.running = True

    def reader(self):
        while self.running:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('<O'):
                    parts = line.strip('<>\n ').split(',')
                    if parts[0] == 'O' and len(parts) == 6:
                        _, x, y, th, v, om = parts
                        x = float(x); y = float(y)
                        with self.lock:
                            self.xs.append(x)
                            self.ys.append(y)
            except Exception:
                continue

    def send_command(self, lin, ang):
        cmd = f'<V,{lin:.2f},{ang:.2f}>\n'
        self.ser.write(cmd.encode('ascii'))

def main():
    parser = argparse.ArgumentParser(description='Neo‑Bot control & odometry visualizer')
    parser.add_argument('--port', '-p', default='/dev/cu.usbserial-0001', help='Serial port')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    ctrl = NeoBotController(args.port, args.baud)
    t = threading.Thread(target=ctrl.reader, daemon=True)
    t.start()

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], '-o', label='Pose')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Neo‑Bot Odometry')
    ax.legend()
    ax.grid(True)

    try:
        while True:
            user = input("Enter linear m/s and angular rad/s (e.g. 0.2 0.0): ").strip()
            if not user:
                continue
            parts = user.split()
            try:
                lin = float(parts[0])
                ang = float(parts[1])
            except:
                print("Invalid input. Enter two numbers separated by space.")
                continue
            ctrl.send_command(lin, ang)
            with ctrl.lock:
                line.set_data(ctrl.xs, ctrl.ys)
            ax.relim()
            ax.autoscale_view()
            fig.canvas.draw()
            plt.pause(0.01)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ctrl.running = False
        time.sleep(0.1)
        ctrl.ser.close()

if __name__ == '__main__':
    main()
