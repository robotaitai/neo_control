#!/usr/bin/env python3
"""
neo_bot_control_arrow.py

Arrow-key control and real‑time odometry display for the Neo‑Bot over USB‑CDC.

Usage:
  python3 neo_bot_control_arrow.py --port /dev/cu.usbserial-0001 --baud 115200

Controls:
  ↑ : forward
  ↓ : backward
  ← : rotate left
  → : rotate right
  q : quit
"""

import argparse
import serial
import threading
import curses
import time

def curses_main(stdscr, ser, odom, lin_speed, ang_speed):
    # Curses setup
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    # Start reader thread to update odometry
    def reader():
        while True:
            try:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('<O'):
                    parts = line.strip('<>\n ').split(',')
                    if parts[0] == 'O' and len(parts) == 6:
                        _, x, y, th, v, om = parts
                        odom['x'] = float(x)
                        odom['y'] = float(y)
                        odom['th'] = float(th)
                        odom['v'] = float(v)
                        odom['om'] = float(om)
            except:
                continue

    threading.Thread(target=reader, daemon=True).start()

    # Main control loop
    while True:
        key = stdscr.getch()
        lin = 0.0
        ang = 0.0
        if key == curses.KEY_UP:
            lin = lin_speed
        elif key == curses.KEY_DOWN:
            lin = -lin_speed
        elif key == curses.KEY_LEFT:
            ang = ang_speed
        elif key == curses.KEY_RIGHT:
            ang = -ang_speed
        elif key in (ord('q'), ord('Q')):
            break

        # send command
        cmd = f'<V,{lin:.2f},{ang:.2f}>\n'
        ser.write(cmd.encode('ascii'))

        # display
        stdscr.clear()
        stdscr.addstr(0, 0, "Neo‑Bot arrow-key control (q to quit)")
        stdscr.addstr(2, 0, f"Command: lin = {lin:.2f} m/s, ang = {ang:.2f} rad/s")
        stdscr.addstr(4, 0, "Odometry:")
        stdscr.addstr(5, 0, f"  x = {odom['x']:.2f} m")
        stdscr.addstr(6, 0, f"  y = {odom['y']:.2f} m")
        stdscr.addstr(7, 0, f"  th= {odom['th']:.2f} rad")
        stdscr.addstr(8, 0, f"  v = {odom['v']:.2f} m/s")
        stdscr.addstr(9, 0, f"  ω = {odom['om']:.2f} rad/s")
        stdscr.refresh()

        time.sleep(0.05)

def main():
    parser = argparse.ArgumentParser(description='Neo‑Bot arrow-key control')
    parser.add_argument('--port', '-p', default='/dev/cu.usbserial-0001',
                        help='Serial port')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                        help='Baud rate')
    parser.add_argument('--lin', type=float, default=0.2,
                        help='Linear speed step (m/s)')
    parser.add_argument('--ang', type=float, default=1.0,
                        help='Angular speed step (rad/s)')
    args = parser.parse_args()

    # open serial port
    ser = serial.Serial(args.port, args.baud, timeout=0.1)

    # shared odom dict
    odom = {'x': 0.0, 'y': 0.0, 'th': 0.0, 'v': 0.0, 'om': 0.0}

    # run curses interface
    curses.wrapper(curses_main, ser, odom, args.lin, args.ang)

    # on exit, send stop command
    ser.write(b'<V,0.00,0.00>\n')
    ser.close()

if __name__ == '__main__':
    main()
