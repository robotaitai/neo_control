#!/usr/bin/env python3

import serial
import time
import threading
import re
import sys
import os
import argparse
from collections import deque

# --- Platform-specific keyboard input ---
try:
    # Linux / macOS
    import curses
    import select

    def get_key_curses(stdscr):
        """Check for keyboard input using curses (non-blocking)."""
        stdscr.nodelay(True)  # Don't block waiting for input
        try:
            return stdscr.getch()
        except curses.error:
            return -1 # No input

    def keyboard_listener(callback):
        """Uses curses for keyboard input."""
        try:
            stdscr = curses.initscr()
            curses.cbreak() # Don't wait for Enter
            curses.noecho() # Don't echo pressed keys
            stdscr.keypad(True) # Handle arrow keys, etc.
            print("curses mode initialized for keyboard input.")
            print_controls() # Print controls again after curses init
            callback(stdscr) # Pass stdscr to the main loop
        finally:
            # Clean up curses
            if 'stdscr' in locals() and stdscr:
                stdscr.keypad(False)
                curses.nocbreak()
                curses.echo()
                curses.endwin()
            print("curses mode terminated.")

except ImportError:
    # Windows
    try:
        import msvcrt

        def get_key_msvcrt():
            """Check for keyboard input using msvcrt (non-blocking)."""
            if msvcrt.kbhit():
                return msvcrt.getch()
            return None

        def keyboard_listener(callback):
            """Uses msvcrt for keyboard input."""
            print("msvcrt mode initialized for keyboard input (Windows).")
            print_controls() # Print controls again
            callback(None) # Pass None as stdscr is not used

    except ImportError:
        print("Error: Neither curses (Linux/macOS) nor msvcrt (Windows) found.")
        print("Non-blocking keyboard input not supported on this platform.")
        sys.exit(1)
# --- End Platform-specific keyboard input ---

# --- Configuration ---
DEFAULT_PORT = "/dev/ttyACM0"  # Change this to your robot's serial port
# On Windows, it might be "COM3", "COM4", etc.
# On Linux, it might be "/dev/ttyUSB0", "/dev/ttyACM0", etc.
# On macOS, it might be "/dev/cu.usbmodemXXXX", "/dev/tty.usbmodemXXXX", etc.

BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1  # Read timeout in seconds
COMMAND_SEND_INTERVAL = 0.1 # Send velocity commands every 100ms
PLOT_UPDATE_INTERVAL = 50 # Plot update interval in milliseconds
PLOT_HISTORY_SIZE = 100 # Number of points to show in the plot history

# Velocity control parameters
LINEAR_VEL_STEP = 0.05  # m/s change per key press
ANGULAR_VEL_STEP = 0.1  # rad/s change per key press
MAX_LINEAR_VEL = 0.5    # Max linear velocity (optional limit)
MAX_ANGULAR_VEL = 2.0   # Max angular velocity (optional limit)

# Regular expression for parsing Odometry frames
ODOM_REGEX = re.compile(r"<O,(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)>")

# --- Global State (Shared between threads) ---
# Use locks for thread safety when accessing shared variables
state_lock = threading.Lock()
desired_lin_vel = 0.0
desired_ang_vel = 0.0
last_odom = {
    'x': 0.0, 'y': 0.0, 'th': 0.0, 'v': 0.0, 'om': 0.0, 'time': time.time()
}
# Deques for plotting history
time_history = deque(maxlen=PLOT_HISTORY_SIZE)
v_history = deque(maxlen=PLOT_HISTORY_SIZE)
om_history = deque(maxlen=PLOT_HISTORY_SIZE)

running = True  # Flag to signal threads to stop

# --- Functions ---

def print_controls():
    """Prints the keyboard controls."""
    print("\n--- Neo-Bot Controller ---")
    print("Controls:")
    print("  W/S: Increase/Decrease Linear Velocity (+/- {:.2f} m/s)".format(LINEAR_VEL_STEP))
    print("  A/D: Increase/Decrease Angular Velocity (+/- {:.2f} rad/s)".format(ANGULAR_VEL_STEP))
    print("  SPACE: Stop (Set velocities to 0)")
    print("  Q: Quit")
    print("-------------------------\n")

def serial_worker(ser):
    """
    Thread function to handle serial reading (odometry) and writing (commands).
    """
    global running, last_odom, time_history, v_history, om_history
    last_command_time = time.time()

    while running:
        # --- Read Odometry ---
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                # print(f"DEBUG RX: {line}") # Uncomment for raw serial debugging
                if line.startswith("<O,"):
                    match = ODOM_REGEX.match(line)
                    if match:
                        try:
                            x, y, th, v, om = map(float, match.groups())
                            current_time = time.time()
                            with state_lock:
                                last_odom = {'x': x, 'y': y, 'th': th, 'v': v, 'om': om, 'time': current_time}
                                time_history.append(current_time)
                                v_history.append(v)
                                om_history.append(om)
                            # print(f"Odom: X={x:.3f} Y={y:.3f} Th={th:.3f} V={v:.3f} Om={om:.3f}") # Uncomment for parsed odom
                        except ValueError:
                            print(f"Warning: Could not parse odom frame: {line}")
                        except Exception as e:
                            print(f"Error processing odom: {e}")
                # elif not line.startswith("<V,"): # Optionally ignore command echo or other known frames
                #     print(f"Ignoring: {line}")

        except serial.SerialException as e:
            print(f"Serial error: {e}")
            running = False
            break
        except IOError: # Sometimes happens on port close / disconnect
             print(f"Serial IO error.")
             running = False
             break
        except Exception as e:
            print(f"Error in serial loop: {e}")
            # Decide if this error is fatal
            # running = False
            # break

        # --- Send Commands Periodically ---
        current_time = time.time()
        if current_time - last_command_time >= COMMAND_SEND_INTERVAL:
            with state_lock:
                lin = desired_lin_vel
                ang = desired_ang_vel

            command = f"<V,{lin:.3f},{ang:.3f}>\n"
            try:
                ser.write(command.encode('ascii'))
                # print(f"DEBUG TX: {command.strip()}") # Uncomment for command debugging
                last_command_time = current_time
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                running = False
                break
            except Exception as e:
                 print(f"Error writing command: {e}")
                 # Decide if this error is fatal
                 # running = False
                 # break

        # Small sleep to prevent busy-waiting if no data and command interval not reached
        time.sleep(0.01)

    print("Serial worker thread finished.")

def keyboard_control_loop(stdscr=None):
    """
    Main loop for keyboard input.
    Uses stdscr if curses is active, otherwise uses msvcrt functions directly.
    """
    global running, desired_lin_vel, desired_ang_vel

    while running:
        # Get key press (platform-specific)
        if stdscr: # curses (Linux/macOS)
            key = get_key_curses(stdscr)
            key_char = chr(key) if key != -1 and key < 256 else key # Convert code to char if possible
        else: # msvcrt (Windows)
            key_bytes = get_key_msvcrt()
            key_char = key_bytes.decode('ascii').lower() if key_bytes else None

        new_command = False
        if key_char:
            with state_lock: # Lock when modifying shared desired velocities
                if key_char == 'w':
                    desired_lin_vel = min(desired_lin_vel + LINEAR_VEL_STEP, MAX_LINEAR_VEL)
                    new_command = True
                elif key_char == 's':
                    desired_lin_vel = max(desired_lin_vel - LINEAR_VEL_STEP, -MAX_LINEAR_VEL)
                    new_command = True
                elif key_char == 'a':
                    desired_ang_vel = min(desired_ang_vel + ANGULAR_VEL_STEP, MAX_ANGULAR_VEL)
                    new_command = True
                elif key_char == 'd':
                    desired_ang_vel = max(desired_ang_vel - ANGULAR_VEL_STEP, -MAX_ANGULAR_VEL)
                    new_command = True
                elif key_char == ' ':
                    desired_lin_vel = 0.0
                    desired_ang_vel = 0.0
                    new_command = True
                elif key_char == 'q':
                    print("Q pressed, shutting down...")
                    desired_lin_vel = 0.0 # Send stop command before quitting
                    desired_ang_vel = 0.0
                    new_command = True
                    running = False # Signal other threads to stop

            if new_command and running: # Don't print if quitting
                 print(f"Command: Lin={desired_lin_vel:.2f} m/s, Ang={desired_ang_vel:.2f} rad/s")

        time.sleep(0.05) # Prevent CPU hogging

    print("Keyboard listener finished.")


# --- Matplotlib Visualization ---
# Import here so script can run without GUI if matplotlib not installed or display not available
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    VISUALIZATION_ENABLED = True
except ImportError:
    print("\nWarning: Matplotlib not found. Real-time plotting disabled.")
    print("Install it with: pip install matplotlib\n")
    VISUALIZATION_ENABLED = False

fig, ax = None, None
line_v, line_om = None, None
text_v, text_om = None, None
start_time = time.time()

def setup_plot():
    global fig, ax, line_v, line_om, text_v, text_om
    fig, ax = plt.subplots()
    line_v, = ax.plot([], [], 'r-', label='Linear Vel (V)')
    line_om, = ax.plot([], [], 'b-', label='Angular Vel (OM)')
    ax.legend()
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s or rad/s)")
    ax.set_title("Neo-Bot Measured Velocities")
    ax.grid(True)

    # Add text display for current values
    text_v = ax.text(0.05, 0.95, '', transform=ax.transAxes, ha='left', va='top', color='red')
    text_om = ax.text(0.05, 0.90, '', transform=ax.transAxes, ha='left', va='top', color='blue')


def update_plot(frame):
    global time_history, v_history, om_history, last_odom
    with state_lock:
        # Copy data to avoid holding lock during plot updates
        times = list(time_history)
        vs = list(v_history)
        oms = list(om_history)
        current_v = last_odom['v']
        current_om = last_odom['om']

    if not times:
        return line_v, line_om, text_v, text_om # Nothing to plot yet

    relative_times = [t - times[0] for t in times] # Show time relative to start of plot history

    line_v.set_data(relative_times, vs)
    line_om.set_data(relative_times, oms)

    # Update text display
    text_v.set_text(f'V: {current_v:.3f} m/s')
    text_om.set_text(f'OM: {current_om:.3f} rad/s')

    # Adjust plot limits
    ax.relim()
    ax.autoscale_view(scalex=True, scaley=True)
    # Optional: Set fixed Y limits if desired
    # ax.set_ylim(-MAX_LINEAR_VEL - 0.1, MAX_LINEAR_VEL + 0.1) # Example limits

    return line_v, line_om, text_v, text_om

# --- Main Execution ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Control Neo-Bot via serial and visualize odometry.")
    parser.add_argument('-p', '--port', default=DEFAULT_PORT,
                        help=f"Serial port connected to the Neo-Bot (default: {DEFAULT_PORT})")
    parser.add_argument('--no-plot', action='store_true',
                        help="Disable real-time plotting.")
    args = parser.parse_args()

    if args.no_plot:
        VISUALIZATION_ENABLED = False

    ser = None
    serial_thread = None

    try:
        print(f"Attempting to connect to {args.port} at {BAUD_RATE} baud...")
        ser = serial.Serial(args.port, BAUD_RATE, timeout=SERIAL_TIMEOUT)
        print("Serial port opened successfully.")

        # Give the connection a moment
        time.sleep(1.0)
        ser.reset_input_buffer() # Clear any old data

        # Start the serial communication thread
        serial_thread = threading.Thread(target=serial_worker, args=(ser,), daemon=True)
        serial_thread.start()
        print("Serial worker thread started.")

        # Start keyboard listener (platform-specific)
        # We run the keyboard listener in the main thread if no plot,
        # otherwise in a separate thread.
        if VISUALIZATION_ENABLED and plt:
             # Start keyboard listener in its own thread
             keyboard_thread = threading.Thread(target=keyboard_listener, args=(keyboard_control_loop,), daemon=True)
             keyboard_thread.start()
             print("Keyboard listener thread started.")

             # Set up and run the plot in the main thread (plt.show() is blocking)
             setup_plot()
             ani = animation.FuncAnimation(fig, update_plot, interval=PLOT_UPDATE_INTERVAL, blit=True, cache_frame_data=False)
             plt.show() # This blocks until the plot window is closed

             print("Plot window closed.")
             running = False # Signal threads to stop if plot window is closed

        else:
             # No visualization, run keyboard listener directly in main thread
             keyboard_listener(keyboard_control_loop) # This blocks until 'q' is pressed


    except serial.SerialException as e:
        print(f"Error opening serial port {args.port}: {e}")
        print("Please check the port name, permissions, and if the device is connected.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Shutting down...")
        running = False # Ensure running flag is false

        # Wait briefly for threads to potentially notice the flag
        time.sleep(0.2)

        # Clean up serial port
        if ser and ser.is_open:
            try:
                # Send a final stop command
                stop_command = "<V,0.000,0.000>\n"
                ser.write(stop_command.encode('ascii'))
                print("Sent final stop command.")
                time.sleep(0.1) # Allow time for command to be sent
                ser.close()
                print("Serial port closed.")
            except Exception as e:
                print(f"Error during serial port cleanup: {e}")

        # Wait for threads to finish (optional, as daemon=True)
        # if serial_thread and serial_thread.is_alive():
        #     print("Waiting for serial thread...")
        #     serial_thread.join(timeout=1.0)
        # if 'keyboard_thread' in locals() and keyboard_thread.is_alive():
        #      print("Waiting for keyboard thread...")
        #      keyboard_thread.join(timeout=1.0)

        print("Cleanup complete. Exiting.")
        # Ensure curses is cleaned up if it was used and exited abnormally
        if 'curses' in sys.modules and 'stdscr' not in locals():
            try:
                curses.endwin()
                print("Forcing curses cleanup.")
            except:
                pass