"""
PC-side user interface for line following robot control via Bluetooth.
Sends calibration commands and velocity parameters to robot, then monitors output.

Hardware:
    - Bluetooth Module: HC-05 or similar at 460800 baud
    - Serial Port: COM6 (configurable)

Notes:
    - Preset velocity: 50 mm/s
    - Automatic calibration sequence on each trial start
    - Press Ctrl+C to stop trial and send END_COMM command
    - Continuously displays MCU debug output during operation
"""
import serial
import time


def run_trial(bt):
    """
    Execute single trial with automatic calibration and monitoring.

    :param bt: Serial connection object to robot
    :type bt: serial.Serial
    """
    try:
        params_cmd = "PARAMS,0,50\n"
        print(f"Sending: {params_cmd.strip()}")
        bt.write(params_cmd.encode())
        time.sleep(0.5)

        bt.write(b"CALIBRATE_BLACK\n")
        print("Sent: CALIBRATE_BLACK (preset)")
        time.sleep(0.5)

        bt.write(b"CALIBRATE_WHITE\n")
        print("Sent: CALIBRATE_WHITE (preset)")
        time.sleep(0.5)

        bt.write(b"CALIBRATION_COMPLETE\n")
        print("Sent: CALIBRATION_COMPLETE")
        time.sleep(0.5)

        if bt.in_waiting:
            response = bt.read(bt.in_waiting).decode('utf-8', errors='ignore')
            print(f"MCU response: {response}")

        print("\nBot running (press Ctrl+C to stop)...")
        while True:
            if bt.in_waiting:
                response = bt.read(bt.in_waiting).decode('utf-8', errors='ignore')
                print(f"MCU: {response}", end='')
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nStopping trial...\n")
        bt.write(b"END_COMM\n")
        time.sleep(0.5)
        return


def main():
    """
    Main program loop for robot control interface.
    Establishes serial connection and runs trials on user input.
    """
    port = 'COM6'

    try:
        bt = serial.Serial(port, baudrate=460800, timeout=1)
        print(f"Connected to {port}")
        time.sleep(2)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    try:
        while True:
            input("Press Enter to start bot...")
            run_trial(bt)

    except Exception as e:
        print(f"Error: {e}")
    finally:
        bt.close()
        print("Serial connection closed")


if __name__ == "__main__":
    main()