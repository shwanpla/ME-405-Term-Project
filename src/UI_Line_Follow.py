import serial
import time

def run_trial(bt):
    """Run a single trial with preset calibration and speed"""
    try:
        # Send preset parameters to MCU (desired_vel = 50 mm/s)
        params_cmd = "PARAMS,0,50\n"
        print(f"Sending: {params_cmd.strip()}")
        bt.write(params_cmd.encode())
        time.sleep(0.5)
        
        # Send calibration commands automatically
        bt.write(b"CALIBRATE_BLACK\n")
        print("Sent: CALIBRATE_BLACK (preset)")
        time.sleep(0.5)
        
        bt.write(b"CALIBRATE_WHITE\n")
        print("Sent: CALIBRATE_WHITE (preset)")
        time.sleep(0.5)

        bt.write(b"CALIBRATION_COMPLETE\n")
        print("Sent: CALIBRATION_COMPLETE")
        time.sleep(0.5)
        
        # Check for any response
        if bt.in_waiting:
            response = bt.read(bt.in_waiting).decode('utf-8', errors='ignore')
            print(f"MCU response: {response}")
        
        # Monitor MCU output
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
    # Configure the serial port
    port = 'COM6'
    
    # Open serial connection with same baud rate as MCU (460800)
    try:
        bt = serial.Serial(port, baudrate=460800, timeout=1)
        print(f"Connected to {port}")
        time.sleep(2)  # Give time for connection to establish
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return
    
    try:
        # Keep running trials until user exits
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