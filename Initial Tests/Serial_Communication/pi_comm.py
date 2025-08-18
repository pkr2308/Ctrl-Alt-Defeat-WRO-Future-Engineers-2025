import serial

def main():
    # Open serial port to RP2040 UART (adjust port as needed)
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)

    print("Enter steering angle and speed separated by space (e.g., 45 120):")
    while True:
        try:
            # Read input from user
            user_input = input("Steering Speed > ")
            if not user_input:
                continue

            # Parse input
            parts = user_input.split()
            if len(parts) != 2:
                print("Please enter exactly two values: steering and speed")
                continue
            steering = float(parts[0])
            speed = float(parts[1])

            # Send command to RP2040
            command = f"S,{steering},{speed}\n"
            ser.write(command.encode())

            # Wait for response from RP2040
            response = ser.readline().decode().strip()
            if response.startswith("R"):
                values = response.split(",")[1:]
                print(f"Received Data -> Yaw: {values}, Distance: {values}, LiDAR1: {values}, LiDAR2: {values}, LiDAR3: {values}")
            else:
                print("No valid response received")

        except KeyboardInterrupt:
            print("\nExiting.")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
