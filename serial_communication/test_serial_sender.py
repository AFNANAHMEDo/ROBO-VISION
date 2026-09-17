import serial
import time

PORT = "COM8"
BAUD = 115200
TIMEOUT = 1


def main():

    print(f"Opening {PORT} at {BAUD} baud...")

    ser = serial.Serial(
        PORT,
        BAUD,
        timeout=TIMEOUT
    )

    time.sleep(2)

    # These are the SAME commands
    # that detect.py will send.
    test_commands = [
        "C",
        "D,10,0",
        "D,-10,0",
        "D,0,10",
        "D,0,-10",
        "D,10,10",
        "D,-10,-10",
    ]

    for command in test_commands:

        print(f"\n--> Sending: {command}")

        ser.write((command + "\n").encode("utf-8"))
        ser.flush()

        time.sleep(0.5)

        while ser.in_waiting:

            line = ser.readline().decode(
                "utf-8",
                errors="replace"
            ).strip()

            print(f"<-- ESP32: {line}")

        time.sleep(1)


    ser.close()

    print("\nTest finished.")


if __name__ == "__main__":
    main()