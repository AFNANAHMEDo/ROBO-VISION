import serial

ser = serial.Serial("COM8", 115200, timeout=1)

print("Connected!")

while True:
    ser.write(b"Hello\n")