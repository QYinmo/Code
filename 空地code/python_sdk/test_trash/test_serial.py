import serial
import time

rxbuffer = []
rx_length = 4
comlist = [170,0, 0, 0, 0, 255]

with serial.Serial(port="/dev/ttyUSB2", baudrate=38400) as ser:
    ser.setRTS(False)
    try:
        time.sleep(2)#########################实际重点######################
        while True:
            for value in comlist:
                hex_value = hex(value)[2:].zfill(2)
                ser.write(bytes.fromhex(hex_value))
                print(f"Sent: {hex_value}")
            ser.flush()

            time.sleep(0.15)

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        ser
    finally:
        ser.close()


# No need for explicit ser.close() here, it is automatically handled by the 'with' statement
