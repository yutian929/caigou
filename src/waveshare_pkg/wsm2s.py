import serial
import argparse
import threading

def read_serial():
    while True:
        data = ser.readline().decode('utf-8')
        if data:
            print(f"Received: {data}", end='')

def main():
    global ser
    # parser = argparse.ArgumentParser(description='Serial JSON Communication')
    # parser.add_argument('port', type=str, help='Serial port name (e.g., COM1 or /dev/ttyUSB0)')

    # args = parser.parse_args()

    # ser = serial.Serial(args.port, baudrate=115200, dsrdtr=None)
    ser = serial.Serial("/dev/arm_waveshare_serial",115200)
    ser.setRTS(False)
    ser.setDTR(False)

    serial_recv_thread = threading.Thread(target=read_serial)
    serial_recv_thread.daemon = True
    serial_recv_thread.start()
    print("hello")
    try:
        while True:
            command = input("")
            print(f"Sending: {command}")
            ser.write(command.encode() + b'\n')
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
# {"T":202,"name":"boot.mission"}
# {"T":206,"name":"boot.mission","lineNum":3,"content":'{"T":2, "pos":3, "ea":0, "eb":35}'}
# {"T": 104, "x": 456, "y": 38, "z": -60, "t": 3.14, "spd": 0.25}
#  0.4567356036803964, 0.03868430954247791, -0.15731061641243083
if __name__ == "__main__":
    main()


# {"T":2,"pos":3,"ea":30,"eb":55}