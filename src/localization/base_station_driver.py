import rospy
from serial import Serial
from pyubx2 import UBXReader, UBX_PROTOCOL, RTCM3_PROTOCOL, protocol

def main():
    with Serial('/dev/ttyACM0', 115200, timeout=1) as ser:
        reader = UBXReader(ser, protfilter=(UBX_PROTOCOL | RTCM3_PROTOCOL))
        while True:
            if ser.in_waiting:
                (raw_data, parsed_data) = reader.read()
                if parsed_data:
                    print(parsed_data.identity)
                if protocol(raw_data) == RTCM3_PROTOCOL:
                    print("rtcm3")


if __name__ == '__main__':
    main()