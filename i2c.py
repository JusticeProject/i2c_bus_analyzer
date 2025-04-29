import serial
import time
import sys

###################################################################################################

capture_seconds = 0
bus_scan = False

for i in range(1, len(sys.argv)):
    if (sys.argv[i].isnumeric()):
        capture_seconds = int(sys.argv[i])
    elif (sys.argv[i] == "scan"):
        bus_scan = True

print(f"Capturing for {capture_seconds} seconds.")
print(f"Bus scan = {bus_scan}.")

if (capture_seconds == 0) and (bus_scan == False):
    # nothing to do
    print("Exiting")
    exit()

###################################################################################################

def get_line(ser):
    data = ser.readline()
    #print(f"read {len(data)} bytes")
    data_str = data.decode(encoding="utf-8") # convert from bytes to string
    data_str = data_str.rstrip("\r\n")
    return data_str

###################################################################################################

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)


if (bus_scan):
    ser.write(b"a")

fd = None
if (capture_seconds > 0):
    ser.write(b"g")
    fd = open("data.txt", "w")

start_time = time.time()

while (True):
    data_str = get_line(ser)

    if (len(data_str) > 0):
        if (data_str.count(",") == 3) and (capture_seconds > 0):
            fd.write(data_str + "\n")
        elif (data_str.count(",") == 3) and (capture_seconds == 0):
            pass
        else:
            print(data_str)

    if ("Done." in data_str) and (capture_seconds == 0):
        break

    # check if we passed the time limit
    if (capture_seconds > 0):
        now = time.time()
        if (now - start_time) > capture_seconds:
            break


if (fd is not None):
    print("Data written to data.txt")
    fd.close()

ser.close()
