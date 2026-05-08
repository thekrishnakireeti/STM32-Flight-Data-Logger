import serial
import csv
import time
import os
import re
from datetime import datetime

PORT = "COM4"
BAUD = 115200
CSV_FILE = "sensor_log.csv"

columns = [
    "Date-Day",
    "Time",
    "LAT","LON","SPEED",
    "TEMP","PRESS","ALT",
    "Ax","Ay","Az"
]

# Create CSV if not present
if not os.path.exists(CSV_FILE):

    with open(CSV_FILE,'w',newline='') as f:

        writer = csv.writer(f)
        writer.writerow(columns)

print("CSV ready:",CSV_FILE)

def wait_for_port():

    while True:
        try:
            ser = serial.Serial(PORT,BAUD,timeout=1)
            print("Connected to",PORT)
            return ser

        except:
            print("Waiting for serial port",PORT)
            time.sleep(2)


# This dictionary holds values from 3 lines
data_buffer = {}

def extract_numbers(line):

    pairs = re.findall(r'([A-Za-z]+)\s*=\s*(-?\d+\.?\d*)',line)

    result = {}

    for key,val in pairs:

        if "." in val:
            result[key] = float(val)
        else:
            result[key] = int(val)

    return result


ser = wait_for_port()

while True:

    try:

        line = ser.readline().decode(errors='ignore').strip()

        if not line:
            continue

        now = datetime.now()
        time_str = now.strftime("%H:%M:%S")
        date_day = now.strftime("%Y-%m-%d %A")

        print(time_str,"|",line)

        values = extract_numbers(line)

        data_buffer.update(values)

        # When IMU line arrives we assume packet complete
        if "Ax" in values:

            Ax = data_buffer.get("Ax",0)
            Ay = data_buffer.get("Ay",0)
            Az = data_buffer.get("Az",0)

            # Skip writing if all IMU values are zero
            if Ax==0 and Ay==0 and Az==0:
                data_buffer.clear()
                continue

            row = [
                date_day,
                time_str,
                data_buffer.get("LAT",""),
                data_buffer.get("LON",""),
                data_buffer.get("SPEED",""),
                data_buffer.get("TEMP",""),
                data_buffer.get("PRESS",""),
                data_buffer.get("ALT",""),
                Ax,
                Ay,
                Az
            ]

            with open(CSV_FILE,'a',newline='') as f:

                writer = csv.writer(f)
                writer.writerow(row)

            data_buffer.clear()

    except serial.SerialException:

        print("Serial disconnected. Waiting...")

        try:
            ser.close()
        except:
            pass

        ser = wait_for_port()

    except Exception as e:

        print("Error:",e)
