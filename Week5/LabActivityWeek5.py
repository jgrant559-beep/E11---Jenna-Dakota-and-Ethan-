# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import csv
import numpy as np
import board
import adafruit_bme680
import busio
import sys

from digitalio import DigitalInOut, Direction, Pull

from adafruit_pm25.i2c import PM25_I2C

reset_pin = None
# If you have a GPIO, its not a bad idea to connect it to the RESET pin
# reset_pin = DigitalInOut(board.G0)
# reset_pin.direction = Direction.OUTPUT
# reset_pin.value = False


# For use with a computer running Windows:
# import serial
# uart = serial.Serial("COM30", baudrate=9600, timeout=1)

# For use with microcontroller board:
# (Connect the sensor TX pin to the board/computer RX pin)
# uart = busio.UART(board.TX, board.RX, baudrate=9600)

# For use with Raspberry Pi/Linux:
import serial
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=0.25)

# For use with USB-to-serial cable:
# import serial
# uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=0.25)

# Connect to a PM2.5 sensor over UART
from adafruit_pm25.uart import PM25_UART
pm25 = PM25_UART(uart, reset_pin)

# Create library object, use 'slow' 100KHz frequency!
# i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
# Connect to a PM2.5 sensor over I2C
# pm25 = PM25_I2C(i2c, reset_pin)

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)

# change this to match the location's pressure (hPa) at sea level
bme680.sea_level_pressure = 1013.25

file = open('lab5.csv', 'w', newline=None)

csvwriter = csv.writer(file, delimiter=',')
# file first row: time, temperature, pressure,...

meta = ['time',"Particles > 0.3um / 0.1L air", "Temperature", "Gas", "Humidity", "Pressure", "Altitude"]
# writerow(column_names)
csvwriter.writerow(meta)

# You will usually have to add an offset to account for the temperature of
# the sensor. This is usually around 5 degrees but varies by use. Use a
# separate temperature sensor to calibrate this one.
temperature_offset = -5
start_time = time.time()
# while True:
    # print(time.strftime("%H:%M:%S", time.localtime()), end=" ")
    # print("\nTemperature: %0.1f C" % (bme680.temperature + temperature_offset), end=", ")
    # print("Gas: %d ohm" % , end=", ")
    # print("Humidity: %0.1f %%" % bme680.relative_humidity, end=", ")
    # print("Pressure: %0.3f hPa" % bme680.pressure, end=", ")
    # print("Altitude = %0.2f meters" % bme680.altitude)
    
    
    # if time.time() > start_time + 10:
        # break
    # time.sleep(1)

# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Example sketch to connect to PM2.5 sensor with either I2C or UART.
"""



arguments=sys.argv




print("Found PM2.5 sensor, reading data...")

i = 0

while i < (int(arguments[1])):
    time.sleep(1)

    try:
        aqdata = pm25.read()
        temp = bme680.temperature + temperature_offset
        gas = bme680.gas
        humidity = bme680.relative_humidity
        pressure = bme680.pressure
        altitude = bme680.altitude
        
        # print(aqdata)
    except RuntimeError:
        print("Unable to read from sensor, retrying...")
        continue

    # print()
    # print("Concentration Units (standard)")
    # print("---------------------------------------")
    # print(
        # "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
        # % (aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"])
    # )
    # print("Concentration Units (environmental)")
    # print("---------------------------------------")
    # print(
        # "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
        # % (aqdata["pm10 env"], aqdata["pm25 env"], aqdata["pm100 env"])
    # )
    # print("---------------------------------------")
    now = time.strftime("%H:%M:%S", time.localtime())
    value = [aqdata["particles 03um"], temp, gas, humidity, pressure, altitude]
    csvwriter.writerow([now] + value)
    print(aqdata["particles 03um"])
    
    #print("Particles > 0.3um / 0.1L air:", aqdata["particles 03um"])
    # print("Particles > 0.5um / 0.1L air:", aqdata["particles 05um"])
    # print("Particles > 1.0um / 0.1L air:", aqdata["particles 10um"])
    # print("Particles > 5.0um / 0.1L air:", aqdata["particles 50um"])
    # print("Particles > 10 um / 0.1L air:", aqdata["particles 100um"])
    # print("---------------------------------------")
    i+=1
