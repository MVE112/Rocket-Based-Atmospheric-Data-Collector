# Made by: Miguel Velasco Espinosa

from adafruit_bme280 import basic as adafruit_bme280
import board
import time
import datetime
import pymysql
i2c = board.I2C()
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
import smbus
import RPi.GPIO as GPIO
import time,os
from signal import signal, SIGINT, SIGTERM
from sys import exit
GPIO.setwarnings(False)
addr = 0x19
maxScale = 100
LED = 26

allData = open("AllSensorData.txt", "a")

alrtData = open("alertData.txt", "a")

CTRL_REG1 = 0x20
CTRL_REG4 = 0x23
OUT_X_L = 0x28
OUT_X_H = 0x29
OUT_Y_L = 0x2A
OUT_Y_H = 0x2B
OUT_Z_L = 0x2C
OUT_Z_H = 0x2D

POWERMODE_NORMAL = 0x27
RANGE_100G = 0x00
RANGE_200G = 0x10
RANGE_400G = 0x30

bus = smbus.SMBus(1)

GPIO.setmode(GPIO.BCM)
GPIO.setup(LED, GPIO.OUT)
GPIO.output(LED, GPIO.LOW)

def initialize(addr, maxScale):
    scale = int(maxScale)
    
    bus.write_byte_data(addr, CTRL_REG1, POWERMODE_NORMAL)

    if maxScale == 100:
        bus.write_byte_data(addr, CTRL_REG4, RANGE_100G)
    elif maxScale == 200:
        bus.write_byte_data(addr, CTRL_REG4, RANGE_200G)
    elif maxScale == 400:
        bus.write_byte_data(addr, CTRL_REG4, RANGE_400G)
    else:
        print("Error")
def readAxes(addr):
    data0 = bus.read_byte_data(addr, OUT_X_L)
    data1 = bus.read_byte_data(addr, OUT_X_H)
    data2 = bus.read_byte_data(addr, OUT_Y_L)
    data3 = bus.read_byte_data(addr, OUT_Y_H)
    data4 = bus.read_byte_data(addr, OUT_Z_L)
    data5 = bus.read_byte_data(addr, OUT_Z_H)
    
    x = data0 | data1 << 8
    y = data2 | data3 << 8
    z = data4 | data5 << 8

    if x > 32767:
        x -= 65536
    if y > 32767:
        y -= 65536
    if z > 32767:
        z -= 65536
    x = ~x
    y = ~y
    z = ~z
    return x,y,z
def convertToG(maxScale, xAcc1, yAcc1, zAcc1):
    X = (2*float(maxScale) * float(xAcc1))/(2**16);
    Y = (2*float(maxScale) * float(yAcc1))/(2**16);
    Z = -1*((2*float(maxScale) * float(zAcc1))/(2**16))-1;
    return X, Y, Z
def convertToA(maxScale, x, y, z):
    A = abs(x) * 9.81
    B = abs(y) * 9.81
    C = abs(z) * 9.81
    return A, B, C

def cleanup(signal_received, frame):
    allData.close()
    alrtData.close()
    GPIO.cleanup()
    exit(0)
def acc():
    signal(SIGINT, cleanup)
    signal(SIGTERM, cleanup)
    print("Starting Stream")
    initialize(addr,100)

    ts = time.ctime()

    allData.write(str(ts) + "\t")

    xAcc1, yAcc1, zAcc1 = readAxes(addr)

    x, y, z = convertToG(maxScale, xAcc1, yAcc1, zAcc1)

    i, h, u = convertToA(maxScale, x, y, z)
    fo = (0.146)*((i + h + u)/3) #146 grams total rocket weight with payload

    allData.write("x: " + str(i) + "\t" + "y: " + str(h) + "\t" + "z: " + str(u) + "\n")
    if (i >= h and i >= u):
        return i * 0.146
    if(h >= i and h >= u):
        return h * 0.146
    if(u >= i and u >= h):
        return u * 0.146
db = pymysql.connect(host="localhost",user="newuser",password="newuserpassword",database="RaspberryPi")

RD = """INSERT INTO ROCKETDATAC (date_YYYY_MM_DD,time_HH_MM_SS,temp_C,humidity_percent,pressure_hPa,altitude_Meters,launch_force_N) values(%s,%s,%s,%s,%s,%s,%s);"""
cursor = db.cursor()
while True:
    bme280.sea_level_pressure = 1025.096 # value changes depending on day
    pascals = (int)(bme280.pressure + (bme280.altitude / 8.3))
    degrees = (int)(bme280.temperature)
    humidity = (int)(bme280.relative_humidity)
    altitude = (int)(bme280.altitude)
    csvt = datetime.datetime.now()
    today = datetime.date.today()
    date = datetime.date.isoformat(today)
    tim = datetime.datetime.now()
    t = tim.strftime("%H:%M")
    force = acc()

    cursor.execute(RD, (date, t, degrees, humidity, pascals, altitude, force))
    db.commit()
