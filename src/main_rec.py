from mpu6050 import MPU6050
from apds9960 import *
from apds9960 import uAPDS9960 as APDS9960
from nrf24l01 import NRF24L01
from time import time, sleep, sleep_us, ticks_ms, ticks_diff, ticks_add
from pid import PID
from machine import Pin, I2C, PWM, SPI
from vl53l0x2 import setup_tofl_device, TBOOT
import sys
import struct
from array import array
from random import randint

green_led = Pin(26, Pin.OUT)
red_led = Pin(14, Pin.OUT)

csn = Pin(12, mode=Pin.OUT, value=1) # Chip Select Not
ce = Pin(11, mode=Pin.OUT, value=0)  # Chip Enable
led = Pin(25, Pin.OUT)               # Onboard LED
payload_size = 20

i2c0 = I2C(id=0, sda=Pin(0), scl=Pin(1))
i2c = I2C(1, sda=Pin(2), scl=Pin(3), freq=400000)
imu = MPU6050(i2c)

apds = APDS9960(i2c)
apds.enableProximitySensor()
apds.enableLightSensor()

device_1_xshut.value(0)
tof0 = setup_tofl_device(i2c0, 40000, 12, 8)
tof0.set_address(0x31)

pwm_B = PWM(Pin(22))
pwm_A = PWM(Pin(20))

b_bk = Pin(17, Pin.OUT)
b_fd = Pin(16, Pin.OUT)

a_fd = Pin(19, Pin.OUT)
a_bk = Pin(18, Pin.OUT)

stby = Pin(21, Pin.OUT)

stby.on()
pwm_B.duty_u16(0)
pwm_A.duty_u16(0)

start_speed = 30000

Kp = 2
Ki = 0.01
Kd = 0.2
max_inc = 2
pid = PID(p=Kp, i = Ki, d = Kd, imax=max_inc)
target=0

out_L, out_R = start_speed, start_speed

left_turn = 90
right_turn = -90
half_left = 45
half_right = -45
right_adj = -1
left_adj = -1
half_turn = 180
turnCounter = 0
travel_time = array('i')

lastInstruct = 0

red = "N"
blue = "N"
redFound = False
blueFound = False

send_pipe = b"\xd2\xf0\xf0\xf0\xf0"
receive_pipe = b"\xe1\xf0\xf0\xf0\xf0"

def setup():
    nrf = NRF24L01(SPI(0), csn, ce, payload_size=payload_size)
    nrf.open_tx_pipe(send_pipe)
    nrf.open_rx_pipe(1, receive_pipe)
    nrf.flush_tx()
    nrf.flush_rx()
    nrf.start_listening()
    return nrf

nrf = setup()

#RF transceiver methods for receiving robot
def recAngle(angle):
    rfid1 = 0
    rfid2 = 0
    rf1a = 999
    rf2a = 999
    if nrf.any():
        while nrf.any():
            package = nrf.recv()
            (r_angle,rfid, ) = struct.unpack("ii",package)
        

        nrf.stop_listening()
#         print("received: ",message, " from: ", rfid,"; sending: ", num2)
        if rfid == 1:
            rfid1 = rfid
            rf1a = r_angle
        elif rfid == 2:
            rfid2 = rfid
            rf2a = r_angle
        try:
            nrf.send(struct.pack("iiiii", angle, rfid1, rf1a, rfid2, rf2a))
        except OSError:
            pass
        nrf.start_listening()
    r_angle1 = rf1a
    r_angle2 = rf2a
    return r_angle1, r_anlge2

def recCol(r,b):
    msg_r = ""
    msg_b = ""
    if nrf.any():
        while nrf.any():
            package = nrf.recv()
            (message_r,message_b, ) = struct.unpack("ss",package)
        utime.sleep_ms(25)
        nrf.stop_listening()
        msg_r = message_r.decode()
        msg_b = message_b.decode()

        try:
            encoded_r = r.encode()
            encoded_b = b.encode()
            byte_r = bytearray(encoded_r)
            byte_b = bytearray(encoded_b)
            print("sending colours: ", r,b)
            nrf.send(struct.pack("ss", byte_r, byte_b))
        except OSError:
            pass
        nrf.start_listening()
    return msg_r,msg_b

    
def searchLED():
    red_led.value(1)
    green_led.value(0)

def homingLED():
    red_led.value(0)
    green_led.value(1)

def resetPathing():
    global travel_time
    global turnCounter
    turnCounter = 0
    travel_time = array('i')
    
def forwardControl():
    global out_L, out_R
    lout = max(0, out_L)
    rout = max(0, out_R)
    
    gz=round(imu.gyro.z)+1
    adj = pid.get_pid(gz, target)
    
    if adj < 0:
        if adj/2 >= -1:
            out_L = out_L
            out_R = out_R
        else:
            adj = -1*adj
            out_L = out_L+ adj
            out_R = out_R-adj
    elif adj > 0:
        out_R = out_R+adj
        out_L = out_L-adj
    else:
        out_L = out_L
        out_R = out_R
     
    lout = int(max(0, out_L))
    rout = int(max(0, out_R)) 
     
    return lout, rout

def moveForward(speedL, speedR):
    a_fd.value(1)
    a_bk.value(0)
    b_fd.value(1)
    b_bk.value(0)
    pwm_A.duty_u16(speedR)
    pwm_B.duty_u16(speedL)
    
def moveBack(speedL, speedR, t_time):
    a_fd.value(0)
    a_bk.value(1)
    b_fd.value(0)
    b_bk.value(1)
    pwm_A.duty_u16(speedR)
    pwm_B.duty_u16(speedL)
    sleep(t_time)

def turnR(out_L, out_R):
    a_fd.value(0)
    a_bk.value(1)
    b_fd.value(1)
    b_bk.value(0)
    pwm_A.duty_u16(out_R)
    pwm_B.duty_u16(out_L)
    
def turnL(out_L, out_R):
    a_fd.value(1)
    a_bk.value(0)
    b_fd.value(0)
    b_bk.value(1)
    pwm_A.duty_u16(out_R)
    pwm_B.duty_u16(out_L)

def turnAngle(deg,out_L,out_R):
    stop()
    moveBack(out_L, out_R, 0.1)
    angle = 0
    gz = 0
    deltaT_s = 0
    if deg < 0:
        turn = deg*0.944444444
        lastSample = ticks_ms()
        while angle>turn:
            if(ticks_diff(ticks_ms(),lastSample) > 9):
                gz=round(imu.gyro.z)+0.6785
                deltaT_s = (ticks_ms() - lastSample)/1000
                angle = angle + gz*deltaT_s
                lastSample = ticks_ms()
            turnR(out_L,out_R)
    else:
        turn = deg*0.944444444
        lastSample = ticks_ms()
        while angle<turn:
            if(ticks_diff(ticks_ms(),lastSample) > 9):
                gz=round(imu.gyro.z)+0.6785
                deltaT_s = (ticks_ms() - lastSample)/1000
                angle = angle + gz*deltaT_s
                lastSample = ticks_ms()
            turnL(out_L,out_R)
    stop()
    
def stop():
    a_fd.value(0)
    a_bk.value(0)
    b_fd.value(0)
    b_bk.value(0)
    pwm_A.duty_u16(0)
    pwm_B.duty_u16(0)

def wallDetectLeft():
    distL = tof.ping()-50
    if distL <= 100:
        return True
    return False

def wallDetectFront():
    dist = apds.readProximity()
    if dist >= 230:
        return True
    return False

def detectColours():
    redFound="N"
    blueFound = "N"
    bDetect = 100
    rDetect = 50
    r = apds.readRedLight()
    g = apds.readGreenLight()
    b = apds.readBlueLight()
    if(r>=rDetect and b < rDetect):
        redFound = "Y"
    if(b>=bDetect and r < bDetect):
        blueFound= "Y"
    return redFound, blueFound

def updateTargets(red,blue):
    global blueFound, redFound
    return False
    n_red,n_blue = sendDist(red,blue)
    while n_red == "" and n_blue == "":
        n_red,n_blue = sendDist(red,blue)
        if n_blue == "Y":
            blueFound = True
            blue = n_blue
        if n_red == "Y":
            redFound = True
            red = n_red
    return True

def initialiseHeading():
    global initialHeadingComplete
    startHeading = randint(-250,250)
    (angle1,angle2)=sendAngle(startHeading)
    while angle1 == 999 or angle2 == 999:
        (angle1,angle2)=sendAngle(startHeading)
        while startHeading == angle1 or startHeading == angle2:
            (angle1,angle2)=sendAngle(startHeading)
            startHeading = randint(-250,250)
    turnAngle(startHeading)

def newHeading():
    startHeading = randint(-250,250)
    turnAngle(startHeading)
    
initaliseHeading()

try:
    device_1_xshut.value(1)
    sleep_us(TBOOT)

    tof1 = setup_tofl_device(i2c0, 40000, 12, 8)
    
    def wallDetectRight():
        distR = tof1.ping()
        if distR <= 100:
            return True
        return False
    
    def returnHome(counter):
        homingLED()
        global red, blue
        sendColour(red,blue)
        while counter > 0:
            duration = travel_time[counter-2]
            instruct = travel_time[counter-1]
            if instruct == 3:
                turnAngle(half_turn,forwardControl()[0],forwardControl()[1])
            elif instruct == 1:
                turnAngle(left_turn,forwardControl()[0],forwardControl()[1])
            elif instruct == 2:
                turnAngle(half_left,forwardControl()[0],forwardControl()[1])
                 
            start = ticks_ms()
            while ticks_ms() < ticks_add(duration,start):
                sendDist(red,blue)
                if wallDetectFront():
                    
                elif wallDetectLeft():
                    turnAngle(right_adj,forwardControl()[0],forwardControl()[1])
                elif wallDetectRight():
                    turnAngle(left_adj,forwardControl()[0],forwardControl()[1])
                else:
                    moveForward(forwardControl()[0],forwardControl()[1])
            counter=counter-2   
        stop()
        sleep(2)
        newHeading()
    
    last_time = ticks_ms()
    while not blueFound and not redFound:
        searchLED()
        sendColour()
        if wallDetectFront():
            newTravel = ticks_diff(ticks_ms(), last_time)
            travel_time.append(newTravel)
            if detectColours()[0] == "Y":
                red = "Y"
                redFound = True 
                stop()
                sleep(2)
                updateTargets()
                moveBack(forwardControl()[0],forwardControl()[1], 0.1)
                returnHome(turnCounter)
                resetPathing()
                turnAngle(half_turn,forwardControl()[0],forwardControl()[1])
                newHeading()
                last_time = ticks_ms()
            elif detectColours()[1] == "Y":
                blue = "Y"
                blueFound = True
                stop()
                sleep(2)
                updateTargets()
                travel_time.append(3)
                turnCounter = turnCounter + 2
                moveBack(forwardControl()[0],forwardControl()[1], 0.1)
                returnHome(turnCounter)
                resetPathing()
                 turnAngle(half_turn,forwardControl()[0],forwardControl()[1])
                last_time = ticks_ms()
            else:
                travel_time.append(1)
                turnAngle(right_turn,forwardControl()[0],forwardControl()[1])
                turnCounter = turnCounter + 2
                last_time = ticks_ms()
        elif wallDetectLeft():
             turnAngle(right_adj,forwardControl()[0],forwardControl()[1])
        elif wallDetectRight():
            turnAngle(left_adj,forwardControl()[0],forwardControl()[1])
        else:
            moveForward(forwardControl()[0],forwardControl()[1])
        sleep(3)
    stop()
    sleep(1)
    returnHome(turnCounter)        
    stop()
finally:
    tof0.set_address(0x29)
