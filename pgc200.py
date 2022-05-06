   #3_22-21_12_01
#-----  uncomment in real code  --------------------------
# import RPi.GPIO as GPIO         #GPIO
# from bluetooth import *         # bluetooth communication
# import picamera                 # camera
# import picamera.array           # camera

from time import sleep          #delays
import time                     #import sleep  #delays
import cv2                      #image processing
# import serial                   #UART communication
import struct                   #converting numbers
from subprocess import call     #calling OS commands
# import subprocess
import json                     #json format
from math import log10          #calculate log 10
import datetime
from datetime import timedelta
from datetime import datetime
import numpy
import decimal
# import pdb
import base64
import os
# import signal
import sys
import logging
import zlib
# from threading import Thread
import zipfile
# import ftplib
# import re
# from pprint import pprint
import traceback

#------------  comment out in real code, this is emulation  -------------------
from externaldev import picamera,BluetoothSocket
import externaldev
from externaldev import serial

GPIO = externaldev.RPiGPIO() # --- device substitution ---
client_sock = externaldev.bluetooth() #  --- device substitution ---
RFCOMM = 'RFCOMM'
PORT_ANY = 3
# -----------------------------------------------------------------------------

swversion='3.0.34' 
TARGET = (190,10) 
REGULAR=0
LITE=1
CALIBDISTANCEPARAMETER=-213
GROUNDDISTANCEPARAMETER=100
XCALIBRATIONPOSITION=900   
DEFAULT_PLASTIC_Z=3850
Y_REFERENCE_DIFFERENCE=158                                                  
Y_PIXEL_MAX=320                                             
REFERENCE_STEPS=800                                             
REFERENCE_PIXELS_LOW_END=246                                        
SMALL_FLOWER_DISTANCE_MOVE_UP=400  #was 700 changed to prevent motor z stucked while analyzing (kliva)
SMALL_FLOWER_DISTANCE=9
TOFAR_DISTANCE=10.3
DEFAULT_DISTANCE=10.4
MEASURE_DISTANCE_PHASE_1=1
MEASURE_DISTANCE_PHASE_2=2
MEASURE_DISTANCE_PHASE_3=3
MEASURE_DISTANCE_PHASE_4=4
STRIP_DETECTED = 10.5 # detecting reflector edge need to nove all the way to it
FCONDISCONNECT=0.032 # disconnect timeout
FROM_HOME=1
FROM_450=2
STEPS_FOR_MEASURING_WHITE_DISTANCE=450
SLEEP_TIME_WHITE_DIFF=5.2
COUNT_CYCLES_FOR_ADJUST_TEMPERATURE= 431
#-----setup GPIO pins------------------
debugflaginspect=0
BLU_LED = 11
WHITE_LED = 9
CAM_LIGHT = 5
DIR_Y = 19 #WAS 26
STEP_Y = 26 #WAS 19
SWITH_Y = 6
DIR_X = 21
STEP_X = 20
SWITH_X = 16
DIR_Z = 23
STEP_Z = 24
SWITH_Z = 25
SLP = 13
COVER = 12
FAULT_X = 7
FAULT_Z = 1
FAULT_Y = 18
ENABLE_X = 3
ENABLE_Z = 4
ENABLE_Y = 2
PAIR = 17
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# GPIO.setup(FAULT_X, GPIO.IN)
# GPIO.setup(FAULT_Y, GPIO.IN)
# GPIO.setup(FAULT_Z, GPIO.IN)

GPIO.setup(ENABLE_X, GPIO.OUT)
GPIO.setup(ENABLE_Y, GPIO.OUT)
GPIO.setup(ENABLE_Z, GPIO.OUT)
GPIO.setup(DIR_X, GPIO.OUT)
GPIO.setup(STEP_X, GPIO.OUT)
GPIO.setup(SWITH_X, GPIO.IN)
GPIO.setup(DIR_Y, GPIO.OUT)
GPIO.setup(STEP_Y, GPIO.OUT)
GPIO.setup(SWITH_Y, GPIO.IN)
GPIO.setup(DIR_Z, GPIO.OUT)
GPIO.setup(STEP_Z, GPIO.OUT)
GPIO.setup(SWITH_Z, GPIO.IN)
GPIO.setup(SLP, GPIO.OUT)
GPIO.setup(BLU_LED, GPIO.OUT)
GPIO.setup(WHITE_LED, GPIO.OUT)
GPIO.setup(CAM_LIGHT, GPIO.OUT)
GPIO.setup(COVER, GPIO.IN)
GPIO.setup(PAIR, GPIO.IN)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18,0)
GPIO.setup(8, GPIO.OUT)
GPIO.output(8,0)
GPIO.setup(7, GPIO.OUT)
GPIO.output(7,0)
GPIO.setup(27, GPIO.OUT)
GPIO.output(27,0)
GPIO.setup(22, GPIO.OUT)
GPIO.output(22,0)
GPIO.setup(10, GPIO.OUT)
GPIO.output(10,0)
GPIO.output(BLU_LED,GPIO.HIGH)
GPIO.output(WHITE_LED,GPIO.HIGH)
blu_led = GPIO.PWM(BLU_LED, 1)
white_led = GPIO.PWM(WHITE_LED, 1)
blu_led.ChangeDutyCycle(50)
GPIO.output(CAM_LIGHT,GPIO.HIGH)
#---------constant values---------------
DEV_MODEL = "GC-200"
HOSTMODE = "Bluetooth"
FIRMWARE_VERSION = '0.01'
SOFTWARE_VERSION = '1.0'
Y_HEAT_DISTANCE=950
safe_y_distance_up=300              # for preventin collision when going up --- для предотвращения столкновения при подъеме
FIX_STEPS_X_DEFAULT=300             # steps of x motor fix microswitch --- шаги мотора по оси x при нажатиии микровыключателя
FIX_STEPS_Y_DEFAULT=300             # steps of y motor fix microswitch --- шаги мотора по оси y при нажатиии микровыключателя
FIX_STEPS_Z_DEFAULT=300             # steps of z motor fix microswitch --- шаги мотора по оси z при нажатиии микровыключателя
SAFE_Y_UP=900             
stabilityWarmupTime =200 # время прогрева стабильности
examinationWarmupTime = 500 # проверочное время прогрева
MICROSWITCH_TOUCH_COUNTER=4
BLUETOOTH_DISCONNECT_CYCLE_COUNTER=4 # СЧЕТЧИК ЦИКЛОВ ОТКЛЮЧЕНИЯ BLUETOOTH
SAVE_RCLOCAL_FLAG=0
SAVE_RCL_FLAG = False
ValueRClocalDefaultB='(/home/pi/GcWatchdog/GcWatchdog proven python /home/pi/gc200B/pgc200.py)&'
ValueRClocalDefaultA='(/home/pi/GcWatchdog/GcWatchdog proven python /home/pi/gc200A/pgc200.py)&'

compensation_topOfFlower = 1200 #while calculating bud center add this to get the right flower length
FORWARD = 0
REVERS = 1
ZFORWARD = 0
ZREVERS = 1
YFORWARD = 0
YREVERS = 1
XFORWARD = 1
XREVERS = 0

delay_x = .0001             # .002                #delay between motor pulses
delay_y = .0001             #delay between motor pulses --- задержка между импульсами двигателя по оси x
delay_z = .0001             #delay between motor pulses --- задержка между импульсами двигателя по оси z
DefaultBudLength = 18.0     # in mm
mm_to_step_Y = 158          # 1 step = 0.0063 mm
mm_to_step_Z = 158          # 1 step = 0.0063 mm
mm_to_step_X = 158          # 1 step = 0.054 mm
num_of_cycles=0

wcsZ= 9500 #6000 #
wcsY=3000  #4000
wcsX=900  #4000
versiondotz=4800

bud_length_start_steps=6000 # начальные шаги по длине бутона
bud_length_in_steps_plus200=6200 # шаги по длине бутона +200
found_counter_threshold=5 # найденный порог счетчика
bud_length_start_mm=35      # bud length start mm --- длина бутона, мм
auto_mode = 1 # режим авто
thresholdForDistance = 80 # пороговое значение для расстояния
manual_mode = 0 # ручной режим
distanceCorrection= 0 # коррекция расстояния
maxtemerature=57 # максимальная температура
firstconnectionTimout="60000" # время ожидания первого подключения
connectionTimout="9000" # время ожидания подключения
# stabilityTimout="90000" # время стабилизации
lenghtMeasureStepTimout="60000" # время ожидания шага измерения длины
distanceMeasureStepTimout="40000" # время ожидания шага измерения расстояния
temperatureTimout="15000" # время ожидания температуры
calibrateTimout="125000" # Время калибровки
singleSpectratMeasurementTimout="40000" # время ожидания одного Спектрального измерения
bitTimout ="60000" # время ожидания бита
examTimout ="170000" # время ожидания проверки
stabilityTimout ="150000" # время ожидания стабильности
functionStartTimout="40000" # время ожидания запуска функции
dir_pathA = '/home/pi/gc200A'
dir_pathB = '/home/pi/gc200B'
full_pathOf_newVersionA = '/home/pi/gc200A/pgc200.py'
full_pathOf_newVersionB = '/home/pi/gc200B/pgc200.py'

bud_length_gap=200 # разрыв в длине бутона
bud_length_gap_bit=500 # разрыв в длине бутона в битах

serialNum = 70000 # серийный номер
spectserialnum = 70001 #  спектральный серийный номер

STATEFORLAMPTIME_STARTUP = 1 # СОСТОЯНИЕ ДЛЯ ЗАПУСКА ВО ВРЕМЯ РАБОТЫ ЛАМПЫ
STATEFORLAMPTIME_LAMPOFF = 2 # СОСТОЯНИЕ ДЛЯ ВРЕМЕНИ ВЫКЛЮЧЕНИЯ ЛАМПЫ
STATEFORLAMPTIME_LAMPON = 3 # СОСТОЯНИЕ ДЛЯ ВРЕМЕНИ ВКЛЮЧЕНИЯ ЛАМПЫ
STATEFORLAMPTIME_SENDBIT = 4 # СОСТОЯНИЕ ДЛЯ ВРЕМЕНИ ОТПРАВКИ ЛАМПЫ
STATEFORLAMPTIME_PREIODIC = 5 # СОСТОЯНИЕ ДЛЯ ВРЕМЕНИ ПЕРЕД ЛАМПЫ

#---------init values-------------------------------
class Point(object): # структура данных Point
    """__init__() functions as the class constructor"""
    def __init__(self, distance=None, measurement_origin=None,zlocation=None):
        self.distance = distance
        self.measurement_origin = measurement_origin
        self.zlocation = zlocation

class locations: # system location class in 3 axis: x,y,z --- класс расположения системы по 3 осям: x,y,z
    def __init__(self,data):  # initialize locations to received value --- инициализировать местоположения в соответствии с полученным значением
        self.x = data
        self.y = data
        self.z = data

class flags:              # инициализировать флаги полученным значением
    def __init__(self,data):  # initialize flags to received value
        self.gspectwhversion = data
        self.ymicroswitch = data
        self.gspectsensortype = data
        self.gspectserialnum = data
        self.distance_list = data
        self.distance_list_measurement_origin = data
        self.z_list = data
        self.PointsList = data
        self.counterNonZeroDistance= data
        self.averageDistance = data
        self.bud_center_z = data
        self.NumberOfLocations = data
        self.correction_factor = data
        self.presetYlocation = data
        self.exam_results = data
        self.BudLength = data
        self.threadFlag = data
        self.bud_length_second_steps  = data
        self.SpectrometerExists=False
        self.config = data
        self.calibration_results = data
        self.calibrationmaterial_results = data
        self.dark_results = data
        self.reference_results = data
        self.watchdog= data # наблюдатель
        self.log=data
        self.WithContainer=data
        self.BlueToothConnected=data
        self.client_sock=data
        self.client_info=data
        self.lastcommand=data
        self.safe_y_distance =data
        self.safe_z_distance =data
        self.MAX_RANGE_Z  =data
        self.MAX_RANGE_Y =data
        self.MAX_RANGE_X = data
        self.bud_center_x = data
        self.bud_center_z=data
        self.spctrometer_number=data
        self.spectrometer_edge_in_image=data
        self.spectrometer_center_in_image=data
        self.image_crop_right=data
        self.pixel_to_mm=data
        self.BUD_FRONT_Y=data
        self.versionprovedsent=data
        self.btcontime_1=data
        self.btcontime_2=data
        self.wasopen=data
        self.budDistance=data
        self.budDistanceFlag=data
        self.strip_edge_in_image_spectrometer_side=data
        self.strip_edge_in_image_reflector=data
        self.whiteExists_edge_in_image_reflector_side=data
        self.whiteExists_edge_in_image_back_side=data
        self.whiteExists_edge_in_image_left_side=data
        self.whiteExists_edge_in_image_right_side=data
        self.whiteExistsBlkCrp_edge_in_image_reflector_side=data
        self.whiteExistsBlkCrp_edge_in_image_back_side=data
        self.whiteExistsBlkCrp_edge_in_image_left_side=data
        self.whiteExistsBlkCrp_edge_in_image_right_side=data
        self.spectrometer_edge_in_image_white=data
        self.y_whiteExists_distance=data
        self.inprocessFlag =data
        self.coolingFlag=data
        self.TemperatureUpperLimit=data
        self.TemperatureLowerLimit=data
        self.LampCoolDownTimeForSoftware=data
        self.CoolDownHysteresis=data
        self.powerButtonPressed=data
        self.powerButtonPressedTime=data
        self.distance_from_home =data
        self.btunpair=data
        self.server_sock=data
        self.small_flower=data
        self.bitPerformed=data
        self.heatingDownPosition=data
        self.xswitch=data
        self.zswitch=data
        self.ZmicroswitchExeption=data
        self.dissconnectioncounter =data
        self.ignorePowerButtonPressed =data
        self.fix_steps_x= data
        self.fix_steps_y=data
        self.fix_steps_z= data
        self.mode= data
        self.configExtract = data
        self.calibration_y_steps = data
        self.lampOn = data
        self.lampStartTime = data
        self.lampElapsedTime = data
        self.gotConfig=data
        self.shorten_log_size=data
        self.shorten_exceptionlog_size=data
        self.deviceTipe=data
        self.extractYoffset=data
        self.canHeatForward=data

class btconnectionTime: # инициализировать время подключения bluetooth до полученного значения , было открыто
    def __init__(self,data):  #initialize bt connection Time to received value , was open
        self.btcontime_1=data
        self.btcontime_2=data

class bitarray: # инициализировать флаги полученным значением
    def __init__(self,data):  #initialize flags to received value ,
        self.bit_arr = {}
        self.bit_arr['MotorY'] = data
        self.bit_arr['MotorX'] = data
        self.bit_arr['MotorZ'] = data
        self.bit_arr['Spectrometer'] = data
        self.bit_arr['Camera'] = data
        self.bit_arr['CameraRed'] = data
        self.bit_arr['CameraGreen'] = data
        self.bit_arr['CameraBlue'] = data
        self.bit_arr['Spectrometer'] = data
        self.bit_arr['TemperatureLampOff'] = data
        self.bit_arr['TemperatureLampOn'] = data
        self.bit_arr['LampCurrentsOff'] = data
        self.bit_arr['LampCurrentsOn'] = data
        self.bit_arr['Switch_X_phase1'] = data
        self.bit_arr['Switch_Z_phase1'] = data
        self.bit_arr['Switch_Y_phase1'] = data
        self.bit_arr['Switch_X_phase2'] = data
        self.bit_arr['Switch_Y_phase2'] = data
        self.bit_arr['Switch_Z_phase2'] = data
        self.bit_arr['Switch_X_phase3'] = data
        self.bit_arr['Switch_X_phase3'] = data
        self.bit_arr['Switch_Y_phase3'] = data
        self.bit_arr['Switch_Z_phase3'] = data
        self.bit_arr['BudInside'] = data
        self.bit_arr['Lastexception']=data
        self.bit_arr['LampElapsedTime'] =data
        self.bit_arr['Command'] =data
        self.bit_arr['SdVersion'] =data
        self.bit_arr['DrawerOpen'] =data

        
class lampReadData:             # считанные данные лампы system location in 3 axis: x,y,z
    def __init__(self,data):    #initialize locations to received value  --- инициализировать местоположения в соответствии с полученным значением
        self.datetimeStr = data
        self.duration = data

# настройка связи спектрометра, через serial
spectrometer = serial.Serial("/dev/ttyS0", baudrate=115200, timeout= 30)      #setup spectrometer communication
camera = None
pipe_name = None   #= 'pipe_test'
pid  = None
pipeout = None

#---------functions-----------------------

# инициализация устройства
def InitDeviceId():
    f = open('/etc/machine-info','r') # считываем информацию об устройстве из этого файла
    line = f.readline()
    words = line.split(" ")
    print (words[0])
    print (words[1])
    f.close()
    try:
        spectrometer.write("!\r")
        spectrometer.readline()
        flgs.ymicroswitch = 0
        spectrometer.write("h0\r")
        flgs.gspectsensortype = spectrometer.readline()
        print(flgs.gspectsensortype)
        spectrometer.write("h1\r")
        flgs.gspectwhversion = spectrometer.readline() # hardware ver
    except Exception as e:
        logexception(e)
        print("Unexpected SPECTROMETER error")
        flgs.gspectserialnum = words[1]
        return False
    if flgs.gspectwhversion =='':
        print ("Unexpected SPECTROMETER error")
        flgs.gspectserialnum = words[1]
        return False
    print(flgs.gspectwhversion)
    spectrometer.write("h2\r")
    spectserialnum = spectrometer.readline()
    print(spectserialnum)
    flgs.gspectserialnum = spectserialnum
    print('after split')
    return True

# установить устройство set device Id
def SetDeviceId(devid):
    f = open('/etc/machine-info','r')
    line = f.readline()
    words = line.split(" ")
    print(words[0])
    print(words[1])
    f.close()
    f1 = open('/etc/machine-info','w+')
    machineName = "%s %s"%(words[0],devid)
    print(machineName)
    f1.writelines(machineName)
    f1.close()

# Get Device Id
def GetDevId():
    f = open('/etc/machine-info','r')
    line = f.readline()
    words = line.split(" ")
    print(words[0])
    print(words[1])
    f.close()
    return words[1].strip()

# get Temperature value
def getTemperature():
    spectrometer.write("St\r")
    Temperature = spectrometer.readline()
    tempr = Temperature.split(" ")
    log( 'Temperature = '+tempr[1].rstrip() )
    return tempr[1].rstrip()

def ledOn():
    GPIO.output(CAM_LIGHT,GPIO.LOW) #turn on camera light включить свет камеры

def ledOff():
    GPIO.output(CAM_LIGHT,GPIO.HIGH) #turn off camera light выключить свет камеры

def GetSpectDevInfo(): #Get Spectrometer Device Info Получить Информацию Об Устройстве Спектрометра
    print('IN GetSpectDevInfo')
    print(flgs.gspectserialnum)
    spectserialnum = flgs.gspectserialnum
    print(spectserialnum)
    spectserial = spectserialnum.split(" ")
    spectserial1 = spectserial[1].split("\r\n")
    print(spectserial1[0])
    z = "%s" %spectserial1[0]
    log('spectserial') # эти номера разные
    log(z)
    deviceId=GetDevId()

    log('deviceId') # эти номера разные
    log(str(deviceId))
    frontDistance = int( flgs.BUD_FRONT_Y/mm_to_step_Y)
    REFLECTORVERSION='0'
    sdvr=str(getSdVersion())
    json_file='/home/pi/jsons/gc200conf.json'
    json_data1=open(json_file)
    configvalues = json.load(json_data1)
    print(configvalues)
    json_data1.close()
    info = {'SpectrometerSerialNumber': z ,'SerialNumber': deviceId ,'FirmwareVersion': FIRMWARE_VERSION ,
            'ReflectorVersion': REFLECTORVERSION  , 'SoftwareVersion': swversion,'BudFrontDistance':frontDistance ,
            'SdVersion': sdvr , 'Command':flgs.lastcommand ,'ConfFile': str(configvalues)}
    SendHostMessage('%s\n' %str(info))
    log('FTR SendHostMessage 590')
    print(info)
    return info

def getpid(process_name):
    import os
    return [item.split()[1] for item in os.popen('tasklist').read().splitlines()[4:] if process_name in item.split()]

# Перемещение по оси x в полученное местоположение, возврат в исходное местоположение после перемещения
def Move_x(new_position): #Move along x axis to received location, return location after moving
    if flgs.deviceTipe == LITE:
        return
    print('Move_x')
    log('Move_x pos.x %d'%pos.x)
    # log( 'Move_x pos.x %d'%pos.x )
    switch_x_counter=0
    GPIO.output(ENABLE_X ,GPIO.LOW)
    if new_position > flgs.MAX_RANGE_X:
        new_position = flgs.MAX_RANGE_X
    print('pos.x %d',%pos.x)
    print('new_position %d' %new_position)
    if new_position > pos.x:
        GPIO.output(DIR_X, XFORWARD)
        print('#new_position > pos.x Move_x()pos.x %d' %pos.x)
        print('#new_position > pos.xMove_x()new_position %d' %new_position)
        for x in range(new_position-pos.x):
            GPIO.output(STEP_X, GPIO.HIGH)
            sleep(delay_x)
            GPIO.output(STEP_X, GPIO.LOW)
            sleep(delay_x)
    elif new_position < pos.x:
        GPIO.output(DIR_X, XREVERS)
        print('Move_x new_position < pos.x')
        print('new_position  pos.x Move_x()pos.x %d' %pos.x)
        print('new_position < pos.xMove_x()new_position %d' %new_position)
        steps_to_move_bk = pos.x - new_position
        if steps_to_move_bk == pos.x :
            steps_to_move_bk=flgs.MAX_RANGE_X+20

        for x in range(steps_to_move_bk):
            GPIO.output(STEP_X, GPIO.HIGH)
            sleep(delay_x)
            GPIO.output(STEP_X, GPIO.LOW)
            sleep(delay_x)
            switch_x=GPIO.input(SWITH_X)
            
            if (switch_x):
                switch_x_counter=switch_x_counter+1
                log('x microswitch switch_x_counter=%d'%switch_x_counter)
            if (switch_x==0 and switch_x_counter>0): # ситуация когда микровыключатель нажат один раз
                switch_x_counter=0
                log('x microswitch switch_x_counter=%d'%switch_x_counter)
                # write_to_exceptionlog('x microswitch pressed ONCE')
                sexeption='x microswitch pressed ONCE'
                check_if_exception_in_file(sexeption)

            if (switch_x_counter >= MICROSWITCH_TOUCH_COUNTER): # анализируем сенсорный счетчик выключателя
                flgs.xswitch=1
                log('MICROSWITCH_TOUCH_COUNTER x microswitch switch_x_counter=%d'%switch_x_counter)
                fixing_home_x()
                pos.x = flgs.fix_steps_x
                return pos.x

    GPIO.output(ENABLE_X ,GPIO.HIGH)
    pos.x = new_position
    return pos.x

    
def Home_x():        # домашнее положение по x
    if flgs.deviceTipe == LITE:
        return
    print('Home_x()pos.x =%d' %pos.x)
    Move_x(0)
    print('FTR Move_x(0) pos.x =%d' %pos.x)
    log( 'Home_x()pos.x %d' %pos.x)
    
def fixing_home_x():                #fix home x after switch
    if flgs.deviceTipe ==  LITE:
        return
    print('fixing_home_x()')
    GPIO.output(DIR_X, XFORWARD)
    for x in range(flgs.fix_steps_x):
        GPIO.output(STEP_X, GPIO.HIGH)
        sleep(delay_x)
        GPIO.output(STEP_X, GPIO.LOW)
        sleep(delay_x)
    GPIO.output(ENABLE_X ,GPIO.HIGH)
    flgs.xswitch=1
    log ('EXIT fixing_home_x()')

# Перемещение по оси y в полученное местоположение, возврат в исходное местоположение после перемещения
def Move_y(new_position):            #Move along y axis to received location, return location after moving
    log('Move_y()')
    switch_y_counter=0
    if pos.z < flgs.safe_z_distance and  new_position > 0 :
        print('Z position is too DOWN!')
        return pos.y
    if new_position > flgs.MAX_RANGE_Y:
        new_position = flgs.MAX_RANGE_Y
    GPIO.output(ENABLE_Y ,GPIO.LOW)
    if new_position > pos.y:
        GPIO.output(DIR_Y, REVERS)
        for x in range(new_position-pos.y):
            GPIO.output(STEP_Y, GPIO.HIGH)
            sleep(delay_y)
            GPIO.output(STEP_Y, GPIO.LOW)
            sleep(delay_y)
    elif new_position < pos.y:
        GPIO.output(DIR_Y, FORWARD)
        steps_to_move_bk= pos.y - new_position
        if steps_to_move_bk== pos.y : #home
            steps_to_move_bk=flgs.MAX_RANGE_Y+20        
        for x in range(steps_to_move_bk):
            GPIO.output(STEP_Y, GPIO.HIGH)
            sleep(delay_y)
            GPIO.output(STEP_Y, GPIO.LOW)
            sleep(delay_y)
            switch_y=GPIO.input(SWITH_Y)
            if (switch_y):
                switch_y_counter=switch_y_counter+1
            if (switch_y==0 and switch_y_counter>0 ):
                switch_y_counter=0
                sexeption='y microswitch pressed ONCE'
                check_if_exception_in_file(sexeption)
                # write_to_exceptionlog('y microswitch pressed ONCE')
            if (switch_y_counter >= MICROSWITCH_TOUCH_COUNTER):
                flgs.ymicroswitch = 1
                print 'flgs.ymicroswitch CLOSED'
                log( 'flgs.ymicroswitch CLOSED')
                fixing_home_y()
                pos.y = flgs.fix_steps_y
                return pos.y
    GPIO.output(ENABLE_Y ,GPIO.HIGH)
    pos.y = new_position
    return pos.y

# Перемещение по оси y в полученное местоположение, возврат в исходное местоположение после перемещения
def Move_yForCalibration(new_position):  #Move along y axis to received location, return location after moving
    log( 'Move_yForCalibration() ')
    if new_position > flgs.MAX_RANGE_Y:
        new_position = flgs.MAX_RANGE_Y
    GPIO.output(ENABLE_Y ,GPIO.LOW)
    if new_position > pos.y:
        GPIO.output(DIR_Y, REVERS)
        for x in range(new_position-pos.y):
            GPIO.output(STEP_Y, GPIO.HIGH)
            sleep(delay_y)
            GPIO.output(STEP_Y, GPIO.LOW)
            sleep(delay_y)
    elif new_position < pos.y:
        GPIO.output(DIR_Y, FORWARD)
        for x in range(pos.y - new_position):
            GPIO.output(STEP_Y, GPIO.HIGH)
            sleep(delay_y)
            GPIO.output(STEP_Y, GPIO.LOW)
            sleep(delay_y)
            if (GPIO.input(SWITH_Y)):
                fixing_home_y()
                pos.y = 0
                return pos.y
    GPIO.output(ENABLE_Y ,GPIO.HIGH)
    pos.y = new_position
    return pos.y
# установим в домашнее состояние после переключения
def fixing_home_y():                #fix home y after switch
    GPIO.output(ENABLE_Y ,GPIO.LOW)
    GPIO.output(DIR_Y, REVERS)
    for x in range(flgs.fix_steps_y):
        GPIO.output(STEP_Y, GPIO.HIGH)
        sleep(delay_y)
        GPIO.output(STEP_Y, GPIO.LOW)
        sleep(delay_y)
    GPIO.output(ENABLE_Y ,GPIO.HIGH)

# Переместить ось y домой, верните значение True или False для встроенного теста
def Home_y():                            #Move y axis home, return True or False for built-in test
    print('Home_y()pos.y =%d' %pos.y)
    log('b4 move flgs.ymicroswitch=%d'%flgs.ymicroswitch)
    Move_y(0)    
    log('ftr move flgs.ymicroswitch=%d'%flgs.ymicroswitch)
    print('FTR Move_y(0) pos.y =%d' %pos.y)
    log( 'Home_y()pos.y %d' %pos.y)
    
# Перемещение по оси z в полученное местоположение, возврат в исходное местоположение после перемещения
def Move_z(new_position):            #Move along z axis to received location, return location after moving
    switch_z_counter= 0
    if new_position== 0:
        yflag = HomeYTest()
        if yflag == False:
            return False
    if new_position > flgs.MAX_RANGE_Z:
        new_position = flgs.MAX_RANGE_Z
    GPIO.output(ENABLE_Z ,GPIO.LOW)
    if new_position > pos.z:
        GPIO.output(DIR_Z, FORWARD)
        for x in range(new_position-pos.z):
            GPIO.output(STEP_Z, GPIO.HIGH)
            sleep(delay_z)
            GPIO.output(STEP_Z, GPIO.LOW)
            sleep(delay_z)
    elif new_position < pos.z:
        GPIO.output(DIR_Z, REVERS)
        steps_to_move_bk= pos.z - new_position
        if steps_to_move_bk== pos.z :
            steps_to_move_bk=flgs.MAX_RANGE_Z+20        
        for x in range(steps_to_move_bk):
            GPIO.output(STEP_Z, GPIO.HIGH)
            sleep(delay_z)
            GPIO.output(STEP_Z, GPIO.LOW)
            sleep(delay_z)
            switch_z=GPIO.input(SWITH_Z)
            if (switch_z):
                switch_z_counter=switch_z_counter+1
            if (switch_z==0 and switch_z_counter>0):
                switch_z_counter=0
                if flgs.ZmicroswitchExeption==False:
                    # write_to_exceptionlog('z microswitch pressed ONCE')
                    sexeption='z microswitch pressed ONCE'
                    check_if_exception_in_file(sexeption)
                    log('z microswitch switch_x_counter=%d'%switch_z_counter)
                    flgs.ZmicroswitchExeption=True
            if (switch_z_counter >= MICROSWITCH_TOUCH_COUNTER):
                flgs.zswitch=1  
                switch_z_counter=0
                log('MICROSWITCH_TOUCH_COUNTER z microswitch switch_z_counter=%d'%switch_z_counter)
                print('z microswitch pressed')
                fixing_home_z()
                pos.z =flgs.fix_steps_z
                return pos.z
    GPIO.output(ENABLE_Z ,GPIO.HIGH)
    pos.z = new_position
    return pos.z

# зафиксировать домашнее состояние z после переключения
def fixing_home_z():                    #fix home z after switch
    GPIO.output(ENABLE_Z ,GPIO.LOW)
    GPIO.output(DIR_Z, FORWARD)
    for x in range(flgs.fix_steps_z):
        GPIO.output(STEP_Z, GPIO.HIGH)
        sleep(delay_z)
        GPIO.output(STEP_Z, GPIO.LOW)
        sleep(delay_z)
    GPIO.output(ENABLE_Z ,GPIO.HIGH)

# Переместите ось z домой, верните значение True или False для встроенного теста
def Home_z():                        #Move z axis home, return True or False for built-in test
    Move_y(250)
    Move_z(0)
    return True

# Переместить всю систему домой
def Home():      #Move all system home
    log( 'Home() ')
    log('pos.y='+str(pos.y))
    # Move_z(flgs.fix_steps_y)
    Move_y(SAFE_Y_UP)
    # Move_y(flgs.fix_steps_y+)
    Home_x()  # home x before home z to prevent hitting drawer axis while going down
    Home_z()
    Home_y()

# Переместить всю систему домой
def HomeYTest():      #Move all system home
    flgs.ymicroswitch = 0
    Home_y()
    if flgs.ymicroswitch == 1:
        return True
    else:
        print('Y is not @ HOME')
        return False

# Переместите всю систему домой поэтапно, чтобы избежать столкновения с крышкой
def HomeFromExtract():      #Move all system home in steps so no cllision with cover
    Move_y(pos.y-550)
    sleep(0.1)
    Move_z(flgs.safe_z_distance)
    Home_y()

# Получить изображение с камеры, вернуть RGB-массив
def GetCameraImage(cam_bit = 0):    #Get image from camera, return RGB array
    log( 'GetCameraImage() ')
    GPIO.output(CAM_LIGHT,GPIO.LOW) #turn on camera light
    sleep(0.5)
    frame = picamera.array.PiRGBArray(flgs.camera)
    flgs.camera.capture(frame,'bgr')
    if cam_bit:
            return True
    return frame.array

# Команда "Получить ток лампы" (строка 'Sc\r' или 'Sc\n').
def Getpectlampcurrents():    # 'Get lamp currents' command (string 'Sc\r' or 'Sc\n').
    spectrometer.write("Sc\r")
    data_received = spectrometer.readline()
    print('Spect lamp currents %s' %data_received)
    return data_received

# Проверка спектрометра
def CheckSpectResponse():    #Check spectrometer
    spect_bit = False
    try:
        spectrometer.write("!\r")
        data_received = spectrometer.readline()
        if(data_received):
            spect_bit = True
        else:
            print('CheckSpectResponse FALSE')
    except Exception as e:
        logexception(e)
        print("Spectrometer FATAL Error")
    return spect_bit

# Получить спектрометр FirmVersion
def GetSpectFirmVersion(): #Get Spectrometer FirmVersion
    spectrometer.write("i\r")
    data_received = spectrometer.readline()
    return data_received

# Установите Интенсивность Лампы Спектрометра
def SetSpectIntensity(lump_lvel): #Set Spectrometer Lamp Intensity
    if(0 <= lump_lvel <= 100):
        spectrometer.write("LI%d\r" %lump_lvel)
        data_received = spectrometer.readline()
        return data_received
    else: 
        return "ERROR, invalid value"

# Установите длины волн для спектрометра
def SpectSetWL(points):        #Set wavelengths to spectrometer
    log( 'SpectSetWL() ')
    if points <= 0:
        points = 1
    if points > 511:
        points = 511
    minWl = 1550.0
    maxWl = 1950.0
    step = (maxWl - minWl)/(points - 1)
    for i in range(points ):
        spectrometer.write('W%d,%f\r' %(i,float(minWl + step*i)))
        log( spectrometer.readline())
    if (points < 511):
        spectrometer.write('W%d,%f\r' %(i+1, 0.0))
        log(spectrometer.readline())
    return True

# Настройте спектрометр в соответствии с полученными данными
def SpectConfig():   #Configure spectrometer according received data
    log( 'SpectConfig() ')
    NumOfPoints=flgs.config["spectrometerAttributes"]['NumOfPoints']
    SpectSetWL(NumOfPoints)
    # set wavelength average and scan average
    # установите среднее значение длины волны и среднее значение сканирования
    spectrometer.write('V%d,%d\r' %(flgs.config["spectrometerAttributes"]['WavelengthAverage'],
                                    flgs.config["spectrometerAttributes"]['ScanAverage']))
    print(spectrometer.readline())
    LampControlMode=flgs.config["spectrometerAttributes"]['LampControlMode']
    # set lamp control mode, 1 = auto, 0 = manual
    # установите режим управления лампой, 1 = auto, 0 = manual
    spectrometer.write('LM%d\r' %LampControlMode)
    print(spectrometer.readline())
    LampWarmupTime=flgs.config["spectrometerAttributes"]['LampWarmupTime']
    # установите время прогрева света
    spectrometer.write('LT%d\r' %LampWarmupTime)        #set light warm up time
    print(spectrometer.readline())
    autoSubtract=1
    # установите режим управления лампой, 1 = auto, 0 = manual
    spectrometer.write('PB%d\r' %autoSubtract)        #set lamp control mode, 1 = auto, 0 = manual
    print(spectrometer.readline())

# Включить лампу
def SpectTurnOnLamp():
    log( 'SpectTurnOnLamp() ')
    if flgs.lampOn==0: #TODO(Ido): Undefined fixed
        flgs.lampStartTime=time.time()
        spectrometer.write('LI100\r' ) # set lamp intensity --- установить интенсивность лампы 100 % ---
        print(spectrometer.readline())
    if flgs.gotConfig==False:  # meaning not gc called yet --- это означает, что gc еще не вызван ---
        sleep(4)
    else:
        sleep(flgs.config["spectrometerAttributes"]['LampWarmupTime']/1000)
    flgs.lampOn=1
    log('flgs.lampOn=1')

# Включить фонарь в процентах освещенности
def SpectTurnOnLamp_percent(percent):
    log( 'SpectTurnOnLamp() ')
    if flgs.lampOn==0: #TODO(Ido): Undefined fixed
        flgs.lampStartTime=time.time()
        strWritetospect= 'LI'+percent+'\r'
        spectrometer.write(strWritetospect ) # set lamp intensity --- установить интенсивность лампы ---
        print(spectrometer.readline())
    if flgs.gotConfig==False:  # meaning not gc called yet  --- это означает, что gc еще не вызван ---
        sleep(4)
    else:
        sleep(flgs.config["spectrometerAttributes"]['LampWarmupTime']/1000)
    flgs.lampOn=1
    log('flgs.lampOn=1')

# Установите интенсивность лампы спектрометра на 0%, выключить лампу
def SpectTurnOffLamp():  #Set spectrometer lamp intensity to 0%
    log( 'SpectTurnOffLamp() ')
    print('SpectTurnOffLamp()')
    spectrometer.write('LI0\r')
    print(spectrometer.readline())
    flgs.lampOn=0
    log('flgs.lampOn=0')

# получить время воследней индикации из log
def getLastLampTimeFromLog():
    print('getLastLampTimeFromLog')
    log( 'getLastLampTimeFromLog')
    print(os.system(' grep -a -n "STARTING" /home/pi/LogsGc200/logGc200.txt>starting.txt'))
    num_lines = sum(1 for line in open('starting.txt'))
    print('linecount=%d'%num_lines)
    lastline=''
    f = open('starting.txt','r')
    Starting=0
    for line in f:
        if '---------------------STARTING GC200' in line:
            Starting=1
        lastline= line        
    f.close()
    print(lastline)
    linenum = lastline.split(":")
    print(linenum[0])
    linenumb4boot= int(linenum[0])-2      
    lineBeforeRebootCmd= 'sed -n '+str(linenumb4boot)+ 'p /home/pi/LogsGc200/logGc200.txt>lineBeforeReboot.txt'
    os.system(lineBeforeRebootCmd)
    lineBeforeReboot=''
    file = open('lineBeforeReboot.txt','r')
    for line in file:        
        lineBeforeReboot=line        
    file.close()
    print(lineBeforeReboot)
    print('----------------')
    stripedlineBeforeReboot = lineBeforeReboot.rstrip() 
    print('----------------')
    print(stripedlineBeforeReboot)
    log('^^^ stripedlineBeforeReboot=%s'%stripedlineBeforeReboot)
    if stripedlineBeforeReboot=='':
        log("empty stripedlineBeforeReboot ")
        timeLampWasOn= '0:00'
        return timeLampWasOn
    print('----------------@')
    hourLine = ''
    timeLampWasOn= '0:00'
    try:
        log("b4 split ")
        hourLine = stripedlineBeforeReboot.split(" ")
        log("FTR split ")
        timeLampWasOn= hourLine[0]+" "+ hourLine[1]
    except Exception as e:
        logexception(e)
    print(timeLampWasOn)
    return timeLampWasOn

# считать индикатор из времени файла
def rdLampElapsedTimeFile():
    f = "/home/pi/LogsGc200/lampElapsedTime.txt"
    fElapsedTimeLine='none'
    lrd=lampReadData(0)
    if os.path.isfile(f):
        fElapsedTimeFile=open(f, "r+")
        fElapsedTimeLine=fElapsedTimeFile.read()   
        fElapsedTimeFile.close()
        print(fElapsedTimeLine)
        lampSavedTimeFields= fElapsedTimeLine.split("|")
        lrd.duration=lampSavedTimeFields[1]    
        lrd.datetimeStr=lampSavedTimeFields[0]
    else:    
        logLampElapsedTime(0)
        return ''
    try:    
        return lrd        # ElapsedTime Прошедшее время
    except Exception as e:
        # logexception(e)
        logLampElapsedTime(0)

# рассчитать время, затраченное лампой
def calculate_LampElapsedTime(sdate2,onstart=0):
    log ('calculate_LampElapsedTime  sdate2=%s'%sdate2)
    newDuration=-1
    try:
        lrd=rdLampElapsedTimeFile()    
        print(lrd.duration) # lampSavedduration --- продолжительность сохранения лампы
        print(lrd.datetimeStr) # lampSavedTime --- время время экономии лампы
        lampSaveddurationSeconds=-1
        if 'day' in lrd.duration:
            durationArray= lrd.duration.split(",")
            print(durationArray[0])
            dayArray= durationArray[0].split(" ")
            print(dayArray[0])
            lampSaveddurationSeconds=86400*int(dayArray[0] )
            print(durationArray[1])
            lampSaveddurationSeconds+=sum(x * int(t) for x, t in zip([3600, 60, 1], durationArray[1].split(":")))
            print(lrd.duration)
        if lampSaveddurationSeconds==-1:
            lampSaveddurationSeconds=sum(x * int(t) for x, t in zip([3600, 60, 1], lrd.duration.split(":")))     
        print(lampSaveddurationSeconds)
        dateLampSavedduration = datetime.strptime(lrd.datetimeStr, '%Y-%m-%d %H:%M:%S')
        dateLastworkingtime = datetime.strptime(sdate2, '%Y-%m-%d %H:%M:%S')
        diff = date_diff_in_seconds(dateLastworkingtime, dateLampSavedduration)
        if onstart==STATEFORLAMPTIME_LAMPON:
            newDuration=lampSaveddurationSeconds
        else:
            newDuration=lampSaveddurationSeconds+diff
        logLampElapsedTime(newDuration)
        log ('calculate_LampElapsedTime  newDuration=%d'%newDuration)
        print(str(timedelta(seconds=newDuration)))
    except Exception as e:
        logexception(e)
        log('calculate_LampElapsedTime  Exception')
        logLampElapsedTime(0)
    return newDuration

# STATEFORLAMPTIME_SENDBIT
# Состояние работы аремени работы лампы
# подсчет разницы во времени
def date_diff_in_seconds(dt2, dt1):
    timedelta = dt2 - dt1
    return timedelta.days * 24 * 3600 + timedelta.seconds

def dhms_from_seconds(seconds):
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    days, hours = divmod(hours, 24)
    return (days, hours, minutes, seconds)

def diffdates(d1, d2):
    #Date format: %Y-%m-%d %H:%M:%S
    return (time.mktime(time.strptime(d2,"%Y-%m-%d %H:%M:%S")) -
            time.mktime(time.strptime(d1, "%Y-%m-%d %H:%M:%S")))

# запись значения аргумента в файл журнал данных lampElapsedTime.txt
def logLampElapsedTime(s):
    flog=open("/home/pi/LogsGc200/lampElapsedTime.txt", "w")
    flog.truncate(0)
    s2log = datetime.now().strftime("%Y-%m-%d %H:%M:%S")+"|" +str(timedelta(seconds=s))
    flog.writelines(s2log +'\n')
    flog.close()

# ----------- Начало измерения спектрометра --------------
def SpectStartMeasurement():      #spectrometer Start measurement
    log( 'SpectStartMeasurement() ')
    measurementTimeout=SpectSpectraMeasurementTime()
    print(measurementTimeout)
    spectrometer.write('XM\r')
    sleeptime=measurementTimeout/1000 +1
    sleep(sleeptime)
    print(spectrometer.readline())

#  Установка интенсивности лампы
def SpectReadMeasurementTime():
    spectrometer.write('E0\r')                    #set lamp intensity
    ReadMeasurementTime=spectrometer.readline()
    timeout=333
    try:
        print('SpectraMeasurementTime '+ReadMeasurementTime)
        timeoutstr = ReadMeasurementTime.split(" ")
        timeout= int(timeoutstr[2].rstrip() )
    except Exception as e:
        logexception(e)
        timeout=333
    return timeout

# Считывание измерений из памяти спектрометра
def ReadMeasurementData(points = 10):  #Read measurement from spectrometer memory
    log( 'ReadMeasurementData() ')
    spectrometer.write('Xm0,%d\r' %points)
    data_received = [0]*points
    for i in range(points):
        # преобразуем 4 байта с плавающей запятой в маленьком конце в значение с плавающей запятой
        data_received[i] = struct.unpack('<f',spectrometer.read(4))[0]
    return data_received

# Спектрометр начинает эталонное измерение
def SpectStartReferenceMeasurement(): #Spectrometer start reference measurement
    spectrometer.write('XR\r')
    print(spectrometer.readline())

# Считывание эталонного измерения из памяти спектрометра
def ReadReferenceMeasurementData(points): #Read reference measurement from spectrometer memory
    spectrometer.write('Xr0,%d\r' %points)
    data_received = [0]*points
    for i in range(points):
        # преобразуем 4 байта с плавающей запятой в маленьком конце в значение с плавающей запятой
        data_received[i] = struct.unpack('<f',spectrometer.read(4))[0]    #convert 4 Bytes floating point in little endian to float
    return data_received

# Спектрометр начинает измерение темноты
def SpectStartDarkMeasurement(): #Spectrometer start dark measurement
    spectrometer.write('XD\r')
    print(spectrometer.readline())

# Считываем измерения темноватости из памяти спектрометра
def ReadDarkMeasurementData(points): #Read dark measurement from spectrometer memory
    spectrometer.write('Xd0,%d\r' %points)
    data_received = [0]*points
    for i in range(points):
        # преобразуем 4 байта с плавающей запятой в маленьком конце в значение с плавающей запятой
        data_received[i] = struct.unpack('<f',spectrometer.read(4))[0]    #convert 4 Bytes floating point in little endian to float
    return data_received

# Отправить строку в Bluetooth
def SendHostMessage(bt_str): # Send string to bluetooth
    log( 'SendHostMessage() ')
    flgs.client_sock.send(bt_str)

# Получать строку от bluetooth
def ReceiveHostMessage(str_size): #Receive string from bluetooth
    try:
        data_received = flgs.client_sock.recv(str_size)
        return data_received
    except Exception as e:
        print('ReceiveHostMessage null')
        return 'null'

#  проверяем исключения в файле исключений
def rdExeptionFile():
    f = "exceptions.txt"
    lastexception='none'
    print(lastexception)
    if os.path.isfile(f):
        fexceptionlog=open(f, "r+")
        print(lastexception)
        lastexception=fexceptionlog.read()
        print(lastexception)
        fexceptionlog.close()
        print(lastexception)
        os.remove(f)

def boolToInt(b):
    if b==True:
        return 1
    else:
        return 0

# Выполняет встроенный тест, возращает результаты bitarr
def ActivateBITNew():  #Perform built-in test, return results bitarr
    log( 'start-ActivateBITNew ')
    if flgs.deviceTipe == REGULAR:
        print("regular device")
        log( 'regular device')
    if flgs.deviceTipe == LITE:
        print("LITE device")
        log("LITE device")
    bitarr.bit_arr['Command'] = 'bt' 
    try:
        whistleDog(functionStartTimout)
    except Exception as e:
        logexception(e)
        log( 'whistleDog failed')
    flgs.lampOn=0
    Home() # Переместить всю систему домой
    # BIT = False
    cover = GPIO.input(COVER)
    print('Drawer : ',cover)
    log( 'ActivateBITNew   Drawer : %d ' %cover)
    if cover==0:
        Home() # to make sure user can close drawer
        bitarr.bit_arr['DrawerOpen'] =0 #TODO(Ido): Create constants for bit status
        data_str = json.dumps(bitarr.bit_arr)
        print(data_str)
        log(data_str)
        flgs.bitPerformed=0
        return data_str
    else:
        bitarr.bit_arr['DrawerOpen'] =1
    Zfailed=False
    GPIO.output(CAM_LIGHT,GPIO.LOW) # turn on camera light
    fexceptions=    "/home/pi/LogsGc200/exceptions.txt"
    lastexception='none'
    if os.path.isfile(fexceptions):
        fexceptionlog=open(fexceptions, "r+")
        lastexception=fexceptionlog.read()   #+" "+fexceptionlog.readline(1)
        fexceptionlog.close()
        print(lastexception)
    bitarr.bit_arr['LastException'] =str(lastexception)
    IsPowerButtonPressed(True)
    log('in ActivateBITNew 1')
    bitarr.bit_arr['Camera'] = 2
    print(' Before GetCameraImage()')
    try:
        bitarr.bit_arr['Camera'] = boolToInt(GetCameraImage(1))
    except Exception as e:
        logexception(e)
        print("Camera error")
        log( "BIT Camera error")
        bitarr.bit_arr['Camera'] = 0
    print(bitarr.bit_arr['Camera'])
    whistleDog(bitTimout)
    IsPowerButtonPressed(True) 
    if bitarr.bit_arr['Camera']==0:
        bitarr.bit_arr['TemperatureLampOff'] = 2
        bitarr.bit_arr['TemperatureLampOn']  = 2
        bitarr.bit_arr['LampcurrentsOff']    = 2
        bitarr.bit_arr['LampcurrentsOn']     = 2
        bitarr.bit_arr['WhiteExists']        = 2
    print(' ftr GetCameraImage()')
    bitarr.bit_arr['Spectrometer'] = 0
    lampcurrentsOff='-1,-1'
    lampcurrentsOn='-1,-1'
    temperatureLampOff='-1'
    temperatureLampOn='-1'
    log('in ActivateBITNew 2')
    try:
        bitarr.bit_arr['Spectrometer'] = boolToInt(CheckSpectResponse())
        bitLampWarmupTime=200
        spectrometer.write('LT%d\r' %bitLampWarmupTime)
        print(spectrometer.readline())
        SpectTurnOffLamp()
        sleep(0.3)
        temperatureLampOff=getTemperature()
        lampcurrentsOff=Getpectlampcurrents()
        SpectTurnOnLamp()
        sleep(1)
        lampcurrentsOn=Getpectlampcurrents()
        temperatureLampOn=getTemperature()
    except Exception as e:
        logexception(e)
        bitarr.bit_arr['Spectrometer'] =0
        print("Spectrometer FATAL Error")
        log( "BIT Spectrometer FATAL Error")
        data_str = json.dumps(bitarr.bit_arr)
        print(data_str)
    whistleDog(bitTimout)
    IsPowerButtonPressed(True)
    print(bitarr.bit_arr['Spectrometer'])
    if bitarr.bit_arr['Spectrometer']==1:
        bitarr.bit_arr['TemperatureLampOff'] = str(temperatureLampOff)
        bitarr.bit_arr['TemperatureLampOn']  = str(temperatureLampOn)
        bitarr.bit_arr['LampCurrentsOff']    = str(lampcurrentsOff)
        bitarr.bit_arr['LampCurrentsOn']     = str(lampcurrentsOn)
    else:
        bitarr.bit_arr['TemperatureLampOff'] = 2
        bitarr.bit_arr['TemperatureLampOn']  = 2
        bitarr.bit_arr['LampCurrentsOff']    = 2
        bitarr.bit_arr['LampCurrentsOn']     = 2
    log('in ActivateBITNew 3')
    if flgs.deviceTipe == REGULAR:
        Move_x(flgs.bud_center_x )
        xswitch= GPIO.input(SWITH_X)
        print('Switch X: ',xswitch)
        if xswitch==0:
            bitarr.bit_arr['Switch_X_phase1'] = 1
        else:
            bitarr.bit_arr['Switch_X_phase1'] = 0
            log( "BIT Switch_X_phase1 Error")
    if flgs.deviceTipe == LITE:        
        bitarr.bit_arr['Switch_X_phase1'] = 2
    zswitch= GPIO.input(SWITH_Z)
    print('Switch Z: ',zswitch)
    print('pos.z=',pos.z)
    if zswitch==0:
        bitarr.bit_arr['Switch_Z_phase1'] = 1
    else:
        bitarr.bit_arr['Switch_Z_phase1'] = 0
        log( "BIT Switch_Z_phase1 Error")
        Zfailed=True
    log('in ActivateBITNew 4')
    Move_y(safe_y_distance_up)
    Move_z(flgs.safe_z_distance)
    print("bitarr.bit_arr['Camera'] ",bitarr.bit_arr['Camera'])
    whistleDog(bitTimout)
    IsPowerButtonPressed(True)
    fnjwhite = '/home/pi/pct/BITwhitew.jpg'
    fnjwide = '/home/pi/pct/budcenterw.jpg'
    if  os.path.exists(fnjwhite):
        os.system('sudo rm  /home/pi/pct/BITwhitew.jpg')
    if  os.path.exists(fnjwide):
        os.system('sudo rm  /home/pi/pct/budcenterw.jpg')
    if bitarr.bit_arr['Camera'] == 1:
        whistleDog(distanceMeasureStepTimout)
        Move_y(250)
        Home_z()
        log('**FTR Home_z pos.z=%d'%pos.z)
        Home_y()
        loc_y = flgs.y_whiteExists_distance
        print('y_whiteExists_distance: ',loc_y)
        Move_yForCalibration(loc_y)     # mm_to_step_Y*(flgs.config['calibrationAttributes']['Y_Distance']/1000))
        log('mv2calib 4')
        log('whiteExists pos.y=%d'%pos.y)
        log('whiteExists pos.z=%d'%pos.z)
        whiteExists=0
        if flgs.spectrometer_edge_in_image_white != -1:
            whiteExists=find_white_exists()
        Home()   # Переместить всю систему домой
        #FINDING Y CALIBRATION DISTANCE
        Move_x(XCALIBRATIONPOSITION)     # 900                                                   
        spectrometerBottomPixel=from_bottom_find_white_exists_diff()                                                        
        distancPixelsHome=from_top_find_white_exists_diff(FROM_HOME)   #test_find_white_exists_diff()
        Move_yForCalibration(STEPS_FOR_MEASURING_WHITE_DISTANCE)             #TODO(Ido): Change to a constant
        distancPixels450=from_top_find_white_exists_diff(FROM_450)    #TODO(Ido): Change variable name
        diffDistancepixels=   distancPixels450-distancPixelsHome
        Difference_between_white_fronts= Y_REFERENCE_DIFFERENCE - distancPixelsHome
        Pixels_to_lower_end_of_Experiment_device_spectrometer=Y_PIXEL_MAX-spectrometerBottomPixel
        print('Pixels_to_lower_end_of_Experiment_device_spectrometer='+str(Pixels_to_lower_end_of_Experiment_device_spectrometer))
        Step_to_move_for_Reference_device=REFERENCE_STEPS
        Pixels_to_lower_end_of_Reference_device_spectrometer=REFERENCE_PIXELS_LOW_END
        
        #TODO(Ido): Consider splitting this to several lines/to a function
        PIXELS_TO_WHITE_FRONT=Pixels_to_lower_end_of_Experiment_device_spectrometer + Difference_between_white_fronts
        stepsToMove= Step_to_move_for_Reference_device *(PIXELS_TO_WHITE_FRONT) / Pixels_to_lower_end_of_Reference_device_spectrometer
        print('spectrometerBottomPixel= '+str(spectrometerBottomPixel))
        log( 'spectrometerBottomPixel= '+str(spectrometerBottomPixel))
        print('WHITE distancPixelsHome= '+str(distancPixelsHome))
        log( 'WHITE distancPixelsHome= '+str(distancPixelsHome))
        print('distancPixels450= '+str(distancPixels450))
        log( 'WHITE distancPixels450= '+str(distancPixels450))
        print('diffDistancepixels= '+str(diffDistancepixels))
        log( 'diffDistancepixels= '+str(diffDistancepixels))
        print('stepsTillWhite= '+str(stepsToMove))
        log( 'stepsTillWhite= '+str(stepsToMove))
        flgs.calibration_y_steps=int(stepsToMove+CALIBDISTANCEPARAMETER)
        log( '@flgs.calibration_y_steps= '+str(flgs.calibration_y_steps))
        print('whiteExists=%d'%whiteExists)
        fwhiteExists=float(whiteExists)
        if fwhiteExists<=0:
            bitarr.bit_arr['WhiteExists'] = 0
            log( "BIT WhiteExists Error")
        else:
            bitarr.bit_arr['WhiteExists'] = 1
        if flgs.spectrometer_edge_in_image_white == -1:
            bitarr.bit_arr['WhiteExists'] = 2
            log( "BIT WhiteExists Not Tested")
        Move_z(versiondotz)
    Zfailed=False  # test because no white dot --- тест, потому что нет белой точки ---
    print(flgs.safe_y_distance)
    whistleDog(bitTimout)
    IsPowerButtonPressed(True)
    if Zfailed==False:
        Move_y(flgs.safe_y_distance)
        yswitch= GPIO.input(SWITH_Y)
        if yswitch==0:
            bitarr.bit_arr['Switch_Y_phase1'] =1
        else:
            bitarr.bit_arr['Switch_Y_phase1'] =0
    else:
         bitarr.bit_arr['Switch_Y_phase1'] =2
    log('in ActivateBITNew 5')
    flgs.xswitch=0
    if flgs.deviceTipe == REGULAR:
        Home_x()
        if flgs.xswitch== 1:
            bitarr.bit_arr['Switch_X_phase2'] =1
        else:
            bitarr.bit_arr['Switch_X_phase2'] =0
            log( "BIT Switch_X_phase2 Error")
    if flgs.deviceTipe == LITE:        
        bitarr.bit_arr['Switch_X_phase2'] =2
    Move_x(700)
    flgs.zswitch=0
    Home_z()
    if flgs.zswitch==0:
        bitarr.bit_arr['Switch_Z_phase2'] =0
        log( "BIT Switch_Z_phase2 Error")
    else:
        bitarr.bit_arr['Switch_Z_phase2'] =1
    log('**ActivateBITNew b4 Home_y()')
    flgs.ymicroswitch=0
    Home_y()
    log('**ActivateBITNew FTR Home_y() flgs.ymicroswitch=%d'%flgs.ymicroswitch)
    if flgs.ymicroswitch== 1:
        bitarr.bit_arr['Switch_Y_phase2'] =1
    else:
        bitarr.bit_arr['Switch_Y_phase2'] =0
        log( "BIT Switch_Y_phase2 Error")
    yswitch= GPIO.input(SWITH_Y)
    zswitch= GPIO.input(SWITH_Z)
    if flgs.deviceTipe == REGULAR:
        xswitch= GPIO.input(SWITH_X)
        if xswitch==0:
            bitarr.bit_arr['Switch_X_phase3'] =1
        else:
            bitarr.bit_arr['Switch_X_phase3'] =0
    if flgs.deviceTipe == LITE:        
        bitarr.bit_arr['Switch_X_phase3'] =2    
    yswitch= GPIO.input(SWITH_Y)
    if yswitch==0:
        bitarr.bit_arr['Switch_Y_phase3'] =1
    else:
        bitarr.bit_arr['Switch_Y_phase3'] =0
    print('Switch Y: ',yswitch)
    zswitch= GPIO.input(SWITH_Z)
    if zswitch==0:
        bitarr.bit_arr['Switch_Z_phase3'] =1
    else:
        bitarr.bit_arr['Switch_Z_phase3'] =0
    print('Switch Z: ',zswitch)
    if flgs.deviceTipe == REGULAR:
        if( (bitarr.bit_arr['Switch_X_phase1'] ==1) and (bitarr.bit_arr['Switch_X_phase2'] ==1) and (bitarr.bit_arr['Switch_X_phase3'] ==1)):
            bitarr.bit_arr['MotorX'] =1
        else:
            bitarr.bit_arr['MotorX'] =0
    if flgs.deviceTipe == LITE:        
        bitarr.bit_arr['MotorX'] =2
    if( (bitarr.bit_arr['Switch_Y_phase1'] ==1) and (bitarr.bit_arr['Switch_Y_phase2'] ==1) and (bitarr.bit_arr['Switch_Y_phase3'] ==1)):
        bitarr.bit_arr['MotorY'] =1
    else:
        bitarr.bit_arr['MotorY'] =0
    if( (bitarr.bit_arr['Switch_Z_phase1'] ==1) and (bitarr.bit_arr['Switch_Z_phase2'] ==1) and (bitarr.bit_arr['Switch_Z_phase3'] ==1)):
        bitarr.bit_arr['MotorZ'] =1
    else:
        bitarr.bit_arr['MotorZ'] =0
    log('in ActivateBITNew 6')
    IsPowerButtonPressed(True)
    print('pos.x ',pos.x)
    M_SwitchTest()
    bitarr.bit_arr['WhitePhoto'] =2
    fnjwhite = '/home/pi/pct/BITwhitew.jpg'
    if  os.path.exists(fnjwhite):
        print('/home/pi/pct/BITwhitew.jpg exists')
        log( '/home/pi/pct/BITwhitew.jpg exists')
        encode_string =''
        with open(fnjwhite, "rb") as f:
            bytes = f.read()
            encode_string = base64.b64encode(bytes)
        bitarr.bit_arr['WhitePhoto'] =encode_string
    bitarr.bit_arr['Command'] = 'bt'
    if bitarr.bit_arr['Camera'] == 1:
        get_budcenter_photo()
        # Move_z(flgs.safe_z_distance+500)
        fnjwide = '/home/pi/pct/budcenterw.jpg'
        if  os.path.exists(fnjwide):
            encode_string =''
            with open(fnjwide, "rb") as f:
                bytes = f.read()
                encode_string = base64.b64encode(bytes)   
            bitarr.bit_arr['ReflectorPhoto'] =encode_string
    else:
        bitarr.bit_arr['ReflectorPhoto'] =''
        
    if flgs.ignorePowerButtonPressed==1:
        bitarr.bit_arr['PowerButtonPressed'] =0
    else:
        bitarr.bit_arr['PowerButtonPressed'] =1
    data_str = json.dumps(bitarr.bit_arr)
    print(data_str)
    log(data_str)
    BIT = False
    if all(v == True for v in bitarr.bit_arr.values()):
        BIT = True
        blu_led.start(0)
    else:
        blu_led.ChangeDutyCycle(20)
    log('BIT result' +str(BIT))
    GPIO.output(CAM_LIGHT,GPIO.HIGH) #turn off camera light --- выключить свет камеры ---
    Move_z(flgs.safe_z_distance)
    Home() # Переместить всю систему домой
    print('pos.y ',pos.y)
    print('pos.z ',pos.z)
    print('pos.x ', pos.x)
    blu_led.ChangeDutyCycle(50)
    white_led.ChangeDutyCycle(50)
    log ("PowerButtonPressed duringBIT=%d" %flgs.ignorePowerButtonPressed)
    log( 'END ActivateBITNew ')
    flgs.bitPerformed = 1
    return  data_str

# получить фотографию центра бутона
def get_budcenter_photo():
    print ("get_budcenter_photo()")
    Home() # Переместить всю систему домой
    Move_x(flgs.bud_center_x)
    Move_z(flgs.safe_z_distance)
    Move_y(flgs.safe_y_distance)
    Move_z(flgs.bud_center_z)
    GetCameraImageSaveForImageAnalysis('budcenter')

# Получить изображение с камеры, вернуть RGB-массив
def GetCameraImageSaveForImageAnalysis(phototype):    #Get image from camera, return RGB array
    fn = '%s.bmp'%str(pos.z)
    fnjwide = '/home/pi/pct/budcenterw.jpg'
    if  os.path.exists(fnjwide):
        os.remove(fnjwide)
    if  os.path.exists(fn):
        os.remove(fn)
    sleep(0.1)
    img = GetCameraImage()
    GPIO.output(CAM_LIGHT,GPIO.HIGH) #turn off camera light
    cv2.imwrite(fnjwide, img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return True

# убедиться, что двигатель Z вышел на 7800
def find_reflector_dots(): #make sure Z motor went to 7800
    log('find_reflector_dots()')
    SpectTurnOffLamp()
    ddebugbl=True
    safe_x_distance=900
    Move_x(safe_x_distance)
    Home_y()
    safe_y_distance=3050 
    Move_y(safe_y_distance)
    sleep(0.2)
    min_distance = 0
    firstpixel=-1
    tt=datetime.now().time()
    stt = str(tt)
    print ('b4 GetCameraImage() START %s',stt)
    img = GetCameraImage()
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTP GetCameraImage() START %s',stt)
    fn = '/home/pi/pct/BIT%s.bmp'%str(pos.z)
    if ddebugbl==True:
        GetCameraImageSaveLOFN(fn)
    SpectTurnOnLamp()
    fnj = '/home/pi/pct/BIT%s.jpg'%str(pos.z)
    fnjreflectorcropped = '/home/pi/pct/reflectorcropped.jpg'
    fnjreflector = '/home/pi/pct/reflector%s.jpg'%str(pos.z)
    fnjrv = '/home/pi/pct/BITrev%s.jpg'%str(pos.z)
    im_gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    thresholdForDistance=50 #100
    thresh_img = cv2.threshold(im_gray, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    cropped_img = thresh_img[flgs.spectrometer_center_in_image:flgs.spectrometer_center_in_image+70,flgs.spectrometer_edge_in_image-75:flgs.spectrometer_edge_in_image]
    if ddebugbl==True:
        cv2.imwrite(fnjreflector, img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cv2.imwrite(fnjreflectorcropped, cropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if  os.path.exists(fnjreflector):
            encode_string =''
            with open(fnjreflector, "rb") as f:
                bytes = f.read()
                encode_string = base64.b64encode(bytes)
            bitarr.bit_arr['ReflectorPhoto'] =encode_string
        cv2.imwrite(fnj, img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    cropped_imgrev=numpy.flipud(cropped_img)
    if ddebugbl==True:
        cv2.imwrite(fnjrv, cropped_imgrev, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    try:
        white_pixels = numpy.array(numpy.where(cropped_imgrev == 255))
        first_white_pixel = white_pixels[:,0]
        print('first_white_pixel=')
        print(first_white_pixel)
        firstpixel= int(first_white_pixel[0])
    except Exception as e:
        logexception(e)
        firstpixel=-1
    print('firstpixel ',firstpixel)
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTR CROP START %s',stt)
    if ddebugbl==True:
        fnjpgb = '/home/pi/pct/BITdistthresh_img%s.jpg'%str(pos.z)
        cv2.imwrite(fnjpgb, thresh_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    min_distance =(firstpixel)*flgs.pixel_to_mm  # compensate for crop 130 instead of 120
    print('distance:', (min_distance))
    return min_distance
def upload_log():
    shorten_log()
    logzip = '/home/pi/LogsGc200/log.zip'
    if os.path.isfile(logzip):
        os.remove(logzip)
    logfn = '/home/pi/LogsGc200/logGc200.txt'
    jungle_zip = zipfile.ZipFile(logzip, 'w')
    jungle_zip.write(logfn, compress_type=zipfile.ZIP_DEFLATED)
    jungle_zip.close()
    f = open("/home/pi/LogsGc200/log.zip", "r")
    ziptext = f.read()
    print(ziptext)
    f.close()   
    data_to_GCA = {'logfile': ziptext, 'Command':flgs.lastcommand}
    SendHostMessage('%s\n' %str(data_to_GCA))
    whistleDog(firstconnectionTimout)

# журнал исключений загрузки
def upload_exceptionlog():
    shorten_log()
    logzip = '/home/pi/LogsGc200/exceptionlog.zip'
    if os.path.isfile(logzip):
        os.remove(logzip)
    logfn = '/home/pi/LogsGc200/exceptionsFull.txt' # архивируем файл ошибок
    jungle_zip = zipfile.ZipFile(logzip, 'w')
    jungle_zip.write(logfn, compress_type=zipfile.ZIP_DEFLATED)
    jungle_zip.close()
    f = open("/home/pi/LogsGc200/exceptionlog.zip", "r")
    ziptext = f.read()
    print(ziptext)
    f.close()   
    data_to_GCA = {"logfile": ziptext, "Command":flgs.lastcommand}
    SendHostMessage('%s\n' %str(data_to_GCA))
    whistleDog(firstconnectionTimout)

# журнал перемещений
def move_log():
    whistleDog(firstconnectionTimout)
    try:
        os.system("sudo mv /home/pi/LogsGc200/logGc200.txt  /home/pi/LogsGc200/savedlogGc200.txt")
    except Exception as e:
        logexception(e)

        log('Log file does not exists')

#  найти белые существующие
def find_white_exists():
    ddebugbl=True
    sleep(0.2)
    min_distance = 0
    firstpixel=-1
    tt=datetime.now().time()
    stt = str(tt)
    print ('b4 GetCameraImage() START %s',stt)
    img = GetCameraImage()
    tt=datetime.now().time()
    stt = str(tt)
    print ('pos.y %s'%str(pos.y))
    print ('FTP GetCameraImage() START %s',stt)
    fn = '/home/pi/pct/BITwhite%s.bmp'%str(pos.z)
    if  os.path.exists(fn):
        os.system('sudo rm  '+fn)
    if ddebugbl==True:
        GetCameraImageSaveLOFN(fn)
    if  os.path.exists(fn)==False:
        return -1
    fnj = '/home/pi/pct/BITwhite%s.jpg'%str(pos.z)
    fnjblack = '/home/pi/pct/BITwhiteblack%s.jpg'%str(pos.z)
    fnjwide = '/home/pi/pct/BITwhitew.jpg'
    fnjrv = '/home/pi/pct/BITwhiterev%s.jpg'%str(pos.z)
    cropLeft=flgs.spectrometer_center_in_image-5
    cropRight=flgs.spectrometer_center_in_image+5
    widecropped_img = img[0:320,cropLeft:cropRight]

    cv2.imwrite(fnjwide, widecropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90]) # запишем изображение в файл

    im_gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    thresholdForDistance=100
    thresh_img = cv2.threshold(im_gray, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    try:
        if ddebugbl==True:
            cv2.imwrite(fnjblack, thresh_img , [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    except Exception as e:
        logexception(e)

        diff = -1
    cropDown=flgs.spectrometer_edge_in_image_white-5
    cropUp=cropDown-flgs.whiteExists_crop_size

    cropped_img = thresh_img[cropUp:cropDown,cropLeft:cropRight]

    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img , [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    cropped_imgrev=numpy.flipud(cropped_img)
    diff = -1

    if ddebugbl==True:
        cv2.imwrite(fnjrv, cropped_imgrev, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    try:
        white_pixels = numpy.array(numpy.where(cropped_imgrev == 255))
        first_white_pixel = white_pixels[:,0]
        last_white_pixel = white_pixels[:,-1]
        print('first_white_pixel=')
        print(first_white_pixel)
        firstpixel= int(first_white_pixel[0])
        print('last_white_pixel=')
        print(last_white_pixel)
        firstpixelwhite= int(first_white_pixel[0])
        lastpixelwhite= int(last_white_pixel[0])
        diff = lastpixelwhite-firstpixelwhite

    except Exception as e:
        logexception(e)
        firstpixel=-1
    print('diff  %d' %diff)
    print('firstpixel %d' %firstpixel)
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTR CROP START %s',stt)
    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cv2.imwrite(fnjwide, widecropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        fnjpgb = '/home/pi/pct/BITdistthresh_img%s.jpg'%str(pos.z)
        cv2.imwrite(fnjpgb, thresh_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if diff== -1:
        min_distance = -1
    else:
        min_distance = 1
    print('find_white_exists() distance: %f mm' %(min_distance))
    return min_distance

# найти белый существующие разница
def find_white_exists_diff():
    ddebugbl=True
    sleep(0.2)
    min_distance = 0
    firstpixel=-1
    tt=datetime.now().time()

    stt = str(tt)

    print ('b4 GetCameraImage() START %s',stt)
    img = GetCameraImage()
    tt=datetime.now().time()

    stt = str(tt)

    print ('pos.y %s'%str(pos.y))
    print ('FTP GetCameraImage() START %s',stt)
    fn = '/home/pi/pct/BITwhite%s.bmp'%str(pos.z)
    if ddebugbl==True:
        GetCameraImageSaveLOFN(fn)
    fnj = '/home/pi/pct/BITwhite%s.jpg'%str(pos.z)
    fnjblack = '/home/pi/pct/BITwhiteblack%s.jpg'%str(pos.z)
    fnjwide = '/home/pi/pct/BITwhitew.jpg'
    fnjrv = '/home/pi/pct/BITwhiterev%s.jpg'%str(pos.z)
    cropLeft=flgs.spectrometer_center_in_image-5
    cropRight=flgs.spectrometer_center_in_image+5
    widecropped_img = img[0:320,cropLeft:cropRight]

    cv2.imwrite(fnjwide, widecropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    im_gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    thresholdForDistance=100
    thresh_img = cv2.threshold(im_gray, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    if ddebugbl==True:
        cv2.imwrite(fnjblack, thresh_img , [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    cropDown=flgs.spectrometer_edge_in_image_white-5
    cropUp=cropDown-flgs.whiteExists_crop_size

    cropped_img = thresh_img[cropUp:cropDown,cropLeft:cropRight]

    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img , [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    cropped_imgrev=numpy.flipud(cropped_img)
    diff = -1

    if ddebugbl==True:
        cv2.imwrite(fnjrv, cropped_imgrev, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    try:
        white_pixels = numpy.array(numpy.where(cropped_imgrev == 255))
        first_white_pixel = white_pixels[:,0]
        last_white_pixel = white_pixels[:,-1]
        print('first_white_pixel=')
        print(first_white_pixel)
        firstpixel= int(first_white_pixel[0])
        print('last_white_pixel=')
        print(last_white_pixel)
        firstpixelwhite= int(first_white_pixel[0])
        lastpixelwhite= int(last_white_pixel[0])
        diff = lastpixelwhite-firstpixelwhite
    except Exception as e:
        logexception(e)
        firstpixel=-1
    print('diff  %d' %diff)
    print('firstpixel %d' %firstpixel)
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTR CROP START %s',stt)
    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        cv2.imwrite(fnjwide, widecropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        fnjpgb = '/home/pi/pct/BITdistthresh_img%s.jpg'%str(pos.z)
        cv2.imwrite(fnjpgb, thresh_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if diff== -1:
        min_distance = -1
    else:
        min_distance = 1
    print('find_white_exists() distance: %f mm' %(min_distance))
    return min_distance

# Включить интерфейс Bluetooth и сканировать устройство
def bt_pairing(): #Turn on bluetooth interface and scan devices
    log(' bt_pairing()')
    call(['sudo','hciconfig','hci0','up'])
    call(['sudo','hciconfig','hci0','piscan'])

# Отключить интерфейс Bluetooth
def bt_unpairing(): #Turn on bluetooth interface and scan devices 
    log(' bt_unpairing()')
    call(['sudo','hciconfig','hci0','down'])

# перезагрузить устройство
def reboot_device():
    call(['sudo','reboot','now'])

# проверить, открыт ли контейнер
def  IsOpen():   #check if container is closed  flgs.wasopen = False
    # closed1 = False
    if (GPIO.input(COVER)):
        closed1 = True
    else:
        flgs.wasopen = True
#   установливает flgs.wasopen = True

# проверить, закрыт ли контейнер
def IsClosed():   #check if container is closed  flgs.wasopen = False
    closed = False
    if (GPIO.input(COVER)):
        closed = True
        Home_y()
    else:
        flgs.wasopen = True
        flgs.canHeatForward = True
    SendHostMessage('%s\n' %str({'IsClosed': closed,'WasOpen': flgs.wasopen, 'Command':flgs.lastcommand}))
    if (GPIO.input(COVER)):
        flgs.wasopen = False
    print('%s\n' %str({'IsClosed': closed}))
    return closed

# запись в журнал
def log(s):
    flog=open("/home/pi/LogsGc200/logGc200.txt", "a+")
    s2log = datetime.now().strftime("%Y-%m-%d %H:%M:%S")+" " +s
    flog.writelines(s2log+'\n')
    flog.close()

# запись в журнал исключений Полная
def write_to_exceptionlogFull(s):
    flog=open("/home/pi/LogsGc200/exceptionsFull.txt", "a+")
    s2log = datetime.now().strftime("%Y-%m-%d %H:%M:%S")+" " +s
    flog.writelines(s2log+'\n')
    flog.close()

# запись в журнал исключений
def write_to_exceptionlog(s):
    flog=open("/home/pi/LogsGc200/exceptions.txt", "a+")
    s2log = datetime.now().strftime("%Y-%m-%d %H:%M:%S")+" " +s
    flog.writelines(s2log+'\n')
    flog.close()

# журнал исключений
def logexception( ex ):
    exc_type, exc_obj, exc_tb = sys.exc_info()
    sexeption=str(exc_type) +' '+str(exc_tb.tb_lineno) +' '+ ' ' + traceback.format_exc()
    log("$$Exception= "+sexeption)
    # битовое сообщение слишком длинное, если мы записываем все данные исключения и загружаем их с битовыми результатами
    sexeption=traceback.format_exc() # bit message is too long if we write all exception data and upload it with bit results
    sexeptionlines = sexeption.split("\n")
    try:
        sexeptionstring= sexeptionlines[1].replace("File","")
        sexeptionstring= sexeptionstring.replace("pgc200.py","")
        sexeptionstring= sexeptionstring.replace("/home/pi/gc200","")
        check_if_exception_in_file(sexeptionstring)
    except:
        log('error spliting exception trace')         
        write_to_exceptionlogFull(sexeption)
        
# проверяем, есть ли исключение в файле
def check_if_exception_in_file(sexeptionstr):
    exceptionAlready_in_log=False
    line_counter=0
    count=0
    if  os.path.exists("/home/pi/LogsGc200/exceptions.txt"):
        with open("/home/pi/LogsGc200/exceptions.txt", "r") as f:
            lines = f.readlines()
        with open("/home/pi/LogsGc200/exceptions.txt", "w") as f:
            for line in lines:
                if sexeptionstr in line:
                    exceptionAlready_in_log=True
                    sexeptionwords = line.split("^")
                    try:
                        count= int(sexeptionwords[1])
                        line_counter=count+1
                        print('exceptionAlready_in_log=True' +str(count))
                        log( 'exceptionAlready_in_log=True' +str(count))
                    except:
                        count=1
                            # if line.strip("\n") != "nickname_to_delete":
                else:
                    f.write(line)
        if exceptionAlready_in_log==False:
            sexeptionstr= sexeptionstr+ "^" + "1"
        else:
            sexeptionstr= sexeptionstr+ "^" + str(line_counter)
            write_to_exceptionlog(sexeptionstr)
    else:
        sexeptionstr= sexeptionstr+ "^" + "1"
        write_to_exceptionlog(sexeptionstr)
        
# найдите ближайший белый
def find_nearest_white(img, target):
    nonzero = numpy.argwhere(img == 255)
    distances = numpy.sqrt((nonzero[:,0] - TARGET[0]) ** 2 + (nonzero[:,1] - TARGET[1]) ** 2)
    nearest_index = numpy.argmin(distances)
    return nonzero[nearest_index]

# Получить минимальное расстояние от черного цвета спектрометра
def get_min_distance(): #Get minimum distance from spectrometer  Black
    ddebugbl=True
    # if pos.y==1600:
    if flgs.distance_from_home==MEASURE_DISTANCE_PHASE_1:
        print(pos.y)
        if pos.y>flgs.fix_steps_y:
            numIterations=4
            for i in range(numIterations):
                Move_y(pos.y-250)
            Home_y()
    if flgs.distance_from_home==MEASURE_DISTANCE_PHASE_2:
        Move_y(1600) #was 1000
        print(pos.y)
    if flgs.distance_from_home==MEASURE_DISTANCE_PHASE_3:
        Move_y(flgs.safe_y_distance)
        print(pos.y)
    if flgs.distance_from_home==MEASURE_DISTANCE_PHASE_4:
        Move_y(flgs.safe_y_distance)
        print(pos.y)
        log('MEASURE_DISTANCE_PHASE_4' +str(pos.y))
    # включить свет камеры
    GPIO.output(CAM_LIGHT,GPIO.LOW) #turn on camera light
    sleep(0.2)
    min_distance = 0
    firstpixel=-1
    tt=datetime.now().time()
    stt = str(tt)
    print ('b4 GetCameraImage() START %s',stt)
    tt=datetime.now().time()
    stt = str(tt)
    pos_yz='%s_%s'%(str(pos.y),str(pos.z))
    print ('pos_yz=%s',pos_yz)
    print ('FTP GetCameraImage() START %s',stt)
    fn = '/home/pi/pct/%s.bmp'%str(pos_yz)
    img = GetCameraImage()
    if ddebugbl==True:
        print('pos_y=%s'%str(pos.y))
        GetCameraImageSaveLOFN(fn)
    ledOff()
    fnj = '/home/pi/pct/%s.jpg'%str(pos.z)
    fnjwide = '/home/pi/pct/w%s.jpg'%str(pos.z)
    fnjwidezip = '/home/pi/pct/w%s.zip'%str(pos.z)
    fnjrv = '/home/pi/pct/rev%s.jpg'%str(pos.z)
    fnwhitestrip = '/home/pi/pct/whitestrip%s.jpg'%str(pos.z)
    left_img_edge= flgs.spectrometer_center_in_image-5  # левый край изображения
    right_img_edge= flgs.spectrometer_center_in_image+5  # правый край изображения
    img_crop_end_y=320-flgs.image_crop_right # обрезка изображения
    widecropped_img = img[0:img_crop_end_y,left_img_edge:right_img_edge] # широко обрезанное изображение
    if ddebugbl==True:
        cv2.imwrite(fnjwide, widecropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    im_gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    thresholdForDistance=100
    thresh_img = cv2.threshold(im_gray, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    cropped_img = thresh_img[40:flgs.spectrometer_edge_in_image,left_img_edge:right_img_edge]
    print('flgs.strip_edge_in_image_spectrometer_side=%d flgs.strip_edge_in_image_reflector=%d' %(flgs.strip_edge_in_image_spectrometer_side,
                                                                                                  flgs.strip_edge_in_image_reflector))
    flgs.strip_edge_in_image_spectrometer_side=flgs.spectrometer_edge_in_image-90
    flgs.strip_edge_in_image_reflector_side=flgs.spectrometer_edge_in_image-70
    print('flgs.strip_edge_in_image_spectrometer_side=%d flgs.strip_edge_in_image_reflector=%d' %(flgs.strip_edge_in_image_spectrometer_side,
                                                                                                  flgs.strip_edge_in_image_reflector))
    whitestripcropped_img = thresh_img[flgs.strip_edge_in_image_spectrometer_side:flgs.strip_edge_in_image_reflector,
                            left_img_edge:right_img_edge]  # reflector strip -- отражающая полоса --
    if ddebugbl==True:
        cv2.imwrite(fnwhitestrip, whitestripcropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    diffpixelws=-1
    try:
        white_pixelsws = numpy.array(numpy.where(whitestripcropped_img == 255))
        if (len(white_pixelsws[0]) > 0):
            first_white_pixelws = white_pixelsws[:,0]
            last_white_pixelws = white_pixelsws[:,-1]
            print('first_white_pixel ws=')
            print(first_white_pixelws)
            print('last_white_pixel ws=')
            print(last_white_pixelws)
            diffpixelws= int(last_white_pixelws[0])-int(first_white_pixelws[0])
    except Exception as e:
        logexception(e)
        log('get_min_distance Failed')
    print('diffpixelws %d' %diffpixelws)
    if (diffpixelws>6 and diffpixelws<=23) : # if strip is seen meaning the bud is small we move as close as we can
        min_distance =STRIP_DETECTED 
        print('distance: %f mm' %(min_distance))
        return min_distance  #test
    print('firstpixel %d' %firstpixel)
    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    cropped_imgrev=numpy.flipud(cropped_img)
    if ddebugbl==True:
        cv2.imwrite(fnjrv, cropped_imgrev, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    try:
        white_pixels = numpy.array(numpy.where(cropped_imgrev == 255))
        first_white_pixel = white_pixels[:,0]
        print('first_white_pixel=')
        print(first_white_pixel)
        firstpixel= int(first_white_pixel[0])
    except Exception as e:
        logexception(e)
        firstpixel=-1
    print('firstpixel %d' %firstpixel)
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTR CROP START %s',stt)
    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        jungle_zip = zipfile.ZipFile(fnjwidezip, 'w')
        jungle_zip.write(fnjwide, compress_type=zipfile.ZIP_DEFLATED)
        jungle_zip.close()
        fnjpgb = 'distthresh_img%s.jpg'%str(pos.z)
        cv2.imwrite(fnjpgb, thresh_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    linear_compensation=0
    if firstpixel>0: # 45 - линейная компенсация разницы между световым пятном и фактической поверхностью спектрометра, компенсирующая обрезку
        linear_compensation=45 # 45 is linear compensation for diff between lightspot and actual spectrometer surface compensate for crop
    else:
        if min_distance == 9.5:
            return min_distance
    # компенсировать обрезку 130 вместо 120 -- compensate for crop 130 instead of 120
    min_distance =(firstpixel+linear_compensation)*flgs.pixel_to_mm
    print('distance: %f mm' %(min_distance))
    log('distance: %f mm' %min_distance)
    return min_distance

# получить загруженные значения конфигурации
def getdownloadedconfigvalues():
    json_file='test.json'
    json_data1=open(json_file)
    configvalues = json.load(json_data1)
    print(configvalues)
    json_data1.close()
    return configvalues

#  Получить конфигурацию в виде файла json и редактировать
def get_config_and_update_conf_file():         #Get configuration as json file
    log(' get_config_and_update_conf_file()')
    ConfigurationFileAccepted = False
    cnf1=''
    cnf2=''
    sleep(2)
    cnf1= ReceiveHostMessage(10000)
    try:
        sleep(2)
        cnf2= ReceiveHostMessage(10000)
    except Exception as e:
        logexception(e)
        cnf2= ''
    if cnf2== 'null':
        cnf2= ''
    print('cnf2=%s '%cnf2)
    configuration_file =cnf1+cnf2   #+cnf3+cnf4
    print(configuration_file)
    log(configuration_file)
    suffix='hk'
    # prefix ='{"'
    startJsonChar =0
    if configuration_file[0]=='{':
        startJsonChar= 0
    if configuration_file[2]=='{':
        startJsonChar= 2
    if configuration_file[4]=='{':
        startJsonChar= 4
    ln= len(configuration_file)-2
    print(len(configuration_file))
    if configuration_file.endswith(suffix)==True:
        configuration_file= configuration_file[:ln]
    if startJsonChar==2:
        configuration_file= configuration_file[2:len(configuration_file)]
    if startJsonChar==4:
        configuration_file= configuration_file[4:len(configuration_file)]
    print(len(configuration_file))
    try:
        json_configuration_file = json.loads(configuration_file)
        jsonFile1 = open("test.json", "w+")
        jsonFile1.write(json.dumps(json_configuration_file))
        jsonFile1.close()
        sleep(1.5)
        testvals=getdownloadedconfigvalues()
        flgs.extractYoffset =   int(testvals['gc200Attributes']['extract_offset_y'])       
        nMAX_RANGE_Z = int(testvals['gc200Attributes']['MAX_RANGE_Z'])
        log('nMAX_RANGE_Z'+str(nMAX_RANGE_Z))
        print('flgs.extractYoffset'+str(flgs.extractYoffset))
        log('flgs.extractYoffset'+str(flgs.extractYoffset))
        os.system("sudo mv /home/pi/jsons/gc200conf.json /home/pi/jsons/oldgc200conf.json")
        sleep(1)
        os.system("sudo mv test.json /home/pi/jsons/gc200conf.json")
        os.system("sudo sync")
        os.system("sudo cat /home/pi/jsons/gc200conf.json")
        sleep(1)
        rereadConfValuesFromFile()
        print(' AFTER json.loads(configuration_file)')
        ConfigurationFileAccepted = True
        SendHostMessage('%s\n' %str({'ConfigurationFileAccepted': ConfigurationFileAccepted, 'Command':flgs.lastcommand}))
    except Exception as e:
        logexception(e)
        # exc_type, exc_obj, exc_tb = sys.exc_info()
        # print str(exc_type)
        # log( str(exc_type))
        print('FAILED parse/download json configuration_file')
        log ('FAILED parse/download json configuration_file')
        SendHostMessage('%s\n' %str({'ConfigurationFileAccepted': ConfigurationFileAccepted, 'Command':flgs.lastcommand}))
    return configuration_file

# Получить конфигурацию в виде файла json
def get_config():         #Get configuration as json file
    log(' get_config()')
    ConfigurationAccepted = False
    cnf1=''
    cnf2=''
    sleep(2)
    cnf1= ReceiveHostMessage(10000)
    try:
        sleep(2)
        cnf2= ReceiveHostMessage(10000)
    except Exception as e:
        logexception(e)
        cnf2= ''
    if cnf2== 'null':
        cnf2= ''
    print('cnf2=%s '%cnf2)
    config_file =cnf1+cnf2   #+cnf3+cnf4
    log (config_file)
    try:
        config_file = json.loads(config_file)
        ConfigurationAccepted = True
        log('ConfigurationAccepted = False sent')
    except Exception as e:
        logexception(e)
        ConfigurationAccepted = False
        log('ConfigurationAccepted = False sent')
    if ConfigurationAccepted:
        flgs.gotConfig=True
        print(' AFTER json.loads(config_file)')
        try:
            flgs.CoolDownHysteresis=    float(config_file["spectrometerAttributes"]['CoolDownHysteresis'])
        except Exception as e:
            logexception(e)
        flgs.config = config_file
        set_date_time()
        SpectConfig()
        getSpectrometerActualParams()
        print(flgs.config["spectrometerCenterAttributes"]['CenterZ'])
        fCenterZ=float(flgs.config["spectrometerCenterAttributes"]['CenterZ'])*(mm_to_step_Z/1000.0)
        flgs.WithContainer=flgs.config['calibrationAttributes']['WithContainer']
        flgs.bud_center_z=int(fCenterZ)
        log('####in get_config flgs.bud_center_z='+str(flgs.bud_center_z))
        flgs.TemperatureLowerLimit=float(flgs.config["spectrometerAttributes"]['TemperatureLowerLimit']    )
        flgs.TemperatureUpperLimit=float(flgs.config["spectrometerAttributes"]['TemperatureUpperLimit']    )
        flgs.mode=int(flgs.config['mode']    )
        print("flgs.mode="+str(flgs.mode))
        # confFile=flgs.config["spectrometerAttributes"]['TemperatureUpperLimit']
        if flgs.coolingFlag==False:
            SpectTurnOnLamp()
    return ConfigurationAccepted

# версия подтверждена
def versionProved():
    print('versionProved()')
    log('versionProved()')
    str2watchdog = '3;' + "\n" # DIRECT MESSAGE SW UPDATE --- ПРЯМОЕ ОБНОВЛЕНИЕ SW СООБЩЕНИЙ ---
    if flgs.versionprovedsent == True: # версия подтверждена
        return
    if flgs.watchdog == True: # проверяем сторожок, если все хорошо запишем информацию в файл
        os.write(flgs.pipeout, str2watchdog)
        flgs.versionprovedsent = True
        print(str2watchdog)
        log(str2watchdog)
        # Странно тут все организовано?!

# сменить версию
def swapVersion():
    log(' swapVersion()')
    print('swapVersion()')
    running_dir_path = os.path.dirname(os.path.realpath(__file__))
    if dir_pathA==running_dir_path:
        full_pathOf_newVersion = full_pathOf_newVersionB
    else:
        full_pathOf_newVersion = full_pathOf_newVersionA
    print('swapVersion() 1')
    str2watchdog = '2;' + 'python ' + full_pathOf_newVersion + "\n"  # DIRECT MESSAGE SW UPDATE
    print('swapVersion() 2')
    if flgs.watchdog == True: # проверяем сторожок, если все хорошо запишем информацию в файл
        print('swapVersion() 3')
        log( 'swapVersion() 3')
        os.write(flgs.pipeout, str2watchdog)
        flgs.watchdog = False
        print(str2watchdog)
        log( str2watchdog)

# Получает swcode в виде файла json
def get_swcode():        #Get swcode as json file
    log(' get_swcode()')
    print('get_swcode')
    sc1 = ''
    sc2 = ''
    sc11 = ''
    scode = ''
    crc = 0x00000000
    json_file = ''
    endmessagefound = False
    while True:
        try:
            sleep(1)
            sc1 = ReceiveHostMessage(10000)
            print('sc1=%s ' % sc1)
            if sc1 == 'null':
                sc1 = ''
                break
            if sc11 == 'null':
                sc11 = ''
                break
            json_file = json_file + sc1 + sc11
        except Exception as e:
            logexception(e)
            sc2 = ''
            break
    if sc2 == 'null':
        sc2 = ''
    print('sc2=%s ' % sc2)
    print('*****************************************************')
    try:
        f1 = open('/home/pi/downloaded.txt','w+')
        f1.writelines(json_file)
        f1.close()
    except Exception as e:
        logexception(e)
        print('-FILE ERROR-')
    flagstr="}"+"\n"
    if flagstr  in json_file:
        print('!!!!!!!!!!!!!!!!!!!!!FOUND!!!!!!!!!!!!!!!!!!!!!!!')
        endmessagefound = True
    if endmessagefound == True:
        print(json_file.index(flagstr))
        idx= len(json_file) -json_file.index(flagstr)-1
        json_file=json_file[:-idx]
    print('json_file=%s ' % json_file)
    log('json_file=%s ' % json_file)
    print('******************************************************************')
    swcode_file = json.loads(json_file)
    print('ftr swcode_file load')
    try:
        scode64 = swcode_file['BlockAsString']
        sentcrc = swcode_file['BlockCrc']
        msgformat = swcode_file['MessageFormat']
        print(msgformat)
        scode = base64.b64decode(scode64)
        print(scode)
        scodeTmp = scode + '0000000' + str(msgformat)
        crc = zlib.crc32(scodeTmp) & 0xffffffff
        print('FTR CRC')
        if crc == sentcrc:
            log('crc == sentcrc')
        else:
            print('bad CRC')
            # errcode = 3
            # errmsg = 'Bad CRC'
            log('errcode = 3  Bad CRC')
            return None
    except Exception as e:
        logexception(e)
        # type, value, traceback = sys.exc_info()
        # print('Error opening %s: %s' % (value.filename, value.strerror))
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print('dir_path=%s' % str(dir_path))
    # file_path = '/home/pi/testgc200/pgc200.py' #swcode_file['code']['path']
    running_dir_path = os.path.dirname(os.path.realpath(__file__))
    log('running_dir_path =%s'%running_dir_path)
    if dir_pathA==running_dir_path:
        full_pathOf_newVersion = full_pathOf_newVersionB
        dir_newVersion = dir_pathB
    else:
        full_pathOf_newVersion = full_pathOf_newVersionA
        dir_newVersion = dir_pathA
    print('b4 file')
    log('full_pathOf_newVersion =%s'%full_pathOf_newVersion)
    if msgformat==2:
        print('msgformat==2')
        fzip = open(dir_newVersion+'/newVersionzip.zip','w+')
        print('b4 file')
        fzip.writelines(scode)
        fzip. close()
        extractall(dir_newVersion,'newVersionzip.zip')
        return swcode_file
    f1 = open(full_pathOf_newVersion,'w+')
    print('b4 file')
    f1.writelines(scode)
    f1. close()
    return swcode_file

# выключение
def shutdown(snt=False):
    print ("shutdown()")
    log ("shutdown()")
    Home() # Переместить всю систему домой
    if snt==True:
        SendHostMessage('%s\n' %str({'shutdown Accepted': '1', 'Command':flgs.lastcommand}))
    sleep(0.1)
    blu_led.start(100)
    sleep(0.1)
    white_led.start(100)
    sleep(0.1)
    SpectTurnOffLamp()
    bt_unpairing()
    os.system("sudo poweroff")
    print('shutdown done')

# проверить, Нажата Ли Кнопка Питания
def IsPowerButtonPressed(duringBIT=False) -> object:
    if flgs.ignorePowerButtonPressed==1:
        return
    bButtonPressed=GPIO.input(PAIR)
    if bButtonPressed==1 :
        flgs.powerButtonPressed=True
        if duringBIT==True:
            flgs.ignorePowerButtonPressed=1
            return
    else:
        flgs.powerButtonPressed=False
    wt1=datetime.now()
    timeDifference=0 # Give that a normal name #fixed
    float_timeDifference=0.0 #Give that a normal name fixed
    if flgs.powerButtonPressedTime==None:
        flgs.powerButtonPressedTime=wt1
    try:
        timeDifference=wt1-flgs.powerButtonPressedTime
        print(str(timeDifference))
        float_timeDifference=timeDifference.total_seconds()
        print(str(float_timeDifference))
    except:
        print('in except')
        if bButtonPressed==1 :
            flgs.powerButtonPressed=1
            flgs.powerButtonPressedTime=wt1
            print(str(flgs.powerButtonPressedTime))
        else :
            flgs.powerButtonPressed=0
            flgs.powerButtonPressedTime=None
            print(str(flgs.powerButtonPressedTime))
        return
    print('flgs.powerButtonPressed='+ str(flgs.powerButtonPressed))
    fbutpressed=3.0
    if flgs.powerButtonPressed==1 and float_timeDifference> fbutpressed and bButtonPressed==1 :
        print ("PowerButtonPressed IN IF")
        log ("PowerButtonPressed")
        if duringBIT==False:
            shutdown(False)
        else:
            flgs.ignorePowerButtonPressed=1

# Время измерения спектральной темноты
def SpectDarkMeasurementTime():
    spectrometer.write('E2\r' )
    ReadMeasurementTime=spectrometer.readline()
    print('DarkMeasurementTime '+ReadMeasurementTime)
    print(ReadMeasurementTime)
    timeoutstr = ReadMeasurementTime.split(" ")
    timeout= int(timeoutstr[2].rstrip() )
    return timeout

# Спектральное время измерения
def SpectReffernceMeasurementTime():
    spectrometer.write('E1\r' )
    ReadMeasurementTime=spectrometer.readline()
    print('ReffernceMeasurementTime '+ReadMeasurementTime)
    timeoutstr = ReadMeasurementTime.split(" ")
    timeout= int(timeoutstr[2].rstrip() )
    return timeout

# Время измерения спектрального диапазона
def SpectSpectraMeasurementTime():
    log(' SpectSpectraMeasurementTime()')
    spectrometer.write('E0\r' )
    ReadMeasurementTime=spectrometer.readline()
    print('ReffernceMeasurementTime '+ReadMeasurementTime)
    timeoutstr = ReadMeasurementTime.split(" ")
    timeout= int(timeoutstr[2].rstrip() )
    return timeout

# Перемещение калибровки
def MoveToCalibration():    # система калибровки для измерения
    log(' MoveToCalibration()')
    log('mv2calib 2')
    if flgs.config["spectrometerAttributes"]['LampControlMode'] == manual_mode:
        SpectTurnOnLamp()
        log('mv2calib 3')
    if (flgs.WithContainer==True):    # and flgs.presetYlocation>0
        Move_z(flgs.bud_center_z)
    else:
        loc_z = int(flgs.config['calibrationAttributes']['CenterZ']*mm_to_step_Z/1000.0)
        Move_z(loc_z)
    print('pos.z %d' %pos.z)
    print('pos.x %d' %pos.x)
    Home_x()
    loc_x = 0
    loc_x = int(flgs.config['calibrationAttributes']['CenterX']*mm_to_step_X/1000.0)
    Move_x(loc_x)
    print(flgs.config['calibrationAttributes']['CenterY'])
    loc_y = 0
    log( 'mv2calib flgs.fix_steps_y ' +str(flgs.fix_steps_y) )
    log( 'mv2calib flgs.calibration_y_steps ' +str(flgs.calibration_y_steps) )
    if flgs.calibration_y_steps==-1:
        loc_y = int(flgs.config['calibrationAttributes']['CenterY']*mm_to_step_Y/1000.0)
    else:
        loc_y =flgs.fix_steps_y + flgs.calibration_y_steps
    print('calib Y_Distance: %f' %loc_y)
    Move_yForCalibration(loc_y)     # mm_to_step_Y*(flgs.config['calibrationAttributes']['Y_Distance']/1000))
    log('CALIBDISTANCEPARAMETER='+str(CALIBDISTANCEPARAMETER))
    log('@mv2calib loc_y'+str(loc_y))
    log( 'mv2calib pos.y %d' %pos.y)
    log( 'mv2calib pos.z %d' %pos.z)
    log( 'mv2calib pos.x %d' %pos.x)

# получить темноту
def getDark(test):    #система калибровки для измерения
    print('getDarktest ')
    darktimeout=SpectDarkMeasurementTime()
    print(darktimeout)
    flgs.dark_results =SpectStartDarkMeasurement()
    sleep(darktimeout/1000.0)
    flgs.dark_results = ReadDarkMeasurementData(flgs.config['spectrometerAttributes']['NumOfPoints'])
    print(flgs.dark_results)

# получить ссылку
def getReference():    #система калибровки для измерения
    log(' getReference()')
    MoveToCalibration()
    reftimeout=SpectReffernceMeasurementTime()
    SpectStartReferenceMeasurement()
    print('After Refference: ')
    if flgs.config["spectrometerAttributes"]['LampControlMode'] == manual_mode:
        SpectTurnOffLamp()
    sleep(reftimeout/1000.0)
    flgs.reference_results = ReadReferenceMeasurementData(flgs.config['spectrometerAttributes']['NumOfPoints'])
    darktimeout=SpectDarkMeasurementTime()
    SpectStartDarkMeasurement()
    sleep(darktimeout/1000.0)
    calib_results = {'Reference':flgs.reference_results,'Dark': flgs.dark_results}
    SendHostMessage('%s\n' %str(calib_results))
    print(calib_results)
    return calib_results

# Измеренные расстояния
def measureDistances():
    log(' measureDistances()')
    print(' ***********measureDistances()*****************')
    print(' ***********pos.z=' +str(pos.z) +'*****************')
    log('measureDistances start')
    ddebug=True
    flgs.small_flower= False
    flgs.distance_list_measurement_origin= [bool]*flgs.config['examinationAttributes']['NumberOfLocations']
    flgs.distance_list = [float]*flgs.config['examinationAttributes']['NumberOfLocations'] # 
    flgs.z_list = [float]*flgs.config['examinationAttributes']['NumberOfLocations'] 
    flgs.PointsList = []
    for i in range(flgs.config['examinationAttributes']['NumberOfLocations']):
        flgs.distance_list[i]=-1.0
        flgs.distance_list_measurement_origin[i]=True
        flgs.PointsList.append(Point(-1.0, True,-1.0))
    flgs.NumberOfLocations=flgs.config['examinationAttributes']['NumberOfLocations']
    print('NumberOfLocations: %f'%(flgs.NumberOfLocations))
    flgs.presetYlocation=0
    lastpoint=int(flgs.NumberOfLocations)-1
    print('lastpoint: %d'%(lastpoint))
    for i in range(flgs.config['examinationAttributes']['NumberOfLocations']):
        print(int(flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0))
    print('--')
    flgs.presetYlocation=flgs.config['examinationAttributes']['Locations'][lastpoint]['Y_Distance']   #no camera
    print('presetYlocation:%f' %(flgs.presetYlocation))
    if flgs.presetYlocation>0:
        flgs.BudLength = DefaultBudLength
    else:
        Move_y(flgs.fix_steps_y)
        flgs.BudLength = -1*flgs.presetYlocation/1000.0
        flgs.NumberOfLocations=flgs.NumberOfLocations-1
    flgs.correction_factor = location_rescale(flgs.BudLength)
    print('correction_factor: %f  bud_center_z:%f' %(flgs.correction_factor ,flgs.bud_center_z))
    Move_z(flgs.bud_center_z)
    z_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0)
    if flgs.small_flower==True:
        z_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0)* 0.81
    Move_z(flgs.bud_center_z+ int(z_distance_mm*mm_to_step_Z))
    Move_x(flgs.bud_center_x)
    totalDistance = 0
    flgs.counterNonZeroDistance = 0
    # distanceExists= False
    # pointDistanceExists= 0
    if flgs.presetYlocation>0: # с камерой
        for i in range(flgs.config['examinationAttributes']['NumberOfLocations']):
            flgs.z_list[i] =float(flgs.config['examinationAttributes']['Locations'][i]['Z_Offset'])/1000.0
            flgs.PointsList[i].zlocation =float(flgs.config['examinationAttributes']['Locations'][i]['Z_Offset'])/1000.0
        print(flgs.z_list)
        for i in range(flgs.config['examinationAttributes']['NumberOfLocations']):
            if flgs.distance_list[i]== -1.0:
                sleep(0.2)
                print('flgs.z_list[i]=[%d],%d'%(i,flgs.z_list[i]))
                decPointVal = decimal.Decimal(flgs.correction_factor*mm_to_step_Z*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0)
                zval=round(decPointVal,0)
                print(zval)
                Move_z(flgs.bud_center_z +int(zval) )
                x_distance_mm = flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['X_Offset']/1000.0
                if x_distance_mm > 5:
                    x_distance_mm = 5
                if x_distance_mm < -5:
                    x_distance_mm = -5
                Move_x(int(pos.x + mm_to_step_X * x_distance_mm))
                sleep(0.5) #TODO(Ido): Check if that is necessary
                flgs.distance_from_home=MEASURE_DISTANCE_PHASE_1
                Move_y(flgs.fix_steps_y)
                current_distance = get_min_distance()
                if current_distance<0:
                    # current_distance=10.4 #TODO(Ido): Make this a constant    DEFAULT_DISTANCE=10.4
                    current_distance=DEFAULT_DISTANCE #TODO(Ido): Make this a constant    DEFAULT_DISTANCE=10.4
                # if current_distance>=10.3: #TODO(Ido): Make this a constant    TOFAR_DISTANCE=10.3
                if current_distance>=TOFAR_DISTANCE: #TODO(Ido): Make this a constant    TOFAR_DISTANCE=10.3
                    # flgs.distance_from_home=2 #TODO(Ido): Make this a constant
                    flgs.distance_from_home=MEASURE_DISTANCE_PHASE_2
                    current_distance = get_min_distance()
                    # if current_distance>=10.3: #TODO(Ido): Make this a constant
                    if current_distance>=TOFAR_DISTANCE: #TODO(Ido): Make this a constant
                        #  flgs.distance_from_home=3  #TODO(Ido): Make this a constant
                         flgs.distance_from_home=MEASURE_DISTANCE_PHASE_3
                         current_distance = get_min_distance()
                         log('MEASURE_DISTANCE_PHASE_3 ' +str(current_distance))
                         if current_distance>=9: #TODO(Ido): Make this a constant SMALL_FLOWER_DISTANCE=9
                            flgs.small_flower=True
                            # Move_z(flgs.bud_center_z+700)  #TODO(Ido): Make this a constant   SMALL_FLOWER_DISTANCE_MOVE_UP=700
                            Move_z(flgs.bud_center_z+SMALL_FLOWER_DISTANCE_MOVE_UP)
                            flgs.distance_from_home=MEASURE_DISTANCE_PHASE_4
                            current_distance = get_min_distance()
                log('current_distance =%f %d'%(current_distance,flgs.distance_from_home ))
                print('current_distance =%f %d'%(current_distance,flgs.distance_from_home ))
                flgs.distance_list[i] = current_distance
                flgs.distance_list_measurement_origin[i] = flgs.distance_from_home
                flgs.PointsList[i].measurement_origin = flgs.distance_from_home
                flgs.PointsList[i].distance = current_distance
                low=  flgs.z_list[i]-1.5
                high= flgs.z_list[i]+1.5
                print('low=%f high=%f'%(low,high))
                for j in range(0,flgs.NumberOfLocations):
                    if flgs.distance_list[j]== -1:
                        if flgs.z_list[j] >= low and flgs.z_list[j] <= high:
                            print('***[j]=[%d] flgs.z_list[i]=[%f] current_distance=[%d]'%(i,j,current_distance))
                            flgs.distance_list[j]= current_distance
                            flgs.distance_list_measurement_origin[j]= flgs.distance_from_home
                    if flgs.PointsList[j].distance== -1:
                        if flgs.PointsList[j].zlocation >= low and flgs.PointsList[j].zlocation <= high:
                            print('***[j]=[%d] flgs.z_list[i]=[%f] current_distance=[%d]'%(i,j,current_distance))
                            flgs.PointsList[j].distance= current_distance
                            flgs.PointsList[j].measurement_origin= flgs.distance_from_home
                if ddebug==True:
                    for i in range(flgs.NumberOfLocations):
                        print('------distance_list[%d] :%d' %(i,flgs.distance_list[i]))
                        print(flgs.distance_list_measurement_origin[i])
                if ddebug==True:
                    for i in range(flgs.NumberOfLocations):
                        print('------distance_list[%d] :%d ' %(i,flgs.PointsList[j].distance))
                        print(flgs.PointsList[j].distance)
                if current_distance>0:
                    totalDistance=totalDistance+current_distance
                    flgs.counterNonZeroDistance=flgs.counterNonZeroDistance+1
    else:   # без камеры
        for i in range(flgs.NumberOfLocations):
            print('i in loop :%d' %(i))
            flgs.distance_list[i] = flgs.config['examinationAttributes']['Locations'][i]['Y_Distance']/1000.0
    if ddebug==True:
        for i in range(flgs.NumberOfLocations):
            print('distance_list[%d] :%f,origi=%d' %(i,flgs.distance_list[i],flgs.distance_list_measurement_origin[i]))
            print(flgs.distance_list_measurement_origin[i])
            log( '------distance_list[%d] :%f' %(i,flgs.distance_list[i]))
    flgs.averageDistance=0
    if flgs.counterNonZeroDistance>0:
        flgs.averageDistance = totalDistance/flgs.counterNonZeroDistance
        print('averageDistance: %f' %flgs.averageDistance)
    else:
        sexeption=  ' measureDistances averageDistance: ' +str(flgs.averageDistance)
        check_if_exception_in_file(sexeption)
    GPIO.output(CAM_LIGHT,GPIO.HIGH)  # только местоположение выключает светодиоды
    log('measureDistances end')

# переместить калибровку Extract
def moveToCalibrateExtract():    
    log(' moveToCalibrateExtract()')
    log('mv2calibExtract 2')
    SpectTurnOnLamp()
    log('mv2calibExtract 3')
    loc_z = int(flgs.configExtract['calibrationAttributes']['CenterZ']*mm_to_step_Z/1000.0)
    flgs.bud_center_z=loc_z
    log('####in moveToCalibrateExtract flgs.bud_center_z='+str(flgs.bud_center_z))
    Move_z(loc_z)
    loc_x = int(flgs.configExtract['calibrationAttributes']['CenterX']*mm_to_step_X/1000.0)
    flgs.bud_center_x=loc_x
    Move_x(loc_x)
    print('pos.z %d' %pos.z)
    print('pos.x %d' %pos.x)
    log( 'pos.z %d' %pos.z)
    log(  'pos.x %d' %pos.x)
    print(flgs.configExtract['calibrationAttributes']['CenterY'])
    loc_y = 0
    loc_y = int(flgs.configExtract['calibrationAttributes']['CenterY']*mm_to_step_Y/1000.0)
    print('calib Y_Distance: %f' %loc_y)
    print('calib_y _offsete: %f' %flgs.extractYoffset)
    Move_y(int(loc_y+flgs.extractYoffset))    
    log('moveToCalibrateExtract 4 '+ str(pos.y))
    sleep(3)
    
#    Калибровка
def calibration(snd2server,numpoints):    # система калибровки для измерения
    whistleDog(functionStartTimout)
    log('calibration 1')
    print('calib 1')
    whistleDog(calibrateTimout)
    print('calib 2')
    log('calib 2')
    flgs.reference_results = [None]* 1 # flgs.config['spectrometerAttributes']['NumOfPoints'] #flgs.config['examinationAttributes']['NumberOfLocations']
    reftimeout=SpectReffernceMeasurementTime()  #test rareearth
    print(numpoints)
    print(reftimeout)
    log(str(reftimeout))
    wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    stt1 = str(wt1)
    IsPowerButtonPressed()
    lgstr= '^*^CALIB reftimeout SpectReffernceMeasurementTime'+str(reftimeout)+' '+stt1
    spectTime=SpectSpectraMeasurementTime()
    print(spectTime)
    log(str(spectTime))
    lgstr= '^*^CALIB readTimeOut SpectSpectraMeasurementTime'+str(spectTime)
    sleep(2)
    wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    stt1 = str(wt1)
    lgstr= '^*^CALIB  b4 measurement '+' '+stt1
    log(lgstr )
    IsPowerButtonPressed()
    log('calib before SpectStartMeasurement')
    SpectStartMeasurement()   #SpectStartReferenceMeasurement()
    flgs.reference_results[0] = ReadMeasurementData(numpoints)
    wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
    stt1 = str(wt1)
    lgstr= '^*^CALIB  after measurement '+' '+stt1
    log(lgstr )
    IsPowerButtonPressed()
    log('measurepoint pos x=%d'%pos.x)
    log('measurepoint pos y=%d'%pos.y)
    log('measurepoint pos z=%d'%pos.z)
    if flgs.mode ==1:
        log('calib 4')
        HomeFromExtract()
    else:
        Home() # Переместить всю систему домой
        log('calib 4')
    bcover=False
    calib_results = {'Reference':flgs.reference_results[0],'Dark': [], 'Command':flgs.lastcommand, 'DrawerOpened':bcover}
    log('calib 5')
    if snd2server==True:
        SendHostMessage('%s\n' %str(calib_results))
        log('calib 6')
    IsPowerButtonPressed()
    print(calib_results)
    log('calib 7')
    return calib_results

#  расчет поглощения
def calculate_Absorbance(reflectance, dark, reference): #calculate Absorbance
    log('calculate_Absorbance ')
    Absorbance = [0.0]*len(reflectance)
    for i in range(len(reflectance)):
        Absorbance[i] = float(-log10((reflectance[i] - dark[i]) / reference[i]))
    return Absorbance

# изменение масштаба местоположения
def location_rescale(bud_length = DefaultBudLength): # Изменение масштаба местоположения в соответствии с длиной бутона
    return float((bud_length / DefaultBudLength)*0.7)

#  Выбрать конфигурацию по атрибутам
def byAttributesSpectConf(sample):
    log('byAttributesSpectConf() %s '%sample)
    print('in byAttributesSpectConf    ')
    autoDark=1 # remove read from attributes
    if sample=='spectrometer':
        autoDark=flgs.config['spectrometerAttributes']['AutomaticDark']
    if sample=='plastic':
        autoDark=flgs.config['plasticSpectrometerAttributes']['AutomaticDark']
    if sample=='white':
        autoDark=flgs.config['whiteSpectrometerAttributes']['AutomaticDark']
    if sample=='dark':
        autoDark=flgs.config['darkSpectrometerAttributes']['AutomaticDark']
    #     установить режим управления лампой , 1 = auto, 0 = manual
    spectrometer.write('PA%d\r' %autoDark)        #set lamp control mode, 1 = auto, 0 = manual
    print(spectrometer.readline())
    LampWarmupTime= examinationWarmupTime   # 1000
    if sample=='spectrometer':
        LampWarmupTime=flgs.config['spectrometerAttributes']['LampWarmUpTimeForSensor']
    if sample=='plastic':
        LampWarmupTime=flgs.config['plasticSpectrometerAttributes']['LampWarmUpTimeForSensor']
    if sample=='white':
        LampWarmupTime=flgs.config['whiteSpectrometerAttributes']['LampWarmUpTimeForSensor']
    if sample=='dark':
        LampWarmupTime=flgs.config['darkSpectrometerAttributes']['LampWarmUpTimeForSensor']
    # установить время прогрева лампы
    spectrometer.write('LT%d\r' %LampWarmupTime)        #set light warm up time    DarkSubtraction
    print(spectrometer.readline())
    DarkSubtraction= 1   # 1000
    if sample=='spectrometer':
        DarkSubtraction=flgs.config['spectrometerAttributes']['DarkSubtraction']
    if sample=='plastic':
        DarkSubtraction=flgs.config['plasticSpectrometerAttributes']['DarkSubtraction']
    if sample=='white':
        DarkSubtraction=flgs.config['whiteSpectrometerAttributes']['DarkSubtraction']
    if sample=='dark':
        DarkSubtraction=flgs.config['darkSpectrometerAttributes']['DarkSubtraction']
    # установить время прогрева лампы Затемнение
    spectrometer.write('PB%d\r' %DarkSubtraction)        #set light warm up time    DarkSubtraction
    print(spectrometer.readline())
    NumOfPoints=flgs.config["spectrometerAttributes"]['NumOfPoints']
    SpectSetWL(NumOfPoints)
    WavelengthAverage = 400
    ScanAverage = 1
    if sample=='spectrometer':
        WavelengthAverage=flgs.config['spectrometerAttributes']['WavelengthAverage']
        ScanAverage=flgs.config['spectrometerAttributes']['ScanAverage']
    if sample=='plastic':
        WavelengthAverage=flgs.config['plasticSpectrometerAttributes']['WavelengthAverage']
        ScanAverage=flgs.config['plasticSpectrometerAttributes']['ScanAverage']
    if sample=='white':
        WavelengthAverage=flgs.config['whiteSpectrometerAttributes']['WavelengthAverage']
        ScanAverage=flgs.config['whiteSpectrometerAttributes']['ScanAverage']
    if sample=='dark':
        WavelengthAverage=flgs.config['darkSpectrometerAttributes']['WavelengthAverage']
        ScanAverage=flgs.config['darkSpectrometerAttributes']['ScanAverage']
    # установить среднее значение длины волны и среднее значение сканирования
    spectrometer.write('V%d,%d\r' %(WavelengthAverage,ScanAverage)) #set wavelength average and scan average
    print(spectrometer.readline())
    LampControlMode=0
    if sample=='spectrometer':
        LampControlMode=flgs.config["spectrometerAttributes"]['LampControlMode']
    if sample=='plastic':
        LampControlMode=flgs.config["plasticSpectrometerAttributes"]['LampControlMode']
    if sample=='white':
        LampControlMode=flgs.config["whiteSpectrometerAttributes"]['LampControlMode']
    if sample=='dark':
        LampControlMode=flgs.config["darkSpectrometerAttributes"]['LampControlMode']
    # установить режим управления лампой, 1 = auto, 0 = manual
    spectrometer.write('LM%d\r' %LampControlMode)        #set lamp control mode, 1 = auto, 0 = manual
    print(spectrometer.readline())
    NumOfPoints=0
    if sample=='spectrometer':
        NumOfPoints=flgs.config["spectrometerAttributes"]['NumOfPoints']
    if sample=='plastic':
        NumOfPoints=flgs.config["plasticSpectrometerAttributes"]['NumOfPoints']
    if sample=='white':
        NumOfPoints=flgs.config["whiteSpectrometerAttributes"]['NumOfPoints']
    if sample=='dark':
        NumOfPoints=flgs.config["darkSpectrometerAttributes"]['NumOfPoints']
    SpectSetWL(NumOfPoints)
    DivideMode=0
    if sample=='spectrometer':
        DivideMode=flgs.config["spectrometerAttributes"]['DivideMode']
    if sample=='plastic':
        DivideMode=flgs.config["plasticSpectrometerAttributes"]['DivideMode']
    if sample=='white':
        DivideMode=flgs.config["whiteSpectrometerAttributes"]['DivideMode']
    if sample=='dark':
        DivideMode=flgs.config["darkSpectrometerAttributes"]['DivideMode']
    # установить режим управления лампой, 1 = auto, 0 = manual
    spectrometer.write('PD%d\r' %DivideMode)        #set lamp control mode, 1 = auto, 0 = manual
    print(spectrometer.readline())
    LogMode=0
    if sample=='spectrometer':
        LogMode=flgs.config["spectrometerAttributes"]['LogMode']
    if sample=='plastic':
        LogMode=flgs.config["plasticSpectrometerAttributes"]['LogMode']
    if sample=='white':
        LogMode=flgs.config["whiteSpectrometerAttributes"]['LogMode']
    if sample=='dark':
        LogMode=flgs.config["darkSpectrometerAttributes"]['LogMode']
    # установить режим управления лампой, 1 = auto, 0 = manual
    spectrometer.write('PC%d\r' %LogMode)        #set lamp control mode, 1 = auto, 0 = manual
    print(spectrometer.readline())
    LampIntensity=0
    if sample=='spectrometer':
        LampIntensity=flgs.config["spectrometerAttributes"]['LampIntensity']
    if sample=='plastic':
        LampIntensity=flgs.config["plasticSpectrometerAttributes"]['LampIntensity']
    if sample=='white':
        LampIntensity=flgs.config["whiteSpectrometerAttributes"]['LampIntensity']
    if sample=='dark':
        LampIntensity=flgs.config["darkSpectrometerAttributes"]['LampIntensity']
    # установить режим управления лампой, 1 = auto, 0 = manual
    SetSpectIntensity(LampIntensity)        #set lamp control mode, 1 = auto, 0 = manual
    LogMode=0
    if sample=='spectrometer':
        LogMode=flgs.config["spectrometerAttributes"]['LogMode']
    if sample=='plastic':
        LogMode=flgs.config["plasticSpectrometerAttributes"]['LogMode']
    if sample=='white':
        LogMode=flgs.config["whiteSpectrometerAttributes"]['LogMode']
    if sample=='dark':
        LogMode=flgs.config["darkSpectrometerAttributes"]['LogMode']
    spectrometer.write('PC%d\r' %LogMode)        # установить режим управления лампой, 1 = auto, 0 = manual
    print(spectrometer.readline())

# конфигурация  проверки спектрометра
def examSpectConf():
    log('examSpectConf() ')
    print('in examSpectConf    ')
    autoDark=1
    spectrometer.write('PA%d\r' %autoDark)        # установить режим управления лампой, 1 = auto, 0 = manual
    print(spectrometer.readline())
    LampWarmupTime= examinationWarmupTime   # 1000 Время прогрева проверки
    spectrometer.write('LT%d\r' %LampWarmupTime)        # установить время прогрева лампы
    print(spectrometer.readline())

# вернуть регулярную конфигурацию
def backtoRegularSpectConf():
    log('examSpectConf() ')
    print('in examSpectConf    ')
    autoDark=0
    spectrometer.write('PA%d\r' %autoDark)        # установить режим управления лампой, 1 = auto, 0 = manual
    print(spectrometer.readline())
    LampWarmupTime=  stabilityWarmupTime   # 200
    spectrometer.write('LT%d\r' %LampWarmupTime)       # установить время прогрева лампы
    print(spectrometer.readline())

# проверить конфигурацию спектрометра не темный
def examConfNoDark():
    log('examSpectConf() ')
    print('in examSpectConf    ')
    autoDark=0
    spectrometer.write('PA%d\r' %autoDark)        # установить режим управления лампой, 1 = auto, 0 = manual
    print(spectrometer.readline())
    LampWarmupTime=  examinationWarmupTime   # 200
    spectrometer.write('LT%d\r' %LampWarmupTime)        # установить время прогрева лампы
    print(spectrometer.readline())

#  переместить в центр
def moveToCenter():
    log('moveToCenter() ')
    Move_z(flgs.bud_center_z)

    Move_x(flgs.bud_center_x)
    return
    # target_distance=0
    # if flgs.presetYlocation>0:
    #     Move_y(flgs.safe_y_distance)
    #     current_distance =flgs.distance_list[0]
    #     target_distance = flgs.config['examinationAttributes']['Locations'][0]['Y_Distance']/1000.0

    # else:
    #     current_distance = flgs.config['stabilityAttributes']['Y_Distance'] / 1000.0

    # target_distance = flgs.config['examinationAttributes']['Locations'][0]['Y_Distance']/1000.0
    # yToMve=int(flgs.safe_y_distance + ( current_distance- target_distance)*mm_to_step_Y)
    # print 'yToMve : %f mm ' %yToMve
    # Move_y(int(yToMve))

#  переместить в центр
def moveToCenter1(z_list = (0,300) , threshold = 3000 , max_y = 3700 , y_step_mm = 1):
    for z_position in z_list:
        print('z_position = ' + str(z_position))
        Move_x(flgs.bud_center_x)
        Move_z(flgs.MAX_RANGE_Z - z_position)
        Home_y()
        print('y_position = ' + str(pos.y))
        average_reflectance = 0
        current_y_position = 0
        counter = 0
        reflectance1 = [None]
        while ((average_reflectance < threshold ) and (current_y_position < max_y)):
            print('current_y_position = ' + str(current_y_position))
            print('average_reflectance = ' + str(average_reflectance))
            counter = counter + 1
            print('counter = ' + str(counter))
            current_y_position = Move_y(pos.y+(y_step_mm * counter)* mm_to_step_Y)
            SpectStartMeasurement()
            print('Measurement started')
            reflectance1 = ReadMeasurementData(flgs.config['spectrometerAttributes']['NumOfPoints'])
            print(reflectance1)
            average_reflectance = numpy.mean(reflectance1)
        if average_reflectance >= threshold:
            return current_y_position , average_reflectance
    return None,None

# ПолучитьТемный Спектр
def getOneSpectraDark(): #get reflectance for define y distance from flower --- получить коэффициент отражения для определения расстояния y от цветка
    whistleDog(calibrateTimout)
    log('getOneSpectraDark() ')
    SpectStartMeasurement()
    print('Measurement started') # Измерение началось
    reflectance1 = ReadMeasurementData(flgs.config['spectrometerAttributes']['NumOfPoints'])
    print(reflectance1)
    return reflectance1
    # average_reflectance = numpy.mean(reflectance1)

# получите один спектр
def getOneSpectra(): #get reflectance for define y distance from flower
    log('getOneSpectra()')
    SpectStartMeasurement()
    print('Measurement started') # Измерение началось
    reflectance1 = ReadMeasurementData(flgs.config['spectrometerAttributes']['NumOfPoints'])
    print(reflectance1)
    return reflectance1

#  только для отладки, вызов из терминала, получить коэффициент отражения для определения расстояния y от цветка
def getOneSpectra2(): #for debug only call from terminal  get reflectance for define y distance from flower
    backtoRegularSpectConf()
    SpectStartMeasurement()
    print('Measurement started')
    reflectance1 = ReadMeasurementData(41)
    print(reflectance1)
    return reflectance1
 
# проверка -- анализ непосредственно ! --
def exam(nocamera=0):
    confsave=flgs.config
    if  flgs.mode==1:
        flgs.config= flgs.configExtract    
    if nocamera==0:
        Move_y(flgs.fix_steps_y)
    else:
        flgs.correction_factor=1
    whistleDog(functionStartTimout)
    flgs.NumberOfLocations = flgs.config['examinationAttributes']['NumberOfLocations'] # 1
    print(flgs.NumberOfLocations)
    log('####in exam flgs.bud_center_z='+str(flgs.bud_center_z))
    for i in range(flgs.NumberOfLocations):
        temperature=getTemperature()
        whistleDog(singleSpectratMeasurementTimout)
        sleep(0.1)
        spectTime=SpectSpectraMeasurementTime()
        print(spectTime)
        log(str(spectTime ))
        if flgs.mode==2:
            Move_y(pos.y - 500)
        if flgs.mode==0:
            if pos.y!=flgs.fix_steps_y:
                Move_y(pos.y - 1000)
        print('after spectTime')
        # для камера не будет возвращаться за каждой точкой
        if (flgs.WithContainer == False and nocamera==0)  :   #for noc will not go back for every point
            if (flgs.presetYlocation > 0):   # с камерой
                Move_y(flgs.safe_y_distance - 500)   # вернуться назад, чтобы не нагревать цветок
        print('pos y=%d'%pos.y)
        log('pos y=%d'%pos.y)
        sleep(0.2)
        decPointVal = decimal.Decimal(flgs.correction_factor*mm_to_step_Z*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0)
        zval=round(decPointVal,0)
        print(zval)
        if i==8:
            print('-8---zval=%f' %zval)
        print(flgs.bud_center_z)
        Move_z(flgs.bud_center_z +int(zval) )
        z_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0)
        if flgs.small_flower==True:
            z_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0)*0.81

        # stepsD_Z=z_distance_mm*mm_to_step_Z
        # log('point '+str(i)+' attribute='+str(flgs.config["examinationAttributes"]["Locations"][i]["Z_Offset"])+' z_distance_mm_steps='+str(stepsD_Z))
        Move_z(flgs.bud_center_z+int(z_distance_mm*mm_to_step_Z))
        # log('point '+str(i)+' attribute='+str(flgs.config["examinationAttributes"]["Locations"][i]["Z_Offset"])+' z_distance_mm_steps='+str(stepsD_Z)+' flgs.bud_center_z='+ str(flgs.bud_center_z)+' pos.z '+ str(pos.z))

        x_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['X_Offset']/1000.0)
        if flgs.small_flower==True:
            x_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['X_Offset']/1000.0)* 0.45
        if x_distance_mm > 5:
            x_distance_mm = 5
        if x_distance_mm < -5:
            x_distance_mm = -5
        # bug fixed
        Move_x(int(flgs.bud_center_x + mm_to_step_X * x_distance_mm))
        if flgs.presetYlocation>0:
            target_distance = float(flgs.config['examinationAttributes']['Locations'][i]['Y_Distance']/1000.0)
        else:
            target_distance = 0.0

        sleep(0.1)

        yToMve=0
        if nocamera==0: # calc yToMve flower --- вычисления для цветка ---
            current_distance =flgs.distance_list[i]
            print('current_distance =%f'%flgs.distance_list[i])
            print('target_distance =%f'%target_distance)
            if current_distance<flgs.averageDistance:  # patch just for the show
                current_distance=flgs.averageDistance

            if flgs.distance_list_measurement_origin[i]==MEASURE_DISTANCE_PHASE_1:
                if current_distance==0:   #big flower

                    yToMve=int(flgs.fix_steps_y + (12 - target_distance)*mm_to_step_Y) #test
                else:
                    yToMve=int(flgs.fix_steps_y + (current_distance - target_distance)*mm_to_step_Y)
            if flgs.distance_list_measurement_origin[i]==MEASURE_DISTANCE_PHASE_2:
                if current_distance==0:   #big flower
                    yToMve=int(1600 + (0 - target_distance)*mm_to_step_Y) #test was 1000
                else:

                    yToMve=int(1600 + (current_distance - target_distance)*mm_to_step_Y)
            if flgs.distance_list_measurement_origin[i]==MEASURE_DISTANCE_PHASE_3:
                if current_distance==0:   #big flower
                    yToMve=int(flgs.safe_y_distance + (0 - target_distance)*mm_to_step_Y) #test
                else:
                    yToMve=int(flgs.safe_y_distance + (current_distance )*mm_to_step_Y) # bug fixed no need to subtract here
            if flgs.distance_list_measurement_origin[i]==MEASURE_DISTANCE_PHASE_4:
                if current_distance==0:   #big flower
                    yToMve=int(flgs.safe_y_distance + (0 - target_distance)*mm_to_step_Y) #test
                else:
                    yToMve=int(flgs.safe_y_distance + (current_distance )*mm_to_step_Y) # bug fixed no need to subtract here

            print('measured_distance: %f mm yToMve : %f  ' %(current_distance,yToMve))

        if nocamera==1: # base yToMve for extract,ground --- вычисления для extract,ground
            fWcsY=float(flgs.config["spectrometerCenterAttributes"]['CenterY'])*(mm_to_step_Y/1000.0)
            wcsY=int(fWcsY)
            yToMve = wcsY
            
        loc_y_offset= 0
        loc_y_offset = flgs.extractYoffset  

        if flgs.mode==1:   # calc yToMve extract --- вычисления для extract
            yToMve = int(flgs.configExtract['calibrationAttributes']['CenterY']*mm_to_step_Y/1000.0)
            print('calib_y _offsete: %f' %loc_y_offset)
            yToMve = yToMve +loc_y_offset
            log('findCenterExtract 5 yToMve ='+ str(yToMve))
            
        if flgs.mode==2:  # calc yToMve ground --- вычисления для ground
            if i==0:
                yToMve = yToMve+GROUNDDISTANCEPARAMETER*2 +loc_y_offset
            else:
                yToMve = yToMve+GROUNDDISTANCEPARAMETER*2 - 20 +loc_y_offset
        Move_y(yToMve)
        print('measurepoint pos y=%d'%pos.y)
        log('point=%d'%i)
        log('measurepoint pos x=%d'%pos.x)
        log('measurepoint pos y=%d'%pos.y)
        log('measurepoint pos z=%d'%pos.z)
        print('flgs.small_flower= %d' %flgs.small_flower)

        readTimeOut=SpectReadMeasurementTime()
        wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        stt1 = str(wt1)
        log('^^readTimeOut b4 measurement '+str(readTimeOut)+' '+stt1)
        whistleDog(calibrateTimout)
        flgs.budDistanceFlag = False
        getSpectrometerActualParams()
        reflectance=getOneSpectra()
        
        pos_y=float(pos.y/mm_to_step_Y)
        pos_y=round(pos_y, 2)
        log('pos_y=%f'%pos_y)
        print('pos_y=%f'%pos_y)
        temperature = getTemperature()
        info ={}
        if i==0:
            fnjwide = '/home/pi/pct/budcenterw.jpg'
            if  os.path.exists(fnjwide):
                os.system('sudo rm  /home/pi/pct/budcenterw.jpg')
            GetCameraImageSaveForImageAnalysis('budcenter')
            if  os.path.exists(fnjwide):
                encode_string =''
                with open(fnjwide, "rb") as f:
                    bytes = f.read()
                    encode_string = base64.b64encode(bytes)

            SpectTurnOnLamp()        
            info = {'Absorbance': [],'Temperature': temperature,'Reference': flgs.calibration_results['Reference'],'Reflectance': reflectance,'CalibrationMaterial': flgs.calibrationmaterial['Reference'], 'PosY':pos_y, 'Command':flgs.lastcommand,'SamplePhoto':encode_string}
        else:
            info = {'Absorbance': [],'Temperature': temperature,'Reference': flgs.calibration_results['Reference'],'Reflectance': reflectance,'CalibrationMaterial': flgs.calibrationmaterial['Reference'], 'PosY':pos_y, 'Command':flgs.lastcommand,'SamplePhoto':""}
        print(info)
        log("exam sent " +str(info))

        SendHostMessage('%s\n' %str(info))
        print(info)

        print('i=%d'%i)
    #     двигайтесь так, чтобы не попасть в крышку
    Move_y(pos.y-400)# move  in order not hit the cover    
    Move_z(pos.z-500)    # move  in order not hit the cover    
    Move_y(pos.y-200)# move  in order not hit the cover    
    Move_z(flgs.safe_z_distance)    # move  in order not hit the cover    
    flgs.config=confsave
    Home() # Переместить всю систему домой
    flgs.canHeatForward = False
    fPlasticZ=float(flgs.config["plasticCenterAttributes"]['CenterZ'])*(mm_to_step_Z/1000.0)
    PlasticZ=int(fPlasticZ)
    print(PlasticZ)
    Move_z(PlasticZ)

# скачать точку для экстракта
def downloadPointsExtract():
    log(' downloadPointsExtract()')
    ConfigurationAccepted = False
    cnf1=''
    cnf2=''
    sleep(2)
    cnf1= ReceiveHostMessage(10000) # получить строку bluetooth, size 10000 signs
    try:
        sleep(2)
        cnf2= ReceiveHostMessage(10000) # получить 2-ю строку bluetooth, size 10000 signs
    except Exception as e:
        logexception(e)
        cnf2= ''
    if cnf2== 'null':
        cnf2= ''
    print('cnf2=%s '%cnf2)
    config_file =cnf1+cnf2   #+cnf3+cnf4 --- string concatenation in config file ---
    log (config_file)
    try:
        config_file = json.loads(config_file)
        ConfigurationAccepted = True
        log('ConfigurationAccepted = True sent')
    except Exception as e:
        logexception(e)
        ConfigurationAccepted = False
        log('ConfigurationAccepted = False sent')
    if ConfigurationAccepted:
        print(' AFTER json.loads(config_file)')
        try:
            flgs.LampCoolDownTimeForSoftware = float(config_file["spectrometerAttributes"]['LampCoolDownTimeForSoftware'])
            flgs.CoolDownHysteresis = float(config_file["spectrometerAttributes"]['CoolDownHysteresis'])
        except Exception as e:
            logexception(e)
        flgs.configExtract = config_file
        bcz=flgs.configExtract["spectrometerCenterAttributes"]['CenterZ']
        print("flgs.configExtract spectrometerCenterAttributes CenterZ " +str(bcz))
        print(flgs.configExtract)
    #     возвращает принятую конфигурацию
    return ConfigurationAccepted

#  поиск центра экстракта
def findCenterExtract(nocamera=1):
    
    whistleDog(functionStartTimout)
    
    if (flgs.WithContainer==True):    # and flgs.presetYlocation>0
        loc_z = int(flgs.config['calibrationAttributes']['CenterZ']*mm_to_step_Z/1000.0)
        Move_z(loc_z)
        loc_x = int(flgs.config['calibrationAttributes']['CenterX']*mm_to_step_X/1000.0)
        Move_x(loc_x)    
        loc_y = int(flgs.config['calibrationAttributes']['CenterY']*mm_to_step_Y/1000.0)
    
    print(flgs.NumberOfLocations)
    flgs.NumberOfLocations = flgs.config['examinationAttributes']['NumberOfLocations'] #TBD ONLY FOR TEST только для теста

    flgs.correction_factor=1

    for i in range(flgs.NumberOfLocations):
        temperature=getTemperature()
        whistleDog(singleSpectratMeasurementTimout)
        sleep(0.1)
        spectTime=SpectSpectraMeasurementTime()
        print(spectTime)
        log(str(spectTime ))
        print('after spectTime')
        if (flgs.WithContainer == False and nocamera==0)  :   #for noc will not go back for every point ??
            if (flgs.presetYlocation > 0):   # with canera
                Move_y(flgs.safe_y_distance - 500)   # отодвиньтесь назад, чтобы не нагревать цветок
        print('pos y=%d'%pos.y)
        log('pos y=%d'%pos.y)
        sleep(0.2)
        Move_y(flgs.safe_y_distance)
        decPointVal = decimal.Decimal(flgs.correction_factor*mm_to_step_Z*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0)
        zval=round(decPointVal,0)
        print(zval)
        if i==8:
            print('-8---zval=%f' %zval)
        print(flgs.bud_center_z)
        z_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000.0)
        Move_z(loc_z+int(z_distance_mm*mm_to_step_Z))

        x_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['X_Offset']/1000.0)
        # bug fixed
        Move_x(int(loc_x + mm_to_step_X * x_distance_mm))
        
        y_distance_mm = float(flgs.config['examinationAttributes']['Locations'][i]['Y_Distance']/1000.0)
        log('loc_y=%d'%loc_y)
        log('y_distance_mm=%d'%y_distance_mm)
        loc_y_offset= 0
        loc_y_offset = flgs.extractYoffset    
        print('calib_y _offsete: %f' %loc_y_offset)
        Move_y(int(loc_y + mm_to_step_Y * y_distance_mm)+loc_y_offset)
        log('findCenterExtract 5 '+ str(pos.y))
        sleep(0.1)
        print('measurepoint pos y=%d'%pos.y)
        print('i=%d'%i)
        log('i=%d'%i)
        log('measurepoint pos x=%d'%pos.x)
        log('measurepoint pos y=%d'%pos.y)
        log('measurepoint pos z=%d'%pos.z)
        print('flgs.small_flower= %d' %flgs.small_flower)
        readTimeOut=SpectReadMeasurementTime()
        wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        stt1 = str(wt1)
        log('^^readTimeOut b4 measurement '+str(readTimeOut)+' '+stt1)
        whistleDog(calibrateTimout)
        flgs.budDistanceFlag = False
        getSpectrometerActualParams()
        reflectance=getOneSpectra()
        pos_y=pos.y/mm_to_step_Y
        log('pos_y=%f'%pos_y)
        print('pos_y=%f'%pos_y)
        temperature = getTemperature()
        info ={}
        if i==0:
            fnjwide = '/home/pi/pct/budcenterw.jpg'
            if  os.path.exists(fnjwide):
                os.system('sudo rm  /home/pi/pct/budcenterw.jpg')
            GetCameraImageSaveForImageAnalysis('budcenter')
            if  os.path.exists(fnjwide):
                encode_string =''
                with open(fnjwide, "rb") as f:
                    bytes = f.read()
                    encode_string = base64.b64encode(bytes)
            SpectTurnOnLamp()        
            info = {'Absorbance': [],'Temperature': temperature,'Reference': [],'Reflectance': reflectance,'CalibrationMaterial': [], 'PosY':pos_y, 'Command':flgs.lastcommand,'SamplePhoto':encode_string}
        else:
            info = {'Absorbance': [],'Temperature': temperature,'Reference': [],'Reflectance': reflectance,'CalibrationMaterial': [], 'PosY':pos_y, 'Command':flgs.lastcommand,'SamplePhoto':""}
        print(info)
        SendHostMessage('%s\n' %str(info))
        print(info)
        print('i=%d'%i)

#  поиск точки экстракта
def findExtractPoint(nocamera=0):
    if nocamera==0:
        Move_y(flgs.fix_steps_y)
    whistleDog(functionStartTimout)
    print(flgs.NumberOfLocations)
    flgs.NumberOfLocations = flgs.config['examinationAttributes']['NumberOfLocations']
    # exam_results = [None]*flgs.NumberOfLocations #flgs.flgs.config['examinationAttributes']['NumberOfLocations']
    # Absorbance = [None]*flgs.NumberOfLocations #   flgs.config['examinationAttributes']['NumberOfLocations']
    byAttributesSpectConf('spectrometer')
    if nocamera==1:
        flgs.correction_factor=1
    for i in range(flgs.NumberOfLocations):
        temperature=getTemperature()
        whistleDog(singleSpectratMeasurementTimout)
        sleep(0.1)
        spectTime=SpectSpectraMeasurementTime()
        print(spectTime)
        log(str(spectTime ))
        print('after spectTime')
        if (flgs.WithContainer == False and nocamera==0)  :   #for noc will not go back for every point
            if (flgs.presetYlocation > 0):   # with canera
                # отодвиньтесь назад, чтобы не нагревать цветок
                Move_y(flgs.safe_y_distance - 500)   # move back in order not heat the flower
        print('pos y=%d'%pos.y)
        log('pos y=%d'%pos.y)
        sleep(0.2)
        decPointVal = decimal.Decimal(flgs.correction_factor*mm_to_step_Z*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000)
        zval=round(decPointVal,0)
        print(zval)
        if i==8:
            print('-8---zval=%f' %zval)
        print(flgs.bud_center_z)
        Move_z(flgs.bud_center_z +int(zval) )
        z_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000)
        if flgs.small_flower==True:
            z_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['Z_Offset']/1000)* 0.81

        Move_z(flgs.bud_center_z+int(z_distance_mm*mm_to_step_Z))

        x_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['X_Offset']/1000)
        if flgs.small_flower==True:
            x_distance_mm = float(flgs.correction_factor*flgs.config['examinationAttributes']['Locations'][i]['X_Offset']/1000)* 0.45
        if x_distance_mm > 5:
            x_distance_mm = 5
        if x_distance_mm < -5:
            x_distance_mm = -5
        if nocamera==0:
            Move_x(int(flgs.bud_center_x + mm_to_step_X * x_distance_mm))
        if flgs.presetYlocation>0:
            target_distance = float(flgs.config['examinationAttributes']['Locations'][i]['Y_Distance']/1000)
        else:
            target_distance = 0.0
        sleep(0.1)
        yToMve=0
        if nocamera==0:
            current_distance =flgs.distance_list[i]
            print('current_distance =%f'%flgs.distance_list[i])
            print('target_distance =%f'%target_distance)
            if current_distance<flgs.averageDistance:  # patch just for the show
                current_distance=flgs.averageDistance
            if flgs.distance_list_measurement_origin[i]==1:
                if current_distance==0:   #big flower
                    yToMve=int(flgs.fix_steps_y + (12 - target_distance)*mm_to_step_Y) #test
                else:
                    yToMve=int(flgs.fix_steps_y + (current_distance - target_distance)*mm_to_step_Y)
            if flgs.distance_list_measurement_origin[i]==2:
                if current_distance==0:   #big flower
                    yToMve=int(1600 + (0 - target_distance)*mm_to_step_Y) #test was 1000
                else:
                    yToMve=int(1600 + (current_distance - target_distance)*mm_to_step_Y)
            if flgs.distance_list_measurement_origin[i]==3:
                if current_distance==0:   #big flower
                    yToMve=int(flgs.safe_y_distance + (0 - target_distance)*mm_to_step_Y) #test
                else:
                    yToMve=int(flgs.safe_y_distance + (current_distance - target_distance)*mm_to_step_Y)

            print('measured_distance: %f mm yToMve : %f  ' %(current_distance,yToMve))
        if nocamera==1:
            fWcsY=float(flgs.config["spectrometerCenterAttributes"]['CenterY'])*(mm_to_step_Y/1000.0)
            wcsY=int(fWcsY)
            yToMve=    wcsY
        Move_y(yToMve)
        print('counter=%d'%flgs.counter)
        print('measurepoint pos x=%d'%pos.x)
        print('measurepoint pos z=%d'%pos.z)
        print('measurepoint pos y=%d'%pos.y)
        log('EXAM')
        log('measurepoint pos x=%d'%pos.x)
        log('measurepoint pos y=%d'%pos.y)
        log('measurepoint pos z=%d'%pos.z)
        print('flgs.small_flower= %d' %flgs.small_flower)
        readTimeOut=SpectReadMeasurementTime()
        wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        stt1 = str(wt1)
        log('^^readTimeOut b4 measurement '+str(readTimeOut)+' '+stt1)
        whistleDog(calibrateTimout)
        flgs.budDistanceFlag = False
        getSpectrometerActualParams()
        sleep(5)
        reflectance=getOneSpectra()
        pos_y=pos.y/mm_to_step_Y
        log('pos_y=%f'%pos_y)
        print('pos_y=%f'%pos_y)
        temperature = getTemperature()
        info ={}
        if i==0:
            fnjwide = '/home/pi/pct/budcenterw.jpg'
            if  os.path.exists(fnjwide):
                os.system('sudo rm  /home/pi/pct/budcenterw.jpg')
            GetCameraImageSaveForImageAnalysis('budcenter')
            if  os.path.exists(fnjwide):
                encode_string =''
                with open(fnjwide, "rb") as f:
                    bytes = f.read()
                    encode_string = base64.b64encode(bytes)
            SpectTurnOnLamp()        
            info = {'Absorbance': [],'Temperature': temperature,'Reference': flgs.calibration_results['Reference'],'Reflectance': reflectance,'CalibrationMaterial': [], 'PosY':pos_y, 'Command':flgs.lastcommand,'SamplePhoto':encode_string}
        else:
            info = {'Absorbance': [],'Temperature': temperature,'Reference': flgs.calibration_results['Reference'],'Reflectance': reflectance,'CalibrationMaterial': flgs.calibrationmaterial['Reference'], 'PosY':pos_y, 'Command':flgs.lastcommand,'SamplePhoto':""}
        print(info)
        SendHostMessage('%s\n' %str(info))
        print(info)
        print('i=%d'%i)
    Move_y(pos.y-550)
    sleep(0.1)
    Move_z(flgs.safe_z_distance)
    Home() # Переместить всю систему домой

#   проверка wcs
def examWcs():
    whistleDog(functionStartTimout)
    examSpectConf()
    print(flgs.NumberOfLocations)
    flgs.NumberOfLocations = flgs.config['examinationAttributes']['NumberOfLocations']
    # exam_results = [None]*flgs.NumberOfLocations #flgs.flgs.config['examinationAttributes']['NumberOfLocations']
    # Absorbance = [None]*flgs.NumberOfLocations #   flgs.config['examinationAttributes']['NumberOfLocations']
    for i in range(flgs.NumberOfLocations):
        temperature=getTemperature()
        whistleDog(singleSpectratMeasurementTimout)
        sleep(0.4)
        spectTime=SpectSpectraMeasurementTime()
        print(spectTime)
        log(str(spectTime ))
        print('after spectTime')
        readTimeOut=SpectReadMeasurementTime()
        wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        stt1 = str(wt1)
        lgstr= '^^readTimeOut b4 measurement '+str(readTimeOut)+' '+stt1
        log(lgstr )
        reflectance=getOneSpectra()
        wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        stt1 = str(wt1)
        lgstr= '^^after measurement '+' '+stt1
        log(lgstr )
        pos_y=0
        info = {'Absorbance': [],'Temperature': temperature,'Reference': flgs.calibration_results['Reference'],'Reflectance': reflectance,'CalibrationMaterial': flgs.calibrationmaterial['Reference'], 'PosY':pos_y, 'Command':flgs.lastcommand}
        print(info)
        SendHostMessage('%s\n' %str(info))
        print(info)
        print('i=%d'%i)
    Home() # Переместить всю систему домой
    fPlasticZ=float(flgs.config["spectrometerCenterAttributes"]['CenterZ'])*(mm_to_step_Z/1000.0)
    PlasticZ=int(fPlasticZ)
    print(PlasticZ)
    Move_z(PlasticZ)

# перезагрузить конфигурацию из файла
def rereadConfValuesFromFile():
    log('rereadConfValuesFromFile() ')
    # configvalues=getconfigvalues()
    readdistancesConfValuesFromFile('all')
    print('rereadConfValuesFromFile flgs.safe_y_distance %d' %flgs.safe_y_distance)
    print('rereadConfValuesFromFile flgs.safe_z_distance %d' %flgs.safe_z_distance)
    print('rereadConfValuesFromFile flgs.MAX_RANGE_Z %d' %flgs.MAX_RANGE_Z)
    print('rereadConfValuesFromFile flgs.BUD_FRONT_Y %d' %flgs.BUD_FRONT_Y)
    print('rereadConfValuesFromFile flgs.pixel_to_mm %f' %flgs.pixel_to_mm)
    print('rereadConfValuesFromFile flgs.spectrometer_edge_in_image %f' %flgs.spectrometer_edge_in_image)
    print('rereadConfValuesFromFile flgs.spectrometer_center_in_image %f' %flgs.spectrometer_center_in_image)
    print('rereadConfValuesFromFile flgs.image_crop_right %f' %flgs.image_crop_right)

# получить версию sd
def getSdVersion():
    sdversion_file='/home/pi/sdversion'
    sdversion= '0'    
    try:
        if os.path.isfile(sdversion_file):
            sdversionfile=open(sdversion_file, "r+")
            sdversion=sdversionfile.read()   #+" "+fexceptionlog.readline(1)
            print(sdversion)
            bitarr.bit_arr['SdVersion'] = sdversion
            sdversionfile.close()
        else:
            bitarr.bit_arr['SdVersion'] = '0'
            sdversion= '0'
            createSdVersion()
    except Exception as e:
        logexception(e)
        bitarr.bit_arr['SdVersion'] = '0'
        sdversion= '0'
        createSdVersion()
    return sdversion

# создать версию sd
def createSdVersion():
    flog=open("/home/pi/sdversion", "w")
    flog.truncate(0)
    s2log = '0'
    flog.writelines(s2log +'\n')
    flog.close()

#    загрузить конфигурацию
def uploadconfigfile():
    json_file='/home/pi/jsons/gc200conf.json'
    json_data1=open(json_file)
    configvalues = json.load(json_data1)
    print(configvalues)
    json_data1.close()
    data_to_GCA = {'configfile': str(configvalues), 'Command':flgs.lastcommand}
    SendHostMessage('%s\n' %str(data_to_GCA))
    whistleDog(firstconnectionTimout)
    # tt=datetime.now().strftime('%H-%M-%S-%f')
    return     configvalues

#   установить конфигурацию значений
def getconfigvalues():
    json_file='/home/pi/jsons/gc200conf.json'
    try:
        json_data1=open(json_file)
        configvalues = json.load(json_data1)
        print(configvalues)
        json_data1.close()
        return     configvalues
    except Exception as e:
        logexception(e)
        os.system("sudo mv /home/pi/jsons/oldgc200conf.json /home/pi/jsons/gc200conf.json")
        sexeption='FAILED LOADING CONFIG FILE!' # НЕ УДАЛОСЬ ЗАГРУЗИТЬ КОНФИГУРАЦИОННЫЙ ФАЙЛ!
        check_if_exception_in_file(sexeption)
        log('FAILED LOADING CONFIG FILE!')
        sleep(1)
        if not os.path.exists('/home/pi/jsons/oldgc200conf.json'):
            log('FAILED REVERT TO OLD CONFIG FILE!') # НЕ УДАЛОСЬ ВЕРНУТЬСЯ К СТАРОМУ КОНФИГУРАЦИОННОМУ ФАЙЛУ!
        try:
            json_data1=open(json_file)
            configvalues = json.load(json_data1)
            print(configvalues)
            json_data1.close()
            return     configvalues
        except Exception as e:
            logexception(e)
 #  Запись исключения в журнал исключений log fixed
 #TODO(Ido): Write exception to the exception log fixed
            # write_to_exceptionlog('FAILED LOADING OLD CONFIG FILE!')
            sexeption='FAILED LOADING OLD CONFIG FILE!'
            check_if_exception_in_file(sexeption)
            log('FAILED LOADING OLD CONFIG FILE!')
            return     None

# считать значение расстояний конфигураций из файла
def readdistancesConfValuesFromFile(param):
    log('readdistancesConfValuesFromFile() ')
    configvalues=None
    configvalues= getconfigvalues()
    log('readdistancesConfValuesFromFile() ')
    confVersion=0
    flgs.y_whiteExists_distance = 700
    if param=='all':
        try:
            flgs.deviceTipe = int(configvalues['gc200Attributes']['device_type']) 
        except:
            flgs.deviceTipe = 0
        # Проверить, откуда взялся этот параметр
        try:
            bitarr.bit_arr['ConfigurationVersion'] = confVersion #TODO(Ido): Check where is this param from
        except Exception as e:
            logexception(e)

            bitarr.bit_arr['ConfigurationVersion'] = 0
            print('bitarr.bit_arr ConfVersion = 0')
        try:
            # край спектрометра на изображении белый
            flgs.spectrometer_edge_in_image_white = int(configvalues['gc200Attributes']['spectrometer_edge_in_image_white'])
            # центр спектрометра на изображении
            flgs.spectrometer_center_in_image = int(configvalues['gc200Attributes']['spectrometer_center_in_image']) #
            # обрезка изображения справа
            flgs.image_crop_right = int(configvalues['gc200Attributes']['image_crop_right']) #
            # край спектрометра на изображении
            flgs.spectrometer_edge_in_image = int(configvalues['gc200Attributes']['spectrometer_edge_in_image']) #
            # край полосы на стороне спектрометра изображения
            flgs.strip_edge_in_image_spectrometer_side = int(configvalues['gc200Attributes']['strip_edge_in_image_spectrometer_side']) #
            # край полосы в отражателе изображения
            flgs.strip_edge_in_image_reflector = int(configvalues['gc200Attributes']['strip_edge_in_image_reflector_side']) #
            # белый Существует размер обрезки
            flgs.whiteExists_crop_size = int(configvalues['gc200Attributes']['whiteExists_crop_size'])
            # расстояние по y существующего белого
            flgs.y_whiteExists_distance = int(configvalues['gc200Attributes']['y_whiteExists_distance'])
            # шаг привода по оси х от микровыключателя
            flgs.fix_steps_x= int(configvalues['gc200Attributes']['fix_steps_x'])              #steps of x motor fix microswitch
            # шаг привода по оси y от микровыключателя
            flgs.fix_steps_y= int(configvalues['gc200Attributes']['fix_steps_y'])               #steps of y motor fix microswitch
            # шаг привода по оси z от микровыключателя
            flgs.fix_steps_z= int(configvalues['gc200Attributes']['fix_steps_z'])               #steps of z motor fix microswitch

        except Exception as e: # НЕ УДАЛОСЬ ПРОЧИТАТЬ КОНФИГУРАЦИОННЫЙ ФАЙЛ!
            logexception(e)
            flgs.fix_steps_x= FIX_STEPS_X_DEFAULT
            flgs.fix_steps_y=FIX_STEPS_Y_DEFAULT
            flgs.fix_steps_z= FIX_STEPS_Z_DEFAULT
            bitarr.bit_arr['ConfigurationVersion'] = -1
            # write_to_exceptionlog('FAILED READING  CONFIG FILE!')
            sexeption='FAILED READING CONFIG FILE!'
            check_if_exception_in_file(sexeption)

        try:
            # distance measurement start
            # начало измерения расстояния
            flgs.safe_y_distance = int(configvalues['gc200Attributes']['safe_y_distance']) # безопасное расстояние y
            flgs.safe_z_distance = int(configvalues['gc200Attributes']['safe_z_distance']) # безопасное расстояние z

            flgs.MAX_RANGE_Y = int(configvalues['gc200Attributes']['MAX_RANGE_Y']) # МАКСИМАЛЬНЫЙ ДИАПАЗОН Y
            flgs.MAX_RANGE_Z = int(configvalues['gc200Attributes']['MAX_RANGE_Z']) # МАКСИМАЛЬНЫЙ ДИАПАЗОН Z
            flgs.MAX_RANGE_X = int(configvalues['gc200Attributes']['MAX_RANGE_X']) # МАКСИМАЛЬНЫЙ ДИАПАЗОН X
            flgs.bud_center_x = int(configvalues['gc200Attributes']['bud_center_x']) # центр бутона x

            flgs.bud_center_z = int(configvalues['gc200Attributes']['bud_center_z'])  # центр бутона z
            log('####in readdistancesConfValuesFromFile flgs.bud_center_z='+str(flgs.bud_center_z))
            flgs.pixel_to_mm = float(configvalues['gc200Attributes']['pixel_to_mm']) # пиксель в мм
            flgs.BUD_FRONT_Y = int(configvalues['gc200Attributes']['BUD_FRONT_Y']) # фронт бутона y

            flgs.TemperatureUpperLimit= 57.0
            flgs.TemperatureUpperLimit= float(configvalues['gc200Attributes']['MAX_TEMPERATURE']) # верхний температурный предел
            flgs.TemperatureLowerLimit= 47.0
            flgs.TemperatureLowerLimit= float(configvalues['gc200Attributes']['MIN_TEMPERATURE']) # нижний температурный предел
            flgs.extractYoffset =   int(configvalues['gc200Attributes']['extract_offset_y'])  # извлечь смещение Y
            log('readdistancesConfValuesFromFile flgs.extractYoffset ='+ str(flgs.extractYoffset))
            print('readdistancesConfValuesFromFile flgs.extractYoffset ='+ str(flgs.extractYoffset))
        except Exception as e:
            logexception(e)
            # write_to_exceptionlog('FAILED READING OLD CONFIG FILE!')
            sexeption='FAILED READING OLD CONFIG FILE!'
            check_if_exception_in_file(sexeption)
            log('FAILED READING OLD CONFIG FILE!')

def getVersion():
    SendHostMessage('%s\n' %str(swversion))

#----------------DEBUG AND TEST-------------
#  печать текущего времени
def printTime():
    tt=datetime.now().strftime('%H-%M-%S-%f') #.time()
    stt = str(tt)
    print ('*********TIME SEND BY APP %s*********',stt)

# какой-то тест выключателя
def M_SwitchTest():
    log('M_SwitchTest() ')
    xswitch= GPIO.input(SWITH_X)
    log('M_SwitchTest()Switch X: %d ' %xswitch)
    print('Switch X: %d ' %xswitch)
    yswitch= GPIO.input(SWITH_Y)
    log('M_SwitchTest()Switch Y: %d ' %yswitch)
    print('Switch Y: %d ' %yswitch)
    zswitch= GPIO.input(SWITH_Z)
    log('M_SwitchTest()Switch Z: %d ' %zswitch)
    print('Switch Z: %d ' %zswitch)
    cover = GPIO.input(COVER)
    print('Drawer : %d ' %cover)

# функции локального перемещения
def Move_zLoop(times):
    for x in range(times):
        whistleDog(firstconnectionTimout)
        Move_z(10000)
        sleep(1)
        Move_z(-10100)

def mvx(npos):
    if npos > 0:
        GPIO.output(DIR_X, FORWARD)
        for x in range(npos-pos.x):
            GPIO.output(STEP_X, GPIO.HIGH)
            sleep(delay_x)
            GPIO.output(STEP_X, GPIO.LOW)
            sleep(delay_x)
    elif npos <0:
        GPIO.output(DIR_X, REVERS)
        xpos= npos*-1
        for x in range(xpos):
            GPIO.output(STEP_X, GPIO.HIGH)
            sleep(delay_x)
            GPIO.output(STEP_X, GPIO.LOW)
            sleep(delay_x)
    pos.x = npos
    return pos.x

def mvz(npos):
    if npos > 0:
        GPIO.output(DIR_Z, FORWARD)
        for x in range(npos-pos.z):
            GPIO.output(STEP_Z, GPIO.HIGH)
            sleep(delay_z)
            GPIO.output(STEP_Z, GPIO.LOW)
            sleep(delay_z)
    elif npos <0:
        GPIO.output(DIR_Z, REVERS)
        xpos= npos*-1
        for x in range(xpos):
            GPIO.output(STEP_Z, GPIO.HIGH)
            sleep(delay_z)
            GPIO.output(STEP_Z, GPIO.LOW)
            sleep(delay_z)
    pos.z = npos
    return pos.z

def mvy(npos):

    if npos > 0:

        GPIO.output(DIR_Y,REVERS )

        for x in range(npos):

            GPIO.output(STEP_Y, GPIO.HIGH)

            sleep(delay_y)

            GPIO.output(STEP_Y, GPIO.LOW)

            sleep(delay_y)
    elif npos <0:

        GPIO.output(DIR_Y, FORWARD)

        xpos= npos*-1

        for x in range(xpos):

            GPIO.output(STEP_Y, GPIO.HIGH)

            sleep(delay_y)

            GPIO.output(STEP_Y, GPIO.LOW)

            sleep(delay_y)

    pos.y = npos

    return pos.y

# Получить изображение с камеры и записать 3 фильтра зеленый, красный, голубой в соответствующие файлы
def GetCameraImageSaveColor():    #Get image from camera, return RGB array
    img = GetCameraImage()
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (36, 0, 0), (70, 255,255))
    imask = mask>0
    green = numpy.zeros_like(img, numpy.uint8)
    green[imask] = img[imask]
    cv2.imwrite("green.png", green)
    mask = cv2.inRange(hsv, (0,50,50), (10,255,255))
    imask = mask>0
    red = numpy.zeros_like(img, numpy.uint8)
    red[imask] = img[imask]
    zerocount=cv2.countNonZero(red.shape)
    print(red.shape)
    print('zerocount %d' %zerocount)
    if cv2.countNonZero(red.shape) == 0:
        print('Image is black')
    else:
        print('Colored image')
    cv2.imwrite("red.png", red)
    mask = cv2.inRange(hsv, (110,50,50), (130,255,255))
    imask = mask>0
    blue = numpy.zeros_like(img, numpy.uint8)
    blue[imask] = img[imask]
    cv2.imwrite("blue.png", blue)

# Получить изображение с камеры, вернуть RGB-массив и записываем в файл foo.bmp
def GetCameraImageSave(cam_bit = 0,led_control = 1):    #Get image from camera, return RGB array
    # включить камеру
    GPIO.output(CAM_LIGHT,GPIO.LOW) #turn on camera light
    sleep(0.2)
    print('1')
    flgs.camera.start_preview()
    print('2')
    sleep(0.5)
    flgs.camera.capture('foo.bmp',format ='bmp')
    print('after pict save')
    flgs.camera.stop_preview()
    print('3')
    if led_control:
        # выключить камеру
        GPIO.output(CAM_LIGHT,GPIO.HIGH)    # turn off camera light
    if cam_bit: # ?
            return True
    return True

# Получить минимальное расстояние от немного светящегося, спектрометра
def get_min_distanceBitlightOn(): #Get minimum distance from spectrometer  Black
    ddebugbl=True
    SpectTurnOffLamp()
    Move_z(flgs.safe_z_distance)
    GPIO.output(CAM_LIGHT,GPIO.LOW) #turn on camera light
    distances_array = [-1] #,-1,-1,-1,-1]
    sleep(0.5)
    min_distance = 0
    firstpixel=-1
    tt=datetime.now().time()
    stt = str(tt)
    print ('b4 GetCameraImage() START %s',stt)
    frame = picamera.array.PiRGBArray(flgs.camera)
    flgs.camera.capture(frame,'bgr')
    img = frame.array
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTP GetCameraImage() START %s',stt)
    fn = '%s.bmp'%str(pos.z)
    fnj = '%s.jpg'%str(pos.z)
    im_gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    thresh_img = cv2.threshold(im_gray, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    cropped_img = thresh_img[155:165,0:320]
    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        fnjpgb = 'distthresh_img%s.jpg'%str(pos.z)
        cv2.imwrite(fnjpgb, thresh_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        log(fnjpgb)
    imagem = cv2.bitwise_not(cropped_img)
    xxxx=[10000 -10000]
    try:
        xxxx=find_nearest_white(cropped_img,255)
        numpy.set_printoptions(formatter={'float': '{: 0.3f}'.format})
        firstpixel= int(xxxx[1])
        print(xxxx)
    except Exception as e:
        logexception(e)
        firstpixel=-1 #whitePoint not found
    print(xxxx[1])
    print('firstpixel %d' %firstpixel)
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTR CROP START %s',stt)
    min_distance =(firstpixel)*flgs.pixel_to_mm  # compensate for crop 130 instead of 120
    print('Bit  distance: %f mm' %(min_distance))
    return min_distance

# Получить минимальное расстояние от черного цвета спектрометра
def get_min_distanceBitDark(): #Get minimum distance from spectrometer  Black
    ddebugbl=True
    SpectTurnOffLamp()
    Move_z(flgs.safe_z_distance)
    # включить камеру
    GPIO.output(CAM_LIGHT,GPIO.HIGH) #turn on camera light
    distances_array = [-1] #,-1,-1,-1,-1]
    sleep(0.5)
    min_distance = 0
    firstpixel=-1
    tt=datetime.now().time()
    stt = str(tt)
    print ('b4 GetCameraImage() START %s',stt)
    frame = picamera.array.PiRGBArray(flgs.camera)
    flgs.camera.capture(frame,'bgr')
    img = frame.array
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTP GetCameraImage() START %s',stt)
    fn = '%s.bmp'%str(pos.z)
    fnj = '%s.jpg'%str(pos.z)
    im_gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    thresh_img = cv2.threshold(im_gray, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    cropped_img = thresh_img[155:165,0:320]
    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        fnjpgb = 'distthresh_img%s.jpg'%str(pos.z)
        cv2.imwrite(fnjpgb, thresh_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        log(fnjpgb)
    imagem = cv2.bitwise_not(cropped_img)
    xxxx=[10000 -10000]
    try:
        xxxx=find_nearest_white(cropped_img,255)
        numpy.set_printoptions(formatter={'float': '{: 0.3f}'.format})
        print(xxxx)
        firstpixel= int(xxxx[1])
    except Exception as e:
        logexception(e)
        firstpixel=-1 # whitePoint not found --- белая точка не найдена
    print('firstpixel %d' %firstpixel)
    tt=datetime.now().time()
    stt = str(tt)
    print ('FTR CROP START %s',stt)
    # компенсировать обрезать 130 вместо 120
    min_distance =(firstpixel)*flgs.pixel_to_mm  # compensate for crop 130 instead of 120
    print('Bit  distance: %f mm' %(min_distance))
    SpectTurnOnLamp()
    return min_distance

# Получить ТЕМНОТУ-изображение с камеры, при выключенной лампе вернуть RGB-массив и записывать в файл dark.bmp
def GetCameraImageSaveDark(cam_bit = 0,led_control = 1):    #Get image from camera, return RGB array
    # выключить свет камеры
    GPIO.output(CAM_LIGHT,GPIO.HIGH) #turn off camera light
    sleep(0.2)
    print('1')
    flgs.camera.start_preview()
    print('2')
    sleep(0.5)
    flgs.camera.capture('dark.bmp',format ='bmp')
    print('after pict save')
    flgs.camera.stop_preview()
    print('3')
    if led_control:
        # выключить свет камеры
        GPIO.output(CAM_LIGHT,GPIO.HIGH)    # turn off camera light
    if cam_bit: # бессмысленное действие какое-то
            return True
    return True

# Получить изображение с камеры, вернуть RGB-массив и записываем в файл foo.bmp
def GetCameraImageSaveLO(cam_bit = 0,led_control = 1):    #Get image from camera, return RGB array
    SpectTurnOffLamp()
    if led_control:
        # включить свет камеры
        GPIO.output(CAM_LIGHT,GPIO.LOW) #turn on camera light
        sleep(0.8)
    print('1')
    flgs.camera.start_preview()
    print('2')
    sleep(0.8)
    flgs.camera.capture('foo.bmp',format ='bmp')
    print('after pict save')
    flgs.camera.stop_preview()
    print('3')
    if led_control:
        # выключить свет камеры
        GPIO.output(CAM_LIGHT,GPIO.HIGH)    # turn off camera light
    if cam_bit: # бессмысленное действие какое-то
            return True
    return True

# Получить изображение с камеры, вернуть RGB-массив
def GetCameraImageSaveLOFN(filename = ''):    #Get image from camera, return RGB array
    print(filename)
    # включить свет камеры
    GPIO.output(CAM_LIGHT,GPIO.LOW) #turn on camera light
    sleep(0.1)
    print('1')
    flgs.camera.start_preview()
    print('2')
    sleep(0.1)
    flgs.camera.capture(filename,format ='bmp')
    print('after pict save')
    flgs.camera.stop_preview()
    return True

# отрегулировать температуру
def adjustTemperature(justTemperature=False):
    log('adjustTemperature() ')
    ready=False
    whistleDog(calibrateTimout)
    temperatureOfLamp=getTemperature()
    print('in adjustTemperature %s' %temperatureOfLamp)
    ftemperature=float(temperatureOfLamp)
    if (flgs.inprocessFlag==True or justTemperature==True ):  
        if (ftemperature>=flgs.TemperatureLowerLimit and ftemperature<=flgs.TemperatureUpperLimit and flgs.coolingFlag==False):
             log('in adjustTemperature %f' %ftemperature)
             ready=True 
        return ftemperature,ready
    print('in adjustTemperature %f' %ftemperature)
    log('in adjustTemperature %f' %ftemperature)
    if (ftemperature>=float(flgs.TemperatureLowerLimit ) and pos.z==flgs.fix_steps_z):
        Home_y()
        log('adjustTemperature()Temperature Reaching MIN_TEMPERATURE')
        PlasticZ=calcPlasticZ()  
        Move_z(PlasticZ)  
        ready=True
    if (ftemperature<float(flgs.TemperatureLowerLimit ) ):  
        if(pos.z!=flgs.fix_steps_z):
            Home() # Переместить всю систему домой
        log('adjustTemperature()Temperature below MIN_TEMPERATURE flgs.canHeatForward ='+str(flgs.canHeatForward))
        SpectTurnOnLamp()
        if (GPIO.input(COVER)):  #drawer closed?
            closed1 = True
            Move_x(flgs.bud_center_x) # heats faster when in center
            if flgs.canHeatForward == True : 
                Move_yForCalibration(Y_HEAT_DISTANCE) 
            else: 
                Home_y() 
        else: 
            ready=False 
    if (ftemperature>=float(flgs.TemperatureUpperLimit) ):  #TBD  57.0
        SpectTurnOffLamp()
        log('adjustTemperature()cooling')
        flgs.coolingFlag=True
        try:
            datetimenow=datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            calculated=calculate_LampElapsedTime(datetimenow,STATEFORLAMPTIME_SENDBIT)
            print(calculated)
            logLampElapsedTime(calculated)
        except Exception as e:
            logexception(e)
            log('adjustTemperature()flgs.lampOn=0')
        ready=False 
    log( 'flgs.coolingFlag=' +str(flgs.coolingFlag))
    if (ftemperature<=(flgs.CoolDownHysteresis) and flgs.coolingFlag==True):  #TBD
        SpectTurnOnLamp()
        datetimenow=datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        calculated=calculate_LampElapsedTime(datetimenow,STATEFORLAMPTIME_LAMPON)
        print(calculated)
        flgs.coolingFlag=False
        ready=True  
        log('adjustTemperature()Temperature greater than  CoolDownHysteresis')
    temperatureOfLamp=getTemperature()
    print('After adjustTemperature() %s' %temperatureOfLamp)
    log( 'After adjustTemperature() %s' %temperatureOfLamp)
    return ftemperature,ready

# записывает информацию в файл, который в свою очередь мониторит GcWatchDog
def whistleDog(timeout):
    str2watchdog='1;'+os.path.dirname(full_path)+';'+swversion+';'+timeout+"\n"
    if flgs.watchdog== True:
        wt=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        stt = str(wt)
        timeouttimeout = "%s %s"%(timeout,stt)
        print('##### whistleDog %s',timeouttimeout)
        # Функция os.write() запишет байтовую строку аргумент str2watchdog в файловый дескриптор flgs.pipeout.
        os.write(flgs.pipeout, str2watchdog)
    else:
        print('flgs.watchdog== False')

# переместить в центр WSC
def moveToCenterWCS():
    log('moveToCenterWCS() ')
    fWcsZ=float(flgs.config["spectrometerCenterAttributes"]['CenterZ'])*(mm_to_step_Z/1000.0)
    wcsZ=int(fWcsZ)
    Move_z(wcsZ)
    fWcsX=float(flgs.config["spectrometerCenterAttributes"]['CenterX'])*(mm_to_step_Z/1000.0)
    wcsX=int(fWcsX)
    Move_x(wcsX)
    fWcsY=float(flgs.config["spectrometerCenterAttributes"]['CenterY'])*(mm_to_step_Y/1000.0)
    wcsY=int(fWcsY)
    yToMve=    wcsY
    print('yToMve : %f  ' %yToMve)
    Move_y(int(yToMve))

# Получить спектрометрические актуальные параметры
def getSpectrometerActualParams():
    log('getSpectrometerActualParams() ')
    spectrometer.write('Li'+'\r')
    response_output = spectrometer.readline()
    print(response_output)
    spectrometer.write('Lm'+'\r')
    response_output = spectrometer.readline()
    print(response_output)
    spectrometer.write('Lt'+'\r')
    response_output = spectrometer.readline()
    print(response_output)
    spectrometer.write('Pa'+'\r')
    response_output = spectrometer.readline()
    print(response_output)
    spectrometer.write('Pb'+'\r')
    response_output = spectrometer.readline()
    print(response_output)
    spectrometer.write('Pc'+'\r')
    response_output = spectrometer.readline()

# перейти к материалу
def movetometerial():
    log(' movetometerial()')
    fPlasticZ=float(flgs.config["plasticCenterAttributes"]['CenterZ'])*(mm_to_step_Z/1000.0)
    PlasticZ=int(fPlasticZ)
    print(PlasticZ)
    Move_z(PlasticZ)
    fWcsX=float(flgs.config["plasticCenterAttributes"]['CenterX'])*(mm_to_step_X/1000.0)
    wcsX=int(fWcsX)
    Move_x(wcsX)
    fPlasticY=float(flgs.config["plasticCenterAttributes"]['CenterY'])*(mm_to_step_Y/1000.0)
    PlasticY=int(fPlasticY)
    Move_y(PlasticY)

# обрезать файлы логов и исключений, оставить только свежие заданной количесвто строк
def shorten_log():  # and exception logs
    # print ("shorten_log() ps -aux |grep python | grep -v grep")
    # print os.system("ps -aux |grep python | grep -v grep") #TODO(Ido): Consider moving to a better place
    try:
        os.system("sudo tail -600000 /home/pi/LogsGc200/logGc200.txt > /home/pi/LogsGc200/newlogGc200.txt")
        sleep(1)
        os.system("sudo mv /home/pi/LogsGc200/newlogGc200.txt  /home/pi/LogsGc200/logGc200.txt")
        sleep(1)
        os.system("sudo tail -60000 /home/pi/GcWatchdog/horror_stories > /home/pi/GcWatchdog/newhorror_stories")
        sleep(1)
        os.system("sudo mv /home/pi/GcWatchdog/newhorror_stories  /home/pi/GcWatchdog/horror_stories")
        print('shorten_logs done')
        exceptions_size=-1
    except:
        log('shorten_log() FAILED')
    try:
        exceptions_size=os.path.getsize("/home/pi/LogsGc200/exeptions.txt")
    except:
        print ('no exception file found')
        log ('no exception file found')
    log('exceptions_size='+str(exceptions_size))
    if  exceptions_size>6000:
        try:
            os.system("sudo tail -6000 /home/pi/LogsGc200/exeptions.txt > /home/pi/LogsGc200/newexeptions.txt")
            sleep(1)
            os.system("sudo mv /home/pi/LogsGc200/newexeptions.txt  /home/pi/LogsGc200/exeptions.txt")
            sleep(1)
        except:
            print ('no exception file found')
    try:
        os.system("sudo tail -6000 /home/pi/LogsGc200/exceptionsFull.txt > /home/pi/LogsGc200/newexeptions.txt")
        sleep(1)
        os.system("sudo mv /home/pi/LogsGc200/newexeptions.txt  /home/pi/LogsGc200/exceptionsFull.txt")
        print('shorten_exceptionsFull done')
    except:
        print ('no exceptionFull file found')

# установить время
def set_date_time():
    print ("set_date_time()")
    try:
        datestr=flgs.config["currentTime"]
        sdate="sudo date -s '%s'"%datestr
        print (sdate)
        log ("set_date_time()"+str(sdate))
        os.system(sdate)
    except Exception as e:
        logexception(e)
        print ('set_date_time FAILED')
        log('set_date_time FAILED')

# извлечь все
def extractall(path='' , filename=''):
    zip_ref = zipfile.ZipFile(path + '/' + filename, 'r')
    zip_ref.extractall(path)
    zip_ref.close()

# Включить интерфейс Bluetooth и сканировать устройства
def bt_Repairing(): #Turn on bluetooth interface and scan devices
    log(' bt_Repairing()')
    call(['sudo','hciconfig','hci0','down'])
    sleep(1.5)
    call(['sudo','hciconfig','hci0','up'])
    call(['sudo','hciconfig','hci0','piscan'])

# проверить находиться ли устройство внизу
def checkDrawerIsDown():
    cover = GPIO.input(COVER)
    print('Drawer : %d ' %cover)
    log( 'ca   Drawer : %d ' %cover)
    return cover

#  расчет пластика
def calcPlasticZ():
    PlasticZ=0
    try:
        fPlasticZ=float(flgs.config["spectrometerCenterAttributes"]['PlasticZ'])*(mm_to_step_Z/1000.0)
        PlasticZ=int(fPlasticZ)
    except:
        PlasticZ=DEFAULT_PLASTIC_Z
    print(PlasticZ)
    return PlasticZ

# cut off unnecessary repetitions --- new code insertion ---
def clipper(iteration_counter, iteration_limit=20):
    allow_iteration = True if iteration_counter % iteration_limit else False
    iteration_counter = iteration_counter % iteration_limit + 1
    return allow_iteration, iteration_counter

# основная процедура ожидания событий
def loop(): # TODO base method for waiting for events
    full_path = os.path.realpath(__file__)
    path, filename = os.path.split(full_path)
    counter = 0
   #  бесконечный цикл
    while True:
        bt_input = ''
        try:
            bt_input = ReceiveHostMessage(2) # Пробуем олучать строку от bluetooth
        except Exception as e:
            logexception(e) # если нет то логируем исключение сообщение ниже
            log('exception ^after ReceiveHostMessage')
        flgs.lastcommand=bt_input
        IsPowerButtonPressed() # проверить, Нажата Ли Кнопка Питания

        if bt_input=='null': # если пустое
            IsOpen() # проверить, открыт ли контейнер
            IsPowerButtonPressed() # проверить, Нажата Ли Кнопка Питания
            str2watchdog='1;'+os.path.dirname(full_path)+';'+swversion+';'+connectionTimout+"\n"
            # если сопряжение с BlueTooth есть
            if flgs.BlueToothConnected == True: # такой выражение бред, потом надо все упростить !
                if flgs.watchdog == True: # если сторожевик True, запишем в файл сигнальную строку
                    os.write(flgs.pipeout, str2watchdog)
                wt=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
                wt1=datetime.now()
                btconn.btcontime_2=btconn.btcontime_1
                btconn.btcontime_1=wt1
                diffff=btconn.btcontime_1-btconn.btcontime_2
                print(str(diffff))
                fdiff=diffff.total_seconds()
                print(str(fdiff))
                fcondisconnect=FCONDISCONNECT # Make this a constant
                if fdiff< fcondisconnect:
                    flgs.dissconnectioncounter =flgs.dissconnectioncounter +1
                else:
                    flgs.dissconnectioncounter = 0
                if flgs.dissconnectioncounter> BLUETOOTH_DISCONNECT_CYCLE_COUNTER:
                    str2watchdog='1;'+os.path.dirname(full_path)+';'+swversion+';'+connectionTimout+"\n"
                    if flgs.watchdog== True:
                        os.write(flgs.pipeout, str2watchdog)
                    flgs.BlueToothConnected = False
                    blu_led.start(100)
                    white_led.start(100)
                    blu_led.ChangeDutyCycle(50)
                    bt_Repairing()
                    btconnect(False)

            counter = counter +1
            print(counter)
            if counter >COUNT_CYCLES_FOR_ADJUST_TEMPERATURE: # Make this a constant
                # flgs.lampElapsedTime=time.time()-flgs.lampStartTime # Проверить, откуда это должно взяться
                counter = 0
                adjustTemperature()  # отрегулировать температуру
        else:
            print(bt_input)
            adjustTemperature()  # отрегулировать температуру
            wt1=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            stt1 = str(wt1)
            lgstr= bt_input+' '+stt1
            log(lgstr )
            flgs.dissconnectioncounter = 0
        try:
            # if bt_input != 'hk':
    
            #     # if bt_input=='null': #TODO(Ido): Consider removing
            #     #     zz=0
    
            if bt_input == 'rb': # команда перезагрузки устройство
                reboot_device()

            elif bt_input == 'bn':
                ActivateBITNew()
    
            elif bt_input == 'id': # COMMAND_ID инициализация ID устройства
                if bitarr.bit_arr['Spectrometer'] ==0:
                    GetSpectDevInfo()
                else:
                    GetSpectDevInfo()
                print('AFTERGetSpectDevInfo()')
                str2watchdog = '3;' + "\n"
                if flgs.watchdog== True: # если сторожевик True, запишем в файл сигнальную строку
                    os.write(flgs.pipeout, str2watchdog)

                versionProved()
                if SAVE_RCLOCAL_FLAG==1:
                    os.system('sudo cat /etc/rc.local | grep GcWatchdog/GcWatchdog > /home/pi/verrclocal.txt')
                    f = open('/home/pi/verrclocal.txt','r')
                    for line in f:
                        log (line)
                    f.close()
    
            elif bt_input == 'bt': # COMMAND_BT
                #bit = str(bitarr.bit_arr)
                #sleep(2)
                print('flgs.bitPerformed=%d' %flgs.bitPerformed)
                log ('flgs.bitPerformed=%d' %flgs.bitPerformed)
                if SAVE_RCLOCAL_FLAG==1:
                    os.system('sudo cat /etc/rc.local | grep GcWatchdog/GcWatchdog > /home/pi/tstrclocal.txt')
                    f = open('/home/pi/tstrclocal.txt','r')
                    for line in f:
                        log (line)
                    f.close()
                if flgs.bitPerformed==0:
                    print('bit not performed bitting now') # BIT не выполнен
                    log( 'bit not performed bitting now')
                    bit = ActivateBITNew()
                    # flgs.bitPerformed=1
                    print('bit after bit new version') # бит за битом новая версия
                    log( 'bit after bit new version')
                    if flgs.bitPerformed==1: # нагревается быстрее при запуске устройства
                        Move_yForCalibration(Y_HEAT_DISTANCE)  #heating faster upon starting the device
                        flgs.heatingDownPosition=1 
                    else:
                        try:
                            whistleDog(functionStartTimout)
                        except Exception as e:
                            logexception(e)
                            log( 'whistleDog failed')
                print('b4 SendHostMessage(')
                datetimenow=datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                calculated=calculate_LampElapsedTime(datetimenow,STATEFORLAMPTIME_SENDBIT)
                print('calculated =%d' %calculated)
                if calculated <0:
                    calculated=0
                log( 'calculated =%d' %calculated)
                bitarr.bit_arr['LampUsage'] =calculated
                data_str = json.dumps(bitarr.bit_arr)
                #bit = str(bitarr.bit_arr)
                SendHostMessage('%s\n' %str(data_str))
                fexceptions=    "/home/pi/LogsGc200/exceptions.txt"
                try:
                    os.remove(fexceptions)
                except Exception as e:
                    logexception(e)
                    log('no exceptions')
                logLampElapsedTime(0)
                log('%s\n' %str(data_str))
                print(str(data_str))
                print('after SendHostMessage()')
                log( '###after SendHostMessage bit###')
                sleep(0.2)
                if SAVE_RCLOCAL_FLAG==1:
                    os.system('sudo cat /etc/rc.local | grep GcWatchdog/GcWatchdog > /home/pi/tstrclocal.txt')
                    f = open('/home/pi/tstrclocal.txt','r')
                    for line in f:
                        log (line)
                    f.close()
 
            elif bt_input == 'fs': # COMMAND_FREE_SPACE
                whistleDog(calibrateTimout) # проверить на залипание
                adjustTemperature() # отрегулировать температуру
                bcover=True
                cover=checkDrawerIsDown() # cover = GPIO.input(COVER) --- проверить открыта ли крышка ---
                if cover==1: #TBD BCOVER CHANGE TO FALSE
                    bcover=False
                    calib_results = {'DrawerOpen': bcover,'Reference':[],'Dark':[],  'Command': flgs.lastcommand}
                    print(str(calib_results))
                    SendHostMessage('%s\n' %str(calib_results))
                else:
                    flgs.inprocessFlag = True # флаг состояния "в процессе"
                    NumOfPoints=flgs.config["spectrometerAttributes"]['NumOfPoints'] # установили атрибут
                    byAttributesSpectConf('dark') #  Выбрать конфигурацию по атрибут 'dark'
                    getSpectrometerActualParams() # Получить спектрометрические актуальные параметры
                    test=1
                    Home() # Переместить всю систему домой
                    byAttributesSpectConf('dark') #  Выбрать конфигурацию по атрибут 'dark'
                    getSpectrometerActualParams() # Получить спектрометрические актуальные параметры
                    SpectTurnOnLamp() # включить лампу
                    cover=checkDrawerIsDown()# cover = GPIO.input(COVER)
                    # проверить еще раз непосредственно перед тем, как взять спектры FS
                    # check again just before taking the FS spectra
                    if cover==1:
                        bcover=False
                        calib_results = {'DrawerOpen': bcover,'Reference':[],'Dark':[],  'Command': flgs.lastcommand}
                        print(str(calib_results))
                        SendHostMessage('%s\n' %str(calib_results)) # отправить сообщение результат калибровки по bt
                    else:
                        reflectance=getOneSpectraDark() # получить темный спектр
                        pos_y=pos.y/mm_to_step_Y
                        log('pos_y=%f'%pos_y)
                        print('pos_y=%f'%pos_y)
                        temperature = getTemperature() # получить температуру спектрометра
                        info = {'DrawerOpen': bcover,'Absorbance':[],'Temperature': temperature,'Reference': [],'Reflectance': reflectance,'CalibrationMaterial': [], 'PosY':pos_y, 'Command':flgs.lastcommand}
                        print(info)
                        SendHostMessage('%s\n' %str(info)) # отправить сообщение с текущей информацией по bt
                        print(info)
                        byAttributesSpectConf('spectrometer') #  Выбрать конфигурацию по атрибут 'spectrometer'
                        getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                        SpectTurnOnLamp() # включить лампу

            elif bt_input == 'ca': # --- calibrate clicked --- COMMAND_CALIBRATE  калибровка
                whistleDog(calibrateTimout) # проверить на залипание
                adjustTemperature()  # отрегулировать температуру
                cover=checkDrawerIsDown()# cover = GPIO.input(COVER) --- проверить открыта ли крышка ---
                if cover==0:
                    bcover=True
                    calib_results = {'DrawerOpen': bcover,'Reference':[],'Dark':[],  'Command': flgs.lastcommand}
                    print(str(calib_results))
                    SendHostMessage('%s\n' %str(calib_results)) # отправить сообщение результат калибровки по bt
                else:
                    flgs.inprocessFlag = True # флаг состояния в процессе
                    flgs.canHeatForward = True # флаг возожности нагреться вперед
                    NumOfPoints=flgs.config["spectrometerAttributes"]['NumOfPoints'] # установили атрибут
                    byAttributesSpectConf('dark') #  Выбрать конфигурацию по атрибут 'dark'
                    getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                    test=1
                    flgs.dark_results = getDark(test) # получить темноту --- система калибровки для измерения
                cover=checkDrawerIsDown()# cover = GPIO.input(COVER) --- проверить открыта ли крышка
                if cover==0:
                    bcover=True
                    calib_results = {'DrawerOpen': bcover,'Reference':[],'Dark':[],  'Command': flgs.lastcommand}
                    print(str(calib_results)) # вывести результат калибровки
                    SendHostMessage('%s\n' %str(calib_results)) # отправить сообщение результат калибровки по bt
                    Home_y() # переместить Y в домашнее состояние
                else:
                    movetometerial()
                    byAttributesSpectConf('plastic') #  Выбрать конфигурацию по атрибуту 'plastic'
                    getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                    SpectTurnOnLamp()
                    flgs.calibrationmaterial = calibration(False,NumOfPoints)
                    print(flgs.calibrationmaterial)
                    print('after plastic')
                    log('after plastic')
                    Home_y()
                    if flgs.mode==1:
                        moveToCalibrateExtract() 
                        byAttributesSpectConf('white') #  Выбрать конфигурацию по атрибуту 'white'
                        getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                        print('b4 white')
                        log('b4 white')
                        flgs.calibration_results = calibration(True,NumOfPoints)
                        print('after white')
                        log('after white')
                        print(flgs.calibration_results)
                        Move_y(pos.y-550)
                        sleep(0.1)
                        Move_z(flgs.safe_z_distance)
                        Home() # Переместить всю систему домой
                    else:
                        cover=checkDrawerIsDown() # cover = GPIO.input(COVER)
                        if cover==0:
                            bcover=True
                            calib_results = {'DrawerOpen': bcover,'Reference':[],'Dark':[],  'Command': flgs.lastcommand}
                            print(str(calib_results))
                            SendHostMessage('%s\n' %str(calib_results)) # отправить сообщение результат калибровки по bt
                        else:
                            MoveToCalibration()
                            byAttributesSpectConf('white')                            
                            getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                            print('b4 white')
                            log('b4 white')
                            flgs.calibration_results = calibration(True,NumOfPoints)                           
                            print('after white')
                            log('after white')
                            print(flgs.bud_center_z)
                            print('calibration_results')
                            print(flgs.calibration_results)
                            fPlasticZ=float(flgs.config["plasticCenterAttributes"]['CenterZ'])*(mm_to_step_Z/1000.0)
                            PlasticZ=int(fPlasticZ)
                            Move_z(PlasticZ)
                            Move_x(flgs.bud_center_x)                

            elif bt_input == 'fc':  # COMMAND_FIND_CENTER  НАЙТИ ЦЕНТР
                byAttributesSpectConf('plastic')
                getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                findCenterExtract()
                Home_y()

            elif bt_input == 'dp': # COMMAND_DOWNLOAD_POINTS  загрузить точку конфигурации с сервера
                whistleDog(calibrateTimout)
                ConfigurationAccepted = downloadPointsExtract()
                # отправить сообщение  по bt
                SendHostMessage('%s\n' %str({'ConfigurationAccepted': ConfigurationAccepted, 'Command':flgs.lastcommand}))
                
            elif bt_input == 'dc':  # download configFile from server --- загрузить конфигурационный файл с сервера ---
                get_config_and_update_conf_file()
                if SAVE_RCL_FLAG:
                    os.system('sudo cat /etc/rc.local | grep new > /home/pi/looprclocal.txt')
                    f = open('/home/pi/looprclocal.txt','r')
                    for line in f:
                        log (line)
                    f.close()
    
            elif bt_input == 'gc': # COMMAND_GET_CONFIG  ПОЛУЧИТЬ КОНФИГУРАЦИЮ
                whistleDog(calibrateTimout)
                ConfigurationAccepted = get_config()
                # отправить сообщение  по bt
                SendHostMessage('%s\n' %str({'ConfigurationAccepted': ConfigurationAccepted, 'Command':flgs.lastcommand}))
                if SAVE_RCL_FLAG:
                    os.system('sudo cat /etc/rc.local | grep new > /home/pi/gclooprclocal.txt')
                    f = open('/home/pi/gclooprclocal.txt','r')
                    for line in f:
                        log (line)
                    f.close()
    
            elif bt_input == 'gs':  #
                getSpectrometerActualParams() # Получить актуальные параметры спектрометра

            elif bt_input == 's1':  # COMMAND_STABILITY1
                whistleDog(calibrateTimout)
                cover = GPIO.input(COVER)
                print('Drawer : %d ' %cover)
                log( 'Drawer : %d ' %cover)
                if cover==0:
                    print('insert flower and try again')
                stabilityFlag =True
                NumOfPoints=flgs.config["stabilityAttributes"]['NumOfPoints']
                SpectSetWL(NumOfPoints)
                MoveToCalibration()
                flgs.calibration_results = calibration(False,NumOfPoints)
                whistleDog(firstconnectionTimout) # stabilityTimout
                backtoRegularSpectConf()
                moveToCenterWCS() # переместить в центр WSC
                num_of_cycles=1
                whistleDog(firstconnectionTimout)
                getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                sleep(1)

            elif bt_input == 'mw': # COMMAND_MOVE_WCS  переместить в центр WSC
                moveToCenterWCS()
    
            elif bt_input == 's2': # COMMAND_STABILITY2
                stabilityFlag =True
                whistleDog(firstconnectionTimout) #
                try:
                    num_of_cycles=num_of_cycles+1
                except Exception as e:
                    logexception(e)
                    num_of_cycles=1
                getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                sleep(1)
                print('num_of_cycles %d'%num_of_cycles)

            elif bt_input == 'st': # Переместить всю систему домой
                Home()
                
            elif bt_input == 'ex': # --- анализ --- COMMAND_ANALYZE
                Move_y(flgs.fix_steps_y+200)
                Move_z(flgs.bud_center_z)
                Move_y(flgs.fix_steps_y)
                whistleDog(calibrateTimout)
                adjustTemperature()  # отрегулировать температуру
                pos_y=pos.y/mm_to_step_Y
                cover = GPIO.input(COVER)
                print('Drawer : %d ' %cover)
                log( 'ex   Drawer : %d ' %cover)
                if cover==0:
                    bcover=True
                    print('insert flower and try again')
                    info = {'DrawerOpen': bcover,'Absorbance':[],'Temperature': temperature,'Reference': flgs.calibration_results['Reference'],'Reflectance': [],'CalibrationMaterial': flgs.calibrationmaterial['Reference'], 'PosY':pos_y, 'Command':flgs.lastcommand}
                    print(info)
                    SendHostMessage('%s\n' %str(info)) # отправить сообщение с текущей информацией по bt
                else:
                    flgs.budDistanceFlag = False
                    flgs.canHeatForward = False
                    getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                    withCamera = flgs.config['examinationAttributes']['WithCamera']
                    max_range_x_save=flgs.MAX_RANGE_X
                    flgs.MAX_RANGE_X=1800
                    if withCamera==True:
                        measureDistances()
                        exam()
                    else:
                        exam(1)
                    flgs.wasopen = False
                    flgs.MAX_RANGE_X=max_range_x_save
                    fPlasticZ=float(flgs.config["plasticCenterAttributes"]['CenterZ'])*(mm_to_step_Z/1000.0)
                    PlasticZ=int(fPlasticZ)
                    print(PlasticZ)
                    Home_y()
                    Move_z(PlasticZ)
                    flgs.inprocessFlag=False

    
            elif bt_input == '1S': # COMMAND_ONE_SPECTRA_WITH_DARK  сделать фото изображения, записать в файл, преобразовать в двоичный формат, получить актуальные параметры спектрометра, получить строку информации, вывести ее, переместить устройство в состояние moveToPlasticInSteps
                if cover==0:
                    bcover=True
                    print('insert flower and try again') # вставьте цветок и повторите попытку
                    info = {'DrawerOpen': bcover,'Absorbance':[],'Temperature': temperature,'Reference': flgs.calibration_results['Reference'],'Reflectance': [],'CalibrationMaterial': flgs.calibrationmaterial['Reference'], 'PosY':pos_y, 'Command':flgs.lastcommand}
                    print(info)
                    SendHostMessage('%s\n' %str(info))
                whistleDog(calibrateTimout)
                flgs.budDistanceFlag = False
                examConfNoDark() 
                getSpectrometerActualParams() # Получить актуальные параметры спектрометра
                reflectance=getOneSpectraDark()
                pos_y=pos.y/mm_to_step_Y
                log('pos_y=%f'%pos_y)
                print('pos_y=%f'%pos_y)
                temperature = getTemperature()
                info = {'Absorbance':[],'Temperature': temperature,'Reference': flgs.calibration_results['Reference'],'Reflectance': reflectance,'CalibrationMaterial': flgs.calibrationmaterial['Reference'], 'PosY':pos_y, 'Command':flgs.lastcommand}
                print(info)
                SendHostMessage('%s\n' %str(info)) # отправить сообщение с текуще информацией по bt
                print(info)
    
            elif bt_input == 'ic': # COMMAND_IS_CLOSED - ЗАКРЫТЬ
                whistleDog(calibrateTimout)
                IsClosed()

            elif bt_input == 'fw':
                find_reflector_dots()

            elif bt_input == 'ul':
                upload_log()

            elif bt_input == 'ue':
                upload_exceptionlog()

            # ---------------------- for debug ---------------------------------
            elif bt_input == 'hp':
                whistleDog(calibrateTimout)
                movetometerial()
    
            elif bt_input == 'hy': # переместь по оси y в домашнюю позицию
                Home_y()
    
            elif bt_input == 'hx': # переместь по оси x в домашнюю позицию
                Home_x()
    
            elif bt_input == 'hz': # переместь по оси z в домашнюю позицию
                Home_z()
    
            elif bt_input == 'my': # переместь по оси y на позицию + 250
                Move_y(pos.y + 250)
                print(pos.y)
    
            elif bt_input == 'y6': # переместь по оси y на позицию + 1500
                Move_y(pos.y + 1500)
    
            elif bt_input == 'yy': # переместь по оси y на позицию flgs.MAX_RANGE_Y
                Move_y(flgs.MAX_RANGE_Y)
                print(pos.y)
    
            elif bt_input == 'ym': # переместь по оси y на позицию - 250
                Move_y(pos.y - 250)
                print(pos.y)
    
            elif bt_input == 'y1':
                whistleDog(firstconnectionTimout)
                bud_front_y=False
                if pos.y>= flgs.BUD_FRONT_Y:
                    bud_front_y=True
                Move_y(pos.y + mm_to_step_Y)
                info = {'Bud_front_y': bud_front_y,  'Command':flgs.lastcommand}
                SendHostMessage('%s\n' %str(info)) # отправить сообщение по bluetooth
                slg="@@@@y1  pos.y= %d\r\n" %(pos.y)
                log(slg)
                print(pos.y)

            elif bt_input == '1y':
                whistleDog(firstconnectionTimout)
                bud_front_y=False
                if pos.y>= flgs.BUD_FRONT_Y:
                    bud_front_y=True
                Move_y(pos.y - mm_to_step_Y)
                info = {'Bud_front_y': bud_front_y,  'Command':flgs.lastcommand}
                SendHostMessage('%s\n' %str(info)) # отправить сообщение по bluetooth
                print(pos.y)
    
            elif bt_input == 'ws':  # blue  on белый включен
                whistleDog(firstconnectionTimout)
                blu_led.start(0)

            elif bt_input == 'w1':     # blue  blink белый блик
                whistleDog(firstconnectionTimout)
                blu_led.ChangeDutyCycle(50)
    
            elif bt_input == 'wo':     # blue  off белый выключен
                whistleDog(firstconnectionTimout)
                blu_led.start(100)

            elif bt_input == 'bs':     # white on синий включен
                whistleDog(firstconnectionTimout)
                white_led.start(0)

            elif bt_input == 'b1':     # white blink синий блик
                whistleDog(firstconnectionTimout)
                white_led.start(0) # test
                sleep(0.1)
                white_led.ChangeDutyCycle(50)

            elif bt_input == 'bo':     # white off синий выключен
                whistleDog(firstconnectionTimout)
                white_led.start(100)

            elif bt_input == 'xt':     # blue off   shutdown(False) синий выключен
                whistleDog(firstconnectionTimout)
                sys.exit("some error message")

            elif bt_input == 'sd':     # blue off синий выключен
                shutdown(True)
                sys.exit("some error message")
    
            elif bt_input == 'mx':
                whistleDog(firstconnectionTimout)
                Move_x(pos.x + 100)
                print(pos.x)

            elif bt_input == 'x4':
                whistleDog(firstconnectionTimout)
                Move_x(pos.x + 400)
                print(pos.x)

            elif bt_input == 'x2':
                whistleDog(firstconnectionTimout)
                Move_x(pos.x + 25)
                print(pos.x)

            elif bt_input == '2x':
                whistleDog(firstconnectionTimout)
                Move_x(pos.x - 25)
                print(pos.x)
    
            elif bt_input == 'xm':
                whistleDog(firstconnectionTimout)
                Move_x(pos.x - 100)
                print(pos.x)
    
            elif bt_input == 'mz':
                whistleDog(firstconnectionTimout)
                Move_z(pos.z + 250)
                print(pos.z)
    
            elif bt_input == 'zm':
                whistleDog(firstconnectionTimout)
                Move_z(pos.z - 250)
                print(pos.z)
    
            elif bt_input == 'z1':
                whistleDog(firstconnectionTimout)
                Move_z(pos.z + mm_to_step_Z)
                print(pos.z)

            elif bt_input == '1z':
                whistleDog(firstconnectionTimout)
                Move_z(pos.z - mm_to_step_Z)
                print(pos.z)

            elif bt_input == 'z6':
                whistleDog(firstconnectionTimout)
                Move_z(pos.z - 1500)
                print(pos.z)
    
            elif bt_input == 'z8':
                whistleDog(firstconnectionTimout)
                Move_z(pos.z + 2000)
                print(pos.z)
    
            elif bt_input == 'z4':
                whistleDog(firstconnectionTimout)
                Move_z(pos.z - 1000)
                print(pos.z)

            elif bt_input == 'zl':
                whistleDog(firstconnectionTimout)
                Home_z()
                Move_zLoop(10)
                print(pos.z)
    
            elif bt_input == 'yl':
                Home_y()
                times=8
                for x in range(times): #TODO(Ido): Find out where is times come from
                    whistleDog(firstconnectionTimout)
                    Move_y(2900)
                    sleep(1)
                    Move_y(-2900)
    
            elif bt_input == 'zz':
                whistleDog(firstconnectionTimout)
                Move_x(pos.x + 400)  # move to center
                print(pos.x)
                Move_z(flgs.MAX_RANGE_Z)
                print(pos.z)

            elif bt_input == 'zc':
                whistleDog(firstconnectionTimout)
                Home_x()
                Move_z(flgs.bud_center_z)
                print(pos.z)
                Move_x(flgs.bud_center_x)  # move to center
                print(pos.x)
                if flgs.budDistanceFlag == False:
                    flgs.budDistance=get_min_distance()
                    log('flgs.budDistance=%s'%str(flgs.budDistance))
                    flgs.budDistanceFlag = True
    
                    ytomove=flgs.safe_y_distance+ int((flgs.budDistance-3) * mm_to_step_Y)
                    Move_y(    ytomove)
                else:
                    ytomove=flgs.safe_y_distance+ int((flgs.budDistance-3) * mm_to_step_Y)
                    Move_y(    ytomove)
    
            elif bt_input == 'za':
                Move_z(0)
                print(pos.z)
    
            elif bt_input == 'xq':
                mvx(40)
                print(pos.x)
    
            elif bt_input == 'qx':
                mvx(-40)
                print(pos.x)
    
            elif bt_input == 'zq':
                mvz(1000)
                print(pos.z)
    
            elif bt_input == 'qz':
                mvz(-1000)
                print(pos.z)
    
            elif bt_input == 'yq':
                mvy(1000)
                print(pos.y)
    
            elif bt_input == 'qy':
                mvy(-1000)
                print(pos.y)
    
            elif bt_input == 'sw':
                M_SwitchTest()
    
            elif bt_input == 'pi':
                pid=os.getpid()
                print(pid)

            elif bt_input == 'kl':
                pid=os.getpid()
                print(pid)
                call(['sudo','kill -9',pid])
    
            elif bt_input == 'dm':
                whistleDog(calibrateTimout)
                flgs.swcode = get_swcode()
                if flgs.swcode != None:
                    swapVersion()
    
            elif bt_input == 'hm':
                whistleDog(firstconnectionTimout)
                flgs.budDistanceFlag = False   #tbd
                Home() # Переместить всю систему домой

            elif bt_input == 'cf':
                GetCameraImageSaveLOFN('zz.bmp')

            elif bt_input == 'mc':
                MoveToCalibration()

            elif bt_input == 'cs':
                whistleDog(firstconnectionTimout)
                GetCameraImageSaveLO(1,1)

            elif bt_input == 'cg':
                GetCameraImageSaveColor()
    
            elif bt_input == 'co':
                whistleDog(firstconnectionTimout)
                GetCameraImageSave(1,1)
                GetCameraImageSaveDark(1,1)

            elif bt_input == 'lo':
                whistleDog(firstconnectionTimout)
                SpectTurnOffLamp()
    
            elif bt_input == 'ln':
                whistleDog(firstconnectionTimout)
                whistleDog(temperatureTimout)
                SpectTurnOnLamp()
    
            elif bt_input == 'dn':
                whistleDog(firstconnectionTimout)
                ledOn()
    
            elif bt_input == 'df':
                ledOff()
    
            elif bt_input == 'tm':
                printTime()

            elif bt_input == '1S':
                whistleDog(calibrateTimout)
                flgs.budDistanceFlag = False
                byAttributesSpectConf('spectrometer')
                getSpectrometerActualParams()
                measureDistances()
                exam()
                # fPlasticZ=float(flgs.config["spectrometerCenterAttributes"]['PlasticZ'])*(mm_to_step_Z/1000.0)
                PlasticZ=calcPlasticZ()
                print(PlasticZ)
                Home_y()
                Move_z(PlasticZ)
    
            elif bt_input == 'uc':
                whistleDog(connectionTimout)
                uploadconfigfile()

            elif bt_input == 'tt':
                whistleDog(connectionTimout)
                flgs.canHeatForward = True
                ftemperature,ready=adjustTemperature(True)    # отрегулировать температуру
                # temperature = getTemperature()
                # ftemperature=float(temperature)
                print(ftemperature)
                info = {'Temperature': ftemperature, 'Command':flgs.lastcommand, 'Operable':ready}
                if (ftemperature<=float(flgs.TemperatureLowerLimit )) :  #TBD
                    log('B4 adjustTemperature')
                    adjustTemperature()  # отрегулировать температуру
                    # flgs.coolingFlag=False
                # log('ftemperature=%f'%ftemperature )
                # sl= 'flgs.TemperatureLowerLimit=%f '%flgs.TemperatureLowerLimit 
                # su= 'flgs.TemperatureUpperLimit=%f '%flgs.TemperatureUpperLimit 
                # s= sl+" "+su
                # log(s)
    
                # if (ftemperature>=flgs.TemperatureLowerLimit and ftemperature<=flgs.TemperatureUpperLimit and flgs.coolingFlag==False):  #TBD
                    # info = {'Temperature': temperature, 'Command':flgs.lastcommand,  'Operable':True}
                SendHostMessage('%s\n' %str(info))
                IsPowerButtonPressed()

            elif bt_input == '1s':
                log('**1s**')
                whistleDog(firstconnectionTimout)
                getSpectrometerActualParams()
                reflectance=getOneSpectra()
                pos_y=pos.y/mm_to_step_Y
                temperature = getTemperature()
                info = {'Absorbance': [],'Temperature': temperature,
                        'Reference': flgs.calibration_results['Reference'],
                        'CalibrationMaterial': flgs.calibrationmaterial_results['Reference'],
                        'Reflectance': reflectance, 'PosY':pos_y, 'Command':flgs.lastcommand}
                SendHostMessage('%s\n' %str(info))
                print(info)

            elif bt_input == '2s':      #for debug only call from terminal getOneSpectraDark()  [] для отладки только вызов из терминала
                whistleDog(firstconnectionTimout)
                reflectance=getOneSpectra2()
                pos_y=pos.y/mm_to_step_Y
                info = {'Reflectance': reflectance, 'PosY':pos_y, 'Command':flgs.lastcommand}
                SendHostMessage('%s\n' %str(info))
    
            elif bt_input == 'cr':
                Getpectlampcurrents()
    
            elif bt_input == 'da':
                flgs.dark_results =getDark()

            elif bt_input == 'gr':
                flgs.dark_results =getReference() # tbd
    
            elif bt_input == 'vv':
                print('vv flgs.safe_y_distance %d' %flgs.safe_y_distance)
                print('vv flgs.safe_z_distance %d' %flgs.safe_z_distance)
    
            elif bt_input == 'dl':
                fPlasticZ=float(flgs.config["plasticCenterAttributes"]['CenterZ'])*(mm_to_step_Z/1000.0)
                PlasticZ=int(fPlasticZ)
                print(PlasticZ)
                Home_y()
                Move_z(PlasticZ)
                flgs.inprocessFlag=False
    
            elif bt_input == 'gd':
                get_min_distance()
    
            elif bt_input == 'gz':
                get_min_distanceBitDark()
            elif bt_input == 'gx':
                get_min_distanceBitlightOn()
            # elif bt_input == 'gb':
            #     print datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            #     get_bud_length(bud_length_gap)
            #     print datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            elif bt_input == 'gz':
                whistleDog(firstconnectionTimout) # test roundtrip
    
                SendHostMessage('gz\n')
    
            elif bt_input == 'sv':
                getVersion()
    
            elif bt_input == 'aa':
                rdExeptionFile()
    
            elif bt_input == 'gi':
                GetDevId()
    
            elif bt_input == 'm1':
                print('We are in...')
                moveToCenter1()
        except Exception as e:
            # произошло исключение цикла
            log('@@!!!!!!! loop exception occurred!!!!!@@')
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            if str(exc_type) == "<class 'bluetooth.btcommon.BluetoothError'>":
                   print(str(exc_type))
                   log( str(exc_type))
                   # выполнить команду linux, создает пустой файл с разрешением изменить время доступа к файлу -a
                   # -m переназначает время последнего изменнения на текущее
                   # -t задает время 2015 12 18 01:30.09
                   call(["touch","-a","-m","-t","201512180130.09","file.txt"])
            sexeption=str(exc_type) +' '+str(exc_tb.tb_lineno) +' '+str(e) + ' ' + traceback.format_exc()
            write_to_exceptionlog(sexeption)

# найти топовый белый цвет, который отличается от существующего
def from_top_find_white_exists_diff(ordinal=FROM_HOME):
    ddebugbl=True
    ddbugd=True
    distances_array = [-1] #,-1,-1,-1,-1]
    sleep(5.2)
    croppedPixels = 0
    athomeflag = 0
    min_distance = 0
    firstpixel=-1
    lastpixelwhite=-1
    lfirstpixelblack=-1
    flgs.cropped450 = 0
    img = GetCameraImage()
    if pos.y==flgs.fix_steps_y:
        flgs.cropped300 = 0
        athomeflag = 1
    else:
        athomeflag = 0
        flgs.cropped450 = 0

    pos_zy='Z'+str(pos.z)+'Y'+str(pos.y)+'_'+str(ordinal)+datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print ('athomeflag %s'%str(athomeflag))
    log ('athomeflag %s'%str(athomeflag))
    print ('pos.y %s'%str(pos.y))
    fn = '/home/pi/pct/BITwhite%s.bmp'%str(pos_zy)
    if  os.path.exists(fn):
        os.system('sudo rm  '+fn)
    if ddebugbl==True:
        GetCameraImageSaveLOFN(fn)
    if  os.path.exists(fn)==False:
        return -1

    fnj = '/home/pi/pct/BITwhite%s.jpg'%str(pos_zy)
    fnj1 = '/home/pi/pct/BIT1%s.jpg'%str(pos_zy)
    fnjblack = '/home/pi/pct/BITwhiteblack%s.jpg'%str(pos_zy)
    fnjwide = '/home/pi/pct/BITwhitew%s.jpg'%str(pos_zy)
    tstfnjwide = '/home/pi/pct/tstBITwhitew%s.jpg'%str(pos_zy)
    fnjrv = '/home/pi/pct/BITwhiterev%s.jpg'%str(pos_zy)
    fnjwidezip = '/home/pi/pct/BITwhitew.zip'
    cropLeft=flgs.spectrometer_center_in_image-5
    cropRight=flgs.spectrometer_center_in_image+5
    widecropped_img = img[0:320,cropLeft:cropRight]
    cv2.imwrite(fnjwide, widecropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    im_gray = cv2.cvtColor( widecropped_img, cv2.COLOR_BGR2GRAY )
    thresholdForDistance= 100
    thresh_img = cv2.threshold(im_gray, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    cropDown=flgs.spectrometer_edge_in_image_white+5
    cropUp=cropDown-flgs.whiteExists_crop_size
    cropUp=0
    cropped_img = widecropped_img[320:cropDown,0:10]
    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img , [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    second_cropped_img = widecropped_img[cropUp:cropDown,0:10]
    fnjSecond = '/home/pi/pct/BITwhiteSecond%s.jpg'%str(pos_zy)
    if ddebugbl==True:
        cv2.imwrite(fnjSecond, second_cropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    im_gray1 = cv2.cvtColor( second_cropped_img, cv2.COLOR_BGR2GRAY )
    thresholdForDistance= 100
    thresh_img1 = cv2.threshold(im_gray1, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    if ddebugbl==True:
        cv2.imwrite(fnjblack, thresh_img1 , [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    cropped_imgrev=numpy.flipud(thresh_img)
    cropped_imgrev_save=numpy.flipud(thresh_img)
    cropped_imgrev_save_2=numpy.flipud(thresh_img)
    diff = -1
    numCols=10
    firstpixelwhiteArray=[None]*numCols
    lastpixelwhiteArray=[None]*numCols
    for i in range(numCols):
        column = thresh_img1[:, i]
        fnjcol = '/home/pi/pct/testCol%s.jpg'%str(i)
        try:
            white_pixels = numpy.array(numpy.where(column == 255))
            first_white_pixel = white_pixels[:,0]
            last_white_pixel = white_pixels[:,-1]
            print('first_white_pixel=')
            print(first_white_pixel)
            firstpixel= int(first_white_pixel[0])
            print('last_white_pixel=')
            print(last_white_pixel)
            firstpixelwhite= int(first_white_pixel[0])
            firstpixelwhiteArray[i]=firstpixelwhite
            lastpixelwhite= int(last_white_pixel[0])
            lastpixelwhiteArray[i]=lastpixelwhite
            diff = lastpixelwhite-firstpixelwhite
        except Exception as e:
            logexception(e)
            # xxxx='[1000 1000]'
        firstpixel=-1

    
    firstpixelwhiteAverage= numpy.average(firstpixelwhiteArray)
    lastpixelwhiteAverage= numpy.average(lastpixelwhiteArray)
    # xxxx='[1000 1000]'

    print ( 'firstpixelwhiteAverage =' + str(firstpixelwhiteAverage)+'      _'+str(ordinal))
    log ( 'firstpixelwhiteAverage =' + str(firstpixelwhiteAverage)+'      _'+str(ordinal))
    print ( 'lastpixelwhiteAverage =' + str(lastpixelwhiteAverage)+'      _'+str(ordinal))
    log ( 'lastpixelwhiteAverage =' + str(lastpixelwhiteAverage)+'      _'+str(ordinal))
    return lastpixelwhiteAverage


#  для кнопки найти расстояние белого до сепктрометра
def from_bottom_find_white_exists_diff(ordinal=FROM_HOME):
    log('from_bottom_find_white_exists_diff')
    ddebugbl=True
    ddbugd=True
    distances_array = [-1] #,-1,-1,-1,-1]
    sleep(SLEEP_TIME_WHITE_DIFF) #TODO(Ido): Consider moving it to a const
    croppedPixels = 0
    athomeflag = 0
    min_distance = 0
    firstpixel=-1
    lastpixelwhite=-1
    lfirstpixelblack=-1
    flgs.cropped450 = 0
    img = GetCameraImage()
    if pos.y==flgs.fix_steps_y:
        flgs.cropped300 = 0
        athomeflag = 1
    else:
        athomeflag = 0
        flgs.cropped450 = 0
    pos_zy='Z'+str(pos.z)+'Y'+str(pos.y)+'_'+str(ordinal)
    print ('athomeflag %s'%str(athomeflag))
    log ('athomeflag %s'%str(athomeflag))
    print ('pos.y %s'%str(pos.y))
    fn = '/home/pi/pct/BITwhite%s.bmp'%str(pos_zy)
    if  os.path.exists(fn):
        os.system('sudo rm  '+fn)
    if ddebugbl==True:
        GetCameraImageSaveLOFN(fn)
    if  os.path.exists(fn)==False:
        return -1
    fnj = '/home/pi/pct/BITwhite%s.jpg'%str(pos_zy)
    fnj1 = '/home/pi/pct/BIT1%s.jpg'%str(pos_zy)
    fnjblack = '/home/pi/pct/BITwhiteblack%s.jpg'%str(pos_zy)
    fnjwide = '/home/pi/pct/BITwhitew%s.jpg'%str(pos_zy)
    tstfnjwide = '/home/pi/pct/tstBITwhitew%s.jpg'%str(pos_zy)
    fnjrv = '/home/pi/pct/BITwhiterev%s.jpg'%str(pos_zy)
    fnjwidezip = '/home/pi/pct/BITwhitew.zip'
    cropLeft=flgs.spectrometer_center_in_image-5
    cropRight=flgs.spectrometer_center_in_image+5
    widecropped_img = img[0:320,cropLeft:cropRight]
    cv2.imwrite(fnjwide, widecropped_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    im_gray = cv2.cvtColor( widecropped_img, cv2.COLOR_BGR2GRAY )
    thresholdForDistance= 100
    thresh_img = cv2.threshold(im_gray, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    cropDown=320
    cropUp=flgs.spectrometer_edge_in_image_white-5
    cropped_img = widecropped_img[cropUp:cropDown,0:10]
    if ddebugbl==True:
        cv2.imwrite(fnj, cropped_img , [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    im_gray1 = cv2.cvtColor( widecropped_img, cv2.COLOR_BGR2GRAY )
    thresholdForDistance= 100
    thresh_img1 = cv2.threshold(im_gray1, thresholdForDistance , 255, cv2.THRESH_BINARY)[1]
    if ddebugbl==True:
        cv2.imwrite(fnjblack, thresh_img1 , [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    diff = -1.0
    cropped_imgrev=numpy.flipud(thresh_img1)
    fnjSecondrev = '/home/pi/pct/revBITwhitebottom%s.jpg'%str(pos_zy)
    if ddebugbl==True:
        cv2.imwrite(fnjSecondrev, cropped_imgrev, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    numCols=10
    firstpixelwhiteArray=[None]*numCols
    lastpixelwhiteArray=[None]*numCols
    for i in range(numCols):
        
        column = cropped_imgrev[:, i]
        fnjcol = '/home/pi/pct/testCol%s.jpg'%str(i)
       
        try:
            white_pixels = numpy.array(numpy.where(column == 255))
            first_white_pixel = white_pixels[:,0]
            last_white_pixel = white_pixels[:,-1]
            print('first_white_pixel=')
            print(first_white_pixel)
            firstpixel= int(first_white_pixel[0])
            print('last_white_pixel=')
            print(last_white_pixel)
            firstpixelwhite= int(first_white_pixel[0])
            firstpixelwhiteArray[i]=firstpixelwhite
            lastpixelwhite= int(last_white_pixel[0])
            lastpixelwhiteArray[i]=lastpixelwhite
            diff = lastpixelwhite-firstpixelwhite
        except Exception as e:
            logexception(e)
            log ('from_bottom_find_white_exists_diff IMAGE ANALISYS FAILED')
        firstpixel=-1
    firstpixelwhiteAverage= numpy.average(firstpixelwhiteArray) # среднее значение белого по первому пикселю
    lastpixelwhiteAverage= numpy.average(lastpixelwhiteArray) # среднее значение белого по последнему пикселю
    xxxx='[1000 1000]'
    bottomBlack= 320.0-firstpixelwhiteAverage

    print ( 'firstpixelwhiteAverage =' + str(firstpixelwhiteAverage)+'      _'+str(ordinal))
    log ( 'from_bottom_find_white_exists_diff  firstpixelwhiteAverage =' + str(firstpixelwhiteAverage)+'      _'+str(ordinal))
    log ( 'bottomBlack =' + str(bottomBlack)+'      _'+str(ordinal))
    print ( 'lastpixelwhiteAverage =' + str(lastpixelwhiteAverage)+'      _'+str(ordinal))
    log ( 'lastpixelwhiteAverage =' + str(lastpixelwhiteAverage)+'      _'+str(ordinal))
    return firstpixelwhiteAverage

#   Bluetooth connect
def btconnect(btpair=True):
    if btpair==True:
        bt_pairing()
    else:
        flgs.server_sock.close()
    flgs.server_sock = BluetoothSocket(RFCOMM) # коннект Bluetooth через RFCOMM
    flgs.server_sock.bind(("",PORT_ANY))
    flgs.server_sock.listen(1)
    log('flgs.gspectserialnum - ' + str(flgs.gspectserialnum))
    print("Waiting for connection to GC200") #  Ожидание коннекта
    print(flgs.BlueToothConnected) # Статус коннекта
    while flgs.BlueToothConnected == False:
        sleep(1)
        try:
            flgs.server_sock.settimeout(1)
            flgs.client_sock, flgs.client_info = flgs.server_sock.accept()
            print("Accepted connection from ", flgs.client_info)
            log('%%%Accepted connection from %%%')
            # ищем строку new в файле /etc/rc.local и запишем в /home/pi/cnctrclocal.txt
            if SAVE_RCL_FLAG:
                os.system('sudo cat /etc/rc.local | grep new > /home/pi/cnctrclocal.txt')
            else:
                os.system('cp /home/pi/verrclocal.txt /home/pi/cnctrclocal.txt')
            f = open('/home/pi/cnctrclocal.txt','r')
            for line in f:
                log (line)
            f.close()
            flgs.BlueToothConnected = True
        except Exception as e:
            log('!!!!!!!BlueTooth exception occurred!!!!!') # Произошло исключение BlueTooth
            exc_type, exc_obj, exc_tb = sys.exc_info()
            print(str(exc_type))
            log( str(exc_type))
            whistleDog(bitTimout)
            adjustTemperature()  # отрегулировать температуру
            IsPowerButtonPressed()
    print('GC200 BlueToothConnected!')
    flgs.client_sock.settimeout(0.5)
    blu_led.ChangeDutyCycle(0)
    print('GC200 ready')

# --------main------------ Gc200Logs

# dir_pathA = '/home/pi/gc200A' # код на Python, для запуска
# dir_pathB = '/home/pi/gc200B' # код на Python, для запуска
dir_pathpct = '/home/pi/pct' # тут  изображения храняться

if __name__ == "__main__":
    try:
        if not os.path.exists('/home/pi/LogsGc200'):
            os.system('sudo mkdir /home/pi/LogsGc200')
    except Exception as e:
        logexception(e)
        print('Failed to create LogsGc200')

    try:
        if not os.path.exists(dir_pathA):
            os.system('sudo mkdir /home/pi/gc200A')
    except Exception as e:
        logexception(e)

        print('Failed to create ' + dir_pathA)
    try:
        if not os.path.exists(dir_pathB):
            os.system('sudo mkdir /home/pi/gc200B')
    except Exception as e:
        logexception(e)
        print('Failed to create ' + dir_pathB)

    try:
        if not os.path.exists(dir_pathpct):
            os.system('sudo mkdir /home/pi/pct')
    except Exception as e:
        logexception(e)
        print('Failed to create ' + dir_pathpct)

    shorten_log()
    log('---------------------STARTING GC200----------------------------------')
    
    lastLampUptimeFromLog=getLastLampTimeFromLog() # получить время воследней индикации из log
    calculated=calculate_LampElapsedTime(lastLampUptimeFromLog,STATEFORLAMPTIME_STARTUP) # рассчитать время, затраченное лампой
    print('STARTUP calculated=%d'%calculated)
    if calculated<0: calculated=0  # время должно быть не меньше 0
    logLampElapsedTime(calculated) # запись значения аргумента в файл журнал данных lampElapsedTime.txt
    try:
        blu_led.start(0)
        pos = locations(0)
        flgs = flags(0)
        # ----- инициализация значений ----- непонятно почему сразу не задать эти значения  в классе?
        btconn = btconnectionTime(0) # задали свой тип данных, два нулевых временных параметра
        flgs.log = logging.getLogger()
        flgs.log.setLevel(logging.DEBUG)
        flgs.log.debug('Logging has started')
        flgs.BlueToothConnected = False
        flgs.versionprovedsent = False # True
        flgs.wasopen = False
        flgs.canHeatForward = True
        flgs.powerButtonPressed = False
        flgs.inprocessFlag = False
        flgs.coolingFlag=False
        flgs.budDistanceFlag = False
        flgs.CoolDownHysteresis = 54.0
        flgs.TemperatureUpperLimit=57.0
        flgs.TemperatureLowerLimit=47.0
        flgs.heatingDownPosition=0
        btconn.btcontime_1=datetime.now()
        btconn.btcontime_2=datetime.now()
        flgs.powerButtonPressedTime=None
        flgs.ignorePowerButtonPressed=0
        flgs.extractYoffset=0
        flgs.calibration_y_steps=-1
        flgs.fix_steps_x= FIX_STEPS_X_DEFAULT
        flgs.fix_steps_y=FIX_STEPS_Y_DEFAULT
        flgs.fix_steps_z= FIX_STEPS_Z_DEFAULT
 
        flgs.lampOn = 0
        # flgs.lampSavedTime = 0
        flgs.lampStartTime = 0
        flgs.lampElapsedTime = 0
        flgs.gotConfig=False
        flgs.deviceTipe = 0
        
        bitarr = bitarray(2)
        log('bitarr.bit_arr[MotorY]='+str( bitarr.bit_arr['MotorY']))
        log('bitarr.bit_arr[MotorX]='+str( bitarr.bit_arr['MotorX']))
        log('bitarr.bit_arr[MotorZ]='+str( bitarr.bit_arr['MotorZ']))
        log('bitarr.bit_arr[Spectrometer]='+str( bitarr.bit_arr['Spectrometer']))
        log('bitarr.bit_arr[Camera]='+str( bitarr.bit_arr['Camera']))


        GPIO.output(ENABLE_Z ,GPIO.HIGH)
        GPIO.output(ENABLE_Y ,GPIO.HIGH)
        GPIO.output(ENABLE_X ,GPIO.HIGH)

        GPIO.output(SLP, GPIO.HIGH)

        try:
            flgs.camera = picamera.PiCamera()  # setup camera --- установка камеры ---
            flgs.camera.resolution =(240, 320)  #( 320,240) # setup camera resolution to 240 X 320 pixels --- установка разрешения камеры ---
            shutterspeed =4000 # 1000000*(9.0987*math.exp(-0.702*10000))
            print(shutterspeed)
            flgs.camera.shutter_speed = int(shutterspeed) # установка выдрежкм затвора камеры

        except Exception as e: # в случае неполадки камеры
            log('@@!!!!!!!main camera FAILED!!! !!!!!@@')
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(str(exc_type))
            log( str(exc_type))
            sexeption=str(exc_type) +' '+str(exc_tb.tb_lineno) +' '+str(e) + ' ' + traceback.format_exc()
            write_to_exceptionlog(sexeption)
            bitarr.bit_arr['Camera'] = 0
            print('camera FAILED!!!')

        # global stabilityFlag

        # global countthread
        # global Thread
        # global lampOn
        # global lampStartTime
        # global lampSavedTime
        # global lampElapsedTime

        flgs.bud_length_second_steps=6000 # второй шаг длины бутона
        flgs.SpectrometerExists=InitDeviceId() # check if machine name == sensor serial --- проверка наименования машины, совпадает с серией сенсора
        log('flgs.SpectrometerExists = ' + str(flgs.SpectrometerExists))
        ledOff() #turn off camera light выключить свет камеры
        countthread =0 # задаем количество потоков
        stabilityFlag=False
     
        flgs.safe_y_distance =-1
        flgs.safe_z_distance =-1

        flgs.MAX_RANGE_Z =-1
        flgs.MAX_RANGE_Y =-1
        flgs.MAX_RANGE_X =-1
        flgs.BUD_FRONT_Y =-1
        flgs.dissconnectioncounter =0
        flgs.shorten_log_size=40000
        flgs.shorten_exceptionlog_size=5000 

        print('main =-1 flgs.safe_y_distance %d' %flgs.safe_y_distance)
        print('main  =-1flgs.safe_z_distance %d' %flgs.safe_z_distance)

        print('main =-1 flgs.MAX_RANGE_Z %d' %flgs.MAX_RANGE_Z)
        configvalues= getconfigvalues() #TODO(Ido): See if this is a duplicate and how to remove it

        rereadConfValuesFromFile()
        print('main flgs.safe_y_distance %d' %flgs.safe_y_distance)
        print('main flgs.safe_z_distance %d' %flgs.safe_z_distance)

        print('main flgs.MAX_RANGE_Z %d' %flgs.MAX_RANGE_Z)
        

        full_path = os.path.realpath(__file__)
        path, filename = os.path.split(full_path)
        pos.x = flgs.MAX_RANGE_X
        pos.y = flgs.MAX_RANGE_Y
        pos.z = flgs.MAX_RANGE_Z
        try:
            flgs.pipe_name = "/home/pi/GcWatchdog/GcWatchdogFifo"
            flgs.pipeout = os.open(flgs.pipe_name, os.O_WRONLY)
            if not os.path.exists(flgs.pipe_name):
                os.mkfifo(pipe_name)
                print('WATCHDOG SUSPICIOUS!!!') # WATCHDOG подозрительный

            flgs.watchdog = True
            print('woof WATCHDOG SUCCESS!!!') # WATCHDOG успешный
            whistleDog(firstconnectionTimout)

        except Exception as e:
            write_to_exceptionlog('watchdog FAILED!!!^1') # WATCHDOG неудачный
            print('watchdog FAILED!!!')
            flgs.watchdog = False

        whistleDog(firstconnectionTimout)
        log('b4 ActivateBITNew')
        rclocalnewfn = '/home/pi/rclocal.txt'
        try:
            if  os.path.exists(rclocalnewfn):
                        os.system('sudo rm  /home/pi/rclocal.txt')
        except Exception as e:
            logexception(e)
            print('Failed to remove ' + rclocalnewfn)

        if SAVE_RCL_FLAG:
            os.system('sudo cat /etc/rc.local | grep new > /home/pi/tstrclocal.txt') #  создаем файл /home/pi/tstrclocal.txt
        else:
            os.system('cp /home/pi/verrclocal.txt /home/pi/tstrclocal.txt')
        f = open('/home/pi/tstrclocal.txt','r')
        for line in f:
            log (line)
        f.close()
        newVersion=0
        if SAVE_RCL_FLAG:
            os.system('sudo cat /etc/rc.local | grep new > /home/pi/rclocal.txt')  # создаем файл /home/pi/rclocal.txt
        else:
            os.system('cp /home/pi/verrclocal.txt /home/pi/rclocal.txt')
        sleep(0.2)
        f = open('/home/pi/rclocal.txt','r')
        for line in f:
            if 'new' in line:
                newVersion=1
                print('newVersion=1')
                log( 'newVersion=1')
                Home() # Переместить всю систему домой
        f.close()

        print('###b4 flgs.bitPerformed = %d'% flgs.bitPerformed)
        log( '###b4 flgs.bitPerformed = %d'% flgs.bitPerformed )
        if newVersion==0:
            bit = ActivateBITNew()
            # flgs.bitPerformed = 1
            if flgs.lampOn==0:
                SpectTurnOnLamp()
        else:
            flgs.bitPerformed = 0

        log( '###ftr flgs.bitPerformed = %d'% flgs.bitPerformed )
        log('ftr ActivateBITNew')
        if SAVE_RCL_FLAG:
            rclocaltxt=str(os.system("sudo cat /etc/rc.local |grep python"))
        else:
            rclocaltxt = ValueRClocalDefaultB  # for test only
        log (rclocaltxt)
        # только в том случае, если бит выполнен, что означает, что драйвер опущен
        if flgs.bitPerformed == 1:  # only if bit performed meaning drawer is down
            # нагревается быстрее при запуске устройства
            Move_yForCalibration(Y_HEAT_DISTANCE) # heating faster upon starting the device
        flgs.heatingDownPosition=1
        whistleDog(bitTimout)
        print(os.system("ps -aux |grep python | grep -v grep"))
        print('mainbit----------')
        if SAVE_RCL_FLAG:
            os.system('sudo cat /etc/rc.local | grep new > /home/pi/tstrclocal.txt') #  создаем файл /home/pi/tstrclocal.txt
        else:
            os.system('cp /home/pi/verrclocal.txt /home/pi/tstrclocal.txt')
            f = open('/home/pi/tstrclocal.txt','r')
            for line in f:
                log (line)
            f.close()

        whistleDog(firstconnectionTimout)
        log('-- begin btconnect --')
        btconnect()  # ---- Bluetooth connect ----
        log('-- loop --')
        loop()   # ---- the main procedure is waiting for events ---- основная процедура - это ожидание событий
    except KeyboardInterrupt:
        print("disconnected")
        log( '!!!!!!!!!!!!!disconnected!!!!!!!!!!!!!')
        SpectTurnOffLamp()
        GPIO.cleanup()
        client_sock.close()
        flgs.server_sock.close()
        flgs.camera.close()

    except Exception as e:
        log('@@!!!!!!!main exception occurred!!!!!@@')
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        if str(exc_type) == "<class 'bluetooth.btcommon.BluetoothError'>":
            print(str(exc_type))
            log( str(exc_type))
            call(["touch","-a","-m","-t","201512180130.09","file.txt"])
            loop()  # ---- the main procedure is waiting for events ---- основная процедура - это ожидание событий

        sexeption=str(exc_type) +' '+str(exc_tb.tb_lineno) +' '+str(e) + ' ' + traceback.format_exc()
        write_to_exceptionlog(sexeption)
        print("disconnected")
        blu_led.stop()
        white_led.stop()
        GPIO.cleanup()
        client_sock.close()
        flgs.server_sock.close()
        flgs.camera.close()
        pid=os.getpid()
        print(pid)

#  end