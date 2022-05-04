# import RPi.GPIO as GPIO         #GPIO
# from bluetooth import *         # bluetooth communication
# import picamera                 # camera
# import picamera.array           # camera

class RPiGPIO():
    PINS = []
    IN = 1
    OUT = 0
    SPI = 41
    I2C = 42
    HARD_PWM = 43
    SERIAL = 40
    UNKNOWN = -1
    BCM = 'BCM'
    HIGH = 1
    LOW = 0

    def __init__(self):
        self.PINS = [0 for item in range(40)]

    def setup(self,PIN,VALUE):
        self.PINS[PIN] = VALUE

    def setmode(self, BOARD_BCM):
        # self.BCM = BOARD_BCM
        print('Установлен режим',BOARD_BCM)

    def getmode(self):
        return self.BCM

    def input(self,PIN):
        return self.PINS[PIN]

    def output(self,PIN,VALUE): # если надо, то предусмотреть работу со списком в переменной PIN
        self.PINS[PIN] = VALUE

    def cleanup(self):
        pass

    def setwarnings(self,BoolVal):
        if not BoolVal:
            print('Предупреждающий канал отключен')

    class PWM():
        PIN = 0
        DutyCycle = 0
        Frequency = 0
        def __init__(self,PIN,Frequency):
            self.PIN = PIN
            self.Frequency = Frequency
            print(f'Установлен режим генерации ШИМ-импульсов {Frequency} в {PIN}')

        def start(self,DutyCycle):
            self.DutyCycle = DutyCycle

        def ChangeFrequency(self,Frequency):
            self.Frequency = Frequency

        def ChangeDutyCycle(self,DutyCycle):
            self.DutyCycle = DutyCycle

        def stop(self):
            self.DutyCycle = 0
            self.Frequency = 0


class picamera():
    resolution = (240, 320)
    shutter_speed = 0
    def PiCamera(self):
        pass
    class array():
        def PiRGBArray(self):
            return
    def start_preview(self):
        pass
    def stop_preview(self):
        pass
    def capture(self,filename, format='bmp'):
        pass

class serial():
    class Serial():
        NameDevice = "/dev/ttyS0"
        baudrate = 115200
        timeout = 30
        StrValue = ''
        def __init__(self,NameDev,baudrate,timeout):
            self.NameDevice = NameDev
            self.baudrate = baudrate
            self.timeout = timeout
        def write(self,StrVal):
            self.StrValue += StrVal
        def readline(self):
            pass

class bluetooth():
    def send(self,StrBt):
        print(StrBt)
    def recv(self,str_size):
        StrBt = input('Enter the command: ')
        return StrBt[str_size]

if __name__ == '__main__':
    pass