from pgc200 import *



# --------main------------ Gc200Logs

if __name__ == '__main__':
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

    lastLampUptimeFromLog = getLastLampTimeFromLog()  # получить время воследней индикации из log
    calculated = calculate_LampElapsedTime(lastLampUptimeFromLog,
                                           STATEFORLAMPTIME_STARTUP)  # рассчитать время, затраченное лампой
    print('STARTUP calculated=%d' % calculated)
    if calculated < 0: calculated = 0  # время должно быть не меньше 0
    logLampElapsedTime(
        calculated)  # запись значения аргумента в файл журнал данных lampElapsedTime.txt
    try:
        blu_led.start(0)
        pos = locations(0)
        flgs = flags(0)
        # ----- инициализация значений ----- непонятно почему сразу не задать эти значения  в классе?
        btconn = btconnectionTime(
            0)  # задали свой тип данных, два нулевых временных параметра
        flgs.log = logging.getLogger()
        flgs.log.setLevel(logging.DEBUG)
        flgs.log.debug('Logging has started')
        flgs.BlueToothConnected = False
        flgs.versionprovedsent = False
        flgs.wasopen = False
        flgs.canHeatForward = True
        flgs.powerButtonPressed = False
        flgs.inprocessFlag = False
        flgs.coolingFlag = False
        flgs.budDistanceFlag = False
        flgs.CoolDownHysteresis = 54.0
        flgs.TemperatureUpperLimit = 57.0
        flgs.TemperatureLowerLimit = 47.0
        flgs.heatingDownPosition = 0
        btconn.btcontime_1 = datetime.now()
        btconn.btcontime_2 = datetime.now()
        flgs.powerButtonPressedTime = None
        flgs.ignorePowerButtonPressed = 0
        flgs.extractYoffset = 0
        flgs.calibration_y_steps = -1
        flgs.fix_steps_x = FIX_STEPS_X_DEFAULT
        flgs.fix_steps_y = FIX_STEPS_Y_DEFAULT
        flgs.fix_steps_z = FIX_STEPS_Z_DEFAULT

        flgs.lampOn = 0
        # flgs.lampSavedTime = 0
        flgs.lampStartTime = 0
        flgs.lampElapsedTime = 0
        flgs.gotConfig = False
        flgs.deviceTipe = 0

        bitarr = bitarray(2)
        log('bitarr.bit_arr[MotorY]=' + str(bitarr.bit_arr['MotorY']))
        log('bitarr.bit_arr[MotorX]=' + str(bitarr.bit_arr['MotorX']))
        log('bitarr.bit_arr[MotorZ]=' + str(bitarr.bit_arr['MotorZ']))
        log('bitarr.bit_arr[Spectrometer]=' + str(
            bitarr.bit_arr['Spectrometer']))
        log('bitarr.bit_arr[Camera]=' + str(bitarr.bit_arr['Camera']))

        GPIO.output(ENABLE_Z, GPIO.HIGH)
        GPIO.output(ENABLE_Y, GPIO.HIGH)
        GPIO.output(ENABLE_X, GPIO.HIGH)

        GPIO.output(SLP, GPIO.HIGH)

        try:
            flgs.camera = picamera.PiCamera()  # setup camera --- установка камеры ---
            flgs.camera.resolution = (240,
                                      320)  # ( 320,240) # setup camera resolution to 240 X 320 pixels --- установка разрешения камеры ---
            shutterspeed = 4000  # 1000000*(9.0987*math.exp(-0.702*10000))
            print(shutterspeed)
            flgs.camera.shutter_speed = int(
                shutterspeed)  # установка выдрежкм затвора камеры

        except Exception as e:  # в случае неполадки камеры
            log('@@!!!!!!!main camera FAILED!!! !!!!!@@')
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(str(exc_type))
            log(str(exc_type))
            sexeption = str(exc_type) + ' ' + str(
                exc_tb.tb_lineno) + ' ' + str(
                e) + ' ' + traceback.format_exc()
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

        flgs.bud_length_second_steps = 6000  # второй шаг длины бутона
        flgs.SpectrometerExists = InitDeviceId()  # check if machine name == sensor serial --- проверка наименования машины, совпадает с серией сенсора
        ledOff()  # turn off camera light выключить свет камеры
        countthread = 0  # задаем количество потоков
        stabilityFlag = False

        flgs.safe_y_distance = -1
        flgs.safe_z_distance = -1

        flgs.MAX_RANGE_Z = -1
        flgs.MAX_RANGE_Y = -1
        flgs.MAX_RANGE_X = -1
        flgs.BUD_FRONT_Y = -1
        flgs.dissconnectioncounter = 0
        flgs.shorten_log_size = 40000
        flgs.shorten_exceptionlog_size = 5000

        print('main =-1 flgs.safe_y_distance %d' % flgs.safe_y_distance)
        print('main  =-1flgs.safe_z_distance %d' % flgs.safe_z_distance)

        print('main =-1 flgs.MAX_RANGE_Z %d' % flgs.MAX_RANGE_Z)
        configvalues = getconfigvalues()  # TODO(Ido): See if this is a duplicate and how to remove it

        rereadConfValuesFromFile()
        print('main flgs.safe_y_distance %d' % flgs.safe_y_distance)
        print('main flgs.safe_z_distance %d' % flgs.safe_z_distance)

        print('main flgs.MAX_RANGE_Z %d' % flgs.MAX_RANGE_Z)

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
                print('WATCHDOG SUSPICIOUS!!!')

            flgs.watchdog = True
            print('woof WATCHDOG SUCCESS!!!')
            whistleDog(firstconnectionTimout)

        except Exception as e:
            write_to_exceptionlog('watchdog FAILED!!!^1')

            print('watchdog FAILED!!!')
            flgs.watchdog = False

        whistleDog(firstconnectionTimout)
        log('b4 ActivateBITNew')
        rclocalnewfn = '/home/pi/rclocal.txt'
        try:
            if os.path.exists(rclocalnewfn):
                os.system('sudo rm  /home/pi/rclocal.txt')
        except Exception as e:
            logexception(e)
            print('Failed to remove ' + rclocalnewfn)
        os.system(
            'sudo cat /etc/rc.local | grep new > /home/pi/tstrclocal.txt')
        f = open('/home/pi/tstrclocal.txt', 'r')
        for line in f:
            log(line)
        f.close()

        os.system(
            'sudo cat /etc/rc.local | grep new > /home/pi/rclocal.txt')
        sleep(0.2)
        newVersion = 0
        f = open('/home/pi/rclocal.txt', 'r')
        for line in f:
            if 'new' in line:
                newVersion = 1
                print('newVersion=1')
                log('newVersion=1')
                Home()  # Переместить всю систему домой
        f.close()

        print('###b4 flgs.bitPerformed = %d' % flgs.bitPerformed)
        log('###b4 flgs.bitPerformed = %d' % flgs.bitPerformed)
        if newVersion == 0:
            bit = ActivateBITNew()
            # flgs.bitPerformed = 1
            if flgs.lampOn == 0:
                SpectTurnOnLamp()
        else:
            flgs.bitPerformed = 0

        log('###ftr flgs.bitPerformed = %d' % flgs.bitPerformed)
        log('ftr ActivateBITNew')
        rclocaltxt = str(os.system("sudo cat /etc/rc.local |grep python"))
        log(rclocaltxt)
        # только в том случае, если бит выполнен, что означает, что драйвер опущен
        if flgs.bitPerformed == 1:  # only if bit performed meaning drawer is down
            # нагревается быстрее при запуске устройства
            Move_yForCalibration(
                Y_HEAT_DISTANCE)  # heating faster upon starting the device
        flgs.heatingDownPosition = 1
        whistleDog(bitTimout)
        print(os.system("ps -aux |grep python | grep -v grep"))
        print('mainbit----------')

        whistleDog(firstconnectionTimout)
        os.system(
            'sudo cat /etc/rc.local | grep new > /home/pi/tstrclocal.txt')
        f = open('/home/pi/tstrclocal.txt', 'r')
        for line in f:
            log(line)
        f.close()

        btconnect()  # ---- Bluetooth connect ----
        loop()  # ---- the main procedure is waiting for events ----
    except KeyboardInterrupt:
        print("disconnected")
        log('!!!!!!!!!!!!!disconnected!!!!!!!!!!!!!')
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
            log(str(exc_type))
            call(["touch", "-a", "-m", "-t", "201512180130.09", "file.txt"])
            loop()

        sexeption = str(exc_type) + ' ' + str(exc_tb.tb_lineno) + ' ' + str(
            e) + ' ' + traceback.format_exc()
        write_to_exceptionlog(sexeption)
        print("disconnected")
        blu_led.stop()
        white_led.stop()
        GPIO.cleanup()
        client_sock.close()
        flgs.server_sock.close()
        flgs.camera.close()
        pid = os.getpid()
        print(pid)


