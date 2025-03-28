import threading
import time
import math
import cv2
import RPi.GPIO as GPIO
import smbus
from flask import Flask, Response
from picamera2 import Picamera2

# ============================================================================
# CameraServer Class
# ============================================================================
class CameraServer:
    def __init__(self):
        # Initialize the CameraServer and Flask app
        self.app = Flask(__name__)
        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size = (640, 480)  # Set resolution
        self.picam2.preview_configuration.main.format = "RGB888"   # Set pixel format
        self.picam2.configure("preview")
        self.server_thread = None
        self.running = False

        # Define Flask routes
        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/', 'index', self.index)

    def generate_frames(self):
        # Capture frames from the camera and yield them as JPEG-encoded bytes
        while self.running:
            frame = self.picam2.capture_array()
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    def video_feed(self):
        # Video streaming route. Put this in the src attribute of an img tag
        return Response(self.generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    def index(self):
        # Home page displaying the video feed
        return '''
        <html>
          <head>
            <title>Raspberry Pi Camera Live Feed</title>
          </head>
          <body>
            <h1>Raspberry Pi Camera Live Feed</h1>
            <img src="/video_feed" width="640" height="480" />
          </body>
        </html>
        '''

    def run_server(self):
        # Start Flask webserver (this runs in a separate thread)
        self.running = True
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

    def start_server(self):
        # Start the Flask webserver in a separate thread
        if self.server_thread is None or not self.server_thread.is_alive():
            print("Starting Camera Webserver...")
            self.picam2.start()
            self.server_thread = threading.Thread(target=self.run_server, daemon=True)
            self.server_thread.start()
        else:
            print("Camera Webserver is already running.")

    def stop_server(self):
        # Stop the camera and terminate the server
        if self.running:
            print("Stopping Camera Webserver...")
            self.running = False
            self.picam2.stop()
        else:
            print("Camera Webserver is not running.")

# ============================================================================
# PCA9685 16-Channel PWM Servo Driver Class
# ============================================================================
class PCA9685:
    __SUBADR1   = 0x02
    __SUBADR2   = 0x03
    __SUBADR3   = 0x04
    __MODE1     = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L  = 0x06
    __LED0_ON_H  = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self, address=0x40, debug=False):
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug
        if self.debug:
            print("Resetting PCA9685")
        self.write(self.__MODE1, 0x00)
	
    def write(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value)
        if self.debug:
            print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))
	  
    def read(self, reg):
        result = self.bus.read_byte_data(self.address, reg)
        if self.debug:
            print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
        return result
	
    def setPWMFreq(self, freq):
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        if self.debug:
            print("Setting PWM frequency to %d Hz" % freq)
            print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        if self.debug:
            print("Final pre-scale: %d" % prescale)
        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10        # sleep
        self.write(self.__MODE1, newmode)        # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)
	
    def setPWM(self, channel, on, off):
        self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
        self.write(self.__LED0_ON_H+4*channel, on >> 8)
        self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
        self.write(self.__LED0_OFF_H+4*channel, off >> 8)
        if self.debug:
            print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel, on, off))
	  
    def setServoPulse(self, channel, pulse):
        pulse = int(pulse*4096/20000)      # PWM frequency is 50Hz; period is 20000us
        self.setPWM(channel, 0, pulse)

# ============================================================================
# TRSensor Class (Line Sensor)
# ============================================================================
class TRSensor:
    def __init__(self, numSensors=5):
        self.numSensors = numSensors
        self.calibratedMin = [0] * self.numSensors
        self.calibratedMax = [1023] * self.numSensors
        self.last_value = 0
        # Assume GPIO.setmode has been called by AlphaBot; only setup needed pins here.
        # Pin assignments for TRSensor hardware:
        self.CS = 5
        self.Clock = 25
        self.Address = 24
        self.DataOut = 23
        self.Button = 7

        GPIO.setup(self.Clock, GPIO.OUT)
        GPIO.setup(self.Address, GPIO.OUT)
        GPIO.setup(self.CS, GPIO.OUT)
        GPIO.setup(self.DataOut, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.Button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def AnalogRead(self):
        value = [0] * (self.numSensors + 1)
        for j in range(self.numSensors + 1):
            GPIO.output(self.CS, GPIO.LOW)
            for i in range(8):
                if i < 4:
                    GPIO.output(self.Address, GPIO.HIGH if ((j >> (3 - i)) & 0x01) else GPIO.LOW)
                else:
                    GPIO.output(self.Address, GPIO.LOW)
                value[j] <<= 1
                if GPIO.input(self.DataOut):
                    value[j] |= 0x01
                GPIO.output(self.Clock, GPIO.HIGH)
                GPIO.output(self.Clock, GPIO.LOW)
            for i in range(4):
                value[j] <<= 1
                if GPIO.input(self.DataOut):
                    value[j] |= 0x01
                GPIO.output(self.Clock, GPIO.HIGH)
                GPIO.output(self.Clock, GPIO.LOW)
            time.sleep(0.0001)
            GPIO.output(self.CS, GPIO.HIGH)
        return [v >> 2 for v in value[1:]]

    def calibrate(self):
        max_sensor_values = [0] * self.numSensors
        min_sensor_values = [1023] * self.numSensors
        for j in range(10):
            sensor_values = self.AnalogRead()
            for i in range(self.numSensors):
                max_sensor_values[i] = max(max_sensor_values[i], sensor_values[i])
                min_sensor_values[i] = min(min_sensor_values[i], sensor_values[i])
        for i in range(self.numSensors):
            self.calibratedMin[i] = max(self.calibratedMin[i], min_sensor_values[i])
            self.calibratedMax[i] = min(self.calibratedMax[i], max_sensor_values[i])

    def readLine(self, white_line=0):
        sensor_values = self.AnalogRead()
        avg = 0
        sum_values = 0
        on_line = False
        for i in range(self.numSensors):
            value = 1000 - sensor_values[i] if white_line else sensor_values[i]
            if value > 200:
                on_line = True
            if value > 50:
                avg += value * (i * 1000)
                sum_values += value
        if on_line:
            self.last_value = avg / sum_values
        else:
            if self.last_value < (self.numSensors - 1) * 1000 / 2:
                self.last_value = 0
            else:
                self.last_value = (self.numSensors - 1) * 1000
        return self.last_value, sensor_values

# ============================================================================
# ServoController Class
# ============================================================================
class ServoController:
    def __init__(self):
        self.pwm = PCA9685(0x40)
        self.pwm.setPWMFreq(50)  # Set PWM frequency to 50Hz

    def move_left(self):
        print("Moving servo left")
        self.pwm.setServoPulse(0, 500)
        time.sleep(1)

    def move_right(self):
        print("Moving servo right")
        self.pwm.setServoPulse(0, 2500)
        time.sleep(1)

    def center(self):
        print("Centering servo")
        self.pwm.setServoPulse(0, 1500)
        time.sleep(1)

    def move_down(self):
        print("Moving servo down")
        self.pwm.setServoPulse(1, 2500)
        time.sleep(1)

    def move_up(self):
        print("Moving servo up")
        self.pwm.setServoPulse(1, 500)
        time.sleep(1)

    def middle(self):
        print("Centering servo")
        self.pwm.setServoPulse(1, 1500)
        time.sleep(1)

    def stop(self):
        self.pwm.setPWM(0, 0, 0)
        self.pwm.setPWM(1, 0, 0)
        print("Servos stopped and PWM signals disabled")

# ============================================================================
# AlphaBot Class
# ============================================================================
class AlphaBot(object):
    def __init__(self, ain1=12, ain2=13, ena=6, bin1=20, bin2=21, enb=26, dr=16, dl=19):
        self.AIN1 = ain1
        self.AIN2 = ain2
        self.BIN1 = bin1
        self.BIN2 = bin2
        self.ENA = ena
        self.ENB = enb
        self.PA = 50
        self.PB = 50
        self.integral = 0
        self.last_proportional = 0
        self.maximum = 35
        self.DR = dr
        self.DL = dl
        self.Button = 7
        self.Buzzer = 4

        # Initialize GPIO only once in AlphaBot
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Setup pins for motor control and additional components
        motor_pins = [self.AIN1, self.AIN2, self.BIN1, self.BIN2, self.ENA, self.ENB]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)

        # Buzzer
        GPIO.setup(self.Buzzer, GPIO.OUT)
        
        # Button
        GPIO.setup(self.Button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Infrared sensors
        GPIO.setup(self.DR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.DL, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize PWM for motor control
        self.PWMA = GPIO.PWM(self.ENA, 500)
        self.PWMB = GPIO.PWM(self.ENB, 500)
        self.PWMA.start(self.PA)
        self.PWMB.start(self.PB)

        # Stop the motors initially
        self.stop()

        # Initialize TRSensor, ServoController, and the integrated CameraServer
        self.tr_sensor = TRSensor()
        self.servo = ServoController()
        self.camera_server = CameraServer()

    # Motor control methods...
    def forward(self):
        self.PWMA.ChangeDutyCycle(self.PA)
        self.PWMB.ChangeDutyCycle(self.PB)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)

    def stop(self):
        self.PWMA.ChangeDutyCycle(0)
        self.PWMB.ChangeDutyCycle(0)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.LOW)

    def backward(self):
        self.PWMA.ChangeDutyCycle(self.PA)
        self.PWMB.ChangeDutyCycle(self.PB)
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.HIGH)
        GPIO.output(self.BIN2, GPIO.LOW)

    def left(self):
        self.PWMA.ChangeDutyCycle(30)
        self.PWMB.ChangeDutyCycle(30)
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        GPIO.output(self.BIN1, GPIO.LOW)
        GPIO.output(self.BIN2, GPIO.HIGH)

    def right(self):
        self.PWMA.ChangeDutyCycle(30)
        self.PWMB.ChangeDutyCycle(30)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        GPIO.output(self.BIN1, GPIO.HIGH)
        GPIO.output(self.BIN2, GPIO.LOW)

    def setPWMA(self, value):
        self.PA = value
        self.PWMA.ChangeDutyCycle(self.PA)

    def setPWMB(self, value):
        self.PB = value
        self.PWMB.ChangeDutyCycle(self.PB)

    def setMotor(self, left, right):
        if right >= 0:
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
            self.PWMA.ChangeDutyCycle(right)
        else:
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
            self.PWMA.ChangeDutyCycle(-right)
        if left >= 0:
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
            self.PWMB.ChangeDutyCycle(left)
        else:
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)
            self.PWMB.ChangeDutyCycle(-left)

    def calibrate_sensors(self):
        print("Calibrating sensors...")
        for i in range(100):
            if i < 25 or i >= 75:
                self.right()
            else:
                self.left()
            self.setPWMA(30)
            self.setPWMB(30)
            self.tr_sensor.calibrate()
        self.stop()
        print(self.tr_sensor.calibratedMin)
        print(self.tr_sensor.calibratedMax)

    def start_line_follow(self):
        print("Starting line following...")
        self.line_following = True  # Set flag to True to start
        try:
            while self.line_following:
                position, sensors = self.tr_sensor.readLine()
                if all(sensor > 900 for sensor in sensors):
                    self.setPWMA(0)
                    self.setPWMB(0)
                else:
                    proportional = position - 2000
                    derivative = proportional - self.last_proportional
                    self.integral += proportional
                    self.last_proportional = proportional
                    power_difference = proportional / 30 + self.integral / 10000 + derivative * 2
                    power_difference = max(min(power_difference, self.maximum), -self.maximum)

                    if power_difference < 0:
                        self.setPWMA(self.maximum + power_difference)
                        self.setPWMB(self.maximum)
                    else:
                        self.setPWMA(self.maximum)
                        self.setPWMB(self.maximum - power_difference)
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.stop()

    def stop_line_follow(self):
        self.line_following = False
        self.stop()

    def infrared_obstacle_check(self):
        DR_status = GPIO.input(self.DR)
        DL_status = GPIO.input(self.DL)
        return DL_status == 0 or DR_status == 0

    def buzzer_on(self):
        GPIO.output(self.Buzzer, GPIO.HIGH)

    def buzzer_off(self):
        GPIO.output(self.Buzzer, GPIO.LOW)

    def start_camera(self):
        #Start the camera webserver
        self.camera_server.start_server()

    def stop_camera(self):
        #Stop the camera webserver
        self.camera_server.stop_server()

# ============================================================================
# Main Section for Testing
# ============================================================================
if __name__ == '__main__':
    bot = AlphaBot()
    try:
        # Example motor control
        bot.forward()
        time.sleep(2)
        bot.stop()

        # Start the camera server
        bot.start_camera()
        print("Camera server started. Visit http://<your_pi_ip>:5000/ in your browser.")
        time.sleep(60)  # Keep the camera running for 60 seconds

        # Stop the camera server
        bot.stop_camera()
        print("Camera server stopped.")

    except KeyboardInterrupt:
        bot.stop()
        bot.stop_camera()
        GPIO.cleanup()