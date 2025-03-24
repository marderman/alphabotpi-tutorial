import time
import RPi.GPIO as GPIO
from TRSensors import TRSensor
from PCA9685 import PCA9685

class AlphaBot(object):

    def __init__(self, ain1=12, ain2=13, ena=6, bin1=20, bin2=21, enb=26):
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
        self.Button = 7
        self.Buzzer = 4

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.BIN1, GPIO.OUT)
        GPIO.setup(self.BIN2, GPIO.OUT)
        GPIO.setup(self.ENA, GPIO.OUT)
        GPIO.setup(self.ENB, GPIO.OUT)
        GPIO.setup(self.Buzzer, GPIO.OUT)
        GPIO.setup(self.Button, GPIO.IN, GPIO.PUD_UP)

        self.PWMA = GPIO.PWM(self.ENA, 500)
        self.PWMB = GPIO.PWM(self.ENB, 500)
        self.PWMA.start(self.PA)
        self.PWMB.start(self.PB)
        self.stop()

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

    # Individually control motors using values from -100 (full power backward) to 100 (full power forward)
    def setMotor(self, left, right):
        # Forward at half speed setMotor(50,50)
        # Backward at full speed setMotor(-100, -100)
        # Turn left setMotor(-50, 50)
        # Stop setMotor(0, 0)
        if((right >= 0) and (right <= 100)):
            GPIO.output(self.AIN1, GPIO.HIGH)
            GPIO.output(self.AIN2, GPIO.LOW)
            self.PWMA.ChangeDutyCycle(right)
        elif((right < 0) and (right >= -100)):
            GPIO.output(self.AIN1, GPIO.LOW)
            GPIO.output(self.AIN2, GPIO.HIGH)
            self.PWMA.ChangeDutyCycle(0 - right)
        if((left >= 0) and (left <= 100)):
            GPIO.output(self.BIN1, GPIO.HIGH)
            GPIO.output(self.BIN2, GPIO.LOW)
            self.PWMB.ChangeDutyCycle(left)
        elif((left < 0) and (left >= -100)):
            GPIO.output(self.BIN1, GPIO.LOW)
            GPIO.output(self.BIN2, GPIO.HIGH)
            self.PWMB.ChangeDutyCycle(0 - left)

    def calibrate_sensors(self, TR):
        print("Calibrating sensors...")
        for i in range(100):
            if i < 25 or i >= 75:
                self.right()
                self.setPWMA(30)
                self.setPWMB(30)
            else:
                self.left()
                self.setPWMA(30)
                self.setPWMB(30)
            TR.calibrate()
        self.stop()
        print(TR.calibratedMin)
        print(TR.calibratedMax)

    def line_follow(self, TR):
        print("Starting line following...")
        while GPIO.input(self.Button) != 0:
            position, Sensors = TR.readLine()
            print(position, Sensors)
            time.sleep(0.05)
        self.forward()

        while True:
            try:
                position, Sensors = TR.readLine()
                if all(sensor > 900 for sensor in Sensors):
                    self.setPWMA(0)
                    self.setPWMB(0)
                else:
                    proportional = position - 2000
                    derivative = proportional - self.last_proportional
                    self.integral += proportional
                    self.last_proportional = proportional
                    power_difference = proportional / 30 + self.integral / 10000 + derivative * 2
                    
                    power_difference = max(min(power_difference, self.maximum), -self.maximum)
                    print(position, power_difference)
                    
                    if power_difference < 0:
                        self.setPWMA(self.maximum + power_difference)
                        self.setPWMB(self.maximum)
                    else:
                        self.setPWMA(self.maximum)
                        self.setPWMB(self.maximum - power_difference)
            except KeyboardInterrupt:
                break

    def infrared_obstacle_avoidance(self, DR=16, DL=19):
        GPIO.setup(DR, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(DL, GPIO.IN, GPIO.PUD_UP)
        try:
            while True:
                DR_status = GPIO.input(DR)
                DL_status = GPIO.input(DL)
                if DL_status == 0 or DR_status == 0:
                    self.stop()
                else:
                    self.forward()
        except KeyboardInterrupt:
            GPIO.cleanup()

    def buzzer_on():
	    GPIO.output(4,GPIO.HIGH)
    
    def buzzer_off():
	    GPIO.output(4,GPIO.LOW)

class ServoController:
    # Servochannel
    def __init__(self):
        self.pwm = PCA9685(0x40)
        self.pwm.setPWMFreq(50)  # Set PWM frequency to 50Hz

    def move_left(self):
        print("Moving servo left")
        self.pwm.setServoPulse(0, 500)  # Move to leftmost position
        time.sleep(1)

    def move_right(self):
        print("Moving servo right")
        self.pwm.setServoPulse(0, 2500)  # Move to rightmost position
        time.sleep(1)

    def center(self):
        print("Centering servo")
        self.pwm.setServoPulse(0, 1500)  # Center position
        time.sleep(1)

    def move_down(self):
        print("Moving servo down")
        self.pwm.setServoPulse(1, 500)  # Move to leftmost position
        time.sleep(1)

    def move_up(self):
        print("Moving servo up")
        self.pwm.setServoPulse(1, 2500)  # Move to rightmost position
        time.sleep(1)
    
    def middle(self):
        print("Centering servo")
        self.pwm.setServoPulse(1, 1500)  # Center position
        time.sleep(1)