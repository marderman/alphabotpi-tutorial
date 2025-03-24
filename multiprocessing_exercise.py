import multiprocessing
import time
from AlphaBot import AlphaBot
import RPi.GPIO as GPIO
from TRSensors import TRSensor

class RobotScheduler:
    def __init__(self):
        self.bot = AlphaBot()
        self.running = multiprocessing.Value('b', True)  # Shared boolean flag for all tasks

    def move_forward_task(self, running):
        """Moves the robot forward continuously."""
        while running.value:
            self.bot.forward()
            time.sleep(1)

    def obstacle_avoidance_task(self, running):
        """Avoids obstacles using ultrasonic sensors."""
        while running.value:
            distance = self.get_ultrasonic_distance()
            if distance < 20:
                print("Obstacle detected! Turning right...")
                self.bot.right()
                time.sleep(0.5)
                self.bot.forward()
            time.sleep(0.1)

    def line_follow_task(self, running, TR):
        """Follows a line using the sensor module."""
        while running.value:
            position, Sensors = TR.readLine()
            print(f"Line Position: {position}, Sensors: {Sensors}")
            time.sleep(0.1)

    def get_ultrasonic_distance(self, TRIG=22, ECHO=27):
        """Measures distance using ultrasonic sensors."""
        GPIO.setup(TRIG, GPIO.OUT)
        GPIO.setup(ECHO, GPIO.IN)

        GPIO.output(TRIG, GPIO.HIGH)
        time.sleep(0.000015)
        GPIO.output(TRIG, GPIO.LOW)

        while not GPIO.input(ECHO):
            pass
        t1 = time.time()
        while GPIO.input(ECHO):
            pass
        t2 = time.time()

        return (t2 - t1) * 34000 / 2

    def start_scheduler(self, TR):
        """Starts all tasks in parallel using multiprocessing."""
        print("Starting Robot Scheduler with multiprocessing...")

        self.processes = [
            multiprocessing.Process(target=self.move_forward_task, args=(self.running,)),
            multiprocessing.Process(target=self.obstacle_avoidance_task, args=(self.running,)),
            multiprocessing.Process(target=self.line_follow_task, args=(self.running, TR))
        ]

        for process in self.processes:
            process.start()

    def stop_scheduler(self):
        """Stops all processes."""
        print("Stopping Robot Scheduler...")
        self.running.value = False  # Signal processes to stop
        for process in self.processes:
            process.terminate()
            process.join()
        self.bot.stop()
        print("All processes stopped.")

if __name__ == "__main__":
    scheduler = RobotScheduler()

    try:
        TR = TRSensor()
        scheduler.start_scheduler(TR)

        while True:
            time.sleep(1)  # Keep the main program alive

    except KeyboardInterrupt:
        scheduler.stop_scheduler()
        print("Scheduler stopped.")
