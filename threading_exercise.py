import threading
import time
from AlphaBot import AlphaBot
import RPi.GPIO as GPIO
from TRSensors import TRSensor


class RobotScheduler:
    def __init__(self):
        self.bot = AlphaBot()
        self.running_tasks = {}
        self.lock = threading.Lock()

    def move_forward_task(self):
        """Moves the robot forward continuously."""
        while self.running_tasks.get("move_forward", False):
            self.bot.forward()
            time.sleep(1)

    def obstacle_avoidance_task(self):
        """Avoids obstacles using ultrasonic sensors."""
        while self.running_tasks.get("obstacle_avoidance", False):
            distance = self.get_ultrasonic_distance()
            if distance < 20:
                print("Obstacle detected! Turning right...")
                self.bot.right()
                time.sleep(0.5)
                self.bot.forward()
            time.sleep(0.1)

    def line_follow_task(self, TR):
        """Follows a line using the sensor module."""
        while self.running_tasks.get("line_follow", False):
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

    def start_task(self, task_name, target, *args):
        """Starts a new task in a separate thread."""
        with self.lock:
            if task_name not in self.running_tasks or not self.running_tasks[task_name]:
                self.running_tasks[task_name] = True
                thread = threading.Thread(target=target, args=args)
                thread.daemon = True
                self.running_tasks[task_name] = thread
                thread.start()
                print(f"Started task: {task_name}")

    def stop_task(self, task_name):
        """Stops a running task."""
        with self.lock:
            if task_name in self.running_tasks and self.running_tasks[task_name]:
                self.running_tasks[task_name] = False
                print(f"Stopping task: {task_name}")

    def stop_all_tasks(self):
        """Stops all tasks and halts the robot."""
        with self.lock:
            for task in list(self.running_tasks.keys()):
                self.running_tasks[task] = False
            self.bot.stop()
            print("All tasks stopped.")

if __name__ == "__main__":
    scheduler = RobotScheduler()
    TR = TRSensor()

    try:
        scheduler.start_task("move_forward", scheduler.move_forward_task)
        time.sleep(3)  # Let it run for 3 seconds

        scheduler.start_task("obstacle_avoidance", scheduler.obstacle_avoidance_task)
        time.sleep(5)  # Let it run for 5 seconds

        # Stop movement and switch to line-following
        scheduler.stop_task("move_forward")
        scheduler.start_task("line_follow", scheduler.line_follow_task, TR)

        time.sleep(10)  # Run for 10 seconds before stopping everything

    except KeyboardInterrupt:
        scheduler.stop_all_tasks()
        print("Scheduler stopped.")
