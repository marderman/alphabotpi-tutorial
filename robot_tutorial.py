import multiprocessing
import time
import os
import cv2
from AlphaBot import AlphaBot  # Import the AlphaBot class if available

# Robot initialization
bot = AlphaBot()

def follow_line():
    """Simulates line-following behavior."""
    while True:
        print("[Line Following] Following the line...")
        bot.forward()  # Move forward
        time.sleep(0.5)

def turn():
    """Simulates turning behavior (e.g., when detecting an obstacle)."""
    while True:
        print("[Turning] Turning left...")
        bot.left()  # Turn left
        time.sleep(2)
        print("[Turning] Resuming normal movement...")

def process_camera():
    """Simulates capturing frames and processing them."""
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if ret:
            print("[Camera] Processing Frame...")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Simulating image processing
        time.sleep(1)  # Simulate processing time

def monitor_cpu():
    """Monitors CPU usage to demonstrate overload."""
    while True:
        print(f"[CPU Monitor] CPU Usage: {os.cpu_count()} cores")
        time.sleep(2)

def cpu_stress():
    """Performs CPU-intensive calculations to keep the core busy."""
    x = 0
    while True:
        x += 1  # Simulate some work
        x = x ** 2 % 1234567  # Prevent optimizations

def scheduler():
    """Manages execution of tasks while ensuring full CPU utilization."""
    num_cores = os.cpu_count()

    # Start normal robot tasks
    processes = {
        "line_following": multiprocessing.Process(target=follow_line),
        "turning": multiprocessing.Process(target=turn),
        "camera": multiprocessing.Process(target=process_camera),
        "cpu_monitor": multiprocessing.Process(target=monitor_cpu)
    }

    # Start all processes
    for key, process in processes.items():
        process.start()

    # Now, stress all CPU cores
    stress_processes = []
    for _ in range(num_cores):
        p = multiprocessing.Process(target=cpu_stress)
        p.start()
        stress_processes.append(p)

    print(f"All {num_cores} CPU cores are now fully utilized.")

    # Run for some time
    time.sleep(15)

    # Stop all processes
    for key, process in processes.items():
        process.terminate()
        process.join()

    for p in stress_processes:
        p.terminate()
        p.join()

if __name__ == "__main__":
    scheduler()

