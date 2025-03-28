import threading
import time
import cv2
import RPi.GPIO as GPIO
from flask import Flask, Response
from picamera2 import Picamera2

class CameraServer:
    def __init__(self):
        #Initialize the CameraServer and Flask app
        self.app = Flask(__name__)
        self.picam2 = Picamera2()
        self.picam2.preview_configuration.main.size = (640, 480)  # Set resolution
        self.picam2.preview_configuration.main.format = "RGB888"  # Set pixel format
        self.picam2.configure("preview")
        self.server_thread = None
        self.running = False

        # Define Flask routes
        self.app.add_url_rule('/video_feed', 'video_feed', self.video_feed)
        self.app.add_url_rule('/', 'index', self.index)

    def generate_frames(self):
        #Capture frames from the camera and yield them as JPEG-encoded bytes
        while self.running:
            frame = self.picam2.capture_array()
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

    def video_feed(self):
        #Video streaming route. Put this in the src attribute of an img tag
        return Response(self.generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    def index(self):
        #Home page displaying the video feed
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
        #Start Flask webserver (this runs in a separate thread)
        self.running = True
        self.app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

    def start_server(self):
        #Start the Flask webserver in a separate thread
        if self.server_thread is None or not self.server_thread.is_alive():
            print("Starting Camera Webserver...")
            self.picam2.start()
            self.server_thread = threading.Thread(target=self.run_server, daemon=True)
            self.server_thread.start()
        else:
            print("Camera Webserver is already running.")

    def stop_server(self):
        #Stop the camera and terminate the server
        if self.running:
            print("Stopping Camera Webserver...")
            self.running = False
            self.picam2.stop()
        else:
            print("Camera Webserver is not running.")

if __name__ == "__main__":
    cam_server = CameraServer()

    try:
        cam_server.start_server()  # Start the webserver
        while True:
            print("Webserver running. Press Ctrl+C to stop.")
            time.sleep(5)

    except KeyboardInterrupt:
        print("Shutting down...")
        cam_server.stop_server()