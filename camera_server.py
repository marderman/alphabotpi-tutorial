from flask import Flask, Response
import cv2
from picamera2 import Picamera2

app = Flask(__name__)

# Initialize and configure the camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)  # Set resolution
picam2.preview_configuration.main.format = "RGB888"    # Set pixel format
picam2.configure("preview")
picam2.start()

def generate_frames():
    """Capture frames from the camera and yield them as JPEG-encoded bytes."""
    while True:
        # Capture a frame from the camera
        frame = picam2.capture_array()

        # Encode the frame in JPEG format
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret:
            continue

        # Convert to bytes and yield in a multipart response format
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    """Home page that displays the video feed."""
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

if __name__ == '__main__':
    # Run the Flask app on all interfaces so it can be accessed via the Raspberry Pi's IP address
    app.run(host='0.0.0.0', port=5000, debug=False)
