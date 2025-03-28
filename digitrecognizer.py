import cv2
import numpy as np
import pickle
import time
from picamera2 import Picamera2

# Load the trained model
with open("digit_classifier.pkl", "rb") as f:
    clf = pickle.load(f)

# Initialize the PiCamera2
picam2 = Picamera2()
# Configure for still capture at 640x480 resolution
config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(0.5)  # Allow the camera to warm up

# Capture an image using PiCamera2
image = picam2.capture_array()

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# Apply Gaussian blur to reduce noise
blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Use adaptive thresholding for more robust binarization
thresh = cv2.adaptiveThreshold(
    blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    cv2.THRESH_BINARY_INV, 11, 2)

# Apply morphological closing to remove small holes and connect parts of the digit
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
morph = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

# Find contours in the processed image
contours, _ = cv2.findContours(morph.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
if len(contours) == 0:
    print("No digit found in the image.")
    exit(0)

# Assume the largest contour corresponds to the digit
c = max(contours, key=cv2.contourArea)
x, y, w, h = cv2.boundingRect(c)
roi = morph[y:y+h, x:x+w]

# Optional: Add a border to the ROI if the digit is near the edge
roi = cv2.copyMakeBorder(roi, 5, 5, 5, 5, cv2.BORDER_CONSTANT, value=[0, 0, 0])

# Resize ROI to 8x8 pixels to match the digits dataset dimensions
roi_resized = cv2.resize(roi, (8, 8), interpolation=cv2.INTER_AREA)

# Normalize pixel values to match training scale (0-16)
roi_resized = (roi_resized.astype(np.float32) / 255.0) * 16
roi_flatten = roi_resized.flatten().reshape(1, -1)

# Use the trained classifier to predict the digit
prediction = clf.predict(roi_flatten)
print("Predicted digit:", prediction[0])
