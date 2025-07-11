from picamera2 import Picamera2
import cv2
import dlib
import RPi.GPIO as GPIO
from scipy.spatial import distance
import time
import numpy as np

# Buzzer setup (GPIO 18)
BUZZER_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# EAR Calculation
def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    return (A + B) / (2.0 * C)

# Head pose estimation
def head_pose_estimation(shape):
    # 2D image points for nose, eyes, and chin
    image_points = np.array([
        shape[30],  # Nose tip
        shape[36],  # Left eye corner
        shape[45],  # Right eye corner
        shape[48],  # Left mouth corner
        shape[54]   # Right mouth corner
    ], dtype="double")

    # 3D model points
    model_points = np.array([
        (0.0, 0.0, 0.0),       # Nose tip
        (-30.0, -60.0, -60.0), # Left eye corner
        (30.0, -60.0, -60.0),  # Right eye corner
        (-30.0, 60.0, 0.0),    # Left mouth corner
        (30.0, 60.0, 0.0)      # Right mouth corner
    ])

    # Camera matrix (assuming a simple camera setup)
    focal_length = 640  # Use width of image for focal length approximation
    center = (320, 240) # Approx center of image
    camera_matrix = np.array([[focal_length, 0, center[0]],
                              [0, focal_length, center[1]],
                              [0, 0, 1]], dtype="double")

    # Distortion coefficients (assuming no lens distortion)
    dist_coeffs = np.zeros((4, 1))

    # Solve PnP to get rotation and translation vectors
    _, rotation_vector, translation_vector = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)

    # Project 3D points onto the 2D image plane
    axis = np.float32([[50, 0, 0], [0, 50, 0], [0, 0, 50]])  # X, Y, Z axis
    img_points, _ = cv2.projectPoints(axis, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

    return rotation_vector, img_points

# Load models
print("â³ Loading models...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Eye indexes
LEFT_EYE = list(range(36, 42))
RIGHT_EYE = list(range(42, 48))

# Start PiCamera2
print("ð· Starting camera...")
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
time.sleep(2)  # give time to warm up

print("â Camera started. Press Ctrl+C to exit.")

# Drowsiness and Distraction parameters
frame_counter = 0
EYE_AR_THRESH = 0.30  # Adjusted threshold for better accuracy
EYE_AR_CONSEC_FRAMES = 20  # Increased frames for better alert detection
DISTRACTION_THRESH = 20  # Distraction angle threshold in degrees
DISTRACTION_CONSEC_FRAMES = 10  # Number of frames to trigger distraction alert

# Head pose detection counters
distraction_counter = 0

try:
    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray)

        for face in faces:
            shape = predictor(gray, face)
            shape = [(shape.part(i).x, shape.part(i).y) for i in range(68)]

            # Eye aspect ratio calculation
            left_eye = [shape[i] for i in LEFT_EYE]
            right_eye = [shape[i] for i in RIGHT_EYE]

            left_ear = eye_aspect_ratio(left_eye)
            right_ear = eye_aspect_ratio(right_eye)
            ear = (left_ear + right_ear) / 2.0

            # Check drowsiness
            if ear < EYE_AR_THRESH:
                frame_counter += 1
                if frame_counter >= EYE_AR_CONSEC_FRAMES:
                    cv2.putText(frame, "DROWSINESS ALERT!", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)
            else:
                frame_counter = 0
                GPIO.output(BUZZER_PIN, GPIO.LOW)

            # Head pose estimation for distraction
            rotation_vector, img_points = head_pose_estimation(shape)

            # Calculate the yaw angle (rotation around the Y-axis)
            yaw = rotation_vector[0][1]

            # Convert yaw from radians to degrees
            yaw_deg = np.degrees(yaw)

            # Check if yaw angle exceeds distraction threshold
            if abs(yaw_deg) > DISTRACTION_THRESH:
                distraction_counter += 1
                if distraction_counter >= DISTRACTION_CONSEC_FRAMES:
                    cv2.putText(frame, "DISTRACTION ALERT!", (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)
            else:
                distraction_counter = 0

        cv2.imshow("Driver Monitor", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\n Interrupted by user.")

finally:
    cv2.destroyAllWindows()
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    GPIO.cleanup()
    picam2.close()
    print("Program exited cleanly.")
