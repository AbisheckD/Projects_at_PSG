from picamera2 import Picamera2
import cv2
import dlib
import RPi.GPIO as GPIO
from scipy.spatial import distance
import time
import numpy as np
from datetime import datetime

# GPIO for buzzer and LED
BUZZER_PIN = 18
LED_PIN = 17  # GPIO pin for LED

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)

# Thresholds
EYE_AR_THRESH = 0.30
EYE_AR_CONSEC_FRAMES = 20
DISTRACTION_THRESH = 10  # degrees
DISTRACTION_CONSEC_FRAMES = 5

# Logging
log_file = open("/home/abisheck/project_linux_demo/sd_log.txt", "w")
start_time = time.time()

# EAR calculation
def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    return (A + B) / (2.0 * C)

# Head pose estimation (yaw)
def head_pose_estimation(shape):
    image_points = np.array([
        shape[30], shape[8], shape[36],
        shape[45], shape[48], shape[54]
    ], dtype="double")

    model_points = np.array([
        (0.0, 0.0, 0.0), (0.0, -330.0, -65.0),
        (-30.0, -60.0, -60.0), (30.0, -60.0, -60.0),
        (-30.0, 60.0, -60.0), (30.0, 60.0, -60.0)
    ])

    focal_length = 640
    center = (320, 240)
    camera_matrix = np.array([
        [focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]
    ], dtype="double")

    dist_coeffs = np.zeros((4, 1))
    success, rotation_vector, translation_vector = cv2.solvePnP(
        model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return np.degrees(yaw)

# Dlib setup
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
LEFT_EYE = list(range(36, 42))
RIGHT_EYE = list(range(42, 48))

# Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
time.sleep(2)

frame_counter = 0
distraction_counter = 0
buzzer_on = False
led_on = False
drowsy_start_time = None  # for slowing down time

try:
    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray)
        status = "NORMAL"

        for face in faces:
            shape = predictor(gray, face)
            shape = [(shape.part(i).x, shape.part(i).y) for i in range(68)]

            left_eye = [shape[i] for i in LEFT_EYE]
            right_eye = [shape[i] for i in RIGHT_EYE]
            ear = (eye_aspect_ratio(left_eye) + eye_aspect_ratio(right_eye)) / 2.0
            yaw_angle = head_pose_estimation(shape)

            print(f"\033[92m[INFO] EAR: {ear:.2f} | Yaw: {yaw_angle:.2f}°\033[0m")

            # Drowsiness detection
            if ear < EYE_AR_THRESH:
                frame_counter += 1
                if frame_counter >= EYE_AR_CONSEC_FRAMES:
                    if drowsy_start_time is None:
                        drowsy_start_time = time.time()
                    status = "DROWSY"
                    print(f"\033[91m[ALERT] DROWSY detected! EAR: {ear:.2f}\033[0m")
            else:
                frame_counter = 0
                drowsy_start_time = None

            # Distraction detection
            if abs(yaw_angle) > DISTRACTION_THRESH:
                distraction_counter += 1
                if distraction_counter >= DISTRACTION_CONSEC_FRAMES:
                    status = "DISTRACTED"
                    print(f"\033[93m[ALERT] DISTRACTED detected! Yaw: {yaw_angle:.2f}°\033[0m")
            else:
                distraction_counter = 0

            # Buzzer and LED logic
            if status in ["DROWSY", "DISTRACTED"]:
                if not buzzer_on:
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)
                    buzzer_on = True
                if not led_on:
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    led_on = True
            else:
                if buzzer_on:
                    GPIO.output(BUZZER_PIN, GPIO.LOW)
                    buzzer_on = False
                if led_on:
                    GPIO.output(LED_PIN, GPIO.LOW)
                    led_on = False

            # Head-up Display (HUD)
            hud_frame = np.zeros((200, 640, 3), dtype=np.uint8)
            font = cv2.FONT_HERSHEY_SIMPLEX
            speed = 60  # hardcoded for now

            cv2.putText(hud_frame, f"STATUS: {status}", (20, 40), font, 1.2, (0, 0, 255) if status != "NORMAL" else (0, 255, 0), 3)
            cv2.putText(hud_frame, f"Speed: {speed} kmph", (20, 80), font, 1, (255, 255, 255), 2)
            cv2.putText(hud_frame, f"EAR: {ear:.2f}", (20, 120), font, 1, (255, 255, 255), 2)
            cv2.putText(hud_frame, f"YAW: {yaw_angle:.2f}°", (20, 160), font, 1, (255, 255, 255), 2)

            if status == "DROWSY" and drowsy_start_time:
                elapsed = int(time.time() - drowsy_start_time)
                slowdown = max(0, 10 - elapsed)
                cv2.putText(hud_frame, f"Slowing in: {slowdown}s", (350, 160), font, 1, (0, 0, 255), 2)

            combined = np.vstack((hud_frame, frame))

            # Log if needed
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if time.time() - start_time <= 60 or status in ["DROWSY", "DISTRACTED"]:
                log_file.write(f"{timestamp}, Status: {status}, Yaw: {yaw_angle:.2f}, EAR: {ear:.2f}\n")
                log_file.flush()

            cv2.imshow("Driver Monitor HUD", combined)
            break  # Process one face only for now

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()
    log_file.close()
    picam2.close()
    cv2.destroyAllWindows()
    print("Log saved and cleanup done.")from picamera2 import Picamera2
import cv2
import dlib
import RPi.GPIO as GPIO
from scipy.spatial import distance
import time
import numpy as np
from datetime import datetime

# GPIO for buzzer and LED
BUZZER_PIN = 18
LED_PIN = 17  # GPIO pin for LED

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)

# Thresholds
EYE_AR_THRESH = 0.30
EYE_AR_CONSEC_FRAMES = 20
DISTRACTION_THRESH = 10  # degrees
DISTRACTION_CONSEC_FRAMES = 5

# Logging
log_file = open("/home/abisheck/project_linux_demo/sd_log.txt", "w")
start_time = time.time()

# EAR calculation
def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    return (A + B) / (2.0 * C)

# Head pose estimation (yaw)
def head_pose_estimation(shape):
    image_points = np.array([
        shape[30], shape[8], shape[36],
        shape[45], shape[48], shape[54]
    ], dtype="double")

    model_points = np.array([
        (0.0, 0.0, 0.0), (0.0, -330.0, -65.0),
        (-30.0, -60.0, -60.0), (30.0, -60.0, -60.0),
        (-30.0, 60.0, -60.0), (30.0, 60.0, -60.0)
    ])

    focal_length = 640
    center = (320, 240)
    camera_matrix = np.array([
        [focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]
    ], dtype="double")

    dist_coeffs = np.zeros((4, 1))
    success, rotation_vector, translation_vector = cv2.solvePnP(
        model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return np.degrees(yaw)

# Dlib setup
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
LEFT_EYE = list(range(36, 42))
RIGHT_EYE = list(range(42, 48))

# Picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
time.sleep(2)

frame_counter = 0
distraction_counter = 0
buzzer_on = False
led_on = False
drowsy_start_time = None  # for slowing down time

try:
    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = detector(gray)
        status = "NORMAL"

        for face in faces:
            shape = predictor(gray, face)
            shape = [(shape.part(i).x, shape.part(i).y) for i in range(68)]

            left_eye = [shape[i] for i in LEFT_EYE]
            right_eye = [shape[i] for i in RIGHT_EYE]
            ear = (eye_aspect_ratio(left_eye) + eye_aspect_ratio(right_eye)) / 2.0
            yaw_angle = head_pose_estimation(shape)

            print(f"\033[92m[INFO] EAR: {ear:.2f} | Yaw: {yaw_angle:.2f}°\033[0m")

            # Drowsiness detection
            if ear < EYE_AR_THRESH:
                frame_counter += 1
                if frame_counter >= EYE_AR_CONSEC_FRAMES:
                    if drowsy_start_time is None:
                        drowsy_start_time = time.time()
                    status = "DROWSY"
                    print(f"\033[91m[ALERT] DROWSY detected! EAR: {ear:.2f}\033[0m")
            else:
                frame_counter = 0
                drowsy_start_time = None

            # Distraction detection
            if abs(yaw_angle) > DISTRACTION_THRESH:
                distraction_counter += 1
                if distraction_counter >= DISTRACTION_CONSEC_FRAMES:
                    status = "DISTRACTED"
                    print(f"\033[93m[ALERT] DISTRACTED detected! Yaw: {yaw_angle:.2f}°\033[0m")
            else:
                distraction_counter = 0

            # Buzzer and LED logic
            if status in ["DROWSY", "DISTRACTED"]:
                if not buzzer_on:
                    GPIO.output(BUZZER_PIN, GPIO.HIGH)
                    buzzer_on = True
                if not led_on:
                    GPIO.output(LED_PIN, GPIO.HIGH)
                    led_on = True
            else:
                if buzzer_on:
                    GPIO.output(BUZZER_PIN, GPIO.LOW)
                    buzzer_on = False
                if led_on:
                    GPIO.output(LED_PIN, GPIO.LOW)
                    led_on = False

            # Head-up Display (HUD)
            hud_frame = np.zeros((200, 640, 3), dtype=np.uint8)
            font = cv2.FONT_HERSHEY_SIMPLEX
            speed = 60  # hardcoded for now

            cv2.putText(hud_frame, f"STATUS: {status}", (20, 40), font, 1.2, (0, 0, 255) if status != "NORMAL" else (0, 255, 0), 3)
            cv2.putText(hud_frame, f"Speed: {speed} kmph", (20, 80), font, 1, (255, 255, 255), 2)
            cv2.putText(hud_frame, f"EAR: {ear:.2f}", (20, 120), font, 1, (255, 255, 255), 2)
            cv2.putText(hud_frame, f"YAW: {yaw_angle:.2f}°", (20, 160), font, 1, (255, 255, 255), 2)

            if status == "DROWSY" and drowsy_start_time:
                elapsed = int(time.time() - drowsy_start_time)
                slowdown = max(0, 10 - elapsed)
                cv2.putText(hud_frame, f"Slowing in: {slowdown}s", (350, 160), font, 1, (0, 0, 255), 2)

            combined = np.vstack((hud_frame, frame))

            # Log if needed
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            if time.time() - start_time <= 60 or status in ["DROWSY", "DISTRACTED"]:
                log_file.write(f"{timestamp}, Status: {status}, Yaw: {yaw_angle:.2f}, EAR: {ear:.2f}\n")
                log_file.flush()

            cv2.imshow("Driver Monitor HUD", combined)
            break  # Process one face only for now

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()
    log_file.close()
    picam2.close()
    cv2.destroyAllWindows()
    print("Log saved and cleanup done.")
