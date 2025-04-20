import cv2
import pickle
import numpy as np
import mediapipe as mp
import os
import time
import socket
import json

# --- CONFIGURATION ---
# IP‑webcam stream URLs for your two phones
CAM_URLS = [
    'http://192.168.1.15:8080/video',
    'http://192.168.1.18:8080/video'
]

# Paths to each phone’s calibration pickle (camera_matrix + dist_coeffs)
CALIB_PATHS = [
    r'E:\vr tracking\output_cam0\calibration_data.pkl',
    r'E:\vr tracking\output_cam1\calibration_data.pkl'
]
UDP_IP   = "127.0.0.1"
UDP_PORT = 5555
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# ArUco board settings
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()
MARKER_LENGTH = 0.05  # real marker side length in meters

# --- INITIALIZATION ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=2,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
def fuse(poses_list):
    if not poses_list:
        return None, None
    rvs = np.array([p[0].reshape(3) for p in poses_list])
    tvs = np.array([p[1].reshape(3) for p in poses_list])
    return np.mean(rvs, axis=0), np.mean(tvs, axis=0)

# 1) Open all streams
caps = [cv2.VideoCapture(url) for url in CAM_URLS]

# 2) Load calibration & build undistort maps
undistort_data = []
for path in CALIB_PATHS:
    print("Looking for:", path)
    if not os.path.exists(path):
        raise FileNotFoundError(f"No file at {path}")
    size = os.path.getsize(path)
    print(f" File size = {size} bytes")
    if size == 0:
        raise ValueError(f"Calibration file {path} is empty! "
                         "Something went wrong in the calibration script.")

    with open(path, 'rb') as f:
        data = pickle.load(f)
    print("Loaded data keys:", data.keys())

for calib_file, cap in zip(CALIB_PATHS, caps):
    # Load camera intrinsics
    with open(calib_file, 'rb') as f:
        data = pickle.load(f)
    mtx = data['camera_matrix']
    dist = data['distortion_coefficients']

    # Grab a frame to get resolution
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError(f"Cannot read from {calib_file}")
    h, w = frame.shape[:2]

    # Compute optimal new camera matrix
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    # Initialize undistort rectify map
    mapx, mapy = cv2.initUndistortRectifyMap(
        mtx, dist, None, new_mtx, (w, h), cv2.CV_32FC1
    )

    undistort_data.append((mapx, mapy, roi))


# Add this function to extract hand landmark data from MediaPipe results
def process_hand_landmarks(results, frame_width, frame_height):
    if not results.multi_hand_landmarks:
        return None, None

    # Process the first detected hand
    hand_landmarks = results.multi_hand_landmarks[0]
    # Determine if it's left or right hand
    hand_type = "left" if results.multi_handedness[0].classification[0].label == "Left" else "right"

    # Extract landmark positions
    landmarks_3d = []
    for landmark in hand_landmarks.landmark:
        # MediaPipe gives normalized coordinates, convert to meters
        # Assuming a reasonable hand size for scale
        landmarks_3d.append([
            landmark.x - 0.5,  # Center origin
            landmark.y - 0.5,  # Center origin
            landmark.z
        ])

    return hand_type, landmarks_3d
# --- MAIN LOOP: CAPTURE → UNDISTORT → DETECT → POSE → FUSE ---
while True:
    head_poses = []
    hand_poses = {1: [], 2: []}
    frames = []

    # 1) CAPTURE & UNDISTORT from both cameras
    for i, cap in enumerate(caps):
        ret, frame = cap.read()
        if not ret:
            print(f"[Camera {i}] NO FRAME")
            continue

        mapx, mapy, roi = undistort_data[i]
        frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)
        x, y, w, h = roi
        frame = frame[y:y + h, x:x + w]
        if frame is None or frame.size == 0:
            print(f"[Camera {i}] INVALID FRAME")
            continue

        # collect for side‑by‑side later
        frames.append(frame)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,  # grayscale image
            ARUCO_DICT,  # e.g., cv2.aruco.getPredefinedDictionary(...)
            parameters=aruco_params  # created above
        )

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, MARKER_LENGTH, mtx, dist
            )
            for _id, rv, tv in zip(ids.flatten(), rvecs, tvecs):
                if _id == 0:
                    head_poses.append((rv, tv))
                elif _id in (1, 2):
                    hand_poses[_id].append((rv, tv))

        # 2) PROCESS BOTH FRAMES IF AVAILABLE
        if len(frames) == 2:
            frame0, frame1 = frames
            h0, w0 = frame0.shape[:2]
            if frame1.shape[:2] != (h0, w0):
                frame1 = cv2.resize(frame1, (w0, h0))

            # Fuse ArUco poses if you want
            head_r, head_t = fuse(head_poses)
            left_r, left_t = fuse(hand_poses[1])
            right_r, right_t = fuse(hand_poses[2])

            # RUN MEDIAPIPE HAND TRACKING ON BOTH
            rgb0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2RGB)
            rgb1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)
            res0 = hands.process(rgb0)
            res1 = hands.process(rgb1)

            # Process hand landmarks from both cameras
            hand0_type, hand0_landmarks = process_hand_landmarks(res0, frame0.shape[1], frame0.shape[0])
            hand1_type, hand1_landmarks = process_hand_landmarks(res1, frame1.shape[1], frame1.shape[0])
            # Determine which hands are visible
            left_hand_visible = (hand0_type == "left" or hand1_type == "left")
            right_hand_visible = (hand0_type == "right" or hand1_type == "right")

            # Get the landmarks for each hand (use whichever camera sees it)
            left_landmarks = hand0_landmarks if hand0_type == "left" else hand1_landmarks
            right_landmarks = hand0_landmarks if hand0_type == "right" else hand1_landmarks

            # Build data structure for UDP transmission
            data = {
                "timestamp": time.time(),
                "head": {
                    "pos": head_t.ravel().tolist() if head_t is not None else None,
                    "rot": head_r.tolist() if head_t is not None else None
                },
                "left": {
                    "pos": left_t.ravel().tolist() if left_t is not None else None,
                    "rot": left_r.tolist() if left_t is not None else None,
                    "visible": left_hand_visible,
                    "landmarks": left_landmarks
                },
                "right": {
                    "pos": right_t.ravel().tolist() if right_t is not None else None,
                    "rot": right_r.tolist() if right_t is not None else None,
                    "visible": right_hand_visible,
                    "landmarks": right_landmarks
                },
                "hands_found": {
                    "cam0": bool(res0.multi_hand_landmarks),
                    "cam1": bool(res1.multi_hand_landmarks)
                }
            }

            msg = json.dumps(data).encode('utf-8')
            sock.sendto(msg, (UDP_IP, UDP_PORT))
            print(f"Sending UDP data: {len(msg)} bytes to {UDP_IP}:{UDP_PORT}")
            # PRINT STATUS
            t = time.time()
            print(f"{t:.2f} — Cam0 hands? {bool(res0.multi_hand_landmarks)} "
                  f"| Cam1 hands? {bool(res1.multi_hand_landmarks)}")

            # OPTIONAL: print your fused head pose
            if head_t is not None:
                x, y, z = head_t.ravel()
                print(f"  Head Pose → x={x:.3f}, y={y:.3f}, z={z:.3f}")
            if left_t is not None:
                print(f"  Left hand pos: {left_t.ravel()}, rot: {left_r.ravel()}")
            if right_t is not None:
                print(f"  Right hand pos: {right_t.ravel()}, rot: {right_r.ravel()}")
        time.sleep(0.01)


# Cleanup
for cap in caps:
    cap.release()
hands.close()
#cv2.destroyAllWindows()

