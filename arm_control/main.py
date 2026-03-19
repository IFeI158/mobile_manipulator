"""
main.py — ArUco Vision + IK + Arduino Serial integration

Detects ArUco markers via an OAK-D (DepthAI) stereo camera,
computes 3D positions, and commands a 4-DOF robot arm over serial.

Key controls
------------
  p : move arm to the last detected ID 9 marker position
  o : print current ID 10 marker coordinates (debug)
  q : quit

Hardware
--------
  Camera  : Luxonis OAK-D (DepthAI)
  Arduino : connected via /dev/ttyACM0 at 9600 baud
  Markers : ArUco DICT_4X4_50  (ID 9 = target object, ID 10 = drop zone)

Dependencies
------------
  pip install depthai opencv-python numpy pyserial
"""

import cv2
import cv2.aruco as aruco
import depthai as dai
import numpy as np
import serial
import time

from ik_solver import compute_servo_angles

# ── Serial settings ────────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 9600

# ── Depth estimation: IQR window size (pixels) ────────────────────────────────
DEPTH_WINDOW = 3

# ── Camera preview resolution ─────────────────────────────────────────────────
PREVIEW_W, PREVIEW_H = 640, 400

# ── ArUco dictionary ──────────────────────────────────────────────────────────
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)


# ── DepthAI pipeline ──────────────────────────────────────────────────────────
def build_pipeline():
    """Create and return a DepthAI pipeline for RGB + stereo depth."""
    pipeline = dai.Pipeline()

    # RGB camera
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setPreviewSize(PREVIEW_W, PREVIEW_H)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)

    # Stereo depth
    mono_l = pipeline.create(dai.node.MonoCamera)
    mono_r = pipeline.create(dai.node.MonoCamera)
    mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.initialConfig.setConfidenceThreshold(200)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    stereo.depth.link(xout_depth.input)

    return pipeline


# ── Depth estimation ──────────────────────────────────────────────────────────
def estimate_depth(depth_frame, u, v, window=DEPTH_WINDOW):
    """
    Estimate depth at pixel (u, v) using an IQR-filtered window.

    Returns depth in metres, or None if not enough valid samples.
    """
    h, w = depth_frame.shape
    samples = [
        depth_frame[v + dy, u + dx]
        for dx in range(-window, window + 1)
        for dy in range(-window, window + 1)
        if 0 <= u + dx < w and 0 <= v + dy < h
        and 0 < depth_frame[v + dy, u + dx] < 10_000
    ]

    if len(samples) < 10:
        return None

    arr = np.array(samples, dtype=float)
    q1, q3 = np.percentile(arr, [25, 75])
    filtered = arr[(arr > q1) & (arr < q3)]
    mean_mm = np.mean(filtered) if len(filtered) > 0 else np.mean(arr)
    return mean_mm / 1000.0   # mm → m


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("[INFO] Arduino connected.")

    pipeline  = build_pipeline()
    detector_params = aruco.DetectorParameters()
    last_pose_id9 = None   # (x_cm, y_cm, z_cm) in camera frame

    with dai.Device(pipeline) as device:
        rgb_q   = device.getOutputQueue("rgb",   maxSize=4, blocking=False)
        depth_q = device.getOutputQueue("depth", maxSize=4, blocking=False)

        calib      = device.readCalibration()
        intrinsics = calib.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A, PREVIEW_W, PREVIEW_H)
        fx, fy = intrinsics[0][0], intrinsics[1][1]
        cx, cy = intrinsics[0][2], intrinsics[1][2]

        print("[INFO] Camera calibration loaded.")
        print("[INFO] Press 'p' to move arm | 'o' for ID-10 coords | 'q' to quit")

        while True:
            frame = rgb_q.get().getCvFrame()
            depth = depth_q.get().getFrame()
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            key   = cv2.waitKey(1) & 0xFF

            corners, ids, _ = aruco.detectMarkers(
                gray, ARUCO_DICT, parameters=detector_params)

            if ids is not None:
                ids = ids.flatten()
                for i, marker_id in enumerate(ids):
                    pts = corners[i][0]
                    u = int(np.mean(pts[:, 0]))
                    v = int(np.mean(pts[:, 1]))

                    Z = estimate_depth(depth, u, v)
                    if Z is None:
                        continue

                    X = (u - cx) * Z / fx
                    Y = -(v - cy) * Z / fy   # screen-up = +Y

                    if marker_id == 9:
                        # Convert to cm, reorder axes for arm coordinate frame
                        last_pose_id9 = (Z * 100, -X * 100, Y * 100)

                    elif marker_id == 10 and key == ord('o'):
                        print(f"[O] ID 10 → X={X:.3f}m  Y={Y:.3f}m  Z={Z:.3f}m")

                    aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[marker_id]]))
                    cv2.circle(frame, (u, v), 6, (0, 255, 0), -1)
                    cv2.putText(frame, f"ID {marker_id}",
                                (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # ── 'p': move arm to last detected ID 9 ───────────────────────────
            if key == ord('p') and last_pose_id9 is not None:
                x_c, y_c, z_c = last_pose_id9
                print(f"[P] ID 9 → x={x_c:.1f}  y={y_c:.1f}  z={z_c:.1f} cm")
                s0, s1, s2, s3 = compute_servo_angles(x_c, y_c, z_c)
                print(f"    angles: s0={s0}  s1={s1}  s2={s2}  s3={s3}")
                arduino.write(f"{s0},{s1},{s2},{s3}\n".encode())
                time.sleep(0.2)

            if key == ord('q'):
                break

            # Draw camera principal point
            cv2.circle(frame, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            cv2.imshow("Frame", frame)

    cv2.destroyAllWindows()
    arduino.close()


if __name__ == "__main__":
    main()
