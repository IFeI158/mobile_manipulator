"""
color_vision.py — HSV color-based object detection (reference only)

Early prototype that detects a yellow object via HSV thresholding
and estimates its 3D position using the OAK-D stereo depth camera.

Replaced by ArUco marker-based detection (aruco_vision.py / main.py)
due to instability under varying lighting conditions.

Kept for reference.

Dependencies
------------
  pip install depthai opencv-python numpy
"""

import cv2
import depthai as dai
import numpy as np

# ── Tunable parameters ────────────────────────────────────────────────────────
OBJ_HEIGHT_CM = 10.0   # Approximate real height of target object (cm)
EMA_ALPHA     = 0.2    # Exponential moving average smoothing factor

# ── Yellow HSV range ──────────────────────────────────────────────────────────
HSV_LOWER = (20, 150, 150)
HSV_UPPER = (30, 255, 255)


def build_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName('rgb')
    cam_rgb.preview.link(xout_rgb.input)

    mono_l = pipeline.create(dai.node.MonoCamera)
    mono_r = pipeline.create(dai.node.MonoCamera)
    mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    mono_l.out.link(stereo.left)
    mono_r.out.link(stereo.right)

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName('depth')
    stereo.depth.link(xout_depth.input)

    return pipeline


def main():
    pipeline = build_pipeline()
    prev_z_mono = None

    with dai.Device(pipeline) as device:
        rgb_q  = device.getOutputQueue('rgb',   maxSize=4, blocking=False)
        depth_q = device.getOutputQueue('depth', maxSize=4, blocking=False)

        calib = device.readCalibration()
        intr  = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, (640, 480))
        fx, fy = intr[0][0], intr[1][1]
        cx, cy = intr[0][2], intr[1][2]
        dist_coeffs = calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

        while True:
            frame     = rgb_q.get().getCvFrame()
            depth_map = depth_q.get().getFrame()

            # Undistort
            frame = cv2.undistort(frame, K, np.array(dist_coeffs))

            # Yellow detection
            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if cnts:
                c = max(cnts, key=cv2.contourArea)
                rect = cv2.minAreaRect(c)
                (cx_r, cy_r), (w, h), _ = rect
                h_pixel = min(w, h)

                if h_pixel > 0:
                    z_new = fx * OBJ_HEIGHT_CM / h_pixel
                    z_mono = (z_new if prev_z_mono is None
                              else EMA_ALPHA * z_new + (1 - EMA_ALPHA) * prev_z_mono)
                    prev_z_mono = z_mono

                    x_cm = (cx_r - cx) * z_mono / fx
                    y_cm = -(cy_r - cy) * z_mono / fy
                    print(f"X={x_cm:.2f}  Y={y_cm:.2f}  Z={z_mono:.2f} cm")

                box = cv2.boxPoints(rect).astype(int)
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                cv2.circle(frame, (int(cx_r), int(cy_r)), 5, (0, 0, 255), -1)

            cv2.imshow('Color Vision', frame)
            if cv2.waitKey(1) == ord('q'):
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
