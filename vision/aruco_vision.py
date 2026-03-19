"""
aruco_vision.py — ArUco marker detection (standalone, no arm control)

Used for debugging and verifying marker detection and 3D coordinate
estimation independently of the arm control pipeline.

Key controls
------------
  p : print last detected ID 9 coordinates
  o : print last detected ID 10 coordinates
  q : quit

Dependencies
------------
  pip install depthai opencv-python numpy
"""

import cv2
import cv2.aruco as aruco
import depthai as dai
import numpy as np

PREVIEW_W, PREVIEW_H = 640, 400
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)


def build_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setPreviewSize(PREVIEW_W, PREVIEW_H)

    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)

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


def main():
    pipeline = build_pipeline()
    params   = aruco.DetectorParameters()

    last_pose = {9: None, 10: None}

    with dai.Device(pipeline) as device:
        rgb_q   = device.getOutputQueue("rgb",   maxSize=4, blocking=False)
        depth_q = device.getOutputQueue("depth", maxSize=4, blocking=False)

        calib      = device.readCalibration()
        intrinsics = calib.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_A, PREVIEW_W, PREVIEW_H)
        fx, fy = intrinsics[0][0], intrinsics[1][1]
        cx, cy = intrinsics[0][2], intrinsics[1][2]

        print("[INFO] Press 'p' for ID-9 coords | 'o' for ID-10 coords | 'q' to quit")

        while True:
            frame = rgb_q.get().getCvFrame()
            depth = depth_q.get().getFrame()
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            key   = cv2.waitKey(1) & 0xFF

            corners, ids, _ = aruco.detectMarkers(gray, ARUCO_DICT, parameters=params)

            if ids is not None:
                ids = ids.flatten()
                for i, marker_id in enumerate(ids):
                    pts = corners[i][0]
                    u = int(np.mean(pts[:, 0]))
                    v = int(np.mean(pts[:, 1]))

                    z = depth[v, u] / 1000.0
                    x = (u - cx) * z / fx
                    y = -(v - cy) * z / fy

                    if marker_id in last_pose:
                        last_pose[marker_id] = (x, y, z)

                    aruco.drawDetectedMarkers(frame, [corners[i]], np.array([[marker_id]]))
                    cv2.circle(frame, (u, v), 4, (0, 255, 0), -1)
                    cv2.putText(frame, f"ID {marker_id}", (u + 5, v - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if key == ord('p') and last_pose[9]:
                x, y, z = last_pose[9]
                print(f"[P] ID 9  → X={x:.3f}m  Y={y:.3f}m  Z={z:.3f}m")

            if key == ord('o') and last_pose[10]:
                x, y, z = last_pose[10]
                print(f"[O] ID 10 → X={x:.3f}m  Y={y:.3f}m  Z={z:.3f}m")

            if key == ord('q'):
                break

            cv2.imshow("ArUco Vision", frame)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
