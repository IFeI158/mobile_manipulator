"""
test_ik.py — Interactive terminal test for IK solver + Arduino

Allows manual input of target coordinates (camera frame, cm)
and sends the computed servo angles to the Arduino over serial.

Usage
-----
  python3 test_ik.py
  > 10 0 -5    # x=10cm, y=0cm, z=-5cm (camera frame)
  > q          # quit
"""

import serial
import time

from ik_solver import compute_servo_angles, CAM_OFFSET_X, CAM_OFFSET_Y, CAM_OFFSET_Z

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 9600


def main():
    print("[INFO] Camera offset at home pose:")
    print(f"  CAM_OFFSET_X = {CAM_OFFSET_X:.3f} cm")
    print(f"  CAM_OFFSET_Y = {CAM_OFFSET_Y:.3f} cm")
    print(f"  CAM_OFFSET_Z = {CAM_OFFSET_Z:.3f} cm\n")

    arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("[INFO] Arduino connected.")
    print("Example input: 10 0 -5   (x y z in cm, camera frame)\n")

    while True:
        usr = input("Target x y z (cm) or 'q' to quit > ").strip()
        if usr.lower() == 'q':
            break
        try:
            x_c, y_c, z_c = map(float, usr.split())
            s0, s1, s2, s3 = compute_servo_angles(x_c, y_c, z_c)
            print(f"  → s0={s0}  s1={s1}  s2={s2}  s3={s3}")
            arduino.write(f"{s0},{s1},{s2},{s3}\n".encode())
            time.sleep(0.1)
        except ValueError as e:
            print(f"  Error: {e}")

    arduino.close()


if __name__ == "__main__":
    main()
