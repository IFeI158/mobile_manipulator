# Mobile Manipulator Robot (지능형 도우미 로봇)

거동이 불편한 사용자가 원격으로 물체를 지정하면, 지능형 도우미 로봇이 해당 물체를 스스로 찾아 집어 가져오는 시스템입니다.

모바일 플랫폼(TurtleBot3)과 직접 설계·제작한 4-DOF 로봇팔을 결합한 **Mobile Manipulator** 로, ArUco 마커 기반 비전과 역기구학(IK)을 통해 물체를 자율적으로 집어 초기 위치로 복귀합니다.

---

## Project Overview

| 항목 | 내용 |
|------|------|
| 기간 | 2025.03 – 2025.06 (1학기 캡스톤 디자인) |
| 인원 | 5명 |
| 역할 | Hardware 설계 / Vision 개발 |
| 플랫폼 | TurtleBot3 Burger + 직접 제작 로봇팔 |

---

## System Architecture

```
User ('p' key press)
        ↓
OAK-D Camera — ArUco Marker Detection
        ↓
3D Position Estimation (Stereo Depth + IQR Filter)
        ↓
Inverse Kinematics (ik_solver.py)
        ↓
Serial Command → Arduino (amurokena.ino)
        ↓
Servo Motion → Pick Object → Return to Home
```

---

## Repository Structure

```
mobile-manipulator/
├── vision/
│   ├── aruco_vision.py        ArUco 인식 + 3D 좌표 추출 (디버깅용)
│   └── color_vision.py        HSV 색상 인식 초기 버전 (참고용)
│
├── arm_control/
│   ├── main.py                메인 실행 파일 (ArUco + IK + 시리얼 통합)
│   ├── ik_solver.py           역기구학 계산 모듈
│   └── test_ik.py             터미널 수동 테스트용
│
├── arduino/
│   ├── amurokena/
│   │   └── amurokena.ino      최종 Arduino (moveAllSmooth + 자동 파지/복귀)
│   └── slow/
│       └── slow.ino           이전 버전 Arduino (참고용)
│
├── ros2_ws/
│   └── src/
│       ├── capstone_design_description/   URDF + STL 메시
│       └── capstone_design_config/        MoveIt2 설정
│
├── robotarm_capstone.xacro    로봇팔 XACRO 모델
└── README.md
```

---

## Hardware

| 부품 | 사양 |
|------|------|
| Mobile Platform | TurtleBot3 Burger |
| Controller | OpenCR 1.0 |
| Camera | Luxonis OAK-D (DepthAI) |
| LiDAR | YDLiDAR |
| Servo ×6 | MG996R / SG90 |
| MCU | Arduino Uno |

### Robot Arm Link Parameters

| Link | Length | Description |
|------|--------|-------------|
| L1 | 20.404 cm | Shoulder → Elbow |
| L2 | 17.617 cm | Elbow → Wrist |
| L3 | 4.65 cm | Wrist → End-effector |
| BASE_Z_OFFSET | 7.8 cm | Base → first joint height |

### Arduino Pin Mapping

| Servo | Pin | Role |
|-------|-----|------|
| servo0 | 12 | BASE yaw |
| servo1a | 11 | SHOULDER (forward) |
| servo1b | 10 | SHOULDER (reverse, 180 − s1) |
| servo2 | 9 | ELBOW |
| servo3 | 8 | WRIST |
| servo4 | 7 | GRIPPER |

---

## Software Stack

- **ROS2 (Humble)** — Navigation, TF, SLAM
- **Python 3** — Vision, IK, serial communication
- **DepthAI** — OAK-D stereo depth pipeline
- **OpenCV** — ArUco detection
- **Arduino** — Servo control

---

## Quick Start

### 1. Install dependencies

```bash
pip install depthai opencv-python numpy pyserial
```

### 2. Upload Arduino firmware

Open `arduino/amurokena/amurokena.ino` in Arduino IDE and upload to Arduino Uno.

### 3. Run main program

```bash
cd arm_control
python3 main.py
```

### 4. Key controls

| Key | Action |
|-----|--------|
| `p` | Move arm to ArUco ID 9 marker position |
| `o` | Print ArUco ID 10 coordinates (debug) |
| `q` | Quit |

### 5. Manual IK test (without camera)

```bash
cd arm_control
python3 test_ik.py
# > 10 0 -5    (x y z in cm, camera frame)
```

---

## Serial Protocol

```
Python → Arduino : "s0,s1,s2,s3\n"         (4 angles, comma-separated)
Arduino → Python : "Received: s0,s1,s2,s3"  (debug echo)
Arduino → Python : "done"                    (pick-and-return complete)

Port  : /dev/ttyACM0
Baud  : 9600
```

---

## ArUco Marker IDs

| ID | Role |
|----|------|
| 9 | Target object — arm moves here on `p` key |
| 10 | Drop zone — coordinates printed on `o` key |

Dictionary: `DICT_4X4_50`

---

## Demonstration

> 시연 영상 추가 예정

---

## Future Improvements

- `hujin.py` — 물체 파지 후 Nav2를 통한 후진 복귀 로직 구현
- Object detection 기반 자동 마커 없는 물체 인식
- MoveIt2 기반 Motion Planning 통합
- SLAM 환경 인식 개선

---

## Author

**IFeI158** — Mechanical Engineering / Robotics & Autonomous Systems
