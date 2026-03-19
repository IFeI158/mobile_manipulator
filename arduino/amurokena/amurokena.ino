/**
 * amurokena.ino — Robot arm servo controller (final version)
 *
 * Receives servo angles from Python via serial (format: "s0,s1,s2,s3\n"),
 * moves all joints smoothly to the target, closes the gripper to pick
 * an object, lifts link-2, then returns to the home pose.
 *
 * Serial protocol
 * ---------------
 *   Python → Arduino : "s0,s1,s2,s3\n"   (4 angles, comma-separated)
 *   Arduino → Python : "Received: s0,s1,s2,s3\n"
 *   Arduino → Python : "done\n"           (after pick-and-return completes)
 *
 * Pin mapping
 * -----------
 *   pin 12  servo0  — BASE yaw
 *   pin 11  servo1a — SHOULDER (forward)
 *   pin 10  servo1b — SHOULDER (reverse, writes 180 - s1)
 *   pin  9  servo2  — ELBOW
 *   pin  8  servo3  — WRIST
 *   pin  7  servo4  — GRIPPER
 */

#include <Servo.h>

// ── Servo objects ─────────────────────────────────────────────────────────────
Servo servo0;   // Base
Servo servo1a;  // Shoulder (forward)
Servo servo1b;  // Shoulder (reverse)
Servo servo2;   // Elbow
Servo servo3;   // Wrist
Servo servo4;   // Gripper

// ── Pin assignments ───────────────────────────────────────────────────────────
const int PIN_BASE      = 12;
const int PIN_SHOULDER_A = 11;
const int PIN_SHOULDER_B = 10;
const int PIN_ELBOW     =  9;
const int PIN_WRIST     =  8;
const int PIN_GRIPPER   =  7;

// ── Home pose angles (degrees) ────────────────────────────────────────────────
const int INIT_S0 = 90;
const int INIT_S1 = 130;
const int INIT_S2 = 0;
const int INIT_S3 = 130;

// ── Gripper angles ────────────────────────────────────────────────────────────
const int GRIPPER_OPEN   = 120;
const int GRIPPER_CLOSED = 60;

// ── Motion timing (ms per step) ───────────────────────────────────────────────
const int SMOOTH_DELAY   = 40;   // Normal joint step delay
const int ELBOW_DELAY    = 20;   // Elbow moves at 2× speed


// ── Utility ───────────────────────────────────────────────────────────────────
int clamp(int val, int minVal, int maxVal) {
  return max(minVal, min(maxVal, val));
}

/** Move a single servo smoothly to target. */
void moveSmooth(Servo &sv, int target, int delayMs = SMOOTH_DELAY) {
  int curr = sv.read();
  int dir  = (target > curr) ? 1 : -1;
  for (int p = curr; p != target; p += dir) {
    sv.write(p);
    delay(delayMs);
  }
  sv.write(target);
}

/**
 * Move base first (yaw), then shoulder / elbow / wrist simultaneously.
 * Elbow runs at double speed via a separate timer.
 */
void moveAllSmooth(int v0, int v1, int v2, int v3) {
  // 1. Base yaw
  moveSmooth(servo0, v0, SMOOTH_DELAY);

  // 2. Shoulder + elbow + wrist in parallel
  int curr1 = servo1a.read();
  int curr2 = servo2.read();
  int curr3 = servo3.read();
  int maxStep = max({ abs(v1 - curr1), abs(v2 - curr2), abs(v3 - curr3) });

  int dir2 = (v2 > curr2) ? 1 : -1;
  int stepCount2 = 0;
  unsigned long lastElbowTime = millis();

  for (int i = 0; i <= maxStep; i++) {
    if (i <= abs(v1 - curr1)) {
      int pos = curr1 + (v1 > curr1 ? i : -i);
      servo1a.write(pos);
      servo1b.write(180 - pos);
    }
    if (i <= abs(v3 - curr3)) {
      servo3.write(curr3 + (v3 > curr3 ? i : -i));
    }
    // Elbow at double speed
    if (stepCount2 <= abs(v2 - curr2) &&
        millis() - lastElbowTime >= ELBOW_DELAY) {
      servo2.write(curr2 + dir2 * stepCount2);
      stepCount2++;
      lastElbowTime = millis();
    }
    delay(SMOOTH_DELAY);
  }
}

/** Full pick-and-return sequence after reaching target pose. */
void pickAndReturn() {
  // Close gripper
  servo4.write(GRIPPER_CLOSED);
  delay(1000);

  // Lift elbow slightly
  int lifted_s2 = clamp(servo2.read() + 20, 0, 180);
  moveSmooth(servo2, lifted_s2, SMOOTH_DELAY);

  // Partial shoulder retract (1 second worth)
  {
    int curr1    = servo1a.read();
    int diff1    = INIT_S1 - curr1;
    int dir1     = (diff1 > 0) ? 1 : -1;
    int maxSteps = min(abs(diff1), 1000 / SMOOTH_DELAY);
    for (int i = 1; i <= maxSteps; i++) {
      int pos = curr1 + dir1 * i;
      servo1a.write(pos);
      servo1b.write(180 - pos);
      delay(SMOOTH_DELAY);
    }
    int elapsed = maxSteps * SMOOTH_DELAY;
    if (elapsed < 1000) delay(1000 - elapsed);
  }

  // Return to home pose
  moveAllSmooth(INIT_S0, INIT_S1, INIT_S2, INIT_S3);
}


// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Write before attach to prevent servo twitch on power-up
  servo0.write(INIT_S0);
  servo1a.write(INIT_S1);
  servo1b.write(180 - INIT_S1);
  servo2.write(INIT_S2);
  servo3.write(INIT_S3);
  servo4.write(GRIPPER_OPEN);

  servo0.attach(PIN_BASE);
  servo1a.attach(PIN_SHOULDER_A);
  servo1b.attach(PIN_SHOULDER_B);
  servo2.attach(PIN_ELBOW);
  servo3.attach(PIN_WRIST);
  servo4.attach(PIN_GRIPPER);

  delay(300);

  // Smooth move to home pose
  moveSmooth(servo0, INIT_S0);
  moveSmooth(servo1a, INIT_S1);
  moveSmooth(servo1b, 180 - INIT_S1);
  moveSmooth(servo2, INIT_S2);
  moveSmooth(servo3, INIT_S3);
}


// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  if (!Serial.available()) return;

  // Parse "s0,s1,s2,s3\n"
  String line = Serial.readStringUntil('\n');
  int vals[4] = { INIT_S0, INIT_S1, INIT_S2, INIT_S3 };
  int idx = 0, start = 0;
  for (int i = 0; i <= (int)line.length(); i++) {
    if (i == (int)line.length() || line.charAt(i) == ',') {
      vals[idx++] = line.substring(start, i).toInt();
      start = i + 1;
      if (idx >= 4) break;
    }
  }

  // Echo for debugging
  Serial.print("Received: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(vals[i]);
    Serial.print(i < 3 ? "," : "\n");
  }

  // Move to target, then pick and return
  moveAllSmooth(vals[0], vals[1], vals[2], vals[3]);
  delay(1000);
  pickAndReturn();

  Serial.println("done");
}
