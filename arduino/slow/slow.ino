#include <Servo.h>

// 서보 객체 정의
Servo servo0;       // 베이스
Servo servo1a;      // 1번 링크 (정방향)
Servo servo1b;      // 1번 링크 (역방향)
Servo servo2;       // 2번 링크
Servo servo3;       // 3번 링크
Servo servo4;       // 그리퍼

// 핀 번호 매핑
const int pin0  = 12;
const int pin1a = 11;
const int pin1b = 10;
const int pin2  = 9;
const int pin3  = 8;
const int pin4  = 7;

// 초기 각도 정의
const int INIT_S0 = 90;
const int INIT_S1 = 130;
const int INIT_S2 = 0;
const int INIT_S3 = 130;

// === 유틸 함수 ===
int clamp(int val, int minVal, int maxVal) {
  return max(minVal, min(val, maxVal));
}

void moveSmooth(Servo &sv, int target, int delayMs = 40) {
  int curr = sv.read();
  if (curr < target) {
    for (int p = curr; p <= target; p++) {
      sv.write(p);
      delay(delayMs);
    }
  } else {
    for (int p = curr; p >= target; p--) {
      sv.write(p);
      delay(delayMs);
    }
  }
}

void moveAllSmooth(int v0, int v1, int v2, int v3) {
  // 1. servo0 먼저 천천히 이동
  int curr0 = servo0.read();
  int step0 = abs(v0 - curr0);
  for (int i = 0; i <= step0; i++) {
    int pos = curr0 + (v0 > curr0 ? i : -i);
    servo0.write(pos);
    delay(40);
  }

  // 2. 나머지 동시에 부드럽게 이동
  int curr1 = servo1a.read();
  int curr2 = servo2.read();
  int curr3 = servo3.read();
  int maxStep = max(max(abs(v1 - curr1), abs(v2 - curr2)), abs(v3 - curr3));

  for (int i = 0; i <= maxStep; i++) {
    if (i <= abs(v1 - curr1)) {
      int step = curr1 + (v1 > curr1 ? i : -i);
      servo1a.write(step);
      servo1b.write(180 - step);
    }
    if (i <= abs(v2 - curr2)) {
      int pos = curr2 + (v2 > curr2 ? i : -i);
      servo2.write(pos);
    }
    if (i <= abs(v3 - curr3)) {
      int pos = curr3 + (v3 > curr3 ? i : -i);
      servo3.write(pos);
    }
    delay(40);
  }
}

void setup() {
  Serial.begin(9600);

  // ✴️ 튐 방지: 먼저 write
  servo0.write(INIT_S0);
  servo1a.write(INIT_S1);
  servo1b.write(180 - INIT_S1);
  servo2.write(INIT_S2);
  servo3.write(INIT_S3);
  servo4.write(180);  // 그리퍼 열림

  // ✴️ 그 후 attach
  servo0.attach(pin0);
  servo1a.attach(pin1a);
  servo1b.attach(pin1b);
  servo2.attach(pin2);
  servo3.attach(pin3);
  servo4.attach(pin4);

  delay(300);  // 초기 안정화

  // ✴️ 부드럽게 초기 위치로 복귀
  moveSmooth(servo0, INIT_S0, 40);
  moveSmooth(servo1a, INIT_S1, 40);
  moveSmooth(servo1b, 180 - INIT_S1, 40);
  moveSmooth(servo2, INIT_S2, 40);
  moveSmooth(servo3, INIT_S3, 40);
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  int vals[4] = {INIT_S0, INIT_S1, INIT_S2, INIT_S3};
  int idx = 0, start = 0;

  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line.charAt(i) == ',') {
      vals[idx++] = line.substring(start, i).toInt();
      start = i + 1;
      if (idx >= 4) break;
    }
  }

  // 디버깅 출력
  Serial.print("Received: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(vals[i]);
    Serial.print(i < 3 ? "," : "\n");
  }

  // 1. 받은 각도로 이동 (베이스 먼저 → 나머지 동시에)
  moveAllSmooth(vals[0], vals[1], vals[2], vals[3]);

  // 2. 그리퍼 닫기
  delay(1000);
  moveSmooth(servo4, 50, 15);  // 그리퍼는 빠르게 닫기

  // 3. 링크2 (servo2) 20도 올리기
  int curr_s2 = servo2.read();
  int new_s2 = clamp(curr_s2 + 20, 0, 180);
  moveSmooth(servo2, new_s2, 40);

  // 4. 초기 위치로 복귀
  delay(1000);
  moveAllSmooth(INIT_S0, INIT_S1, INIT_S2, INIT_S3);
}
