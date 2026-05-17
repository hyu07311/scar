/**
 * @file   cleaning_test.ino
 * @brief  청소 모듈 단독 테스트 — 로드셀 피드백 하강/상승 포함
 *
 *  동작 시퀀스:
 *    1. 핀/ESC/다이나믹셀/로드셀 초기화 + tare
 *    2. [DESCENT] 리니어 하강 → 양측 로드셀 0.5 kg 접지 감지 즉시 정지
 *                 (50초 이내 미감지 시 비상 정지)
 *    3. [RETRACT]  ACT_UP 방향 1.5초 소폭 재상승 (과압 방지)
 *    4. [CLEAN FWD] 슬라이드 -200 + 흡입팬 ON — 20초
 *    5. [CLEAN RET] 슬라이드 +200 + 흡입팬 OFF — 20초 (복귀)
 *    6. 슬라이드 정지
 *    7. [RISE]    리니어 복귀 → 추정 거리 ≤ 0 도달 즉시 정지
 *                 (50초 이내 미완료 시 비상 정지)
 *    ※ 언제든 시리얼 입력 시 즉시 비상 정지
 */

#include <DynamixelSDK.h>
#include <Servo.h>

/* ── 다이나믹셀 레지스터 ── */
#define ADDR_OPERATING_MODE  11
#define ADDR_TORQUE_ENABLE   64
#define ADDR_GOAL_VELOCITY  104

#define BAUDRATE          1000000
#define PROTOCOL_VERSION  2.0
#define DEVICENAME        "OpenCR_DXL_Port"

/* ── 하드웨어 핀 ── */
#define SLIDE_ID   18
#define F60_PIN     5

#define M1_INA    2
#define M1_PWM    3
#define M1_INB    4

#define M2_INA    7
#define M2_INB    8
#define M2_PWM    9

#define LC_DT1  A0
#define LC_SCK  A1
#define LC_DT2  A2

/* ── 제어 상수 ── */
#define ACT_DOWN          200
#define ACT_UP          (-200)
#define SUCTION_ON       1600   // μs
#define SUCTION_OFF      1000   // μs

#define LC_THRESHOLD_KG  0.5f  // 필터링 정지: N=4 평균 기준
#define LC_FAST_KG       1.0f  // 빠른 정지: 단일 원시 읽기 기준 (과하강 방지)
#define ACT_MAX_SPEED    15.0f  // mm/s (추정)
#define TARE_SAMPLES     10

#define STEP_DELAY_MS       2000UL
#define DESCENT_TIMEOUT_MS 50000UL
#define RETRACT_MS          1500UL
#define CLEAN_MS           20000UL
#define RISE_TIMEOUT_MS    50000UL
#define DT_MS                 20UL  // 50Hz

/* ── 전역 객체 ── */
dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;
Servo suction_esc;

long  tare1 = 0, tare2 = 0;
const float scale = -58000.0f;

float   est_dist1 = 0.0f, est_dist2 = 0.0f;
int32_t cur_pwm1  = 0,    cur_pwm2  = 0;

/* ─────────────────────────────────────────────────────
 *  HAL 함수
 * ───────────────────────────────────────────────────── */

bool readLoadCell(long *v1, long *v2) {
  if (digitalRead(LC_DT1) == HIGH || digitalRead(LC_DT2) == HIGH)
    return false;
  unsigned long c1 = 0, c2 = 0;
  for (int i = 0; i < 24; i++) {
    digitalWrite(LC_SCK, HIGH); delayMicroseconds(1);
    c1 <<= 1; c2 <<= 1;
    digitalWrite(LC_SCK, LOW);  delayMicroseconds(1);
    if (digitalRead(LC_DT1)) c1++;
    if (digitalRead(LC_DT2)) c2++;
  }
  digitalWrite(LC_SCK, HIGH); delayMicroseconds(1);
  digitalWrite(LC_SCK, LOW);  delayMicroseconds(1);
  if (c1 & 0x800000) c1 |= 0xFF000000;
  if (c2 & 0x800000) c2 |= 0xFF000000;
  *v1 = (long)c1;
  *v2 = (long)c2;
  return true;
}

void setActuator(int32_t s1, int32_t s2) {
  cur_pwm1 = s1;
  cur_pwm2 = s2;
  auto drive = [](int pA, int pB, int pP, int v) {
    if (v == 0) {
      digitalWrite(pA, LOW); digitalWrite(pB, LOW); analogWrite(pP, 0);
    } else {
      digitalWrite(pA, v > 0 ? HIGH : LOW);
      digitalWrite(pB, v < 0 ? HIGH : LOW);
      analogWrite(pP, (uint8_t)constrain(abs(v), 0, 255));
    }
  };
  drive(M1_INA, M1_INB, M1_PWM, (int)s1);
  drive(M2_INA, M2_INB, M2_PWM, (int)s2);
}

void setSlideVelocity(int32_t vel) {
  packetHandler->write4ByteTxRx(
      portHandler, SLIDE_ID, ADDR_GOAL_VELOCITY, (uint32_t)vel, NULL);
}

void emergencyStop() {
  setActuator(0, 0);
  setSlideVelocity(0);
  suction_esc.writeMicroseconds(SUCTION_OFF);
  Serial.println("[STOP] 비상 정지 — 재부팅 필요");
  while (1);
}

void checkEmergency() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    emergencyStop();
  }
}

/* 거리 추정 갱신 — setActuator() 호출 후 DT_MS마다 실행 */
void updateDist() {
  est_dist1 += (cur_pwm1 / 255.0f) * ACT_MAX_SPEED * (DT_MS / 1000.0f);
  est_dist2 += (cur_pwm2 / 255.0f) * ACT_MAX_SPEED * (DT_MS / 1000.0f);
}

/* ─────────────────────────────────────────────────────
 *  setup — 전체 시퀀스
 * ───────────────────────────────────────────────────── */
void setup() {
  /* [1] 리니어 핀 — 부팅 플로팅 방지 (Serial보다 먼저) */
  pinMode(M1_INA, OUTPUT); digitalWrite(M1_INA, LOW);
  pinMode(M1_INB, OUTPUT); digitalWrite(M1_INB, LOW);
  pinMode(M1_PWM, OUTPUT); analogWrite(M1_PWM,  0);
  pinMode(M2_INA, OUTPUT); digitalWrite(M2_INA, LOW);
  pinMode(M2_INB, OUTPUT); digitalWrite(M2_INB, LOW);
  pinMode(M2_PWM, OUTPUT); analogWrite(M2_PWM,  0);

  /* [2] 로드셀 핀 */
  pinMode(LC_SCK, OUTPUT);
  pinMode(LC_DT1, INPUT);
  pinMode(LC_DT2, INPUT);

  /* [3] 시리얼 */
  Serial.begin(57600);

  /* [4] ESC 초기화 */
  suction_esc.attach(F60_PIN);
  suction_esc.writeMicroseconds(SUCTION_OFF);
  delay(2000);

  /* [5] 다이나믹셀 — 슬라이드(ID 18) 속도 모드 */
  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);

  packetHandler->reboot(portHandler, SLIDE_ID, NULL);
  delay(2000);
  for (int retry = 0; retry < 3; retry++) {
    packetHandler->write1ByteTxRx(portHandler, SLIDE_ID, ADDR_TORQUE_ENABLE,  0, NULL); delay(50);
    packetHandler->write1ByteTxRx(portHandler, SLIDE_ID, ADDR_OPERATING_MODE, 1, NULL); delay(200);
    uint8_t cur_mode = 0;
    packetHandler->read1ByteTxRx(portHandler, SLIDE_ID, ADDR_OPERATING_MODE, &cur_mode, NULL);
    if (cur_mode == 1) break;
  }
  packetHandler->write1ByteTxRx(portHandler, SLIDE_ID, ADDR_TORQUE_ENABLE, 1, NULL);
  delay(20);

  /* [6] 로드셀 tare */
  long sum1 = 0, sum2 = 0;
  int  cnt  = 0;
  while (cnt < TARE_SAMPLES) {
    long t1, t2;
    if (readLoadCell(&t1, &t2)) { sum1 += t1; sum2 += t2; cnt++; }
    delay(10);
  }
  tare1 = sum1 / TARE_SAMPLES;
  tare2 = sum2 / TARE_SAMPLES;
  Serial.println("[TARE] 로드셀 영점 완료");

  delay(STEP_DELAY_MS);

  /* ── [DESCENT] 리니어 하강 ─────────────────────────────────── */
  Serial.println("[DESCENT] 하강 시작 — 접지 감지 대기 (최대 50초)");
  {
    float buf1[4] = {}, buf2[4] = {};
    uint8_t bidx = 0;
    float raw1 = 0.0f, raw2 = 0.0f;  // 루프 밖 선언: readLoadCell false 시 이전 유효값 유지
    unsigned long t0 = millis();
    bool landed = false;

    while (millis() - t0 < DESCENT_TIMEOUT_MS) {
      checkEmergency();

      long r1, r2;
      if (readLoadCell(&r1, &r2)) {
        raw1 = fabsf((float)(r1 - tare1) / scale);
        raw2 = fabsf((float)(r2 - tare2) / scale);
        buf1[bidx] = raw1;
        buf2[bidx] = raw2;
        bidx = (bidx + 1) % 4;
      }

      // 빠른 정지: 단일 원시 읽기 ≥ LC_FAST_KG (HX711 10Hz 과하강 방지)
      bool fast1 = (raw1 >= LC_FAST_KG);
      bool fast2 = (raw2 >= LC_FAST_KG);

      // 필터링 정지: N=4 이동평균 ≥ LC_THRESHOLD_KG (완만한 접지 포착)
      float w1 = 0.0f, w2 = 0.0f;
      for (uint8_t i = 0; i < 4; i++) { w1 += buf1[i]; w2 += buf2[i]; }
      w1 /= 4.0f; w2 /= 4.0f;
      bool lc1 = (w1 >= LC_THRESHOLD_KG);
      bool lc2 = (w2 >= LC_THRESHOLD_KG);

      bool stopped1 = fast1 || lc1;
      bool stopped2 = fast2 || lc2;
      setActuator(stopped1 ? 0 : ACT_DOWN, stopped2 ? 0 : ACT_DOWN);
      updateDist();

      if (stopped1 && stopped2) {
        landed = true;
        setActuator(0, 0);
        Serial.print("[DESCENT] 접지! L="); Serial.print(w1, 2);
        Serial.print("kg R="); Serial.print(w2, 2);
        if (fast1 || fast2) Serial.print(" (fast-stop)");
        Serial.println("kg");
        break;
      }
      delay(DT_MS);
    }

    if (!landed) {
      Serial.println("[DESCENT] 타임아웃 — 비상 정지");
      emergencyStop();
    }
  }

  delay(STEP_DELAY_MS);

  /* ── [RETRACT] 소폭 재상승 1.5초 ────────────────────────────── */
  Serial.println("[RETRACT] 소폭 재상승 1.5초");
  setActuator(ACT_UP, ACT_UP);
  {
    unsigned long t0 = millis();
    while (millis() - t0 < RETRACT_MS) {
      checkEmergency();
      updateDist();
      if (est_dist1 <= 0.0f && est_dist2 <= 0.0f) break;  // 단거리 하강 시 원점 초과 방지
      delay(DT_MS);
    }
  }
  setActuator(0, 0);
  Serial.println("[RETRACT] 완료");

  delay(STEP_DELAY_MS);

  /* ── [CLEAN FWD] 슬라이드 -200 + 흡입팬 ON — 20초 ─────────── */
  Serial.println("[CLEAN FWD] 슬라이드 -200, 흡입팬 ON — 20초");
  setSlideVelocity(-200);
  suction_esc.writeMicroseconds(SUCTION_ON);
  {
    unsigned long t0 = millis();
    while (millis() - t0 < CLEAN_MS) {
      checkEmergency();
      delay(DT_MS);
    }
  }

  delay(STEP_DELAY_MS);

  /* ── [CLEAN RET] 슬라이드 +200 + 흡입팬 OFF — 20초 ─────────── */
  Serial.println("[CLEAN RET] 슬라이드 +200, 흡입팬 OFF — 20초");
  suction_esc.writeMicroseconds(SUCTION_OFF);
  setSlideVelocity(200);
  {
    unsigned long t0 = millis();
    while (millis() - t0 < CLEAN_MS) {
      checkEmergency();
      delay(DT_MS);
    }
  }

  delay(STEP_DELAY_MS);

  /* 슬라이드 정지 */
  setSlideVelocity(0);

  delay(STEP_DELAY_MS);

  /* ── [RISE] 리니어 복귀 ─────────────────────────────────────── */
  Serial.println("[RISE] 원위치 복귀 시작 (최대 50초)");
  {
    unsigned long t0 = millis();
    bool recovered = false;

    while (millis() - t0 < RISE_TIMEOUT_MS) {
      checkEmergency();
      bool d1 = (est_dist1 <= 0.0f);
      bool d2 = (est_dist2 <= 0.0f);
      setActuator(d1 ? 0 : ACT_UP, d2 ? 0 : ACT_UP);
      updateDist();

      if (d1 && d2) {
        recovered = true;
        break;
      }
      delay(DT_MS);
    }

    setActuator(0, 0);
    if (!recovered) {
      Serial.println("[RISE] 타임아웃 — 비상 정지");
      emergencyStop();
    }
  }
  Serial.println("[RISE] 완료");

  delay(STEP_DELAY_MS);

  /* ── [DONE] ─────────────────────────────────────────────────── */
  Serial.println("[DONE] 모든 동작 완료");
  while (1);
}

void loop() {}
