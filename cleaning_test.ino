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

#define LC_RAW_DELTA   30000L  // 필터링 정지: N=4 평균 기준 (실측 후 조정)
#define LC_FAST_DELTA  60000L  // 빠른 정지: 단일 원시 읽기 기준 (과하강 방지)
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
  Serial.print("[TARE] 완료 tare1="); Serial.print(tare1);
  Serial.print(" tare2="); Serial.println(tare2);

  delay(STEP_DELAY_MS);

  /* ── [DESCENT] 리니어 하강 ─────────────────────────────────── */
  Serial.println("[DESCENT] 하강 시작 — 접지 감지 대기 (최대 50초)");
  {
    long rbuf1[4] = {}, rbuf2[4] = {};
    uint8_t bidx = 0;
    long rdelta1 = 0, rdelta2 = 0;  // 루프 밖 선언: false 시 마지막 유효 delta 래치
    unsigned long t0 = millis();
    unsigned long last_print = 0;
    bool landed = false;

    while (millis() - t0 < DESCENT_TIMEOUT_MS) {
      checkEmergency();

      long r1, r2;
      if (readLoadCell(&r1, &r2)) {
        rdelta1 = abs(r1 - tare1);
        rdelta2 = abs(r2 - tare2);
        rbuf1[bidx] = rdelta1;
        rbuf2[bidx] = rdelta2;
        bidx = (bidx + 1) % 4;
      }

      // 빠른 정지: 단일 원시 delta ≥ LC_FAST_DELTA (과하강 방지, 래치 유지)
      bool fast1 = (rdelta1 >= LC_FAST_DELTA);
      bool fast2 = (rdelta2 >= LC_FAST_DELTA);

      // 필터링 정지: N=4 이동평균 ≥ LC_RAW_DELTA (완만한 접지 포착)
      long avg1 = 0, avg2 = 0;
      for (uint8_t i = 0; i < 4; i++) { avg1 += rbuf1[i]; avg2 += rbuf2[i]; }
      avg1 /= 4; avg2 /= 4;
      bool lc1 = (avg1 >= LC_RAW_DELTA);
      bool lc2 = (avg2 >= LC_RAW_DELTA);

      bool stopped1 = fast1 || lc1;
      bool stopped2 = fast2 || lc2;
      setActuator(stopped1 ? 0 : ACT_DOWN, stopped2 ? 0 : ACT_DOWN);
      updateDist();

      // 500ms마다 delta 출력 (임계값 교정용)
      if (millis() - last_print >= 500) {
        Serial.print("[LC] delta1="); Serial.print(rdelta1);
        Serial.print(" delta2="); Serial.println(rdelta2);
        last_print = millis();
      }

      if (stopped1 && stopped2) {
        landed = true;
        setActuator(0, 0);
        Serial.print("[DESCENT] 접지! delta1="); Serial.print(rdelta1);
        Serial.print(" delta2="); Serial.print(rdelta2);
        if (fast1 || fast2) Serial.print(" (fast-stop)");
        Serial.println();
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
  {
    unsigned long t0 = millis();
    while (millis() - t0 < RETRACT_MS) {
      checkEmergency();
      bool d1 = (est_dist1 <= 0.0f);
      bool d2 = (est_dist2 <= 0.0f);
      setActuator(d1 ? 0 : ACT_UP, d2 ? 0 : ACT_UP);  // 각 축 독립 정지
      updateDist();
      if (d1 && d2) break;
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
