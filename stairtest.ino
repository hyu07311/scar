/**
 * @file   scar_stair_test_v2.ino
 * @brief  SCAR 계단 극복 독립 테스트 — OpenCR 직접 업로드용 (No ROS2 / No Xavier)
 * (평지 부하 테스트 및 공중 구동 테스트 결과 반영본)
 */

#include <DynamixelSDK.h>

/* ================================================================
 * [1] 다이나믹셀 레지스터 (Protocol 2.0 공통)
 * ================================================================ */
#define ADDR_OPERATING_MODE   11
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_VELOCITY   104
#define ADDR_PRESENT_LOAD    126

#define LEN_GOAL_VELOCITY     4

#define BAUDRATE         1000000
#define PROTOCOL_VERSION 2.0
#define DEVICENAME       "OpenCR_DXL_Port"

/* ================================================================
 * [2] 주행 모터 ID — MX-106 FL/FR/RL/RR
 * ================================================================ */
const uint8_t DRIVE_IDS[4] = {1, 3, 5, 7};
//  인덱스  0=FL(ID1)  1=FR(ID3)  2=RL(ID5)  3=RR(ID7)

/* ================================================================
 * [3] 방향 규약 (수정됨)
 * 공중 테스트 결과 반영: 모든 바퀴에 같은 부호를 주어야 같은 방향으로 구름.
 * 1 = 정방향, -1 = 역방향
 * (만약 RR 통신 복구 후 RR만 반대로 돈다면 {1, 1, 1, -1}로 수정하세요)
 * ================================================================ */
const int DIR_MULT[4] = {-1, -1, 1, 1};

/* ================================================================
 * [4] 속도 파라미터  (단위: 0.229 RPM/unit, wheel_radius=0.12m)
 * ================================================================ */
#define SPEED_NORMAL    20   //  ~4.6 RPM ≈ 0.06 m/s  (접근 / 평지, 최초 테스트)
#define SPEED_CLIMB     80   //  ~6.9 RPM ≈ 0.09 m/s  (등반 부스트)

/* ================================================================
 * [5] 부하 임계값 (수정됨: 평지 부하 테스트 결과 반영)
 * ================================================================ */
#define LOAD_STAIR_DETECT  400   // 평지 노이즈(314) 이상으로 설정
#define LOAD_CLIMB_BOOST   500   // 부스트 진입 기준값 상향
#define LOAD_CLEAR         200   // 4개 모두 이하 → 계단 통과 완료

/* ================================================================
 * [6] 타이밍
 * ================================================================ */
#define LOOP_INTERVAL_MS    20   // 제어 루프 주기 (50 Hz)
#define STAIR_CLEAR_COUNT   10   // 완료 연속 판정 횟수 (10 × 20ms = 200ms)

/* ================================================================
 * [7] 상태 머신
 * ================================================================ */
enum DriveState {
  STATE_IDLE,         // 대기 — 키 입력 또는 5초 후 시작
  STATE_APPROACH,     // 전진 접근 (SPEED_NORMAL)
  STATE_STAIR_CLIMB,  // 계단 등반 (부하 따라 NORMAL / CLIMB)
  STATE_DONE          // 완료 또는 수동 정지 — 모터 정지
};

DriveState state           = STATE_IDLE;
uint32_t   stateEntryMs    = 0;
int        stairClearCount = 0;
bool       rl_ever_loaded  = false;  // RL이 계단에 한 번이라도 걸렸는지
bool       rr_ever_loaded  = false;  // RR이 계단에 한 번이라도 걸렸는지

/* ================================================================
 * [8] DynamixelSDK 전역 객체
 * ================================================================ */
dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;

/* ================================================================
 * [9] 함수 선언
 * ================================================================ */
void        initDrive();
void        setDrive(int32_t speed);
void        stopDrive();
int16_t     getPresentLoad(uint8_t id);
void        printStatus(int16_t fl, int16_t fr, int16_t rl, int16_t rr, bool boost);
const char *stateName(DriveState s);
void        errorHalt(const char *msg);

/* ================================================================
 * [10] setup()
 * ================================================================ */
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== SCAR Stair Climb Test (v2) ===");
  Serial.println("Initializing DXL drive motors...");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // 초기화 중 LED ON

  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort())
    errorHalt("[ERROR] Port open failed");
  if (!portHandler->setBaudRate(BAUDRATE))
    errorHalt("[ERROR] Baud rate set failed");

  initDrive();

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("[OK] Init complete. Press any key (or wait 5s) to start.");
  Serial.println("     *** Press any key at any time to STOP ***");

  uint32_t t = millis();
  while (!Serial.available() && (millis() - t) < 5000);
  while (Serial.available()) Serial.read();

  state        = STATE_APPROACH;
  stateEntryMs = millis();
  Serial.println("[START] State: APPROACH");
}

/* ================================================================
 * [11] loop()  — 50 Hz 제어 루프
 * ================================================================ */
void loop() {
  // ── 키 입력 감시 (항상 최우선) ────────────────────────────────
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    if (state != STATE_IDLE && state != STATE_DONE) {
      stopDrive();
      state = STATE_DONE;
      Serial.println("[STOP] Key pressed -> DONE (motors stopped)");
    }
    return;
  }

  static uint32_t lastLoopMs = 0;
  uint32_t now = millis();
  if (now - lastLoopMs < LOOP_INTERVAL_MS) return;
  lastLoopMs = now;

  // ── 부하 읽기 (수정됨: abs 함수가 내부 적용됨) ─────────────────
  int16_t load_fl = getPresentLoad(DRIVE_IDS[0]);  // ID 1
  int16_t load_fr = getPresentLoad(DRIVE_IDS[1]);  // ID 3
  int16_t load_rl = getPresentLoad(DRIVE_IDS[2]);  // ID 5
  int16_t load_rr = getPresentLoad(DRIVE_IDS[3]);  // ID 7

  int16_t load_max = std::max(std::max(load_fl, load_fr),
                              std::max(load_rl, load_rr));
  bool boost = (load_max > LOAD_CLIMB_BOOST);

  // ── 상태 머신 ──────────────────────────────────────────────────
  switch (state) {

    // ── 전진 접근 ────────────────────────────────────────────────
    case STATE_APPROACH:
      setDrive(SPEED_NORMAL);
      stairClearCount = 0;

      if (load_fl > LOAD_STAIR_DETECT || load_fr > LOAD_STAIR_DETECT) {
        state        = STATE_STAIR_CLIMB;
        stateEntryMs = now;
        // 어느 쪽이 먼저 닿았는지 로그로 기록
        Serial.print("[STATE] APPROACH -> STAIR_CLIMB  (");
        if (load_fl > LOAD_STAIR_DETECT && load_fr > LOAD_STAIR_DETECT)
          Serial.print("FL+FR");
        else if (load_fl > LOAD_STAIR_DETECT)
          Serial.print("FL");
        else
          Serial.print("FR");
        Serial.print("=");
        Serial.print(std::max(load_fl, load_fr));
        Serial.println(")");
      }
      break;

    // ── 계단 등반 ────────────────────────────────────────────────
    case STATE_STAIR_CLIMB:
      // 부하에 따라 전체 속도 부스트 (4바퀴 동일)
      setDrive(boost ? SPEED_CLIMB : SPEED_NORMAL);

      // 후륜 접촉 여부 누적 (한 번이라도 걸렸는지 기록)
      if (load_rl > LOAD_STAIR_DETECT) rl_ever_loaded = true;
      if (load_rr > LOAD_STAIR_DETECT) rr_ever_loaded = true;

      // 완료 조건:
      //   1) 후륜 둘 다 계단에 한 번 이상 걸렸고 (전륜만 올라간 직후 오판 방지)
      //   2) 4개 부하 모두 안정 상태가 200ms 지속
      if (rl_ever_loaded && rr_ever_loaded &&
          load_fl < LOAD_CLEAR && load_fr < LOAD_CLEAR &&
          load_rl < LOAD_CLEAR && load_rr < LOAD_CLEAR) {
        stairClearCount++;
        if (stairClearCount >= STAIR_CLEAR_COUNT) {
          stopDrive();
          stairClearCount = 0;
          state           = STATE_DONE;
          stateEntryMs    = now;
          Serial.println("[STATE] STAIR_CLIMB -> DONE  (all 4 wheels cleared)");
        }
      } else {
        stairClearCount = 0;
      }
      break;

    // ── 완료 / 수동 정지 ─────────────────────────────────────────
    case STATE_DONE:
      stopDrive();
      digitalWrite(LED_BUILTIN, (millis() / 500) % 2);  // 느린 점멸
      break;

    default:
      break;
  }

  // ── 시리얼 상태 출력 (5 Hz) ────────────────────────────────────
  printStatus(load_fl, load_fr, load_rl, load_rr, boost);
}

/* ================================================================
 * [12] 주행 모터 초기화
 * Torque OFF → 속도 제어 모드(1) → Torque ON
 * ================================================================ */
void initDrive() {
  for (int i = 0; i < 4; i++) {
    uint8_t id = DRIVE_IDS[i];
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, NULL);
    delay(10);
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, 1, NULL);
    delay(10);
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, NULL);
    delay(10);
    Serial.print("[DXL] ID ");
    Serial.print(id);
    Serial.println(" initialized (velocity mode)");
  }
}

/* ================================================================
 * [13] 4바퀴 동기 전진 속도 지령 (수정됨: DIR_MULT 배열 적용)
 * ================================================================ */
void setDrive(int32_t speed) {
  int32_t v[4];
 
  // 각 바퀴별로 설정된 방향 부호 곱하기
  for (int i = 0; i < 4; i++) {
    v[i] = speed * DIR_MULT[i];
  }

  dynamixel::GroupSyncWrite groupDrive(
      portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);

  for (int i = 0; i < 4; i++) {
    uint8_t param[4];
    param[0] = DXL_LOBYTE(DXL_LOWORD(v[i]));
    param[1] = DXL_HIBYTE(DXL_LOWORD(v[i]));
    param[2] = DXL_LOBYTE(DXL_HIWORD(v[i]));
    param[3] = DXL_HIBYTE(DXL_HIWORD(v[i]));
    groupDrive.addParam(DRIVE_IDS[i], param);
  }
  groupDrive.txPacket();
}

/* ================================================================
 * [14] 전체 정지
 * ================================================================ */
void stopDrive() {
  dynamixel::GroupSyncWrite groupStop(
      portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
  uint8_t zero[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) groupStop.addParam(DRIVE_IDS[i], zero);
  groupStop.txPacket();
}

/* ================================================================
 * [15] 부하 읽기 (수정됨: Protocol 2.0 음수 부하 절댓값 처리)
 * ================================================================ */
int16_t getPresentLoad(uint8_t id) {
  int16_t load = 0;
  packetHandler->read2ByteTxRx(
      portHandler, id, ADDR_PRESENT_LOAD, (uint16_t *)&load, NULL);
  return abs(load); // 부호 제거하고 순수 부하 크기만 반환
}

/* ================================================================
 * [16] 시리얼 상태 출력 (5 Hz)
 * ================================================================ */
void printStatus(int16_t fl, int16_t fr, int16_t rl, int16_t rr, bool boost) {
  static uint32_t lastPrintMs = 0;
  if (millis() - lastPrintMs < 200) return;
  lastPrintMs = millis();

  Serial.print("[");
  Serial.print(stateName(state));
  Serial.print("] ");
  Serial.print("FL="); Serial.print(fl);
  Serial.print(" FR="); Serial.print(fr);
  Serial.print(" RL="); Serial.print(rl);
  Serial.print(" RR="); Serial.print(rr);
  if (boost) Serial.print("  <BOOST>");
  Serial.println();
}

/* ================================================================
 * [17] 유틸리티
 * ================================================================ */
const char *stateName(DriveState s) {
  switch (s) {
    case STATE_IDLE:        return "IDLE";
    case STATE_APPROACH:    return "APPROACH";
    case STATE_STAIR_CLIMB: return "CLIMB";
    case STATE_DONE:        return "DONE";
    default:                return "?";
  }
}

void errorHalt(const char *msg) {
  Serial.println(msg);
  while (1) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}
