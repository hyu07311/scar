/**
 * @file   scar_stair_test_v2.ino
 * @brief  SCAR 계단 극복 + 조향 90° 연속 반복 테스트 — OpenCR 직접 업로드용
 * (계단 극복 → CCW 90° 조향 → 직진 복귀 → 반복)
 */

#include <DynamixelSDK.h>

/* ================================================================
 * [1] 다이나믹셀 레지스터 (Protocol 2.0 공통)
 * ================================================================ */
#define ADDR_OPERATING_MODE    11
#define ADDR_TORQUE_ENABLE     64
#define ADDR_GOAL_VELOCITY    104
#define ADDR_PRESENT_LOAD     126
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

#define LEN_GOAL_VELOCITY      4
#define LEN_GOAL_POSITION      4

#define BAUDRATE         1000000
#define PROTOCOL_VERSION 2.0
#define DEVICENAME       "OpenCR_DXL_Port"

/* ================================================================
 * [2] 주행 모터 ID — MX-106 FL/FR/RL/RR
 * ================================================================ */
const uint8_t DRIVE_IDS[4] = {1, 3, 5, 7};

/* ================================================================
 * [3] 방향 규약
 * ================================================================ */
const int DIR_MULT[4] = {-1, -1, 1, 1};

/* ================================================================
 * [4] 속도 파라미터
 * ================================================================ */
#define SPEED_NORMAL    20
#define SPEED_CLIMB     80

/* ================================================================
 * [5] 부하 임계값
 * ================================================================ */
#define LOAD_STAIR_DETECT  400
#define LOAD_CLIMB_BOOST   500
#define LOAD_CLEAR         200

/* ================================================================
 * [6] 타이밍
 * ================================================================ */
#define LOOP_INTERVAL_MS       20
#define STAIR_CLEAR_COUNT      10
#define STAIR_CLIMB_TIMEOUT  8000

/* ================================================================
 * [7] 조향 모터 (ID 12=전륜, ID 13=후륜) — Extended Position Mode(4)
 * ================================================================ */
const uint8_t STEER_IDS[2]  = {12, 13};
const int32_t STEER_ZERO[2] = {4017, 3225};

#define STEER_VELOCITY    100
#define STEER_ACCEL        50
#define STEER_TOL          30
#define STEER_SETTLE_MS  1000
#define STEER_TIMEOUT_MS 12000
#define STEER_HOLD_MS    2000

/* ================================================================
 * [8] 상태 머신
 * ================================================================ */
enum DriveState {
  STATE_IDLE,
  STATE_APPROACH,
  STATE_STAIR_CLIMB,
  STATE_STEER_90,
  STATE_STEER_RETURN,
  STATE_DONE
};

DriveState state              = STATE_IDLE;
uint32_t   stateEntryMs       = 0;
int        stairClearCount    = 0;
int        stairsClimbed      = 0;
bool       rl_ever_loaded     = false;
bool       rr_ever_loaded     = false;
bool       steerCmdSent       = false;
bool       steerRetried       = false;
int32_t    dynamic_steer90[2]     = {0, 0};
int32_t    dynamic_steer_origin[2] = {0, 0};
uint32_t   steerReachedMs     = 0;

/* ================================================================
 * [9] DynamixelSDK 전역 객체
 * ================================================================ */
dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;

/* ================================================================
 * [10] 함수 선언
 * ================================================================ */
void    initDrive();
void    initSteer();
void    setDrive(int32_t speed);
void    stopDrive();
void    recoverSteer(uint8_t id);
void    setSteerGoal(const int32_t pos[2]);
int32_t getSteerPos(uint8_t id);
bool    steerReached(const int32_t target[2]);
int16_t getPresentLoad(uint8_t id);

/* ================================================================
 * [11] setup()
 * ================================================================ */
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort())   { while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); } }
  if (!portHandler->setBaudRate(BAUDRATE)) { while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(100); } }

  initDrive();
  initSteer();
  setSteerGoal(STEER_ZERO);
  delay(9000);

  // LED 점멸로 준비 완료 표시, 키 입력 또는 5초 후 시작
  digitalWrite(LED_BUILTIN, LOW);
  uint32_t t = millis();
  while (!Serial.available() && (millis() - t) < 5000);
  while (Serial.available()) Serial.read();

  state        = STATE_APPROACH;
  stateEntryMs = millis();
}

/* ================================================================
 * [12] loop() — 50 Hz 제어 루프
 * ================================================================ */
void loop() {
  // 키 입력 시 즉시 정지
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    if (state != STATE_IDLE && state != STATE_DONE) {
      stopDrive();
      state = STATE_DONE;
    }
    return;
  }

  static uint32_t lastLoopMs = 0;
  uint32_t now = millis();
  if (now - lastLoopMs < LOOP_INTERVAL_MS) return;
  lastLoopMs = now;

  int16_t load_fl = getPresentLoad(DRIVE_IDS[0]);
  int16_t load_fr = getPresentLoad(DRIVE_IDS[1]);
  int16_t load_rl = getPresentLoad(DRIVE_IDS[2]);
  int16_t load_rr = getPresentLoad(DRIVE_IDS[3]);

  int16_t load_max = std::max(std::max(load_fl, load_fr),
                              std::max(load_rl, load_rr));
  bool boost = (load_max > LOAD_CLIMB_BOOST);

  switch (state) {

    case STATE_APPROACH:
      setDrive(SPEED_NORMAL);
      stairClearCount = 0;
      rl_ever_loaded  = false;
      rr_ever_loaded  = false;

      if (load_fl > LOAD_STAIR_DETECT || load_fr > LOAD_STAIR_DETECT) {
        state        = STATE_STAIR_CLIMB;
        stateEntryMs = now;
      }
      break;

    case STATE_STAIR_CLIMB:
      setDrive(boost ? SPEED_CLIMB : SPEED_NORMAL);

      if (load_rl > LOAD_STAIR_DETECT) rl_ever_loaded = true;
      if (load_rr > LOAD_STAIR_DETECT) rr_ever_loaded = true;

      if ((rl_ever_loaded || rr_ever_loaded) &&
          load_fl < LOAD_CLEAR && load_fr < LOAD_CLEAR &&
          load_rl < LOAD_CLEAR && load_rr < LOAD_CLEAR) {
        stairClearCount++;
        if (stairClearCount >= STAIR_CLEAR_COUNT) {
          stopDrive();
          stairClearCount = 0;
          steerCmdSent    = false;
          stairsClimbed++;
          if (stairsClimbed >= 2) {
            stairsClimbed = 0;
            state        = STATE_STEER_90;
          } else {
            state        = STATE_APPROACH;
          }
          stateEntryMs = now;
        }
      } else {
        stairClearCount = 0;
      }

      if (now - stateEntryMs > STAIR_CLIMB_TIMEOUT) {
        stopDrive();
        stairClearCount = 0;
        steerCmdSent    = false;
        stairsClimbed++;
        if (stairsClimbed >= 2) {
          stairsClimbed = 0;
          state        = STATE_STEER_90;
        } else {
          state        = STATE_APPROACH;
        }
        stateEntryMs = now;
      }
      break;

    case STATE_STEER_90:
      if (!steerCmdSent && now - stateEntryMs >= STEER_SETTLE_MS) {
        int32_t p0 = getSteerPos(STEER_IDS[0]);
        int32_t p1 = getSteerPos(STEER_IDS[1]);
        dynamic_steer_origin[0] = p0;
        dynamic_steer_origin[1] = p1;
        dynamic_steer90[0] = p0 - 8192;
        dynamic_steer90[1] = p1 - 8192;
        setSteerGoal(dynamic_steer90);
        steerCmdSent   = true;
        steerReachedMs = 0;
      }
      if (steerReached(dynamic_steer90) && steerReachedMs == 0) steerReachedMs = now;
      if (steerReachedMs != 0 && now - steerReachedMs >= STEER_HOLD_MS) {
        steerCmdSent = false;
        steerRetried = false;
        state        = STATE_STEER_RETURN;
        stateEntryMs = now;
      } else if (now - stateEntryMs >= STEER_TIMEOUT_MS) {
        if (!steerRetried) {
          setSteerGoal(dynamic_steer90);
          steerRetried   = true;
          steerReachedMs = 0;
          stateEntryMs   = now;
        } else {
          steerCmdSent = false;
          steerRetried = false;
          state        = STATE_STEER_RETURN;
          stateEntryMs = now;
        }
      }
      break;

    case STATE_STEER_RETURN:
      if (!steerCmdSent) {
        setSteerGoal(dynamic_steer_origin);
        steerCmdSent   = true;
        steerReachedMs = 0;
      }
      if (steerReached(dynamic_steer_origin) && steerReachedMs == 0) steerReachedMs = now;
      if (steerReachedMs != 0 && now - steerReachedMs >= STEER_HOLD_MS) {
        steerCmdSent = false;
        steerRetried = false;
        state        = STATE_DONE;
        stateEntryMs = now;
      } else if (now - stateEntryMs >= STEER_TIMEOUT_MS) {
        if (!steerRetried) {
          setSteerGoal(dynamic_steer_origin);
          steerRetried   = true;
          steerReachedMs = 0;
          stateEntryMs   = now;
        } else {
          steerCmdSent = false;
          steerRetried = false;
          state        = STATE_DONE;
          stateEntryMs = now;
        }
      }
      break;

    case STATE_DONE:
      stopDrive();
      digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
      break;

    default:
      break;
  }
}

/* ================================================================
 * [13] 주행 모터 초기화 — 속도 제어 모드(1)
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
  }
}

/* ================================================================
 * [14] 조향 모터 초기화 — Extended Position Mode(4)
 * ================================================================ */
void initSteer() {
  for (int i = 0; i < 2; i++) {
    uint8_t id = STEER_IDS[i];
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, NULL);
    delay(10);
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, 4, NULL);
    delay(10);
    packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, STEER_VELOCITY, NULL);
    delay(10);
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, NULL);
    delay(10);
  }
}

/* ================================================================
 * [15] 4바퀴 동기 전진 속도 지령
 * ================================================================ */
void setDrive(int32_t speed) {
  dynamixel::GroupSyncWrite groupDrive(
      portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);

  for (int i = 0; i < 4; i++) {
    int32_t v = speed * DIR_MULT[i];
    uint8_t param[4];
    param[0] = DXL_LOBYTE(DXL_LOWORD(v));
    param[1] = DXL_HIBYTE(DXL_LOWORD(v));
    param[2] = DXL_LOBYTE(DXL_HIWORD(v));
    param[3] = DXL_HIBYTE(DXL_HIWORD(v));
    groupDrive.addParam(DRIVE_IDS[i], param);
  }
  groupDrive.txPacket();
}

/* ================================================================
 * [16] 전체 주행 정지
 * ================================================================ */
void stopDrive() {
  dynamixel::GroupSyncWrite groupStop(
      portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
  uint8_t zero[4] = {0, 0, 0, 0};
  for (int i = 0; i < 4; i++) groupStop.addParam(DRIVE_IDS[i], zero);
  groupStop.txPacket();
}

/* ================================================================
 * [17a] 조향 모터 Shutdown 복구
 * ================================================================ */
void recoverSteer(uint8_t id) {
  uint8_t hwErr = 0;
  packetHandler->read1ByteTxRx(portHandler, id, 70, &hwErr, NULL);
  if (hwErr == 0) return;

  packetHandler->reboot(portHandler, id, NULL);
  delay(500);

  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, NULL);
  delay(10);
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, 4, NULL);
  delay(10);
  packetHandler->write4ByteTxRx(portHandler, id, 108, STEER_ACCEL, NULL);
  delay(5);
  packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, STEER_VELOCITY, NULL);
  delay(5);
  packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, NULL);
  delay(20);
}

/* ================================================================
 * [17b] 조향 목표 위치 발행
 * ================================================================ */
void setSteerGoal(const int32_t pos[2]) {
  for (int i = 0; i < 2; i++) {
    recoverSteer(STEER_IDS[i]);
    packetHandler->write4ByteTxRx(portHandler, STEER_IDS[i], 108, STEER_ACCEL, NULL);
    packetHandler->write4ByteTxRx(portHandler, STEER_IDS[i], ADDR_PROFILE_VELOCITY, STEER_VELOCITY, NULL);
    delay(5);
  }

  dynamixel::GroupSyncWrite groupSteer(
      portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

  for (int i = 0; i < 2; i++) {
    uint8_t param[4];
    param[0] = DXL_LOBYTE(DXL_LOWORD(pos[i]));
    param[1] = DXL_HIBYTE(DXL_LOWORD(pos[i]));
    param[2] = DXL_LOBYTE(DXL_HIWORD(pos[i]));
    param[3] = DXL_HIBYTE(DXL_HIWORD(pos[i]));
    groupSteer.addParam(STEER_IDS[i], param);
  }
  groupSteer.txPacket();
}

/* ================================================================
 * [18] 조향 현재 위치 읽기
 * ================================================================ */
int32_t getSteerPos(uint8_t id) {
  uint32_t pos = 0;
  packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, &pos, NULL);
  return (int32_t)pos;
}

/* ================================================================
 * [19] 조향 목표 도달 판정
 * ================================================================ */
bool steerReached(const int32_t target[2]) {
  for (int i = 0; i < 2; i++) {
    if (abs(getSteerPos(STEER_IDS[i]) - target[i]) > STEER_TOL) return false;
  }
  return true;
}

/* ================================================================
 * [20] 부하 읽기
 * ================================================================ */
int16_t getPresentLoad(uint8_t id) {
  int16_t load = 0;
  packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRESENT_LOAD, (uint16_t *)&load, NULL);
  return abs(load);
}
