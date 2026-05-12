/**
 * @file   scar_control.ino
 * @brief  SCAR 계단 청소 로봇 — OpenCR 펌웨어 데모 버전
 *
 *  원본(scar_control_real.ino) 대비 변경사항:
 *    - 초음파 센서 제거 (ultrasonic_left/right 항상 0 전송)
 *    - 경광등 릴레이(RELAY_PIN) 제거
 *    - 슬라이드(ID 23) Mode 4(확장위치) → Mode 1(속도)
 *      target_slide_pos 필드를 속도값으로 재사용
 *      slide_pos 항상 0 전송 (Xavier에서 시간 기반 제어)
 */

#include <micro_ros_arduino.h>
#include <DynamixelSDK.h>
#include <Servo.h>
#include <IMU.h>
#include <Wire.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <scar_interfaces/msg/scar_status.h>
#include <scar_interfaces/msg/scar_cmd.h>

/* ================================================================
 *  [1] 다이나믹셀 레지스터 주소 (Protocol 2.0 공통)
 * ================================================================ */
#define ADDR_OPERATING_MODE    11
#define ADDR_TORQUE_ENABLE     64
#define ADDR_GOAL_VELOCITY    104
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_LOAD     126

#define LEN_GOAL_VELOCITY      4
#define LEN_GOAL_POSITION      4
#define LEN_PRESENT_LOAD       2

#define BAUDRATE         1000000
#define PROTOCOL_VERSION 2.0
#define DEVICENAME       "OpenCR_DXL_Port"

/* ================================================================
 *  [2] 모터 ID 배열
 * ================================================================ */
const uint8_t DRIVE_IDS[] = {1, 3, 5, 7};  // MX-106 FL/FR/RL/RR
const uint8_t STEER_IDS[] = {12, 13};       // MX-64  전륜/후륜
const uint8_t BRUSH_IDS[] = {37, 38};       // XL430  브러시 좌/우
const uint8_t SLIDE_ID    = 18;             // MX-64  슬라이드 (속도 제어)

/* ================================================================
 *  [3] 핀 정의
 * ================================================================ */
#define LED_PIN   LED_BUILTIN

// 흡입 ESC
#define F60_PIN   5

// 리니어 액추에이터 1 (청소모듈 좌측)
#define M1_INA    2
#define M1_PWM    3
#define M1_INB    4

// 리니어 액추에이터 2 (청소모듈 우측)
#define M2_INA    7
#define M2_INB    8
#define M2_PWM    9

// 로드셀 HX711 (소프트웨어 SPI)
#define LC_DT1  A0
#define LC_SCK  A1
#define LC_DT2  A2

/* ================================================================
 *  [4] 전역 객체 및 변수
 * ================================================================ */
cIMU IMU;
Servo suction_esc;

dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;
dynamixel::GroupSyncRead *groupSyncRead_steer;

float roll_offset  = 0.0f;
float pitch_offset = 0.0f;
float yaw_offset   = 0.0f;

// 로드셀
long  tare1 = 0, tare2 = 0;
float scale = -58000.0f;

// 리니어 액추에이터 가상 거리 추정
const float ACT_MAX_SPEED  = 15.0f;
int32_t current_act1_pwm   = 0;
int32_t current_act2_pwm   = 0;
float   est_dist1           = 0.0f;
float   est_dist2           = 0.0f;

/* ================================================================
 *  [5] micro-ROS 객체
 * ================================================================ */
rcl_node_t          node;
rclc_support_t      support;
rcl_allocator_t     allocator;
rclc_executor_t     executor;
rcl_publisher_t     publisher;
rcl_subscription_t  subscriber;
rcl_timer_t         timer;

scar_interfaces__msg__ScarStatus status_msg;
scar_interfaces__msg__ScarCmd    cmd_msg;

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

/* ================================================================
 *  [6] 하드웨어 제어 함수 (HAL)
 * ================================================================ */

// ── 로드셀 HX711 소프트웨어 SPI 읽기 ─────────────────────────────
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

// ── 다이나믹셀 부하 읽기 ─────────────────────────────────────────
int16_t getPresentLoad(uint8_t id) {
  int16_t load = 0;
  packetHandler->read2ByteTxRx(
      portHandler, id, ADDR_PRESENT_LOAD, (uint16_t *)&load, NULL);
  return (load % 1024);
}

// ── 비상 정지 (모든 토크 오프) ────────────────────────────────────
void forceStopAll() {
  dynamixel::GroupSyncWrite groupStop(
      portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
  uint8_t zero[4] = {0, 0, 0, 0};
  for (uint8_t id : DRIVE_IDS) groupStop.addParam(id, zero);
  for (uint8_t id : BRUSH_IDS) groupStop.addParam(id, zero);
  groupStop.txPacket();

  for (uint8_t id : STEER_IDS)
    packetHandler->write1ByteTxRx(
        portHandler, id, ADDR_TORQUE_ENABLE, 0, NULL);

  // 슬라이드 속도 0
  packetHandler->write4ByteTxRx(
      portHandler, SLIDE_ID, ADDR_GOAL_VELOCITY, 0, NULL);

  digitalWrite(M1_INA, LOW); digitalWrite(M1_INB, LOW);
  analogWrite(M1_PWM, 0);
  digitalWrite(M2_INA, LOW); digitalWrite(M2_INB, LOW);
  analogWrite(M2_PWM, 0);
  current_act1_pwm = 0;
  current_act2_pwm = 0;

  suction_esc.writeMicroseconds(1000);
}

// ── 리니어 액추에이터 제어 ────────────────────────────────────────
void setActuator(int32_t s1, int32_t s2) {
  current_act1_pwm = s1;
  current_act2_pwm = s2;

  auto drive = [](int pA, int pB, int pP, int v) {
    digitalWrite(pA, v > 0 ? HIGH : LOW);
    digitalWrite(pB, v < 0 ? HIGH : LOW);
    analogWrite(pP, (uint8_t)constrain(abs(v), 0, 255));
  };
  drive(M1_INA, M1_INB, M1_PWM, (int)s1);
  drive(M2_INA, M2_INB, M2_PWM, (int)s2);
}

// ── 조향 동기화 구동 (MX-64, 위치 제어) ──────────────────────────
void setSteerSync(int32_t pos_f, int32_t pos_r) {
  dynamixel::GroupSyncWrite groupSteer(
      portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

  uint8_t param_f[4], param_r[4];
  param_f[0] = DXL_LOBYTE(DXL_LOWORD(pos_f));
  param_f[1] = DXL_HIBYTE(DXL_LOWORD(pos_f));
  param_f[2] = DXL_LOBYTE(DXL_HIWORD(pos_f));
  param_f[3] = DXL_HIBYTE(DXL_HIWORD(pos_f));
  param_r[0] = DXL_LOBYTE(DXL_LOWORD(pos_r));
  param_r[1] = DXL_HIBYTE(DXL_LOWORD(pos_r));
  param_r[2] = DXL_LOBYTE(DXL_HIWORD(pos_r));
  param_r[3] = DXL_HIBYTE(DXL_HIWORD(pos_r));

  groupSteer.addParam(STEER_IDS[0], param_f);
  groupSteer.addParam(STEER_IDS[1], param_r);
  groupSteer.txPacket();
}

// ── 주행 구동 (MX-106, 속도 제어) ────────────────────────────────
void setDriveSplitSync(
    int32_t vFL, int32_t vFR, int32_t vRL, int32_t vRR) {
  dynamixel::GroupSyncWrite groupDrive(
      portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
  int32_t v[4] = {vFL, vFR, vRL, vRR};
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

// ── 청소 브러시 + 흡입팬 구동 ────────────────────────────────────
void setCleaningSync(int32_t brush_speed, int16_t suction_pwm) {
  suction_esc.writeMicroseconds(suction_pwm);

  dynamixel::GroupSyncWrite groupBrush(
      portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY);
  int32_t b[2] = {brush_speed, -brush_speed};
  for (int i = 0; i < 2; i++) {
    uint8_t param[4];
    param[0] = DXL_LOBYTE(DXL_LOWORD(b[i]));
    param[1] = DXL_HIBYTE(DXL_LOWORD(b[i]));
    param[2] = DXL_LOBYTE(DXL_HIWORD(b[i]));
    param[3] = DXL_HIBYTE(DXL_HIWORD(b[i]));
    groupBrush.addParam(BRUSH_IDS[i], param);
  }
  groupBrush.txPacket();
}

// ── 다이나믹셀 초기화 유틸리티 ───────────────────────────────────
void initDXL_Robust(const uint8_t ids[], size_t n, uint8_t opMode) {
  for (size_t i = 0; i < n; i++) {
    packetHandler->write1ByteTxRx(
        portHandler, ids[i], ADDR_TORQUE_ENABLE, 0, NULL); delay(10);
    packetHandler->write1ByteTxRx(
        portHandler, ids[i], ADDR_OPERATING_MODE, opMode, NULL); delay(10);
    packetHandler->write1ByteTxRx(
        portHandler, ids[i], ADDR_TORQUE_ENABLE, 1, NULL); delay(10);
  }
}

/* ================================================================
 *  [7] ROS2 콜백 함수
 * ================================================================ */

void cmd_callback(const void *msgin) {
  const scar_interfaces__msg__ScarCmd *msg =
      (const scar_interfaces__msg__ScarCmd *)msgin;

  if (msg->emergency_stop == 1) {
    forceStopAll();
    return;
  }

  // 1. 주행 명령
  setDriveSplitSync(
      msg->target_vel_fl, msg->target_vel_fr,
      msg->target_vel_rl, msg->target_vel_rr);

  // 2. 조향 명령
  setSteerSync(msg->target_steer_pos_l, msg->target_steer_pos_r);

  // 3. 리니어 액추에이터 명령
  setActuator(msg->target_actuator_1, msg->target_actuator_2);

  // 4. 청소 브러시 + 흡입팬 명령
  setCleaningSync(msg->target_brush_speed, msg->target_suction_pwm);

  // 5. 슬라이드 명령 (데모: 속도 제어 — target_slide_pos를 velocity로 사용)
  packetHandler->write4ByteTxRx(
      portHandler, SLIDE_ID,
      ADDR_GOAL_VELOCITY, (uint32_t)msg->target_slide_pos, NULL);
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  // ── IMU 읽기 ────────────────────────────────────────────────
  IMU.update();
  float roll  = IMU.rpy[0] - roll_offset;
  float pitch = IMU.rpy[1] - pitch_offset;
  float yaw   = IMU.rpy[2] - yaw_offset;

  // ── 로드셀 읽기 ─────────────────────────────────────────────
  long r1, r2;
  float w1 = 0.0f, w2 = 0.0f;
  if (readLoadCell(&r1, &r2)) {
    w1 = (float)(r1 - tare1) / scale;
    w2 = (float)(r2 - tare2) / scale;
  }

  // ── 리니어 액추에이터 거리 추정 ─────────────────────────────
  est_dist1 += (current_act1_pwm / 255.0f) * ACT_MAX_SPEED * 0.01f;
  est_dist2 += (current_act2_pwm / 255.0f) * ACT_MAX_SPEED * 0.01f;

  // ── 조향 현재 위치 읽기 ──────────────────────────────────────
  groupSyncRead_steer->txRxPacket();
  int32_t p_steer_f = 0, p_steer_r = 0;
  if (groupSyncRead_steer->isAvailable(
          STEER_IDS[0], ADDR_PRESENT_POSITION, LEN_GOAL_POSITION))
    p_steer_f = (int32_t)groupSyncRead_steer->getData(
        STEER_IDS[0], ADDR_PRESENT_POSITION, LEN_GOAL_POSITION);
  if (groupSyncRead_steer->isAvailable(
          STEER_IDS[1], ADDR_PRESENT_POSITION, LEN_GOAL_POSITION))
    p_steer_r = (int32_t)groupSyncRead_steer->getData(
        STEER_IDS[1], ADDR_PRESENT_POSITION, LEN_GOAL_POSITION);

  // ── ScarStatus 메시지 패킹 ───────────────────────────────────
  status_msg.pitch_angle      = pitch;
  status_msg.roll_angle       = roll;
  status_msg.yaw_angle        = yaw;
  status_msg.ultrasonic_left  = 0.0f;   // 데모: 초음파 미사용
  status_msg.ultrasonic_right = 0.0f;
  status_msg.loadcell_l       = w1;
  status_msg.loadcell_r       = w2;
  status_msg.linear_dist_1    = est_dist1;
  status_msg.linear_dist_2    = est_dist2;
  status_msg.load_fl          = getPresentLoad(DRIVE_IDS[0]);
  status_msg.load_fr          = getPresentLoad(DRIVE_IDS[1]);
  status_msg.load_rl          = getPresentLoad(DRIVE_IDS[2]);
  status_msg.load_rr          = getPresentLoad(DRIVE_IDS[3]);
  status_msg.steer_pos_l      = p_steer_f;
  status_msg.steer_pos_r      = p_steer_r;
  status_msg.slide_pos        = 0;      // 데모: 슬라이드 위치 미사용

  RCSOFTCHECK(rcl_publish(&publisher, &status_msg, NULL));
}

/* ================================================================
 *  [8] 초기화 (setup)
 * ================================================================ */
void setup() {
  set_microros_transports();

  // ── 핀 모드 설정 ────────────────────────────────────────────
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH);

  pinMode(M1_INA, OUTPUT); pinMode(M1_INB, OUTPUT); pinMode(M1_PWM, OUTPUT);
  pinMode(M2_INA, OUTPUT); pinMode(M2_INB, OUTPUT); pinMode(M2_PWM, OUTPUT);
  digitalWrite(M1_INA, LOW); digitalWrite(M1_INB, LOW); analogWrite(M1_PWM, 0);
  digitalWrite(M2_INA, LOW); digitalWrite(M2_INB, LOW); analogWrite(M2_PWM, 0);

  pinMode(LC_SCK, OUTPUT);
  pinMode(LC_DT1, INPUT);
  pinMode(LC_DT2, INPUT);

  // ── IMU 초기화 및 캘리브레이션 ─────────────────────────────
  IMU.begin();
  delay(500);

  float sum_r = 0.0f, sum_p = 0.0f, sum_y = 0.0f;
  for (int i = 0; i < 100; i++) {
    IMU.update();
    sum_r += IMU.rpy[0];
    sum_p += IMU.rpy[1];
    sum_y += IMU.rpy[2];
    delay(10);
  }
  roll_offset  = sum_r / 100.0f;
  pitch_offset = sum_p / 100.0f;
  yaw_offset   = sum_y / 100.0f;

  // ── 흡입 ESC 초기화 ─────────────────────────────────────────
  suction_esc.attach(F60_PIN);
  suction_esc.writeMicroseconds(1000);
  delay(2000);

  // ── 다이나믹셀 포트 개방 ────────────────────────────────────
  portHandler  = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);

  // ── 모터 모드 초기화 ────────────────────────────────────────
  initDXL_Robust(DRIVE_IDS, 4, 1);   // MX-106: 속도 제어
  initDXL_Robust(STEER_IDS, 2, 3);   // MX-64:  위치 제어

  for (uint8_t id : STEER_IDS) {
    packetHandler->write4ByteTxRx(
        portHandler, id, ADDR_PROFILE_VELOCITY, 20, NULL);
    delay(5);
  }

  initDXL_Robust(BRUSH_IDS, 2, 1);   // XL430: 속도 제어

  // 슬라이드(ID 23): 데모용 속도 제어 모드 (Mode 1)
  uint8_t slide_arr[1] = {SLIDE_ID};
  initDXL_Robust(slide_arr, 1, 1);

  // ── GroupSyncRead 설정 (조향 위치 피드백) ───────────────────
  groupSyncRead_steer = new dynamixel::GroupSyncRead(
      portHandler, packetHandler,
      ADDR_PRESENT_POSITION, LEN_GOAL_POSITION);
  for (uint8_t id : STEER_IDS)
    groupSyncRead_steer->addParam(id);

  // ── 로드셀 영점 측정 ────────────────────────────────────────
  long t1, t2;
  while (!readLoadCell(&t1, &t2));
  tare1 = t1;
  tare2 = t2;

  // ── micro-ROS 초기화 ─────────────────────────────────────────
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "scar_opencr_hal", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(scar_interfaces, msg, ScarStatus),
      "scar_status"));

  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(scar_interfaces, msg, ScarCmd),
      "scar_cmd"));

  RCCHECK(rclc_timer_init_default(
      &timer, &support, RCL_MS_TO_NS(10), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &cmd_msg, &cmd_callback, ON_NEW_DATA));

  digitalWrite(LED_PIN, LOW);
}

/* ================================================================
 *  [9] 메인 루프
 * ================================================================ */
void loop() {
  delay(1);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
