/**
 * @file   cleaning_test.ino
 * @brief  청소 모듈 단독 테스트 — micro-ROS 없음
 *
 *  동작 시퀀스:
 *    0s       : 슬라이드(ID 18) vel=-285 시작 + F60 흡입팬 ON
 *    0~20s    : 구동 유지 (시리얼 입력 시 즉시 전체 정지)
 *    20s      : F60 OFF + 슬라이드 vel=+285 복귀 → 종료
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

/* ── 하드웨어 ── */
#define SLIDE_ID   18
#define F60_PIN     5

#define SUCTION_ON  1600  // μs
#define SUCTION_OFF 1000  // μs
#define RUN_MS      20000UL  // 20초

/* ── 전역 객체 ── */
dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;
Servo suction_esc;

void setSlideVelocity(int32_t vel) {
  packetHandler->write4ByteTxRx(
      portHandler, SLIDE_ID, ADDR_GOAL_VELOCITY,
      (uint32_t)vel, NULL);
}

void emergencyStop() {
  setSlideVelocity(0);
  suction_esc.writeMicroseconds(SUCTION_OFF);
  Serial.println("[STOP] 비상 정지");
  while (1);
}

void setup() {
  Serial.begin(57600);

  /* ── ESC 초기화 ── */
  suction_esc.attach(F60_PIN);
  suction_esc.writeMicroseconds(SUCTION_OFF);
  delay(2000);

  /* ── 다이나믹셀 포트 ── */
  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);

  /* ── 슬라이드(ID 18): velocity mode 초기화 ── */
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

  /* ── 구동 시작 ── */
  setSlideVelocity(-285);
  suction_esc.writeMicroseconds(SUCTION_ON);
  Serial.println("[START] 슬라이드 -285, 흡입팬 ON — 20초 대기 (아무 키: 비상 정지)");

  /* ── 20초 대기 (시리얼 입력 감시) ── */
  unsigned long start = millis();
  while (millis() - start < RUN_MS) {
    if (Serial.available()) emergencyStop();
  }

  /* ── 정상 종료 ── */
  suction_esc.writeMicroseconds(SUCTION_OFF);
  setSlideVelocity(285);
  Serial.println("[DONE] 흡입팬 OFF, 슬라이드 +285 복귀");
}

void loop() {}
