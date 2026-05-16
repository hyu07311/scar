/**
 * @file   scar_state_manager.cpp
 * @brief  SCAR 계단 청소 로봇 — 상태 관리 + 역기구학 통합 노드 (ROS2 Foxy)
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │                      토픽 인터페이스                             │
 * ├──────────────┬──────────────────────────────────┬───────────────┤
 * │ Subscribe    │ /scar_status                     │ ScarStatus    │
 * │              │ /human_detected                  │ Bool          │
 * │              │ /stair_lateral_error             │ Float32 (m)   │
 * │              │ /cmd_vel  (Nav2)                 │ Twist         │
 * ├──────────────┼──────────────────────────────────┼───────────────┤
 * │ Publish      │ /scar_cmd                        │ ScarCmd       │
 * └──────────────┴──────────────────────────────────┴───────────────┘
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │                     역기구학 좌표계 정의                         │
 * │  x = 로봇 전방(+)  y = 로봇 좌측(+)  ω = CCW(+)               │
 * │                                                                  │
 * │  v_FL = (vx − ω·W/2, vy + ω·L/2)   FL:( L/2, W/2)            │
 * │  v_FR = (vx + ω·W/2, vy + ω·L/2)   FR:( L/2,−W/2)            │
 * │  v_RL = (vx − ω·W/2, vy − ω·L/2)   RL:(−L/2, W/2)            │
 * │  v_RR = (vx + ω·W/2, vy − ω·L/2)   RR:(−L/2,−W/2)            │
 * │                                                                  │
 * │  θ_front = atan2(vy+ω·L/2, vx)   ← ID 12 (전륜 조향)          │
 * │  θ_rear  = atan2(vy−ω·L/2, vx)   ← ID 13 (후륜 조향)          │
 * │                                                                  │
 * │  steer_pulse = zero + STEER_SIGN × (θ·4096/2π)                 │
 * │  STEER_SIGN = −1 : 2048→1024 (−1024 pulse) = +90° 실측 확인   │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * ┌──────────────────────────────────────────────────────────────────┐
 * │                     경광등 동작 정책                             │
 * │  IDLE / EMERGENCY_STOP  → OFF (warning_light = 0)              │
 * │  인체 감지 정지 중       → OFF (혼동 방지)                      │
 * │  AUTO 작업 중 (나머지)   → ON  (warning_light = 1)             │
 * │  KEYBOARD 모드           → OFF (수동 제어 중)                   │
 * └──────────────────────────────────────────────────────────────────┘
 *
 * [CMakeLists.txt 추가]
 *   find_package(geometry_msgs REQUIRED)
 *   find_package(std_msgs REQUIRED)
 *   add_executable(scar_state_manager src/scar_state_manager.cpp)
 *   ament_target_dependencies(scar_state_manager
 *     rclcpp scar_interfaces geometry_msgs std_msgs)
 *   install(TARGETS scar_state_manager DESTINATION lib/${PROJECT_NAME})
 */

#include <rclcpp/rclcpp.hpp>
#include <scar_interfaces/msg/scar_status.hpp>
#include <scar_interfaces/msg/scar_cmd.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;
using ScarStatus = scar_interfaces::msg::ScarStatus;
using ScarCmd    = scar_interfaces::msg::ScarCmd;
using Twist      = geometry_msgs::msg::Twist;

/* ================================================================
 *  [1] 고정 하드웨어 상수
 * ================================================================ */
// MX-64: 0.229 RPM/unit → rad/s/unit
constexpr double  DXL_VEL_UNIT  = 0.229 * 2.0 * M_PI / 60.0;
constexpr int32_t PULSE_PER_REV = 4096;

// 조향 부호: 2048→1024 (−1024 pulse) = 원하는 +90° 방향
// 기어 방향이 반대이면 +1로 변경
constexpr int STEER_SIGN = -1;

// ESC PWM (μs)
constexpr int16_t SUCTION_ON  = 1600;
constexpr int16_t SUCTION_OFF = 1000;

// 리니어 액추에이터 PWM (±255)
constexpr int32_t ACT_DOWN    =  200;   // M2 하강
constexpr int32_t ACT_DOWN_M1 =  120;   // M1 하강 (M1이 빠르므로 감속 → 충격 전류 감소)
constexpr int32_t ACT_UP      = -200;
constexpr double  ACT_COAST_S =  0.5;   // 접지 후 드라이버 회복 대기 (초)

// 브러시 (DXL velocity unit)
constexpr int32_t BRUSH_SPEED = 265;

// 초음파 (m)
constexpr float US_POLE_DIST = 0.08f;
constexpr float US_WALL_DIST = 0.06f;

// 전복 비상 정지 각도 (°)
constexpr float TILT_EMERGENCY   = 35.0f;
// 다음 계단 판단 pitch (°)
constexpr float PITCH_NEXT_STAIR = 3.0f;

// Nav2 /cmd_vel 타임아웃 (s)
constexpr double CMD_VEL_TIMEOUT = 0.5;

// 슬라이드 도달 허용 오차 (pulse)
constexpr int32_t SLIDE_TOL = 30;

// 인체 감지 재개 확인 타이머 (s)
// person_node의 resume_wait_sec 과 별개로 Xavier 측에서 추가 확인
constexpr double HUMAN_RESUME_CONFIRM = 1.0;

// 조향 타이밍 / 허용 오차 (테스트 검증값)
constexpr double  STEER_SETTLE_S  = 1.0;   // 등반 후 정착 대기
constexpr double  STEER_TIMEOUT_S = 12.0;  // 조향 타임아웃 (유성 기어 8:1, 720° 이동)
constexpr double  STEER_HOLD_S    = 2.0;   // 목표 도달 후 유지
constexpr int32_t STEER_TOL_PULSE = 30;    // 도달 판정 허용 오차 (pulse)
constexpr int32_t STEER_GEAR_RATIO = 8;    // 유성 기어비

// 계단 극복 확인 (sketch_apr27b 기반)
constexpr int     STAIR_CLEAR_COUNT = 10;  // 연속 확인 횟수 (×50ms = 500ms)
constexpr int32_t BOOST_LOAD        = 500; // 부스트 진입 부하 임계값

/* ================================================================
 *  [2] 열거형
 * ================================================================ */
enum class RobotMode { KEYBOARD = 0, AUTO };

enum class State {
  IDLE = 0,
  VISION_ALIGN,       // YOLO 계단 좌측 정렬
  STAIR_APPROACH,     //  1. 계단 접근 + 전륜 정렬
  FRONT_CLIMB,        //  2. 전륜 등반 (PATS Wheel 독립 감시)
  FRONT_STABILIZE,    //  3. 전륜 안착 대기
  POLE_Y_ALIGN,       //  4. Y축 봉 정렬
  TILTED_CLEAN_INIT,  //  5. 조향 90° 초기화
  CLEANING_DOWN,      //  6. 리니어 하강 → 로드셀 접지 대기
  CRAB_WALK_CLEAN,    //  7. PID 측면 청소 주행 (L→R)
  CLEAN_STABILIZE,    //  8. 청소 완료 + 슬라이드 우측 밀착
  RECOVERY_UP,        //  9. 리니어 복귀 (추정 거리 기반)
  RETURN_LEFT_RUN,    // 10. 좌측 복귀 주행
  RETURN_STABILIZE,   // 11. 복귀 완료 + 슬라이드 좌측 밀착
  STEER_RESET,        // 12. 조향 + 슬라이드 원위치
  STEER_STABILIZE,    // 13. 안정화 대기
  REAR_CLIMB,         // 14. 후륜 등반 (PATS Wheel 독립 감시)
  HUMAN_PAUSE,        // 15. 인체 감지 일시 정지
  EMERGENCY_STOP      // 16. 비상 정지
};

static const char* state_str(State s) {
  switch (s) {
    case State::IDLE:              return "IDLE";
    case State::VISION_ALIGN:      return "VISION_ALIGN";
    case State::STAIR_APPROACH:    return "STAIR_APPROACH";
    case State::FRONT_CLIMB:       return "FRONT_CLIMB";
    case State::FRONT_STABILIZE:   return "FRONT_STABILIZE";
    case State::POLE_Y_ALIGN:      return "POLE_Y_ALIGN";
    case State::TILTED_CLEAN_INIT: return "TILTED_CLEAN_INIT";
    case State::CLEANING_DOWN:     return "CLEANING_DOWN";
    case State::CRAB_WALK_CLEAN:   return "CRAB_WALK_CLEAN";
    case State::CLEAN_STABILIZE:   return "CLEAN_STABILIZE";
    case State::RECOVERY_UP:       return "RECOVERY_UP";
    case State::RETURN_LEFT_RUN:   return "RETURN_LEFT_RUN";
    case State::RETURN_STABILIZE:  return "RETURN_STABILIZE";
    case State::STEER_RESET:       return "STEER_RESET";
    case State::STEER_STABILIZE:   return "STEER_STABILIZE";
    case State::REAR_CLIMB:        return "REAR_CLIMB";
    case State::HUMAN_PAUSE:       return "HUMAN_PAUSE";
    case State::EMERGENCY_STOP:    return "EMERGENCY_STOP";
    default:                       return "UNKNOWN";
  }
}

enum class SlideDir { CENTER = 0, RIGHT, LEFT };

/* ================================================================
 *  [3] PID 컨트롤러
 * ================================================================ */
struct PID {
  double kp, ki, kd, i_max;
  double integral = 0.0, prev_err = 0.0;

  double update(double err, double dt) {
    if (dt < 1e-6) return 0.0;
    integral = std::clamp(integral + err * dt, -i_max, i_max);
    double d = (err - prev_err) / dt;
    prev_err = err;
    return kp * err + ki * integral + kd * d;
  }
  void reset() { integral = prev_err = 0.0; }
};

/* ================================================================
 *  [4] 역기구학 출력
 * ================================================================ */
struct WheelCmds {
  int32_t steer_f = 0;  // ID 12 전륜 조향 pulse
  int32_t steer_r = 0;  // ID 13 후륜 조향 pulse
  int32_t vFL = 0, vFR = 0, vRL = 0, vRR = 0;
};

/* ================================================================
 *  [5] ScarStateManager 노드
 * ================================================================ */
class ScarStateManager : public rclcpp::Node {
 public:
  ScarStateManager() : Node("scar_state_manager"),
                       mode_(RobotMode::KEYBOARD),
                       state_(State::IDLE)
  {
    /* ── [5.1] 파라미터 서버 ─────────────────────────────────── */
    wheel_radius_ = declare_parameter("wheel_radius",        0.12);
    half_L_       = declare_parameter("wheel_base_x",        0.30) / 2.0;
    half_W_       = declare_parameter("wheel_track",         0.28) / 2.0;

    spd_approach_     = declare_parameter("speed_approach_mps",    0.06);
    spd_climb_f_      = declare_parameter("speed_climb_f_mps",     0.23);
    spd_climb_r_      = declare_parameter("speed_climb_r_mps",     0.23);
    spd_climb_boost_  = declare_parameter("speed_climb_boost_mps", 0.30);
    spd_crab_     = declare_parameter("speed_crab_mps",      0.20);
    spd_align_    = declare_parameter("speed_align_mps",     0.10);
    spd_return_   = declare_parameter("speed_return_mps",    0.20);
    spd_slow_     = declare_parameter("speed_slow_mps",      0.05);

    load_contact_ = (int16_t)declare_parameter("load_stair_contact", 400);
    load_stable_  = (int16_t)declare_parameter("load_stair_stable",  200);
    load_diff_    = (int16_t)declare_parameter("load_diff_limit",    250);
    load_emg_     = (int16_t)declare_parameter("load_emergency",     900);

    lc_kg_        = declare_parameter("lc_contact_kg",       1.0);
    vision_tol_   = declare_parameter("vision_align_tol",    0.05);

    // 슬라이드 (ID 18, Velocity Mode): -285 = 우측, +285 = 좌측
    slide_vel_   = (int32_t)declare_parameter("slide_velocity",  285);
    slide_run_s_ = declare_parameter("slide_run_s", 20.0);

    // 인체 감지 임계 거리 (person_node에서 Bool로 변환되어 오지만 예비)
    human_stop_dist_ = declare_parameter("human_stop_distance", 3.0);

    // 데모 환경 시간 기반 제어 (초음파/슬라이드 위치 대체)
    time_pole_align_s_  = declare_parameter("time_pole_align_s",  3.0);
    time_crab_clean_s_  = declare_parameter("time_crab_clean_s",  3.0);
    time_return_left_s_ = declare_parameter("time_return_left_s", 3.0);

    /* ── [5.2] ROS 인터페이스 ────────────────────────────────── */
    cmd_pub_ = create_publisher<ScarCmd>("/scar_cmd", 10);

    // /scar_status 구독 — 조향 zero 첫 수신 시 캡처
    status_sub_ = create_subscription<ScarStatus>(
      "/scar_status", 10,
      [this](ScarStatus::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(st_mtx_);
        st_      = *msg;
        st_ready_ = true;
        if (!steer_zeroed_) {
          zero_f_       = msg->steer_pos_l;  // ID 12 전륜
          zero_r_       = msg->steer_pos_r;  // ID 13 후륜
          steer_zeroed_ = true;
          RCLCPP_INFO(get_logger(),
            "[CALIB] 조향 zero 캡처: 전륜=%d 후륜=%d", zero_f_, zero_r_);
        }
      });

    // /human_detected 구독 — person_node (scar_vision) 에서 발행
    human_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/human_detected", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        bool detected = msg->data;
        human_detected_.store(detected);

        // 감지 해제 시점 기록 (재개 확인 타이머용)
        if (!detected && human_was_detected_) {
          human_clear_time_ = get_clock()->now();
        }
        human_was_detected_ = detected;
      });

    // /stair_lateral_error 구독 — stair_node (scar_vision) 에서 발행
    vision_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/stair_lateral_error", 10,
      [this](std_msgs::msg::Float32::SharedPtr msg) {
        lat_err_.store(msg->data);
        lat_err_stamp_ = get_clock()->now();
      });

    // /cmd_vel 구독 — Nav2 자율주행 (IDLE 상태에서 passthrough)
    cmdvel_sub_ = create_subscription<Twist>(
      "/cmd_vel", 10,
      [this](Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(cv_mtx_);
        cv_       = *msg;
        cv_stamp_ = get_clock()->now();
      });

    // 20Hz 제어 타이머
    timer_ = create_wall_timer(50ms, [this]() { tick(); });

    // 키보드 입력 스레드
    key_thread_ = std::thread([this]() { keyboard_loop(); });

    RCLCPP_INFO(get_logger(), "ScarStateManager 시작 — KEYBOARD 모드");
    print_help();
  }

  ~ScarStateManager() {
    running_.store(false);
    if (key_thread_.joinable()) key_thread_.join();
  }

 private:
  /* ── ROS 인터페이스 ─────────────────────────────────────────── */
  rclcpp::Publisher<ScarCmd>::SharedPtr                    cmd_pub_;
  rclcpp::Subscription<ScarStatus>::SharedPtr              status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr     human_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr  vision_sub_;
  rclcpp::Subscription<Twist>::SharedPtr                   cmdvel_sub_;
  rclcpp::TimerBase::SharedPtr                             timer_;

  /* ── 공유 데이터 ─────────────────────────────────────────────── */
  std::mutex   st_mtx_;
  ScarStatus   st_{};
  bool         st_ready_ = false;

  std::mutex   cv_mtx_;
  Twist        cv_{};
  rclcpp::Time cv_stamp_{0, 0, RCL_ROS_TIME};

  // 비전 데이터
  std::atomic<float> lat_err_{0.0f};
  rclcpp::Time       lat_err_stamp_{0, 0, RCL_ROS_TIME};

  // 인체 감지 데이터
  std::atomic<bool> human_detected_{false};
  bool              human_was_detected_ = false;
  rclcpp::Time      human_clear_time_{0, 0, RCL_ROS_TIME};

  /* ── 파라미터 ────────────────────────────────────────────────── */
  double  wheel_radius_, half_L_, half_W_;
  double  spd_approach_, spd_climb_f_, spd_climb_r_, spd_climb_boost_;
  double  spd_crab_, spd_align_, spd_return_, spd_slow_;
  int16_t load_contact_, load_stable_, load_diff_, load_emg_;
  double  lc_kg_, vision_tol_, human_stop_dist_;
  int32_t slide_vel_;
  double  slide_run_s_;
  double  time_pole_align_s_, time_crab_clean_s_, time_return_left_s_;

  /* ── 조향 zero 기준 ──────────────────────────────────────────── */
  bool    steer_zeroed_ = false;
  int32_t zero_f_ = 2048;
  int32_t zero_r_ = 2048;

  /* ── 슬라이드 (ID 18, Velocity Mode) ────────────────────────── */
  SlideDir slide_dir_      = SlideDir::CENTER;
  bool     slide_cmd_sent_ = false;

  /* ── 상태 머신 ───────────────────────────────────────────────── */
  RobotMode    mode_;
  State        state_;
  State        pre_pause_state_ = State::IDLE;  // 인체 감지 전 복귀용
  rclcpp::Time entry_t_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_t_ {0, 0, RCL_ROS_TIME};

  // PATS Wheel 독립 극복 플래그
  bool fl_climbed_ = false, fr_climbed_ = false;
  bool rl_climbed_ = false, rr_climbed_ = false;

  // 계단 극복 연속 확인 카운터 (sketch_apr27b)
  int  stair_clear_count_ = 0;

  // CLEANING_DOWN 접지 플래그 (transition()에서 리셋)
  bool act1_grounded_ = false;
  bool act2_grounded_ = false;
  rclcpp::Time both_grounded_t_{0, 0, RCL_ROS_TIME};

  double stabilized_pitch_ = 0.0;

  // 조향 동적 목표 + 도달 판정
  bool          steer_cmd_sent_   = false;
  rclcpp::Time  steer_reached_t_{0, 0, RCL_ROS_TIME};
  int32_t       target_steer_f_   = 0;
  int32_t       target_steer_r_   = 0;
  bool          steer_retried_    = false;

  // 2계단 카운터 (데모: 2번째 계단 오른 후에만 청소)
  int           stairs_climbed_   = 0;

  PID pid_cp_{6.0, 0.2, 0.8, 50.0};   // Crab Pitch PID
  PID pid_cy_{4.5, 0.15, 0.5, 50.0};  // Crab Yaw PID

  /* ── 키보드 스레드 ───────────────────────────────────────────── */
  std::atomic<bool> running_{true};
  std::thread       key_thread_;
  std::mutex        key_mtx_;

  double   kv_ = 0.0;
  double   ks_ = 0.0;
  int32_t  kact1_ = 0, kact2_ = 0;
  int32_t  kslide_ = 0, kbrush_ = 0;
  int16_t  ksuction_ = SUCTION_OFF;

  std::atomic<bool> estop_active_{false};
  std::atomic<bool> ev_mode_toggle_{false};
  std::atomic<bool> ev_start_vision_{false};
  std::atomic<bool> ev_start_direct_{false};
  std::atomic<bool> ev_idle_{false};

  /* ================================================================
   *  [6] 경광등 정책
   *
   *  warning_light = 1 (ON)  조건:
   *    - AUTO 모드 + 실제 작업 중 (IDLE/EMERGENCY_STOP/HUMAN_PAUSE 제외)
   *  warning_light = 0 (OFF) 조건:
   *    - KEYBOARD 모드
   *    - IDLE / EMERGENCY_STOP
   *    - 인체 감지로 HUMAN_PAUSE 중 (경광등이 켜져 있으면 혼동 가능)
   * ================================================================ */
  int8_t compute_warning_light() const {
    return 0;  // 발표 버전: 경광등 항상 OFF
  }

  /* ================================================================
   *  [7] 역기구학 엔진
   * ================================================================ */
  WheelCmds inverse_kinematics(double vx, double vy, double omega) const {
    WheelCmds wc;

    double fl_vx = vx - omega * half_W_, fl_vy = vy + omega * half_L_;
    double fr_vx = vx + omega * half_W_, fr_vy = vy + omega * half_L_;
    double rl_vx = vx - omega * half_W_, rl_vy = vy - omega * half_L_;
    double rr_vx = vx + omega * half_W_, rr_vy = vy - omega * half_L_;

    double theta_f = std::atan2(vy + omega * half_L_, vx);
    double theta_r = std::atan2(vy - omega * half_L_, vx);

    auto wheel_rps = [&](double wx, double wy, double theta) {
      return (wx * std::cos(theta) + wy * std::sin(theta)) / wheel_radius_;
    };
    wc.vFL = to_dxl(wheel_rps(fl_vx, fl_vy, theta_f));
    wc.vFR = to_dxl(wheel_rps(fr_vx, fr_vy, theta_f));
    wc.vRL = to_dxl(wheel_rps(rl_vx, rl_vy, theta_r));
    wc.vRR = to_dxl(wheel_rps(rr_vx, rr_vy, theta_r));

    wc.steer_f = zero_f_ + STEER_SIGN *
        static_cast<int32_t>(theta_f * PULSE_PER_REV * STEER_GEAR_RATIO / (2.0 * M_PI));
    wc.steer_r = zero_r_ + STEER_SIGN *
        static_cast<int32_t>(theta_r * PULSE_PER_REV * STEER_GEAR_RATIO / (2.0 * M_PI));
    return wc;
  }

  WheelCmds ik_straight(double mps) const {
    return inverse_kinematics(mps, 0.0, 0.0);
  }
  WheelCmds ik_crab(double mps_y) const {
    return inverse_kinematics(0.0, mps_y, 0.0);
  }
  WheelCmds ik_steer_only(double theta_f_rad, double theta_r_rad) const {
    WheelCmds wc;
    wc.steer_f = zero_f_ + STEER_SIGN *
        static_cast<int32_t>(theta_f_rad * PULSE_PER_REV * STEER_GEAR_RATIO / (2.0 * M_PI));
    wc.steer_r = zero_r_ + STEER_SIGN *
        static_cast<int32_t>(theta_r_rad * PULSE_PER_REV * STEER_GEAR_RATIO / (2.0 * M_PI));
    return wc;
  }

  int32_t to_dxl(double rps) const {
    return static_cast<int32_t>(std::clamp(rps / DXL_VEL_UNIT, -1023.0, 1023.0));
  }
  int32_t mps_to_dxl(double mps) const {
    return to_dxl(std::abs(mps) / wheel_radius_);
  }

  /* ── WheelCmds → ScarCmd 패킹 ──────────────────────────────── */
  // FL/FR 모터는 물리적으로 반대 방향 장착 (sketch_apr27b DIR_MULT {-1,-1,1,1} 동일)
  void pack_wheel(ScarCmd& cmd, const WheelCmds& wc) const {
    cmd.target_vel_fl      = -wc.vFL;
    cmd.target_vel_fr      = -wc.vFR;
    cmd.target_vel_rl      =  wc.vRL;
    cmd.target_vel_rr      =  wc.vRR;
    cmd.target_steer_pos_l = wc.steer_f;
    cmd.target_steer_pos_r = wc.steer_r;
  }

  /* ── 슬라이드 헬퍼 (ID 18, Velocity Mode) ───────────────────── */
  // RIGHT: -slide_vel_ (우측), LEFT: +slide_vel_ (좌측), CENTER: 정지
  void apply_slide(ScarCmd& cmd, SlideDir dir) {
    switch (dir) {
      case SlideDir::CENTER: cmd.target_slide_pos =  0;            break;
      case SlideDir::RIGHT:  cmd.target_slide_pos = -slide_vel_;   break;
      case SlideDir::LEFT:   cmd.target_slide_pos = +slide_vel_;   break;
    }
    slide_dir_ = dir;
  }

  /* ── 안전 체크 ──────────────────────────────────────────────── */
  bool is_tilting(const ScarStatus& s) const {
    return std::abs(s.pitch_angle) > TILT_EMERGENCY ||
           std::abs(s.roll_angle)  > TILT_EMERGENCY;
  }
  bool motor_overloaded(const ScarStatus& s) const {
    return s.load_fl > load_emg_ || s.load_fr > load_emg_ ||
           s.load_rl > load_emg_ || s.load_rr > load_emg_;
  }

  /* ── 유틸리티 ───────────────────────────────────────────────── */
  void transition(State next, const rclcpp::Time& now) {
    RCLCPP_INFO(get_logger(), "[STATE] %s → %s",
                state_str(state_), state_str(next));
    state_          = next;
    entry_t_        = now;
    fl_climbed_        = fr_climbed_ = rl_climbed_ = rr_climbed_ = false;
    stair_clear_count_ = 0;
    act1_grounded_ = act2_grounded_ = false;
    both_grounded_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    slide_cmd_sent_  = false;
    steer_cmd_sent_  = false;
    steer_retried_   = false;
    steer_reached_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    pid_cp_.reset(); pid_cy_.reset();
  }

  double elapsed(const rclcpp::Time& now) const {
    return (entry_t_.nanoseconds() == 0)
           ? 0.0 : (now - entry_t_).seconds();
  }

  /* ================================================================
   *  [8] 메인 틱 (20Hz)
   * ================================================================ */
  void tick() {
    auto now = get_clock()->now();
    double dt = 0.05;
    if (last_t_.nanoseconds() > 0)
      dt = std::clamp((now - last_t_).seconds(), 0.001, 0.2);
    last_t_ = now;

    ScarStatus s;
    {
      std::lock_guard<std::mutex> lk(st_mtx_);
      if (!st_ready_) return;
      s = st_;
    }

    process_events(now);
    (mode_ == RobotMode::KEYBOARD) ? tick_keyboard(s) : tick_auto(s, now, dt);
  }

  /* ── atomic 이벤트 처리 ─────────────────────────────────────── */
  void process_events(const rclcpp::Time& now) {
    if (ev_mode_toggle_.exchange(false)) {
      mode_ = (mode_ == RobotMode::KEYBOARD) ? RobotMode::AUTO : RobotMode::KEYBOARD;
      RCLCPP_INFO(get_logger(), "모드 전환 → %s",
                  mode_ == RobotMode::AUTO ? "AUTO" : "KEYBOARD");
      if (mode_ == RobotMode::AUTO) transition(State::IDLE, now);
    }
    if (ev_start_vision_.exchange(false) &&
        mode_ == RobotMode::AUTO && state_ == State::IDLE) {
      RCLCPP_INFO(get_logger(), "[AUTO] VISION_ALIGN 시작");
      transition(State::VISION_ALIGN, now);
    }
    if (ev_start_direct_.exchange(false) &&
        mode_ == RobotMode::AUTO && state_ == State::IDLE) {
      RCLCPP_INFO(get_logger(), "[AUTO] STAIR_APPROACH 직접 시작");
      transition(State::STAIR_APPROACH, now);
    }
    if (ev_idle_.exchange(false)) {
      estop_active_.store(false);
      pre_pause_state_ = State::IDLE;
      stairs_climbed_  = 0;
      transition(State::IDLE, now);
    }
  }

  /* ================================================================
   *  [9] KEYBOARD 모드 틱
   * ================================================================ */
  void tick_keyboard(const ScarStatus& s) {
    ScarCmd cmd{};
    cmd.warning_light = 0;  // KEYBOARD 모드: 경광등 OFF

    if (is_tilting(s) || estop_active_.load()) {
      cmd.emergency_stop = 1;
      cmd_pub_->publish(cmd);
      if (is_tilting(s))
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
          "[SAFETY] 기울기 초과! pitch=%.1f°", s.pitch_angle);
      return;
    }

    double kv, ks;
    int32_t kact1, kact2, kslide, kbrush;
    int16_t ksuction;
    {
      std::lock_guard<std::mutex> lk(key_mtx_);
      kv=kv_; ks=ks_; kact1=kact1_; kact2=kact2_;
      kslide=kslide_; kbrush=kbrush_; ksuction=ksuction_;
    }

    double vx = kv * std::sin(ks);
    double vy = kv * std::cos(ks);
    WheelCmds wc = inverse_kinematics(vx, vy, 0.0);

    cmd.emergency_stop     = 0;
    pack_wheel(cmd, wc);
    cmd.target_actuator_1  = kact1;
    cmd.target_actuator_2  = kact2;
    cmd.target_brush_speed = kbrush;
    cmd.target_suction_pwm = ksuction;
    cmd.target_slide_pos   = kslide;
    cmd_pub_->publish(cmd);
  }

  /* ================================================================
   *  [10] AUTO 모드 틱 (상태 머신)
   * ================================================================ */
  void tick_auto(const ScarStatus& s, const rclcpp::Time& now, double dt) {

    // ── [안전 1] 전복/과부하/스페이스바 → EMERGENCY_STOP ─────────
    if (state_ != State::EMERGENCY_STOP &&
        state_ != State::HUMAN_PAUSE) {
      if (estop_active_.load()) {
        RCLCPP_WARN(get_logger(), "[SAFETY] 비상 정지 (스페이스바)");
        transition(State::EMERGENCY_STOP, now);
      } else if (is_tilting(s)) {
        RCLCPP_ERROR(get_logger(), "[SAFETY] 전복! pitch=%.1f°", s.pitch_angle);
        transition(State::EMERGENCY_STOP, now);
      } else if (motor_overloaded(s)) {
        RCLCPP_ERROR(get_logger(), "[SAFETY] 모터 과부하!");
        transition(State::EMERGENCY_STOP, now);
      }
    }

    // ── [안전 2] 인체 감지 → HUMAN_PAUSE ──────────────────────
    // IDLE, EMERGENCY_STOP, HUMAN_PAUSE 상태에서는 진입하지 않음
    if (human_detected_.load() &&
        state_ != State::IDLE &&
        state_ != State::EMERGENCY_STOP &&
        state_ != State::HUMAN_PAUSE) {
      RCLCPP_WARN(get_logger(),
        "[HUMAN] 인체 감지 → HUMAN_PAUSE (이전 상태: %s)", state_str(state_));
      pre_pause_state_ = state_;  // 복귀할 상태 저장
      transition(State::HUMAN_PAUSE, now);
    }

    ScarCmd cmd{};
    cmd.emergency_stop     = 0;
    cmd.target_suction_pwm = SUCTION_OFF;
    cmd.warning_light      = compute_warning_light();

    switch (state_) {

      /* ──────────────────────────────────────────────────────────
       *  IDLE: Nav2 /cmd_vel passthrough
       * ────────────────────────────────────────────────────────── */
      case State::IDLE: {
        Twist cv; rclcpp::Time cv_t;
        { std::lock_guard<std::mutex> lk(cv_mtx_); cv=cv_; cv_t=cv_stamp_; }
        if ((now - cv_t).seconds() < CMD_VEL_TIMEOUT) {
          WheelCmds wc = inverse_kinematics(
              cv.linear.x, cv.linear.y, cv.angular.z);
          pack_wheel(cmd, wc);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  HUMAN_PAUSE: 인체 감지 일시 정지
       *
       *  동작:
       *    - 모든 모터 정지 (emergency_stop = 1)
       *    - 경광등 OFF (혼동 방지)
       *    - /human_detected = false 수신 후 HUMAN_RESUME_CONFIRM초
       *      추가 확인 뒤 이전 상태로 복귀
       * ────────────────────────────────────────────────────────── */
      case State::HUMAN_PAUSE: {
        cmd.emergency_stop = 1;
        cmd.warning_light  = 0;  // 인체 감지 중 경광등 OFF

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "[HUMAN_PAUSE] 인체 감지 중 — 정지 유지 (복귀 예정: %s)",
          state_str(pre_pause_state_));

        // 인체 감지 해제 + 확인 시간 경과 시 복귀
        if (!human_detected_.load() &&
            human_clear_time_.nanoseconds() > 0 &&
            (now - human_clear_time_).seconds() > HUMAN_RESUME_CONFIRM) {
          RCLCPP_INFO(get_logger(),
            "[HUMAN_PAUSE] 인체 사라짐 확인 → %s 복귀",
            state_str(pre_pause_state_));
          transition(pre_pause_state_, now);
          pre_pause_state_ = State::IDLE;
        }
        cmd_pub_->publish(cmd);
        return;  // warning_light 이미 설정, 아래 publish 스킵
      }

      /* ──────────────────────────────────────────────────────────
       *  VISION_ALIGN: YOLO 계단 좌측 정렬
       *  /stair_lateral_error (Float32, m) 기반 크랩 이동
       *  양수 = 로봇이 우측 → 좌측으로 이동
       *  음수 = 로봇이 좌측 → 우측으로 이동
       * ────────────────────────────────────────────────────────── */
      case State::VISION_ALIGN: {
        float err = lat_err_.load();

        // /stair_lateral_error 타임아웃 체크 (계단 미감지)
        bool stair_visible = (lat_err_stamp_.nanoseconds() > 0 &&
                              (now - lat_err_stamp_).seconds() < 1.0);

        if (stair_visible && std::abs(err) < (float)vision_tol_) {
          RCLCPP_INFO(get_logger(),
            "[VISION_ALIGN] 정렬 완료 (err=%.3fm) → STAIR_APPROACH", err);
          transition(State::STAIR_APPROACH, now);
          break;
        }

        if (stair_visible) {
          // 오차 방향으로 크랩 이동
          WheelCmds wc = ik_crab(err > 0 ? spd_align_ : -spd_align_);
          pack_wheel(cmd, wc);
        }
        // 계단 미감지 시 정지 (stair_node가 발행 안 하므로 대기)

        if (elapsed(now) > 15.0) {
          RCLCPP_WARN(get_logger(), "[VISION_ALIGN] 타임아웃 → STAIR_APPROACH");
          transition(State::STAIR_APPROACH, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  STAIR_APPROACH: 계단 접근 + 전륜 좌우 정렬
       * ────────────────────────────────────────────────────────── */
      case State::STAIR_APPROACH: {
        // 후륜 접촉 → 후륜 등반 즉시 전환
        if (s.load_rl > load_contact_ || s.load_rr > load_contact_) {
          RCLCPP_INFO(get_logger(), "[APPROACH] 후륜 접촉 → REAR_CLIMB");
          transition(State::REAR_CLIMB, now);
          break;
        }

        bool Lhit = (s.load_fl > load_contact_);
        bool Rhit = (s.load_fr > load_contact_);

        if (Lhit && Rhit) {
          if (std::abs(s.load_fl - s.load_fr) < load_diff_) {
            RCLCPP_INFO(get_logger(), "[APPROACH] 전륜 균등 접촉 → FRONT_CLIMB");
            transition(State::FRONT_CLIMB, now);
          } else {
            bool fl_heavy = (s.load_fl > s.load_fr);
            int32_t lo = mps_to_dxl(0.05), hi = mps_to_dxl(spd_approach_);
            cmd.target_vel_fl = -(fl_heavy ? lo : hi);
            cmd.target_vel_fr = -(fl_heavy ? hi : lo);
            cmd.target_vel_rl =   fl_heavy ? lo : hi;
            cmd.target_vel_rr =   fl_heavy ? hi : lo;
            auto ws = ik_steer_only(0.0, 0.0);
            cmd.target_steer_pos_l = ws.steer_f;
            cmd.target_steer_pos_r = ws.steer_r;
          }
        } else if (Lhit) {
          cmd.target_vel_fl = 0; cmd.target_vel_rl = 0;
          cmd.target_vel_fr = -mps_to_dxl(0.12);
          cmd.target_vel_rr =  mps_to_dxl(0.12);
          auto ws = ik_steer_only(0.0, 0.0);
          cmd.target_steer_pos_l = ws.steer_f;
          cmd.target_steer_pos_r = ws.steer_r;
        } else if (Rhit) {
          cmd.target_vel_fr = 0; cmd.target_vel_rr = 0;
          cmd.target_vel_fl = -mps_to_dxl(0.12);
          cmd.target_vel_rl =  mps_to_dxl(0.12);
          auto ws = ik_steer_only(0.0, 0.0);
          cmd.target_steer_pos_l = ws.steer_f;
          cmd.target_steer_pos_r = ws.steer_r;
        } else {
          pack_wheel(cmd, ik_straight(spd_approach_));
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  FRONT_CLIMB: PATS Wheel 독립 극복 감시
       *
       *  PATS Wheel 동작:
       *    ① 계단 턱 접촉 → load 급증 (> load_contact_)
       *    ② 턱 타넘음   → load 급감 (< load_stable_) = 극복 완료
       *
       *  독립 제어:
       *    FL 극복 → FL 감속, FR 계속 full
       *    FR 극복 → FR 감속, FL 계속 full
       *    RL/RR   → 항상 full (지면 추진)
       * ────────────────────────────────────────────────────────── */
      case State::FRONT_CLIMB: {
        if (s.load_fl < load_stable_) fl_climbed_ = true;
        if (s.load_fr < load_stable_) fr_climbed_ = true;

        // 부스트: 어느 바퀴라도 BOOST_LOAD 초과 시 속도 증가
        bool boost = (s.load_fl > BOOST_LOAD || s.load_fr > BOOST_LOAD ||
                      s.load_rl > BOOST_LOAD || s.load_rr > BOOST_LOAD);
        int32_t full = mps_to_dxl(boost ? spd_climb_boost_ : spd_climb_f_);
        int32_t slow = mps_to_dxl(spd_slow_);
        int32_t rear = full;

        cmd.target_vel_fl = -(fl_climbed_ ? slow : full);
        cmd.target_vel_fr = -(fr_climbed_ ? slow : full);
        cmd.target_vel_rl = rear;
        cmd.target_vel_rr = rear;

        auto ws = ik_steer_only(0.0, 0.0);
        cmd.target_steer_pos_l = ws.steer_f;
        cmd.target_steer_pos_r = ws.steer_r;

        // 극복 완료: FL/FR 극복 + 전/후륜 모두 부하 감소 연속 확인
        // (FRONT_CLIMB 중 후륜은 평지에 있으므로 rl_ever_loaded 조건 불필요)
        bool all_clear = (s.load_fl < load_stable_ && s.load_fr < load_stable_ &&
                          s.load_rl < load_stable_ && s.load_rr < load_stable_);
        if (fl_climbed_ && fr_climbed_ && all_clear) {
          stair_clear_count_++;
          if (stair_clear_count_ >= STAIR_CLEAR_COUNT) {
            RCLCPP_INFO(get_logger(),
              "[FRONT_CLIMB] FL/FR 극복 완료 (%d회 확인) → FRONT_STABILIZE",
              STAIR_CLEAR_COUNT);
            transition(State::FRONT_STABILIZE, now);
          }
        } else {
          stair_clear_count_ = 0;
        }

        if (elapsed(now) > 10.0) {
          RCLCPP_WARN(get_logger(),
            "[FRONT_CLIMB] 타임아웃 (FL=%d FR=%d) → FRONT_STABILIZE",
            fl_climbed_, fr_climbed_);
          transition(State::FRONT_STABILIZE, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  FRONT_STABILIZE: 1초 정지 후 계단 카운트
       * ────────────────────────────────────────────────────────── */
      case State::FRONT_STABILIZE:
        if (elapsed(now) > 1.0) {
          stairs_climbed_++;
          if (stairs_climbed_ < 2) {
            RCLCPP_INFO(get_logger(),
              "[FRONT_STABILIZE] %d번째 계단 완료 → STAIR_APPROACH", stairs_climbed_);
            transition(State::STAIR_APPROACH, now);
          } else {
            stairs_climbed_ = 0;
            stabilized_pitch_ = s.pitch_angle;
            RCLCPP_INFO(get_logger(),
              "[FRONT_STABILIZE] 2번째 계단 완료 → TILTED_CLEAN_INIT (pitch_ref=%.2f°)",
              stabilized_pitch_);
            transition(State::TILTED_CLEAN_INIT, now);
          }
        }
        break;

      case State::POLE_Y_ALIGN:
        transition(State::TILTED_CLEAN_INIT, now);
        break;

      /* ──────────────────────────────────────────────────────────
       *  TILTED_CLEAN_INIT: 조향 −90° (우측 크랩) + 2.5초 대기
       * ────────────────────────────────────────────────────────── */
      case State::TILTED_CLEAN_INIT: {
        // 1) 정착 대기
        if (elapsed(now) < STEER_SETTLE_S) break;

        // 2) 첫 진입 시 현재 위치 기준으로 목표 계산 (dynamic)
        if (!steer_cmd_sent_) {
          int32_t p0 = s.steer_pos_l;
          int32_t p1 = s.steer_pos_r;
          target_steer_f_ = p0 - 8192;
          target_steer_r_ = p1 - 8192;
          steer_cmd_sent_ = true;
          RCLCPP_INFO(get_logger(),
            "[TILTED_CLEAN_INIT] steer goal: f=%d r=%d",
            target_steer_f_, target_steer_r_);
        }

        cmd.target_steer_pos_l = target_steer_f_;
        cmd.target_steer_pos_r = target_steer_r_;

        // 3) 위치 기반 도달 판정
        bool reached = (std::abs(s.steer_pos_l - target_steer_f_) <= STEER_TOL_PULSE &&
                        std::abs(s.steer_pos_r - target_steer_r_) <= STEER_TOL_PULSE);
        if (reached && steer_reached_t_.nanoseconds() == 0)
          steer_reached_t_ = now;

        if (steer_reached_t_.nanoseconds() > 0 &&
            (now - steer_reached_t_).seconds() >= STEER_HOLD_S) {
          RCLCPP_INFO(get_logger(), "[TILTED_CLEAN_INIT] 도달 확인 → CLEANING_DOWN");
          transition(State::CLEANING_DOWN, now);
        } else if (elapsed(now) >= STEER_SETTLE_S + STEER_TIMEOUT_S) {
          if (!steer_retried_) {
            RCLCPP_WARN(get_logger(),
              "[TILTED_CLEAN_INIT] 타임아웃, 재시도: cur f=%d r=%d → target f=%d r=%d",
              s.steer_pos_l, s.steer_pos_r, target_steer_f_, target_steer_r_);
            steer_retried_   = true;
            steer_reached_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            entry_t_         = now;
          } else {
            RCLCPP_WARN(get_logger(),
              "[TILTED_CLEAN_INIT] 재시도 후 타임아웃 → CLEANING_DOWN");
            transition(State::CLEANING_DOWN, now);
          }
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  CLEANING_DOWN: 로드셀 접지까지 리니어 하강
       *
       *  메커니즘:
       *    - 리니어 액추에이터 ACT_DOWN → 청소모듈 실제 하강
       *    - 로드셀이 청소모듈 하단에서 바닥 무게 감지
       *    - 좌/우 독립 감지 → 각자 접지 완료 시 해당 모터 정지
       * ────────────────────────────────────────────────────────── */
      case State::CLEANING_DOWN: {
        // 진입 후 1.5초간 모터 진동 스파이크 무시
        bool stable = (elapsed(now) > 1.5);

        if (stable) {
          if (!act1_grounded_ && fabsf(s.loadcell_l) >= (float)lc_kg_) {
            act1_grounded_ = true;
            RCLCPP_INFO(get_logger(), "[CLEANING_DOWN] ACT1(L) 접지 (%.2f kg)", s.loadcell_l);
          }
          if (!act2_grounded_ && fabsf(s.loadcell_r) >= (float)lc_kg_) {
            act2_grounded_ = true;
            RCLCPP_INFO(get_logger(), "[CLEANING_DOWN] ACT2(R) 접지 (%.2f kg)", s.loadcell_r);
          }
        }

        auto ws = ik_steer_only(-M_PI / 2.0, -M_PI / 2.0);
        cmd.target_steer_pos_l = ws.steer_f;
        cmd.target_steer_pos_r = ws.steer_r;

        if (act1_grounded_ && act2_grounded_) {
          // 양측 접지 완료 → 1초 소폭 상승 후 청소 시작
          if (both_grounded_t_.nanoseconds() == 0) {
            both_grounded_t_ = now;
            RCLCPP_INFO(get_logger(), "[CLEANING_DOWN] 양측 접지 완료. 1초 소폭 상승...");
          }
          double rise_elapsed = (now - both_grounded_t_).seconds();
          if (rise_elapsed < ACT_COAST_S) {
            cmd.target_actuator_1 = 0;   // 드라이버 회복 대기
            cmd.target_actuator_2 = 0;
          } else if (rise_elapsed < ACT_COAST_S + 1.0) {
            cmd.target_actuator_1 = ACT_UP;
            cmd.target_actuator_2 = ACT_UP;
          } else {
            cmd.target_actuator_1 = 0;
            cmd.target_actuator_2 = 0;
            RCLCPP_INFO(get_logger(), "[CLEANING_DOWN] 소폭 상승 완료 → CRAB_WALK_CLEAN");
            transition(State::CRAB_WALK_CLEAN, now);
          }
        } else {
          cmd.target_actuator_1 = act1_grounded_ ? 0 : ACT_DOWN_M1;
          cmd.target_actuator_2 = act2_grounded_ ? 0 : ACT_DOWN;
          if (elapsed(now) > 20.0) {
            RCLCPP_WARN(get_logger(), "[CLEANING_DOWN] 타임아웃 → RECOVERY_UP");
            transition(State::RECOVERY_UP, now);
          }
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  CRAB_WALK_CLEAN: PID 측면 청소 주행 (L→R, 우측 이동)
       *
       *  Pitch PID: 청소 시작 기준 pitch 유지
       *  Yaw PID:   yaw = 0 유지 (직선 이탈 방지)
       *  보정: 전/후 축 속도 차이로 pitch/yaw 제어
       * ────────────────────────────────────────────────────────── */
      case State::CRAB_WALK_CLEAN: {
        double pitch_corr = pid_cp_.update(
            stabilized_pitch_ - s.pitch_angle, dt);
        double yaw_corr = pid_cy_.update(-s.yaw_angle, dt);

        WheelCmds wc = ik_crab(-spd_crab_);  // 우측 크랩

        int32_t pc = to_dxl(pitch_corr / wheel_radius_);
        int32_t yc = to_dxl(yaw_corr   / wheel_radius_);
        wc.vFL += pc + yc;
        wc.vFR += pc + yc;
        wc.vRL -= pc - yc;
        wc.vRR -= pc - yc;

        pack_wheel(cmd, wc);
        cmd.target_brush_speed = BRUSH_SPEED;
        cmd.target_suction_pwm = SUCTION_ON;

        if (elapsed(now) > time_crab_clean_s_) {
          RCLCPP_INFO(get_logger(),
            "[CRAB_WALK] %.1fs 경과 → CLEAN_STABILIZE", time_crab_clean_s_);
          transition(State::CLEAN_STABILIZE, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  CLEAN_STABILIZE: 청소 기기 정지 + 슬라이드 우측 밀착
       * ────────────────────────────────────────────────────────── */
      case State::CLEAN_STABILIZE: {
        auto ws = ik_steer_only(-M_PI / 2.0, -M_PI / 2.0);
        cmd.target_steer_pos_l = ws.steer_f;
        cmd.target_steer_pos_r = ws.steer_r;

        if (!slide_cmd_sent_) {
          RCLCPP_INFO(get_logger(),
            "[CLEAN_STABILIZE] 슬라이드 우측 밀착 시작 (vel=%d, %.1fs)",
            -slide_vel_, slide_run_s_);
          slide_cmd_sent_ = true;
        }

        if (elapsed(now) < slide_run_s_) {
          apply_slide(cmd, SlideDir::RIGHT);  // vel = -slide_vel_
          cmd.target_brush_speed = BRUSH_SPEED;
          cmd.target_suction_pwm = SUCTION_ON;
        } else {
          apply_slide(cmd, SlideDir::CENTER); // 정지
          cmd.target_brush_speed = 0;
          cmd.target_suction_pwm = SUCTION_OFF;
          RCLCPP_INFO(get_logger(),
            "[CLEAN_STABILIZE] 슬라이드 우측 밀착 완료 → RECOVERY_UP");
          transition(State::RECOVERY_UP, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  RECOVERY_UP: 리니어 복귀 (추정 거리 기반)
       *
       *  linear_dist_1/2: OpenCR에서 PWM×시간 적산값
       *  하강 시 양수 누적 → 복귀 중 감산 → 0 이하 = 원위치
       *  로드셀은 접지 감지 전용 → 복귀 판단에 미사용
       * ────────────────────────────────────────────────────────── */
      case State::RECOVERY_UP: {
        bool d1 = (s.linear_dist_1 <= 0.0f);
        bool d2 = (s.linear_dist_2 <= 0.0f);
        cmd.target_actuator_1 = d1 ? 0 : ACT_UP;
        cmd.target_actuator_2 = d2 ? 0 : ACT_UP;

        auto ws = ik_steer_only(-M_PI / 2.0, -M_PI / 2.0);
        cmd.target_steer_pos_l = ws.steer_f;
        cmd.target_steer_pos_r = ws.steer_r;

        if (d1 && d2) {
          RCLCPP_INFO(get_logger(), "[RECOVERY_UP] 복귀 완료 → RETURN_LEFT_RUN");
          transition(State::RETURN_LEFT_RUN, now);
        } else if (elapsed(now) > 8.0) {
          RCLCPP_WARN(get_logger(), "[RECOVERY_UP] 타임아웃 → RETURN_LEFT_RUN");
          transition(State::RETURN_LEFT_RUN, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  RETURN_LEFT_RUN: 좌측 복귀 주행
       * ────────────────────────────────────────────────────────── */
      case State::RETURN_LEFT_RUN: {
        pack_wheel(cmd, ik_crab(spd_return_));  // 좌측 이동
        if (elapsed(now) > time_return_left_s_) {
          RCLCPP_INFO(get_logger(),
            "[RETURN_LEFT] %.1fs 경과 → RETURN_STABILIZE", time_return_left_s_);
          transition(State::RETURN_STABILIZE, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  RETURN_STABILIZE: 정지 + 슬라이드 좌측 밀착
       * ────────────────────────────────────────────────────────── */
      case State::RETURN_STABILIZE: {
        auto ws = ik_steer_only(-M_PI / 2.0, -M_PI / 2.0);
        cmd.target_steer_pos_l = ws.steer_f;
        cmd.target_steer_pos_r = ws.steer_r;

        if (!slide_cmd_sent_) {
          RCLCPP_INFO(get_logger(),
            "[RETURN_STABILIZE] 슬라이드 좌측 밀착 시작 (vel=+%d, %.1fs)",
            slide_vel_, slide_run_s_);
          slide_cmd_sent_ = true;
        }

        if (elapsed(now) < slide_run_s_) {
          apply_slide(cmd, SlideDir::LEFT);   // vel = +slide_vel_
        } else {
          apply_slide(cmd, SlideDir::CENTER); // 정지
          RCLCPP_INFO(get_logger(),
            "[RETURN_STABILIZE] 슬라이드 좌측 밀착 완료 → STEER_RESET");
          transition(State::STEER_RESET, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  STEER_RESET: 조향 직진 + 슬라이드 원위치
       * ────────────────────────────────────────────────────────── */
      case State::STEER_RESET: {
        pack_wheel(cmd, ik_straight(0.0));
        apply_slide(cmd, SlideDir::CENTER);

        bool reached = (std::abs(s.steer_pos_l - zero_f_) <= STEER_TOL_PULSE &&
                        std::abs(s.steer_pos_r - zero_r_) <= STEER_TOL_PULSE);
        if (reached && steer_reached_t_.nanoseconds() == 0)
          steer_reached_t_ = now;

        if (steer_reached_t_.nanoseconds() > 0 &&
            (now - steer_reached_t_).seconds() >= STEER_HOLD_S) {
          RCLCPP_INFO(get_logger(), "[STEER_RESET] 도달 → STEER_STABILIZE");
          transition(State::STEER_STABILIZE, now);
        } else if (elapsed(now) > STEER_TIMEOUT_S) {
          RCLCPP_WARN(get_logger(), "[STEER_RESET] 타임아웃 → STEER_STABILIZE");
          transition(State::STEER_STABILIZE, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  STEER_STABILIZE: 1.5초 후 다음 계단 or IDLE 판단
       * ────────────────────────────────────────────────────────── */
      case State::STEER_STABILIZE:
        if (elapsed(now) > 1.5) {
          State next = (s.pitch_angle > PITCH_NEXT_STAIR)
                       ? State::STAIR_APPROACH : State::IDLE;
          RCLCPP_INFO(get_logger(),
            "[STEER_STABILIZE] pitch=%.1f° → %s",
            s.pitch_angle, state_str(next));
          transition(next, now);
        }
        break;

      /* ──────────────────────────────────────────────────────────
       *  REAR_CLIMB: PATS Wheel 독립 후륜 극복 감시
       *  전륜은 이미 올라가 있으므로 계속 전진 유지
       * ────────────────────────────────────────────────────────── */
      case State::REAR_CLIMB: {
        if (s.load_rl < load_stable_) rl_climbed_ = true;
        if (s.load_rr < load_stable_) rr_climbed_ = true;

        // 부스트: 어느 바퀴라도 BOOST_LOAD 초과 시 속도 증가
        bool boost = (s.load_fl > BOOST_LOAD || s.load_fr > BOOST_LOAD ||
                      s.load_rl > BOOST_LOAD || s.load_rr > BOOST_LOAD);
        int32_t full = mps_to_dxl(boost ? spd_climb_boost_ : spd_climb_r_);
        int32_t slow = mps_to_dxl(spd_slow_);

        cmd.target_vel_fl = -full;
        cmd.target_vel_fr = -full;
        cmd.target_vel_rl = rl_climbed_ ? slow : full;
        cmd.target_vel_rr = rr_climbed_ ? slow : full;

        auto ws = ik_steer_only(0.0, 0.0);
        cmd.target_steer_pos_l = ws.steer_f;
        cmd.target_steer_pos_r = ws.steer_r;

        // 극복 완료: 4바퀴 모두 부하 감소 연속 확인
        bool all_clear = (s.load_fl < load_stable_ && s.load_fr < load_stable_ &&
                          s.load_rl < load_stable_ && s.load_rr < load_stable_);
        if (rl_climbed_ && rr_climbed_ && all_clear) {
          stair_clear_count_++;
          if (stair_clear_count_ >= STAIR_CLEAR_COUNT) {
            State next = (s.pitch_angle > PITCH_NEXT_STAIR)
                         ? State::STAIR_APPROACH : State::IDLE;
            RCLCPP_INFO(get_logger(),
              "[REAR_CLIMB] 완료 (%d회 확인) → %s", STAIR_CLEAR_COUNT, state_str(next));
            transition(next, now);
          }
        } else {
          stair_clear_count_ = 0;
        }

        if (elapsed(now) > 10.0) {
          RCLCPP_WARN(get_logger(), "[REAR_CLIMB] 타임아웃");
          State next = (s.pitch_angle > PITCH_NEXT_STAIR)
                       ? State::STAIR_APPROACH : State::IDLE;
          transition(next, now);
        }
        break;
      }

      /* ──────────────────────────────────────────────────────────
       *  EMERGENCY_STOP
       * ────────────────────────────────────────────────────────── */
      case State::EMERGENCY_STOP:
        cmd.emergency_stop = 1;
        cmd.warning_light  = 0;
        cmd_pub_->publish(cmd);
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "[EMERGENCY_STOP] 비상 정지. 'r' 키로 IDLE 복귀");
        return;

      default: break;
    }

    cmd_pub_->publish(cmd);
  }

  /* ================================================================
   *  [11] 키보드 루프 (별도 스레드)
   * ================================================================ */
  static char get_key_nonblock() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag   &= ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    char ch = 0; read(STDIN_FILENO, &ch, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  }

  void keyboard_loop() {
    while (running_.load()) {
      char k = get_key_nonblock();
      if (k == 0) { std::this_thread::sleep_for(20ms); continue; }
      switch (k) {
        case 'm': case 'M': ev_mode_toggle_.store(true);  break;
        case '1':           ev_start_vision_.store(true); break;
        case '2':           ev_start_direct_.store(true); break;
        case 'r': case 'R': ev_idle_.store(true);         break;
        case ' ': {
          bool p = estop_active_.load(); estop_active_.store(!p);
          RCLCPP_WARN(get_logger(), "[ESTOP] %s", !p ? "활성화" : "해제");
          break;
        }
        case 'h': case 'H': print_help(); break;
        default: handle_drive_key(k); break;
      }
    }
  }

  void handle_drive_key(char k) {
    std::lock_guard<std::mutex> lk(key_mtx_);
    const double STEP = M_PI / 8.0;  // 22.5°
    switch (k) {
      case 'w': case 'W': kv_ =  spd_approach_; break;
      case 's': case 'S': kv_ = -spd_approach_; break;
      case 'x': case 'X': kv_ = 0.0;            break;
      case 'a': case 'A':
        ks_ = std::clamp(ks_ - STEP, -M_PI/2.0, M_PI/2.0);
        RCLCPP_INFO(get_logger(), "[STEER] %.1f°", ks_*180.0/M_PI); break;
      case 'd': case 'D':
        ks_ = std::clamp(ks_ + STEP, -M_PI/2.0, M_PI/2.0);
        RCLCPP_INFO(get_logger(), "[STEER] %.1f°", ks_*180.0/M_PI); break;
      case 'c': case 'C':
        ks_ = 0.0; RCLCPP_INFO(get_logger(), "[STEER] 중앙"); break;
      case 'q': case 'Q':
        ks_ =  M_PI/2.0; kv_ = -spd_approach_;
        RCLCPP_INFO(get_logger(), "[CRAB] 좌측"); break;
      case 'e': case 'E':
        ks_ = -M_PI/2.0; kv_ =  spd_approach_;
        RCLCPP_INFO(get_logger(), "[CRAB] 우측"); break;
      case 'i': case 'I':
        kact1_ = kact2_ =  150;
        RCLCPP_INFO(get_logger(), "[ACT] 상승"); break;
      case 'k': case 'K':
        kact1_ = kact2_ = -150;
        RCLCPP_INFO(get_logger(), "[ACT] 하강"); break;
      case 'o': case 'O':
        kact1_ = kact2_ = 0;
        RCLCPP_INFO(get_logger(), "[ACT] 정지"); break;
      case '[':
        kslide_ = -slide_vel_;  // 우측 이동
        RCLCPP_INFO(get_logger(), "[SLIDE] 우측 vel=%d", kslide_); break;
      case ']':
        kslide_ = +slide_vel_;  // 좌측 이동
        RCLCPP_INFO(get_logger(), "[SLIDE] 좌측 vel=%d", kslide_); break;
      case '\\':
        kslide_ = 0;
        RCLCPP_INFO(get_logger(), "[SLIDE] 정지"); break;
      case 'b': case 'B':
        kbrush_ = kbrush_ ? 0 : BRUSH_SPEED;
        RCLCPP_INFO(get_logger(), "[BRUSH] %s", kbrush_ ? "ON" : "OFF"); break;
      case 'f': case 'F':
        ksuction_ = (ksuction_ == SUCTION_OFF) ? SUCTION_ON : SUCTION_OFF;
        RCLCPP_INFO(get_logger(), "[SUCTION] %s",
                    ksuction_ == SUCTION_ON ? "ON" : "OFF"); break;
    }
  }

  void print_help() {
    RCLCPP_INFO(get_logger(), R"(
╔══════════════════════════════════════════════════════════════╗
║               SCAR 키보드 조작 가이드                         ║
╠══════════════════════════════════════════════════════════════╣
║  [주행 — 역기구학 자동 변환]                                  ║
║   W / S    : 전진 / 후진                                      ║
║   A / D    : 조향 ±22.5° 스텝                                ║
║   C        : 조향 중앙 복귀                                   ║
║   Q / E    : 좌 / 우 크랩 이동 (90°)                         ║
║   X        : 주행 정지                                        ║
║  [청소 모듈]                                                  ║
║   I / K    : 리니어 상승 / 하강                               ║
║   O        : 리니어 정지                                      ║
║   [        : 슬라이드 우측 (vel=-285)                         ║
║   ]        : 슬라이드 좌측 (vel=+285)                         ║
║   \        : 슬라이드 정지                                    ║
║   B        : 브러시 ON / OFF                                  ║
║   F        : 흡입 팬 ON / OFF                                 ║
║  [시스템]                                                     ║
║   M        : KEYBOARD <-> AUTO 모드 전환                      ║
║   1        : [AUTO] VISION_ALIGN -> 자율 시퀀스               ║
║   2        : [AUTO] STAIR_APPROACH 직접 시작                  ║
║   SPACE    : 비상 정지 토글                                   ║
║   R        : IDLE 복귀 / 비상 해제                            ║
║   H        : 도움말                                           ║
╠══════════════════════════════════════════════════════════════╣
║  [경광등 정책]                                                ║
║   AUTO 작업 중          : ON                                  ║
║   IDLE / ESTOP / 인체감지: OFF                                ║
╠══════════════════════════════════════════════════════════════╣
║  [Nav2] AUTO+IDLE: /cmd_vel 수신시 IK 자동 통과               ║
╚══════════════════════════════════════════════════════════════╝)");
  }
};

/* ================================================================
 *  main
 * ================================================================ */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScarStateManager>());
  rclcpp::shutdown();
  return 0;
}