#ifndef R2_CONTROLL_HPP
#define R2_CONTROLL_HPP

#include "bsp_motor.hpp"
#include "bsp_pca9685.hpp"
#include "bsp_ps2.hpp"
#include "bsp_timer.hpp"
#include "chassis_kinematics.hpp"
#include "cmsis_os2.h"
#include "mutex.hpp"
#include "thread.hpp"
#include "uncopyable.hpp"

#include <array>
#include <atomic>
#include <cstdint>

namespace gdut {

struct R2_PS_command {
  float move_fb{0.0f};
  float move_lr{0.0f};
  float rotate{0.0f};

  float left_x{0.0f};
  float left_y{0.0f};
  float right_x{0.0f};
  float right_y{0.0f};

  bool protect_enabled{false};
  bool calibration_active{false};

  bool left_claw_up{false};
  bool left_claw_down{false};
  bool left_claw_rotate_left{false};
  bool left_claw_rotate_right{false};
  float left_claw_rotate_angle{0.0f};
  bool left_claw_open{false};
  bool left_claw_close{false};
  float left_claw_open_angle{0.0f};

  bool right_claw_up{false};
  bool right_claw_down{false};
  bool right_claw_rotate_left{false};
  bool right_claw_rotate_right{false};
  float right_claw_rotate_angle{0.0f};
  bool right_claw_open{false};
  bool right_claw_close{false};
  float right_claw_open_angle{0.0f};

  bool uprise_forward{false};
  bool uprise_backward{false};
  bool pull_forward{false};
  bool pull_backward{false};
};

//按键防抖动
struct button_debounce {
  std::uint8_t press_ticks{4};     // 连续按下多少帧算有效
  std::uint8_t release_ticks{4};   // 连续松开多少帧算有效

  std::uint8_t press_count{0};    // 当前连续按下的帧数
  std::uint8_t release_count{0};    // 当前连续松开的帧数

  bool stable_pressed{false};      // 防抖后的稳定状态
  bool press_event{false};         // 本轮是否产生一次“有效按下”
  bool release_event{false};       // 本轮是否产生一次“有效松开”
};

struct R2_stepper_motor {
  timer *stepper_timer{nullptr};
  uint32_t channel{0};
  GPIO_TypeDef *dir_port{nullptr};
  uint16_t dir_pin{0};
};

struct R2_motor {
  timer *motor_timer{nullptr};
  uint32_t channel{0};
  GPIO_TypeDef *dir_port{nullptr};
  uint16_t dir_pin{0};
};

class R2_command_mailbox : private uncopyable {
public:
  void init() {
    if (!m_mutex) {
      m_mutex = mutex{};
    }
  }
  void publish(const R2_PS_command &cmd);
  R2_PS_command snapshot() const;
  void clear();

private:
  void ensure_mutex_created() const;

private:
  R2_PS_command m_latest_command{};
  mutable mutex m_mutex{empty_mutex};
};

class R2_servo_bus_lock : private uncopyable {
public:
  void lock();
  void unlock();

private:
  std::atomic_flag m_lock{};
};

class R2_ps2_command_controller : private uncopyable {
public:
  void set_parameters(ps2_controller *ps2_controller,
                      R2_command_mailbox *command_mailbox);
  void start();

private:
  static constexpr std::uint8_t k_axis_center = 128U;
  static constexpr float k_axis_deadband = 0.08f;
  static constexpr std::size_t k_calibration_samples = 100U;
  static constexpr float k_servo_angle_step = 1.0f;
  static constexpr float k_servo_min_angle = 0.0f;
  static constexpr float k_servo_max_angle = 270.0f;

  button_debounce m_start_button{};//start按钮的防抖结构体


  void run_in_thread();
  R2_PS_command build_command(const ps2_state &state);
  float normalize_axis(std::uint8_t value, std::uint8_t center,
                       float deadband) const;
  void begin_calibration();
  void update_calibration(const ps2_state &state);
  void finish_calibration();
  void apply_protection(R2_PS_command &cmd) const;
  bool is_pressed(std::uint16_t mask) const;
  bool was_pressed(std::uint16_t mask) const;
  bool is_rising_edge(std::uint16_t mask) const;
  bool mutually_exclusive(bool positive, bool negative) const;
  inline void debounce_button_update(button_debounce &btn, bool raw_pressed);
  inline void debounce_button_reset(button_debounce &btn);

private:
  gdut::ps2_controller *ps2_controller_{nullptr};
  R2_command_mailbox *command_mailbox_{nullptr};
  thread<2048, osPriorityHigh7> m_thread{empty_thread};
  
  std::atomic<std::uint16_t> m_last_buttons{0U};
  std::atomic<bool> m_protect_enabled{false};
  std::atomic<bool> m_calibration_active{false};
  std::atomic<std::size_t> m_calibration_count{0U};

  mutable mutex m_offset_mutex{empty_mutex};
  std::array<float, 4> m_axis_offsets{0.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, 4> m_axis_sums{0.0f, 0.0f, 0.0f, 0.0f};

  std::atomic<float> m_left_claw_rotate_angle{0.0f};
  std::atomic<float> m_left_claw_open_angle{0.0f};
  std::atomic<float> m_right_claw_rotate_angle{0.0f};
  std::atomic<float> m_right_claw_open_angle{0.0f};

  mutable mutex m_ps2_mutex{empty_mutex};
  ps2_state m_current_state{};
};

class R2_chassis_controller : private uncopyable {
public:
  void set_parameters(R2_command_mailbox *command_mailbox,
                      TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2,
                      TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4,
                      TIM_HandleTypeDef *htim5, TIM_HandleTypeDef *htim9);
  void start();

private:
  static constexpr float k_chassis_width = 0.12f;
  static constexpr float k_chassis_length = 0.38f;

  void run_in_thread();
  void apply_chassis_command(const R2_PS_command &cmd);

private:
  R2_command_mailbox *command_mailbox_{nullptr};
  std::atomic<float> m_target_speed1{0.0f};
  std::atomic<float> m_target_speed2{0.0f};
  std::atomic<float> m_target_speed3{0.0f};
  std::atomic<float> m_target_speed4{0.0f};

  timer m_tim1;
  timer m_tim2;
  timer m_tim3;
  timer m_tim4;
  timer m_tim5;
  timer m_tim9;

  motor m_motor1;
  motor m_motor2;
  motor m_motor3;
  motor m_motor4;

  thread<2048, osPriorityRealtime> m_thread{empty_thread};
};

class R2_left_claw_controller : private uncopyable {
public:
  void set_parameters(R2_command_mailbox *command_mailbox,
                      R2_servo_bus_lock *servo_lock,
                      R2_stepper_motor *weapon_stepper_timer3,
                      pca9685 *pca9685);
  void start();

private:
  void run_in_thread();
  void apply_command(const R2_PS_command &cmd);

private:
  R2_command_mailbox *command_mailbox_{nullptr};
  R2_servo_bus_lock *servo_lock_{nullptr};
  gdut::R2_stepper_motor *weapon_stepper_timer3_{nullptr};
  gdut::pca9685 *pca9685_{nullptr};
  thread<1536, osPriorityNormal> m_thread{empty_thread};
};

class R2_right_claw_controller : private uncopyable {
public:
  void set_parameters(R2_command_mailbox *command_mailbox,
                      R2_servo_bus_lock *servo_lock,
                      R2_stepper_motor *clamp_stepper_timer1,
                      pca9685 *pca9685);
  void start();

private:
  void run_in_thread();
  void apply_command(const R2_PS_command &cmd);

private:
  R2_command_mailbox *command_mailbox_{nullptr};
  R2_servo_bus_lock *servo_lock_{nullptr};
  gdut::R2_stepper_motor *clamp_stepper_timer1_{nullptr};
  gdut::pca9685 *pca9685_{nullptr};
  thread<1536, osPriorityNormal> m_thread{empty_thread};
};

class R2_uprise_pull_controller : private uncopyable {
public:
  void set_parameters(R2_command_mailbox *command_mailbox,
                      R2_stepper_motor *lift_stepper_timer2,
                      R2_motor *lift_motor);
  void start();

private:
  void run_in_thread();
  void apply_command(const R2_PS_command &cmd);

private:
  R2_command_mailbox *command_mailbox_{nullptr};
  gdut::R2_stepper_motor *lift_stepper_timer2_{nullptr};
  gdut::R2_motor *lift_motor_{nullptr};
  thread<1536, osPriorityNormal> m_thread{empty_thread};
};

class R2_controller : private uncopyable {
public:
  R2_controller() = default;
  ~R2_controller() = default;

  void set_parameters(ps2_controller *ps2_controller,
                      R2_stepper_motor *clamp_stepper_timer1 = nullptr,
                      R2_stepper_motor *lift_stepper_timer2 = nullptr,
                      R2_stepper_motor *weapon_stepper_timer3 = nullptr,
                      R2_motor *lift_motor = nullptr,
                      pca9685 *pca9685 = nullptr,
                      TIM_HandleTypeDef *htim1 = nullptr,
                      TIM_HandleTypeDef *htim2 = nullptr,
                      TIM_HandleTypeDef *htim3 = nullptr,
                      TIM_HandleTypeDef *htim4 = nullptr,
                      TIM_HandleTypeDef *htim5 = nullptr,
                      TIM_HandleTypeDef *htim9 = nullptr);

  void start();

private:
  R2_command_mailbox m_command_mailbox;
  R2_servo_bus_lock m_servo_bus_lock;
  R2_ps2_command_controller m_ps2_controller;
  R2_chassis_controller m_chassis_controller;
  R2_left_claw_controller m_left_claw_controller;
  R2_right_claw_controller m_right_claw_controller;
  R2_uprise_pull_controller m_uprise_pull_controller;
};

} // namespace gdut

#endif // R2_CONTROLL_HPP
