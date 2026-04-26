#include "R2_control.hpp"

#include "clock.hpp"
#include "pid_controller.hpp"

#include <algorithm>
#include <atomic>
#include <locale>
#include <mutex>

namespace gdut {


void R2_command_mailbox::ensure_mutex_created() const {
  if (!m_mutex) {
    m_mutex = mutex{};
  }
}

// 发布指令
void R2_command_mailbox::publish(const R2_PS_command &cmd) {
  ensure_mutex_created();

  if (!m_mutex) {
    return;
  }

  std::lock_guard<mutex> lock{m_mutex};
  m_latest_command = cmd;
}

// 读取指令快照
R2_PS_command R2_command_mailbox::snapshot() const {
  ensure_mutex_created();

  R2_PS_command cmd{};

  if (!m_mutex) {
    return cmd;
  }

  std::lock_guard<mutex> lock{m_mutex};
  cmd = m_latest_command;

  return cmd;
}

// 清空命令
void R2_command_mailbox::clear() {
  publish(R2_PS_command{});
}

// 上锁
void R2_servo_bus_lock::lock() {
  while (m_lock.test_and_set(std::memory_order_acquire)) {
    osDelay(1);
  }
}
// 解锁
void R2_servo_bus_lock::unlock() { m_lock.clear(std::memory_order_release); }

// 初始化
void R2_ps2_command_controller::set_parameters(
    ps2_controller *ps2_controller, R2_command_mailbox *command_mailbox) {
  ps2_controller_ = ps2_controller;
  command_mailbox_ = command_mailbox;
}

// 控制器的启动函数，创建一个线程来持续读取PS2状态并发布命令
void R2_ps2_command_controller::start() {
  if (ps2_controller_ == nullptr ||
      m_thread.joinable()) {
    return;
  }
  m_ps2_mutex = mutex{};
  m_offset_mutex = mutex{};
  m_thread = thread<2048, osPriorityHigh7>{"r2_ps2_thread",
                                                 [this]() { run_in_thread(); }};
}

// 控制器的主循环，持续读取PS2状态并发布命令
void R2_ps2_command_controller::run_in_thread() {
  ps2_controller_->init();

  for (;;) {
    if (ps2_controller_->poll()) {

      const auto state = ps2_controller_->read_state();
      debounce_button_update(m_start_button,
      (state.buttons & ps2_buttons::k_start) != 0U);
      {
        // 更新当前状态，供其他函数查询按钮状态用
        std::unique_lock<mutex> lock{m_ps2_mutex};
        m_current_state = state;
      }
      if (is_rising_edge(ps2_buttons::k_select)) {
        begin_calibration();
      }
      if (m_calibration_active) {
        update_calibration(state);
      }

      R2_PS_command cmd = build_command(state);
      apply_protection(cmd);
      command_mailbox_->publish(cmd);
      m_last_buttons = state.buttons;
    } else {
      debounce_button_reset(m_start_button);
      m_last_buttons = 0U;
      static_cast<void>(ps2_controller_->handshake());
    }

    osDelay(4);
  }
}

// R2的构建命令
R2_PS_command R2_ps2_command_controller::build_command(const ps2_state &state) {
  // 创建一个空的来存指令
  R2_PS_command cmd{};
  // 保护和采样
  cmd.protect_enabled = m_protect_enabled;
  cmd.calibration_active = m_calibration_active;

  // 摇杆的归一化和死区处理，得到[-1, 1]范围内的值
  cmd.left_x = normalize_axis(state.left_x, k_axis_center, k_axis_deadband) -
               m_axis_offsets[0];
  cmd.left_y = normalize_axis(state.left_y, k_axis_center, k_axis_deadband) -
               m_axis_offsets[1];
  cmd.right_x = normalize_axis(state.right_x, k_axis_center, k_axis_deadband) -
                m_axis_offsets[2];
  cmd.right_y = normalize_axis(state.right_y, k_axis_center, k_axis_deadband) -
                m_axis_offsets[3];

  // 归一化和零点补偿
  cmd.left_x = std::clamp(cmd.left_x, -1.0f, 1.0f);
  cmd.left_y = std::clamp(cmd.left_y, -1.0f, 1.0f);
  cmd.right_x = std::clamp(cmd.right_x, -1.0f, 1.0f);
  cmd.right_y = std::clamp(cmd.right_y, -1.0f, 1.0f);

  // 采集不移动
  if (!m_calibration_active) {
    cmd.move_fb = cmd.left_y;
    cmd.move_lr = cmd.left_x;
    cmd.rotate = cmd.right_x;
  }

  // 保护
  if (m_start_button.press_event) {
    m_protect_enabled = !m_protect_enabled;
    cmd.protect_enabled = m_protect_enabled;

  }

  // 左夹子的限制
  cmd.left_claw_up =
      mutually_exclusive(state.up_is_pressed(), state.down_is_pressed());
  cmd.left_claw_down =
      mutually_exclusive(state.down_is_pressed(), state.up_is_pressed());
  cmd.left_claw_rotate_left =
      mutually_exclusive(state.left_is_pressed(), state.right_is_pressed());
  cmd.left_claw_rotate_right =
      mutually_exclusive(state.right_is_pressed(), state.left_is_pressed());

  // 翻滚限制和保护
  if (cmd.left_claw_rotate_left) {
    m_left_claw_rotate_angle += k_servo_angle_step;
  }
  if (cmd.left_claw_rotate_right) {
    m_left_claw_rotate_angle -= k_servo_angle_step;
  }
  m_left_claw_rotate_angle = std::clamp(m_left_claw_rotate_angle.load(),
                                        k_servo_min_angle, k_servo_max_angle);
  cmd.left_claw_rotate_angle = m_left_claw_rotate_angle;

  // 开合限制和保护
  cmd.left_claw_open =
      mutually_exclusive(state.r1_is_pressed(), state.r2_is_pressed());
  cmd.left_claw_close =
      mutually_exclusive(state.r2_is_pressed(), state.r1_is_pressed());
  if (cmd.left_claw_open) {
    m_left_claw_open_angle += k_servo_angle_step;
  }
  if (cmd.left_claw_close) {
    m_left_claw_open_angle -= k_servo_angle_step;
  }
  m_left_claw_open_angle =
      std::clamp(m_left_claw_open_angle.load(), k_servo_min_angle, k_servo_max_angle);
  cmd.left_claw_open_angle = m_left_claw_open_angle;

  // 右夹子的限制和保护
  cmd.right_claw_up =
      mutually_exclusive(state.triangle_is_pressed(), state.cross_is_pressed());
  cmd.right_claw_down =
      mutually_exclusive(state.cross_is_pressed(), state.triangle_is_pressed());

  cmd.right_claw_rotate_left =
      mutually_exclusive(state.square_is_pressed(), state.circle_is_pressed());
  cmd.right_claw_rotate_right =
      mutually_exclusive(state.circle_is_pressed(), state.square_is_pressed());
  if (cmd.right_claw_rotate_left) {
    m_right_claw_rotate_angle += k_servo_angle_step;
  }
  if (cmd.right_claw_rotate_right) {
    m_right_claw_rotate_angle -= k_servo_angle_step;
  }
  m_right_claw_rotate_angle = std::clamp(m_right_claw_rotate_angle.load(),
                                         k_servo_min_angle, k_servo_max_angle);
  cmd.right_claw_rotate_angle = m_right_claw_rotate_angle;

  // 开合限制和保护
  cmd.right_claw_open =
      mutually_exclusive(state.l1_is_pressed(), state.l2_is_pressed());
  cmd.right_claw_close =
      mutually_exclusive(state.l2_is_pressed(), state.l1_is_pressed());
  if (cmd.right_claw_open) {
    m_right_claw_open_angle += k_servo_angle_step;
  }
  if (cmd.right_claw_close) {
    m_right_claw_open_angle -= k_servo_angle_step;
  }
  m_right_claw_open_angle =
      std::clamp(m_right_claw_open_angle.load(), k_servo_min_angle, k_servo_max_angle);
  cmd.right_claw_open_angle = m_right_claw_open_angle;

  if (cmd.right_y > 0.3f) {
    cmd.uprise_forward = true;
  }
  if (cmd.right_y < -0.3f) {
    cmd.uprise_backward = true;
  }

  // 往前推
  cmd.pull_forward =
      mutually_exclusive(state.l3_is_pressed(), state.r3_is_pressed());
  cmd.pull_backward =
      mutually_exclusive(state.r3_is_pressed(), state.l3_is_pressed());
  return cmd;
}

// 归一化摇杆数据，将摇杆值映射到[-1, 1]范围内，并且应用死区处理
float R2_ps2_command_controller::normalize_axis(std::uint8_t value,
                                                std::uint8_t center,
                                                float deadband) const {
  const int centered = static_cast<int>(value) - static_cast<int>(center);
  float normalized = static_cast<float>(centered) / 127.0f;
  normalized = std::clamp(normalized, -1.0f, 1.0f);
  if (normalized > -deadband && normalized < deadband) {
    return 0.0f;
  }
  return normalized;
}

// 开始校准
void R2_ps2_command_controller::begin_calibration() {
  std::lock_guard<mutex> lock{m_offset_mutex};
  m_calibration_active = true;
  m_calibration_count = 0U;
  m_axis_sums = {0.0f, 0.0f, 0.0f, 0.0f};
}

// 校准过程中不断累积摇杆数据，最后求平均值作为摇杆的中心偏移
void R2_ps2_command_controller::update_calibration(const ps2_state &state) {
  std::unique_lock<mutex> lock{m_offset_mutex};
  m_axis_sums[0] += normalize_axis(state.left_x, k_axis_center, 0.0f);
  m_axis_sums[1] += normalize_axis(state.left_y, k_axis_center, 0.0f);
  m_axis_sums[2] += normalize_axis(state.right_x, k_axis_center, 0.0f);
  m_axis_sums[3] += normalize_axis(state.right_y, k_axis_center, 0.0f);
  ++m_calibration_count;
  lock.unlock();
  if (m_calibration_count >= k_calibration_samples) {
    finish_calibration();
  }
}

// ps2进行校准
void R2_ps2_command_controller::finish_calibration() {
  if (m_calibration_count == 0U) {
    m_calibration_active = false;
    return;
  }
  std::lock_guard<mutex> lock{m_offset_mutex};

  const float sample_count = static_cast<float>(m_calibration_count);
  m_axis_offsets[0] = m_axis_sums[0] / sample_count;
  m_axis_offsets[1] = m_axis_sums[1] / sample_count;
  m_axis_offsets[2] = m_axis_sums[2] / sample_count;
  m_axis_offsets[3] = m_axis_sums[3] / sample_count;
  m_calibration_active = false;
  m_calibration_count = 0U;
}

// ps2的命令控制线程，根据从命令邮箱获取的命令
//  控制底盘的四个电机的速度，以及左爪和右爪的旋转和开合，还有升降和拉伸的步进电机
void R2_ps2_command_controller::apply_protection(R2_PS_command &cmd) const {
  if (!cmd.protect_enabled) {
    return;
  }

  cmd.move_fb = 0.0f;
  cmd.move_lr = 0.0f;
  cmd.rotate = 0.0f;

  cmd.left_claw_up = false;
  cmd.left_claw_down = false;
  cmd.left_claw_rotate_left = false;
  cmd.left_claw_rotate_right = false;
  cmd.left_claw_open = false;
  cmd.left_claw_close = false;

  cmd.right_claw_up = false;
  cmd.right_claw_down = false;
  cmd.right_claw_rotate_left = false;
  cmd.right_claw_rotate_right = false;
  cmd.right_claw_open = false;
  cmd.right_claw_close = false;

  cmd.uprise_forward = false;
  cmd.uprise_backward = false;
  cmd.pull_forward = false;
  cmd.pull_backward = false;
}


bool R2_ps2_command_controller::is_pressed(std::uint16_t mask) const {
  std::lock_guard<mutex> lock{m_ps2_mutex};
  return (m_current_state.buttons & mask) != 0U;
}

bool R2_ps2_command_controller::was_pressed(std::uint16_t mask) const {
  return (m_last_buttons & mask) != 0U;
}

bool R2_ps2_command_controller::is_rising_edge(std::uint16_t mask) const {
  return is_pressed(mask) && !was_pressed(mask);
}

bool R2_ps2_command_controller::mutually_exclusive(bool positive,
                                                   bool negative) const {
  return positive && !negative;
}

//button_debounce的更新函数，根据原始按键状态更新防抖状态，并产生有效按下和松开事件
inline void R2_ps2_command_controller::debounce_button_update(button_debounce &btn, bool raw_pressed) {
  btn.press_event = false;//按下的事件
  btn.release_event = false;//松开的事件

  //不稳定进入
  if (!btn.stable_pressed) {
    //按下就增加次数
    if (raw_pressed) {
      //增加按下次数和清空松开次数
      ++btn.press_count;
      btn.release_count = 0;
      
      //按下后次数大于等于设定的按下阈值，就认为是稳定按下，产生按下事件
      if (btn.press_count >= btn.press_ticks) {
        btn.stable_pressed = true;
        btn.press_count = 0;
        btn.press_event = true;
      }
    } else {
      btn.press_count = 0;
      btn.release_count = 0;
    }
  } else {
    if (!raw_pressed) {
      ++btn.release_count;
      btn.press_count = 0;

      if (btn.release_count >= btn.release_ticks) {
        btn.stable_pressed = false;
        btn.release_count = 0;
        btn.release_event = true;//松开的事件
      }
    } else {
      btn.release_count = 0;
      btn.press_count = 0;
    }
  }
}

inline void R2_ps2_command_controller::debounce_button_reset(button_debounce &btn) {
  btn.press_count = 0;
  btn.release_count = 0;
  btn.stable_pressed = false;
  btn.press_event = false;
  btn.release_event = false;
}
void R2_chassis_controller::set_parameters(R2_command_mailbox *command_mailbox,
                                           TIM_HandleTypeDef *htim1,
                                           TIM_HandleTypeDef *htim2,
                                           TIM_HandleTypeDef *htim3,
                                           TIM_HandleTypeDef *htim4,
                                           TIM_HandleTypeDef *htim5,
                                           TIM_HandleTypeDef *htim9) {
  command_mailbox_ = command_mailbox;

  m_tim1 = timer{htim1};
  m_tim2 = timer{htim2};
  m_tim3 = timer{htim3};
  m_tim4 = timer{htim4};
  m_tim5 = timer{htim5};
  m_tim9 = timer{htim9};

  m_motor1 = motor{&m_tim5, TIM_CHANNEL_1, GPIOF, GPIO_PIN_15, &m_tim1, 13};
  m_motor2 = motor{&m_tim5, TIM_CHANNEL_2, GPIOG, GPIO_PIN_0, &m_tim2, 13};
  m_motor3 = motor{&m_tim5, TIM_CHANNEL_4, GPIOG, GPIO_PIN_1, &m_tim3, 13};
  m_motor4 = motor{&m_tim9, TIM_CHANNEL_2, GPIOE, GPIO_PIN_7, &m_tim4, 13};
}

// R2的底盘控制器启动函数
void R2_chassis_controller::start() {
  if (command_mailbox_ == nullptr || m_thread.joinable()) {
    return;
  }

  m_thread = thread<2048, osPriorityRealtime>{"r2_chassis_thread",
                                              [this]() { run_in_thread(); }};
}

void R2_chassis_controller::run_in_thread() {
  steady_clock::time_point last_time = steady_clock::now();
  pid_controller<float> pid1{1.0f, 0.3f, 0.0f, 0.03f, 1.0f, -1.0f, 1.0f};
  pid_controller<float> pid2{1.0f, 0.3f, 0.0f, 0.03f, 1.0f, -1.0f, 1.0f};
  pid_controller<float> pid3{1.0f, 0.3f, 0.0f, 0.03f, 1.0f, -1.0f, 1.0f};
  pid_controller<float> pid4{1.0f, 0.3f, 0.0f, 0.03f, 1.0f, -1.0f, 1.0f};

  for (;;) {
    const auto now = steady_clock::now();
    const float delta_time_s =
        std::chrono::duration<float>(now - last_time).count();
    last_time = now;

    if (delta_time_s <= 0.0f) {
      osDelay(1);
      continue;
    }

    m_motor1.refresh_encoder_state(delta_time_s);
    m_motor2.refresh_encoder_state(delta_time_s);
    m_motor3.refresh_encoder_state(delta_time_s);
    m_motor4.refresh_encoder_state(delta_time_s);

    const R2_PS_command cmd = command_mailbox_->snapshot();
    apply_chassis_command(cmd);

    const float speed1 =
        m_motor1.get_current_speed() / 3584.61562f * 10.0f / 1.5f;
    const float speed2 =
        -m_motor2.get_current_speed() / 3584.61562f * 10.0f / 1.5f;
    const float speed3 =
        -m_motor3.get_current_speed() / 3584.61562f * 10.0f / 1.5f;
    const float speed4 =
        m_motor4.get_current_speed() / 3584.61562f * 10.0f / 1.5f;

    std::atomic_thread_fence(std::memory_order_acquire);
    const float target1 =-m_target_speed1.load(std::memory_order_relaxed);
    const float target2 =-m_target_speed2.load(std::memory_order_relaxed);
    const float target3 =-m_target_speed3.load(std::memory_order_relaxed);
    const float target4 =-m_target_speed4.load(std::memory_order_relaxed);

    pid1.set_target(target1);
    pid2.set_target(target2);
    pid3.set_target(target3);
    pid4.set_target(target4);

    const float control1 = pid1.update(speed1, delta_time_s * 1000.0f);
    const float control2 = pid2.update(speed2, delta_time_s * 1000.0f);
    const float control3 = pid3.update(speed3, delta_time_s * 1000.0f);
    const float control4 = pid4.update(speed4, delta_time_s * 1000.0f);

    m_motor1.set_pwm_duty(control1);
    m_motor2.set_pwm_duty(control2);
    m_motor3.set_pwm_duty(control3);
    m_motor4.set_pwm_duty(control4);

    osDelay(3);
  }
}

// 底盘控制线程下发指令层
void R2_chassis_controller::apply_chassis_command(const R2_PS_command &cmd) {
  const vector<float, 3> velocities{cmd.move_fb, cmd.move_lr, cmd.rotate};
  const auto wheel_speed =
      mcanum_wheel_kinematics<k_chassis_width,
                              k_chassis_length>::forward_kinematics(velocities);

  m_target_speed1.store(wheel_speed[0], std::memory_order_relaxed);
  m_target_speed2.store(-wheel_speed[1], std::memory_order_relaxed);
  m_target_speed3.store(wheel_speed[2], std::memory_order_relaxed);
  m_target_speed4.store(wheel_speed[3], std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
}

// R2的左爪控制器硬件初始化
void R2_left_claw_controller::set_parameters(
    R2_command_mailbox *command_mailbox, R2_servo_bus_lock *servo_lock,
    R2_stepper_motor *weapon_stepper_timer3, pca9685 *pca9685) {
  command_mailbox_ = command_mailbox;
  servo_lock_ = servo_lock;
  weapon_stepper_timer3_ = weapon_stepper_timer3;
  pca9685_ = pca9685;
}

// 启动控制左爪的线程
/*void R2_left_claw_controller::start() {
  if (command_mailbox_ == nullptr || servo_lock_ == nullptr ||
      m_thread.joinable()) {
    return;
  }

  m_thread = thread<1536, osPriorityNormal>{"r2_left_claw_thread",
                                            [this]() { run_in_thread(); }};
}*/

// 线程进入函数，不断从命令邮箱获取最新的命令，并应用到左爪的控制上
void R2_left_claw_controller::run_in_thread() {
  for (;;) {
    apply_command(command_mailbox_->snapshot());
    osDelay(2);
  }
}

// 左的爪控制线程，根据命令控制左爪的旋转和开合，以及左爪升降的步进电机
void R2_left_claw_controller::apply_command(const R2_PS_command &cmd) {
  if (pca9685_ == nullptr || weapon_stepper_timer3_ == nullptr ||
      weapon_stepper_timer3_->stepper_timer == nullptr) {
    return;
  }

  timer::timer_pwm clamp_pwm(weapon_stepper_timer3_->stepper_timer);
  clamp_pwm.set_duty(weapon_stepper_timer3_->channel, 50);

  servo_lock_->lock();
       pca9685_->set_servo_angle(0, cmd.left_claw_rotate_angle, 500, 2500, 270);
  pca9685_->set_servo_angle(1, cmd.left_claw_open_angle, 500, 2500, 270);
  servo_lock_->unlock();

  if (cmd.left_claw_up) {
    HAL_GPIO_WritePin(weapon_stepper_timer3_->dir_port,
                      weapon_stepper_timer3_->dir_pin, GPIO_PIN_SET);
    clamp_pwm.pwm_start(weapon_stepper_timer3_->channel);
    osDelay(10);
    clamp_pwm.pwm_stop(weapon_stepper_timer3_->channel);
  }
  if (cmd.left_claw_down) {
    HAL_GPIO_WritePin(weapon_stepper_timer3_->dir_port,
                      weapon_stepper_timer3_->dir_pin, GPIO_PIN_RESET);
    clamp_pwm.pwm_start(weapon_stepper_timer3_->channel);
    osDelay(10);
    clamp_pwm.pwm_stop(weapon_stepper_timer3_->channel);
  }
}

// R2的右爪控制器硬件初始化
void R2_right_claw_controller::set_parameters(
    R2_command_mailbox *command_mailbox, R2_servo_bus_lock *servo_lock,
    R2_stepper_motor *clamp_stepper_timer1, pca9685 *pca9685) {
  command_mailbox_ = command_mailbox;
  servo_lock_ = servo_lock;
  clamp_stepper_timer1_ = clamp_stepper_timer1;
  pca9685_ = pca9685;
}

// 启动控制右爪的线程

void R2_right_claw_controller::start() {
  if (command_mailbox_ == nullptr || servo_lock_ == nullptr ||
      m_thread.joinable()) {
    return;
  }

  m_thread = thread<1536, osPriorityNormal>{"r2_right_claw_thread",
                                            [this]() { run_in_thread(); }};
}

// 线程进入函数，不断从命令邮箱获取最新的命令，并应用到右爪的控制上
/*void R2_right_claw_controller::run_in_thread() {
  for (;;) {
    apply_command(command_mailbox_->snapshot());
    osDelay(2);
  }
}*/
// 根据命令控制右爪的旋转和开合，以及右爪升降的步进电机
void R2_right_claw_controller::apply_command(const R2_PS_command &cmd) {
  if (pca9685_ == nullptr || clamp_stepper_timer1_ == nullptr ||
      clamp_stepper_timer1_->stepper_timer == nullptr) {
    return;
  }

  timer::timer_pwm claw_pwm(clamp_stepper_timer1_->stepper_timer);

  servo_lock_->lock();
  pca9685_->set_servo_angle(3, cmd.right_claw_rotate_angle, 500, 2500, 270);
  pca9685_->set_servo_angle(4, cmd.right_claw_open_angle, 500, 2500, 270);
  servo_lock_->unlock();

  if (cmd.right_claw_up) {
    HAL_GPIO_WritePin(clamp_stepper_timer1_->dir_port,
                      clamp_stepper_timer1_->dir_pin, GPIO_PIN_SET);
    claw_pwm.pwm_start(clamp_stepper_timer1_->channel);
    osDelay(10);
    claw_pwm.pwm_stop(clamp_stepper_timer1_->channel);
  }
  if (cmd.right_claw_down) {
    HAL_GPIO_WritePin(clamp_stepper_timer1_->dir_port,
                      clamp_stepper_timer1_->dir_pin, GPIO_PIN_RESET);
    claw_pwm.pwm_start(clamp_stepper_timer1_->channel);
    osDelay(10);
    claw_pwm.pwm_stop(clamp_stepper_timer1_->channel);
  }
}

// R2_uprise_pull_controller的set_parameters函数，设置控制升降机构的线程所需的参数
void R2_uprise_pull_controller::set_parameters(
    R2_command_mailbox *command_mailbox, R2_stepper_motor *lift_stepper_timer2,
    R2_motor *lift_motor) {
  command_mailbox_ = command_mailbox;
  lift_stepper_timer2_ = lift_stepper_timer2;
  lift_motor_ = lift_motor;
}
// 启动控制升降机构的线程
void R2_uprise_pull_controller::start() {
  if (command_mailbox_ == nullptr || m_thread.joinable()) {
    return;
  }

  m_thread = thread<1536, osPriorityNormal>{"r2_uprise_pull_thread",
                                            [this]() { run_in_thread(); }};
}

// 根据命令控制升降机构的步进电机和直流电机的线程跑
void R2_uprise_pull_controller::run_in_thread() {
  for (;;) {
    apply_command(command_mailbox_->snapshot());
    osDelay(2);
  }
}
// 根据命令控制升降机构的步进电机和直流电机
void R2_uprise_pull_controller::apply_command(const R2_PS_command &cmd) {
  if (lift_stepper_timer2_ == nullptr ||
      lift_stepper_timer2_->stepper_timer == nullptr ||
      lift_motor_ == nullptr || lift_motor_->motor_timer == nullptr) {
    return;
  }

  timer::timer_pwm lift_pwm(lift_stepper_timer2_->stepper_timer);
  timer::timer_pwm pull_pwm(lift_motor_->motor_timer);
  lift_pwm.set_duty(lift_stepper_timer2_->channel, 50);
  pull_pwm.set_duty(lift_motor_->channel, 50);

  if (cmd.uprise_forward) {
    HAL_GPIO_WritePin(lift_stepper_timer2_->dir_port,
                      lift_stepper_timer2_->dir_pin, GPIO_PIN_SET);
    lift_pwm.pwm_start(lift_stepper_timer2_->channel);
    osDelay(10);
    lift_pwm.pwm_stop(lift_stepper_timer2_->channel);
  }
  if (cmd.uprise_backward) {
    HAL_GPIO_WritePin(lift_stepper_timer2_->dir_port,
                      lift_stepper_timer2_->dir_pin, GPIO_PIN_RESET);
    lift_pwm.pwm_start(lift_stepper_timer2_->channel);
    osDelay(10);
    lift_pwm.pwm_stop(lift_stepper_timer2_->channel);
  }
  if (cmd.pull_forward) {
    HAL_GPIO_WritePin(lift_motor_->dir_port, lift_motor_->dir_pin,
                      GPIO_PIN_SET);
    pull_pwm.pwm_start(lift_motor_->channel);
    osDelay(10);
    pull_pwm.pwm_stop(lift_motor_->channel);
  }
  if (cmd.pull_backward) {
    HAL_GPIO_WritePin(lift_motor_->dir_port, lift_motor_->dir_pin,
                      GPIO_PIN_RESET);
    pull_pwm.pwm_start(lift_motor_->channel);
    osDelay(10);
    pull_pwm.pwm_stop(lift_motor_->channel);
  }
}

// R2的硬件初始化
void R2_controller::set_parameters(
    ps2_controller *ps2_controller, R2_stepper_motor *clamp_stepper_timer1,
    R2_stepper_motor *lift_stepper_timer2,
    R2_stepper_motor *weapon_stepper_timer3, R2_motor *lift_motor,
    pca9685 *pca9685, TIM_HandleTypeDef *htim1, TIM_HandleTypeDef *htim2,
    TIM_HandleTypeDef *htim3, TIM_HandleTypeDef *htim4,
    TIM_HandleTypeDef *htim5, TIM_HandleTypeDef *htim9) {
  m_ps2_controller.set_parameters(ps2_controller, &m_command_mailbox);
  m_chassis_controller.set_parameters(&m_command_mailbox, htim1, htim2, htim3,
                                      htim4, htim5, htim9);
  m_left_claw_controller.set_parameters(&m_command_mailbox, &m_servo_bus_lock,
                                        weapon_stepper_timer3, pca9685);
  m_right_claw_controller.set_parameters(&m_command_mailbox, &m_servo_bus_lock,
                                         clamp_stepper_timer1, pca9685);
  m_uprise_pull_controller.set_parameters(&m_command_mailbox,
                                          lift_stepper_timer2, lift_motor);
}

// start函数启动各个控制器的线程
void R2_controller::start() {
  m_command_mailbox.init();
  m_ps2_controller.start();
  m_chassis_controller.start();
 // m_left_claw_controller.start();
  //m_right_claw_controller.start();
 // m_uprise_pull_controller.start();
}

} // namespace gdut
