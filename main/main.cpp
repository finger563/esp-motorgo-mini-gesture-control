#include <sdkconfig.h>

#include <chrono>
#include <thread>

#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "esp_mac.h"

#include "espnow.h"
#include "espnow_ctrl.h"
#include "espnow_storage.h"
#include "espnow_utils.h"

#include "butterworth_filter.hpp"
#include "high_resolution_timer.hpp"
#include "motorgo-mini.hpp"
#include "task.hpp"
#include "timer.hpp"

using namespace std::chrono_literals;

enum class EspNowCtrlStatus { INIT, BOUND, MAX };
static EspNowCtrlStatus espnow_ctrl_status = EspNowCtrlStatus::INIT;

static void init_wifi();
static void espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id,
                                 void *event_data);
static esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data, size_t size,
                                 wifi_pkt_rx_ctrl_t *rx_ctrl);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "MotorGo Mini Gesture Control", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  auto &motorgo_mini = espp::MotorGoMini::get();
  motorgo_mini.set_log_level(espp::Logger::Verbosity::INFO);
  motorgo_mini.init_motor_channel_1();
  motorgo_mini.init_motor_channel_2();
  auto &motor1 = motorgo_mini.motor1();
  auto &motor2 = motorgo_mini.motor2();
  auto &button = motorgo_mini.button();

  static constexpr uint64_t core_update_period_us = 1000;                   // microseconds
  static constexpr float core_update_period = core_update_period_us / 1e6f; // seconds

  // TODO: receive commands over esp-now to:
  // - change the motion control type
  // - change the target
  // - start / stop the motors

  // static constexpr auto motion_control_type = espp::detail::MotionControlType::VELOCITY_OPENLOOP;
  // static constexpr auto motion_control_type = espp::detail::MotionControlType::VELOCITY;
  // static const auto motion_control_type = espp::detail::MotionControlType::ANGLE_OPENLOOP;
  static auto motion_control_type = espp::detail::MotionControlType::ANGLE;

  logger.info("Setting motion control type to {}", motion_control_type);
  motor1.set_motion_control_type(motion_control_type);
  motor2.set_motion_control_type(motion_control_type);

  motor1.enable();
  motor2.enable();

  std::atomic<float> target1 = 60.0f;
  std::atomic<float> target2 = 60.0f;
  static bool target_is_angle =
      motion_control_type == espp::detail::MotionControlType::ANGLE ||
      motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP;
  // Function for initializing the target based on the motion control type
  auto initialize_target = [&]() {
    if (target_is_angle) {
      target1 = motor1.get_shaft_angle();
      target2 = motor2.get_shaft_angle();
    } else {
      target1 = 50.0f * espp::RPM_TO_RADS;
      target2 = 50.0f * espp::RPM_TO_RADS;
    }
  };
  // run it once
  initialize_target();

  auto dual_motor_fn = [&]() -> bool {
    motor1.loop_foc();
    motor2.loop_foc();
    motor1.move(target1);
    motor2.move(target2);
    return false; // don't want to stop the task
  };

  auto dual_motor_timer = espp::HighResolutionTimer({.name = "Motor Timer",
                                                     .callback = dual_motor_fn,
                                                     .log_level = espp::Logger::Verbosity::WARN});
  dual_motor_timer.periodic(core_update_period_us);

  static constexpr float sample_freq_hz = 100.0f;
  static constexpr float filter_cutoff_freq_hz = 5.0f;
  static constexpr float normalized_cutoff_frequency =
      2.0f * filter_cutoff_freq_hz / sample_freq_hz;
  static constexpr size_t ORDER = 2;
  // NOTE: using the Df2 since it's hardware accelerated :)
  using Filter = espp::ButterworthFilter<ORDER, espp::BiquadFilterDf2>;
  Filter filter1({.normalized_cutoff_frequency = normalized_cutoff_frequency});
  Filter filter2({.normalized_cutoff_frequency = normalized_cutoff_frequency});

  // if it's a velocity setpoint then target is RPM
  fmt::print("%time(s), "
             "motor 1 target, " // target is either RPM or radians
             "motor 1 angle (radians), "
             "motor 1 speed (rpm), "
             "motor 2 target, " // target is either RPM or radians
             "motor 2 angle (radians), "
             "motor 2 speed (rpm)\n");

  // make the task to periodically poll the encoders and print the state. NOTE:
  // the encoders run their own tasks to maintain state, so we're just polling
  // the current state.
  auto logging_fn = [&]() -> bool {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto _target1 = target1.load();
    if (!target_is_angle)
      _target1 *= espp::RADS_TO_RPM;
    auto _target2 = target2.load();
    if (!target_is_angle)
      _target2 *= espp::RADS_TO_RPM;
    auto rpm1 = filter1(motor1.get_shaft_velocity() * espp::RADS_TO_RPM);
    auto rpm2 = filter2(motor2.get_shaft_velocity() * espp::RADS_TO_RPM);
    auto rads1 = motor1.get_shaft_angle();
    auto rads2 = motor2.get_shaft_angle();
    fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, _target1, rads1,
               rpm1, _target2, rads2, rpm2);
    // don't want to stop the task
    return false;
  };
  auto logging_task = espp::Timer({
      .period = 10ms,
      .callback = logging_fn,
      .task_config = {
        .name = "Logging Task",
        .stack_size_bytes = 5 * 1024,
      },
      .log_level = espp::Logger::Verbosity::WARN});

  std::this_thread::sleep_for(1s);

  logger.info("Starting target task");

  enum class IncrementDirection { DOWN = -1, HOLD = 0, UP = 1 };
  static IncrementDirection increment_direction1 = IncrementDirection::UP;
  static IncrementDirection increment_direction2 = IncrementDirection::DOWN;

  auto update_target = [&](auto &target, auto &increment_direction) {
    float max_target = target_is_angle ? (2.0f * M_PI) : (200.0f * espp::RPM_TO_RADS);
    float target_delta =
        target_is_angle ? (M_PI / 4.0f) : (50.0f * espp::RPM_TO_RADS * core_update_period);
    // update target
    if (increment_direction == IncrementDirection::UP) {
      target += target_delta;
      if (target >= max_target) {
        increment_direction = IncrementDirection::DOWN;
      }
    } else if (increment_direction == IncrementDirection::DOWN) {
      target -= target_delta;
      if (target <= -max_target) {
        increment_direction = IncrementDirection::UP;
      }
    }
  };

  // make a task which will update the target (velocity or angle)
  auto target_task_fn = [&]() -> bool {
    update_target(target1, increment_direction1);
    update_target(target2, increment_direction2);
    return false; // don't want to stop the task
  };
  auto target_task = espp::Timer({
      .period = std::chrono::duration<float>(target_is_angle? 1.0f : core_update_period),
      .callback = target_task_fn,
      .task_config = {.name = "Target Task"},
  });

  // initialize the wifi and esp-now stacks
  espnow_storage_init();

  init_wifi();

  espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
  espnow_init(&espnow_config);
  espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, on_esp_now_recv);
  esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, espnow_event_handler, NULL);

  bool button_state = false;

  while (true) {
    bool new_button_state = button.is_pressed();
    if (new_button_state != button_state) {
      button_state = new_button_state;
      if (button_state) {
        logger.info("Button pressed, changing motion control type");
        // switch between ANGLE and VELOCITY
        if (motion_control_type == espp::detail::MotionControlType::ANGLE ||
            motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP) {
          motion_control_type = espp::detail::MotionControlType::VELOCITY;
          target_is_angle = false;
        } else {
          motion_control_type = espp::detail::MotionControlType::ANGLE;
          target_is_angle = true;
        }
        initialize_target();
        motor1.set_motion_control_type(motion_control_type);
        motor2.set_motion_control_type(motion_control_type);
      } else {
        logger.info("Button released");
      }
    }
    std::this_thread::sleep_for(50ms);
  }
}

void init_wifi() {
  esp_event_loop_create_default();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
  if (base != ESP_EVENT_ESPNOW) {
    return;
  }

  switch (id) {
  case ESP_EVENT_ESPNOW_CTRL_BIND: {
    [[maybe_unused]] espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
    // ESP_LOGI(TAG, "bind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac),
    // info->initiator_attribute);
    // TODO: we are now bound, indicate it and start the sending
    break;
  }

  case ESP_EVENT_ESPNOW_CTRL_BIND_ERROR: {
    [[maybe_unused]] espnow_ctrl_bind_error_t *bind_error = (espnow_ctrl_bind_error_t *)event_data;
    // ESP_LOGW(TAG, "bind error: %s", bind_error_to_string(*bind_error));
    break;
  }

  case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
    [[maybe_unused]] espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
    // ESP_LOGI(TAG, "unbind, uuid: " MACSTR ", initiator_type: %d", MAC2STR(info->mac),
    // info->initiator_attribute); we are now unbound, indicate it and stop the sending
    break;
  }

  default:
    break;
  }
}

esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data, size_t size,
                                     wifi_pkt_rx_ctrl_t *rx_ctrl) {
  // ESP_PARAM_CHECK(src_addr);
  // ESP_PARAM_CHECK(data);
  // ESP_PARAM_CHECK(size);
  // ESP_PARAM_CHECK(rx_ctrl);

  // static uint32_t count = 0;

  // ESP_LOGI(TAG, "espnow_recv, <%" PRIu32 "> [" MACSTR "][%d][%d][%u]: %.*s",
  //          count++, MAC2STR(src_addr), rx_ctrl->channel, rx_ctrl->rssi, size, size, (char
  //          *)data);

  return ESP_OK;
}
