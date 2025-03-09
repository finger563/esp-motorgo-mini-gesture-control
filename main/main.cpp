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
#include "task.hpp"
#include "timer.hpp"
#include "vector2d.hpp"

#include "motorgo-mini.hpp"
using Bsp = espp::MotorGoMini;

#include "command.hpp"

using namespace std::chrono_literals;

static espp::Logger logger({.tag = "MGM", .level = espp::Logger::Verbosity::INFO});

///////////////////////////
/// Motor control variables
///////////////////////////

static auto motion_control_type = espp::detail::MotionControlType::ANGLE;

std::atomic<float> target1 = 60.0f;
std::atomic<float> target2 = 60.0f;
static std::atomic<bool> target_is_angle =
  motion_control_type == espp::detail::MotionControlType::ANGLE ||
  motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP;

std::shared_ptr<espp::MotorGoMini::BldcMotor> motor1_ptr;
std::shared_ptr<espp::MotorGoMini::BldcMotor> motor2_ptr;

///////////////////////////////////////////
/// ESP-NOW related variables and functions
///////////////////////////////////////////
enum class EspNowCtrlStatus { INIT, BOUND };
static EspNowCtrlStatus espnow_ctrl_status = EspNowCtrlStatus::INIT;

static void init_wifi();
static void espnow_event_handler(void *handler_args, esp_event_base_t base, int32_t id,
                                 void *event_data);
static esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data, size_t size,
                                 wifi_pkt_rx_ctrl_t *rx_ctrl);
static void app_responder_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                              espnow_attribute_t responder_attribute,
                              uint32_t status);
static constexpr const char *bind_error_to_string(espnow_ctrl_bind_error_t bind_error);

extern "C" void app_main(void) {

  logger.info("Bootup");

  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::INFO);

  logger.info("Initializing the button");
  auto on_button_pressed = [&](const auto &event) {
    // track the button state change time to determine single press or double
    // press
    static uint64_t last_press_time_us = 0;
    static bool last_double_press_state = false;

    bool pressed = event.active;

    // if the button is pressed
    if (pressed) {
      // get the current time
      auto current_time_us = esp_timer_get_time();
      auto time_since_last_press_us = current_time_us - last_press_time_us;
      // if the button was pressed within 500ms of the last press, it is a
      // double press
      if (time_since_last_press_us < 500'000) {
        last_double_press_state = true;
      } else {
        last_double_press_state = false;
      }
      // update the last press time
      last_press_time_us = current_time_us;
    } else {
      // if the button was released
      auto hold_time_us = esp_timer_get_time() - last_press_time_us;
      // if the button was pressed for more than 500ms, it is a long press
      if (hold_time_us > 500'000) {
        logger.info("Long press detected");
        // on long press, reset the esp-now binding
        if (espnow_ctrl_status == EspNowCtrlStatus::BOUND) {
          logger.info("Resetting esp-now binding");
          espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, false);
          espnow_ctrl_status = EspNowCtrlStatus::INIT;
        }
      }

      if (last_double_press_state) {
        logger.info("Double press detected");
        // on double press, start binding for esp-now
        if (espnow_ctrl_status == EspNowCtrlStatus::INIT) {
          logger.info("Starting esp-now binding");
          espnow_ctrl_initiator_bind(ESPNOW_ATTRIBUTE_KEY_1, true);
          espnow_ctrl_status = EspNowCtrlStatus::BOUND;
        }
        // reset the double press state
        last_double_press_state = false;
      }
    }
  };
  bsp.initialize_button(on_button_pressed);

  bsp.init_motor_channel_1();
  bsp.init_motor_channel_2();
  auto &motor1 = bsp.motor1();
  auto &motor2 = bsp.motor2();

  // set the shared pointers to the motors for the esp-now callback
  motor1_ptr = std::shared_ptr<espp::MotorGoMini::BldcMotor>(std::shared_ptr<espp::MotorGoMini::BldcMotor>{}, &motor1);
  motor2_ptr = std::shared_ptr<espp::MotorGoMini::BldcMotor>(std::shared_ptr<espp::MotorGoMini::BldcMotor>{}, &motor2);

  // TODO: receive commands over esp-now to:
  // - change the motion control type
  // - change the target
  // - start / stop the motors

  logger.info("Setting motion control type to {}", motion_control_type);
  motor1.set_motion_control_type(motion_control_type);
  motor2.set_motion_control_type(motion_control_type);

  motor1.enable();
  motor2.enable();

  // set the initial target
  if (target_is_angle) {
    target1 = motor1.get_shaft_angle();
    target2 = motor2.get_shaft_angle();
  } else {
    target1 = 50.0f * espp::RPM_TO_RADS;
    target2 = 50.0f * espp::RPM_TO_RADS;
  }

  auto dual_motor_fn = [&]() -> bool {
    motor1.loop_foc();
    motor2.loop_foc();
    motor1.move(target1);
    motor2.move(target2);
    return false; // don't want to stop the task
  };

  static constexpr uint64_t core_update_period_us = 1000;                   // microseconds
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

  // initialize the wifi and esp-now stacks
  espnow_storage_init();

  init_wifi();

  espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
  espnow_init(&espnow_config);
  espnow_set_config_for_data_type(ESPNOW_DATA_TYPE_DATA, true, on_esp_now_recv);
  esp_event_handler_register(ESP_EVENT_ESPNOW, ESP_EVENT_ANY_ID, espnow_event_handler, NULL);

  ESP_ERROR_CHECK(espnow_ctrl_responder_bind(30 * 1000, -55, NULL));
  espnow_ctrl_responder_data(app_responder_ctrl_data_cb);

  while (true) {
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
    espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
    logger.info("Bound to {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, initiator_type: {}",
                MAC2STR(info->mac), (int)info->initiator_attribute);
    break;
  }

  case ESP_EVENT_ESPNOW_CTRL_BIND_ERROR: {
    espnow_ctrl_bind_error_t *bind_error = (espnow_ctrl_bind_error_t *)event_data;
    logger.warn("Bind error: {}", bind_error_to_string(*bind_error));
    break;
  }

  case ESP_EVENT_ESPNOW_CTRL_UNBIND: {
    espnow_ctrl_bind_info_t *info = (espnow_ctrl_bind_info_t *)event_data;
    logger.info("Unbound from {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", MAC2STR(info->mac));
    break;
  }

  default:
    break;
  }
}

esp_err_t on_esp_now_recv(uint8_t *src_addr, void *data, size_t size,
                                     wifi_pkt_rx_ctrl_t *rx_ctrl) {
  logger.debug("Received data from {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}", MAC2STR(src_addr));
  logger.debug("RSSI: {}", (int)rx_ctrl->rssi);
  uint8_t *data_ptr = reinterpret_cast<uint8_t *>(data);
  logger.debug("Data: {::02x}", std::vector<uint8_t>(data_ptr, data_ptr + size));

  // parse this into a control command
  Command command;
  if (size == 0 || size > sizeof(command)) {
    logger.warn("Invalid command size: {}", size);
    // don't return fail, as we don't want to stop the esp-now processing
    return ESP_OK;
  }

  // it _should_ be a valid command, so copy it into the command struct
  std::memcpy(&command, data_ptr, size);

  switch (command.code) {
  case CommandCode::NONE:
    return ESP_OK;
  case CommandCode::STOP:
    // disable the motors
    target1 = 0.0f;
    target2 = 0.0f;
    if (motor1_ptr->is_enabled())
      motor1_ptr->disable();
    if (motor2_ptr->is_enabled())
      motor2_ptr->disable();
    break;

  case CommandCode::SET_ANGLE:
    motion_control_type = espp::detail::MotionControlType::ANGLE;
    if (!target_is_angle) {
      // disable the motors
      motor1_ptr->disable();
      motor2_ptr->disable();
      // update the motion control type
      motor1_ptr->set_motion_control_type(motion_control_type);
      motor2_ptr->set_motion_control_type(motion_control_type);
      // reset the angle for each motor, so that when we get a new target, it
      // doesn't jump to the new target from wherever it could have been if the
      // motor was running in speed control mode
      static auto &bsp = Bsp::get();
      bsp.reset_encoder1_accumulator();
      bsp.reset_encoder2_accumulator();
    }
    target1 = command.angle_radians;
    target2 = command.angle_radians;
    target_is_angle = true;
    if (!motor1_ptr->is_enabled())
      motor1_ptr->enable();
    if (!motor2_ptr->is_enabled())
      motor2_ptr->enable();
    break;

  case CommandCode::SET_SPEED:
    motion_control_type = espp::detail::MotionControlType::VELOCITY;
    if (target_is_angle) {
      // disable the motors
      motor1_ptr->disable();
      motor2_ptr->disable();
      // update the motion control type
      motor1_ptr->set_motion_control_type(motion_control_type);
      motor2_ptr->set_motion_control_type(motion_control_type);
    }
    target1 = command.speed_radians_per_second;
    target2 = command.speed_radians_per_second;
    target_is_angle = false;
    if (!motor1_ptr->is_enabled())
      motor1_ptr->enable();
    if (!motor2_ptr->is_enabled())
      motor2_ptr->enable();
    break;
  }

  return ESP_OK;
}

void app_responder_ctrl_data_cb(espnow_attribute_t initiator_attribute,
                              espnow_attribute_t responder_attribute,
                              uint32_t status) {
  // TODO: handle the control data
}

constexpr const char *bind_error_to_string(espnow_ctrl_bind_error_t bind_error) {
    switch (bind_error) {
    case ESPNOW_BIND_ERROR_NONE:
        return "No error";
    case ESPNOW_BIND_ERROR_TIMEOUT:
        return "bind timeout";
    case ESPNOW_BIND_ERROR_RSSI:
        return "bind packet RSSI below expected threshold";
    case ESPNOW_BIND_ERROR_LIST_FULL:
        return "bindlist is full";
    default:
        return "unknown error";
    }
}
