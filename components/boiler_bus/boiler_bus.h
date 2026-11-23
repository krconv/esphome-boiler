#pragma once

#include <atomic>
#include <string>
#include <unordered_map>

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "boiler_core.h"

#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace esphome {
namespace boiler_bus {

// keep in sync with SENSORS / BINARY_SENSORS / TEXT_SENSORS in __init__.py
#define BOILER_SENSORS(X) \
  X(supply_temperature) X(return_temperature) X(cold_in_temperature) X(dhw_out_temperature) \
  X(exhaust_temperature) X(unknown_temperature) X(ch_setpoint) X(dhw_setpoint) \
  X(power_on_hours) X(ch_burner_hours) X(dhw_burner_hours) X(pump_hours) X(ch_ignitions) X(dhw_counter) \
  X(flow) X(fan_rpm) X(water_pressure) X(flame_sensor) \
  X(gas_btu_h) X(propane_gal_h) X(propane_total) \
  X(frames_per_second) X(frames_ok) X(crc_errors) X(malformed_frames) X(unknown_frames) X(adc_overruns) \
  X(adc_baseline) X(dip_depth) X(bit_period) X(last_frame_age)
#define BOILER_BINARY_SENSORS(X) X(pump) X(ch_demand) X(spark) X(flame) X(dhw_demand) X(bus_active)
#define BOILER_TEXT_SENSORS(X) X(state) X(status_raw) X(error_history)

class BoilerBus : public Component {
 public:
#define DECL_S(n) SUB_SENSOR(n)
  BOILER_SENSORS(DECL_S)
#undef DECL_S
#define DECL_B(n) SUB_BINARY_SENSOR(n)
  BOILER_BINARY_SENSORS(DECL_B)
#undef DECL_B
#define DECL_T(n) SUB_TEXT_SENSOR(n)
  BOILER_TEXT_SENSORS(DECL_T)
#undef DECL_T

  void set_tx_pin(uint8_t p) { this->tx_pin_ = p; }
  void set_adc_pin(uint8_t p) { this->adc_pin_ = p; }
  void set_sample_rate(uint32_t r) { this->sample_rate_ = r; }
  void set_dip_depth_mv(float v) { this->dip_depth_mv_ = v; }
  void set_dip_threshold_mv(float v) { this->dip_threshold_mv_ = v; }
  void set_auto_threshold(bool v) { this->auto_threshold_ = v; }
  void set_assume_high_word_zero(bool v) { this->assume_high_zero_ = v; }
  void set_fan_stale_ms(uint32_t v) { this->fan_stale_ms_ = v; }
  void set_log_frames(bool v) { this->log_frames_ = v; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  // before anything else a user component does: GPIO32 must be driven low first
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  struct QItem {
    boiler::Frame frame;
    uint32_t t_ms;
  };

  void drive_tx_low_();
  bool start_adc_();
  static void task_trampoline_(void *arg);
  void sampler_loop_();
  void handle_burst_(const boiler::BurstResult &br);
  void process_frame_(const QItem &it);
  void on_register_(uint16_t reg, uint16_t val, uint32_t now);
  void publish_counter_(uint16_t hi_reg, uint16_t lo_reg, sensor::Sensor *s);
  void publish_error_history_();
  void tick_(uint32_t now);
  void publish_diagnostics_(uint32_t now);
  void pub_(sensor::Sensor *s, float v, uint32_t now);
  void pub_text_(text_sensor::TextSensor *s, const std::string &v);
  float counts_to_mv_(float counts) const;

  uint8_t tx_pin_{0}, adc_pin_{0};
  uint32_t sample_rate_{100000};
  float dip_depth_mv_{170}, dip_threshold_mv_{85};
  bool auto_threshold_{true}, assume_high_zero_{true}, log_frames_{false};
  uint32_t fan_stale_ms_{10000};

  // sampler side (task)
  adc_continuous_handle_t adc_{nullptr};
  adc_cali_handle_t cali_{nullptr};
  TaskHandle_t task_{nullptr};
  QueueHandle_t queue_{nullptr};
  float mv_per_count_{0.8f};
  boiler::DipDetector *dips_{nullptr};
  boiler::BurstDecoder *bursts_{nullptr};
  std::atomic<bool> overflow_{false};
  std::atomic<uint32_t> overruns_{0}, crc_errors_{0}, malformed_{0}, queue_drops_{0};
  std::atomic<float> baseline_counts_{0}, depth_counts_{0}, zero_period_us_{0};
  static bool IRAM_ATTR on_pool_ovf_(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *user);

  // main-loop side
  boiler::RegisterTable table_;
  boiler::Pairer pairer_;
  boiler::Counters counters_;
  uint32_t last_frame_ms_{0};
  bool any_frame_{false};
  std::unordered_map<sensor::Sensor *, std::pair<float, uint32_t>> last_pub_;
  std::string last_state_, last_raw_, last_err_;
  uint32_t last_tick_ms_{0}, last_diag_ms_{0}, last_diag_frames_{0}, last_save_ms_{0};
  double propane_total_{0};
  float saved_total_{0};
  bool total_restored_{false};
  ESPPreferenceObject pref_;
};

}  // namespace boiler_bus
}  // namespace esphome
