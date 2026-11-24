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

#include <vector>

#include "driver/rmt_tx.h"
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
  X(adc_baseline) X(dip_depth) X(bit_period) X(last_frame_age) \
  X(malformed_short) X(malformed_long) X(dip_width) X(rejected_dips) X(tx_dip_depth) X(tx_dip_depth_bus) X(tx_bursts)
#define BOILER_BINARY_SENSORS(X) X(pump) X(ch_demand) X(spark) X(flame) X(dhw_demand) X(bus_active)
#define BOILER_TEXT_SENSORS(X) X(state) X(status_raw) X(error_history) X(tx_last_result) X(malformed_info) X(malformed_trace) X(ok_trace) \
  X(tx_compare) X(req_compare) X(resp_compare) X(tx_trace) X(req_trace) X(resp_trace)

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

  // ---- transmit test controls (called from HA entities, main loop) ----
  // GPIO32 only ever goes HIGH inside a pulse list that boiler::tx_pulses_safe() accepted.
  void set_tx_test_enabled(bool on) { this->tx_test_on_.store(on); }
  void set_tx_dip_us(float us) { this->tx_dip_us_.store(boiler::clamp_tx_dip_us(us)); }
  void set_tx_dip_count(float n) { this->tx_dip_count_.store(n < 1 ? 1 : (n > 32 ? 32 : (uint32_t) n)); }
  bool set_tx_register(const std::string &hex);  // false if not 4 hex digits
  void tx_read_register();                       // one read request, only in a safe gap, at most once per second

  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;
  // before anything else a user component does: GPIO32 must be driven low first
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  enum class Kind : uint8_t { FRAME, MALFORMED, TX_DEPTH, TX_SENT, TRACE, COMPARE };
  struct QItem {
    boiler::Frame frame;
    uint32_t t_ms;
    Kind kind{Kind::FRAME};
    float depth_mv{0};
    uint32_t found{0}, expected{0};
    char info[256];
  };

  void drive_tx_low_();
  bool start_adc_();
  static void task_trampoline_(void *arg);
  void sampler_loop_();
  void handle_burst_(const boiler::BurstResult &br);
  void queue_compare_(const boiler::BurstResult &br, uint32_t sink, const char *tag, const char *extra);
  void enqueue_(const QItem &it);
  bool tx_init_();
  void tx_service_(bool drained);
  bool tx_fire_(const std::vector<boiler::TxPulse> &p, bool is_frame, uint32_t expect_dips);
  void tx_probe_push_(uint64_t idx, uint16_t x);
  void tx_finish_();
  void on_tx_event_(const QItem &it);
  void req_tick_(uint32_t now);
  void req_on_frame_(const boiler::Frame &f, uint32_t t_ms);
  void req_result_(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
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
  std::atomic<float> baseline_counts_{0}, depth_counts_{0}, zero_period_us_{0}, width_samples_{0};
  std::atomic<uint32_t> malformed_short_{0}, malformed_long_{0}, rejected_dips_{0}, malformed_info_drops_{0};
  std::string req_seen_;  // frames decoded while waiting for the response, e.g. "D0010 B0010" (main loop only)
  int64_t last_info_us_{0}, last_ok_trace_us_{0}, last_req_cmp_us_{0}, last_resp_cmp_us_{0};
  uint16_t *ring_{nullptr};  // last RING_N raw ADC samples, for traces
  void queue_trace_(const boiler::BurstResult &br, bool bad, int bitpos, uint32_t sink = 9);

  // transmit side. Shared with the main loop: the atomics. Everything else is sampler-task only.
  std::atomic<bool> tx_test_on_{false}, tx_req_pending_{false};
  std::atomic<uint32_t> tx_dip_us_{boiler::TX_DIP_US_DEFAULT}, tx_dip_count_{8};
  std::atomic<uint16_t> tx_req_reg_{0x0033};
  rmt_channel_handle_t tx_chan_{nullptr};
  rmt_encoder_handle_t tx_enc_{nullptr};
  bool tx_ok_{false}, tx_active_{false}, tx_is_frame_{false}, tx_swallow_{false};
  std::vector<rmt_symbol_word_t> tx_syms_;
  int64_t last_tx_us_{-10000000}, last_test_us_{-10000000}, tx_end_us_{0};
  uint64_t tx_fire_idx_{0}, tx_win_end_{0};
  uint32_t tx_expect_{0};
  float tx_base_{0};
  bool tx_in_{false};
  float tx_min_{0};
  uint32_t tx_len_{0};
  std::vector<float> tx_depths_;
  bool last_burst_board_{false};
  float us_per_sample_{10.0f};
  std::atomic<uint32_t> test_bursts_{0};

  // request tracking (main loop)
  enum class Req : uint8_t { IDLE, WAIT_SLOT, SENT, WAIT_RESP };
  Req req_{Req::IDLE};
  uint16_t req_reg_{0};
  uint32_t req_press_ms_{0}, req_sent_ms_{0}, req_t0_{0}, req_disp_seen_{0}, req_board_seen_{0};
  uint32_t last_press_ms_{0};
  bool any_press_{false};
  bool req_display_between_{false};
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
