#pragma once

#include <algorithm>
#include <atomic>
#include <string>
#include <unordered_map>

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "boiler_core.h"
#include "boiler_poller.h"

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
#define BOILER_BINARY_SENSORS(X) X(pump) X(ch_demand) X(spark) X(flame) X(dhw_demand) X(bus_active) X(display_connected)
#define BOILER_TEXT_SENSORS(X) X(state) X(status_raw) X(error_history) X(poll_result) X(malformed_info) X(malformed_trace) X(ok_trace) \
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

  // ---- bus master (poller); called from the main loop / HA entities ----
  // GPIO32 only ever goes HIGH inside a pulse list that boiler::tx_pulses_safe() accepted.
  void set_ping_delay_ms(uint32_t ms) { this->ping_delay_us_ = ms * 1000u; }  // quiet time after a board frame, no display
  void set_fast_gap_ms(uint32_t ms) { this->fast_gap_us_ = ms * 1000u; }       // quiet time after a board reply, joined
  void set_min_interval_ms(uint32_t ms) { this->tx_min_spacing_us_.store(std::max<uint32_t>(ms, 120) * 1000u); }
  void set_polling_enabled(bool on);
  bool polling_enabled() const { return this->poller_.enabled(); }
  // Heating setpoint (register 0101) as last read from the boiler (poller or display); NAN until seen
  float ch_setpoint_c() const;
  // Queue a write of the heating setpoint. false (and a log line) if refused: out of 30..60 C, display present, polling off, busy.
  bool write_ch_setpoint(float celsius);

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
    bool own{false};  // FRAME: decoded back from our own transmission (TX_SENT: frame = what was sent)
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
  void poll_tick_(uint32_t now);
  void poll_arm_(const boiler::PollRequest &r, bool valid);
  void poll_timing_();
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
  int64_t last_info_us_{0}, last_ok_trace_us_{0}, last_req_cmp_us_{0}, last_resp_cmp_us_{0};
  uint16_t *ring_{nullptr};  // last RING_N raw ADC samples, for traces
  void queue_trace_(const boiler::BurstResult &br, bool bad, int bitpos, uint32_t sink = 9);

  // transmit side. Shared with the main loop: the atomics. Everything else is sampler-task only.
  // tx_req_word_: 0 = nothing armed, else bit 63 | sub << 32 | reg << 16 | val
  std::atomic<uint64_t> tx_req_word_{0};
  std::atomic<uint32_t> tx_req_min_us_{3050000}, tx_req_max_us_{3056000}, tx_min_spacing_us_{250000};
  uint32_t tx_dip_us_{boiler::TX_DIP_US_FIXED};
  rmt_channel_handle_t tx_chan_{nullptr};
  rmt_encoder_handle_t tx_enc_{nullptr};
  bool tx_ok_{false}, tx_active_{false}, tx_is_frame_{false}, tx_swallow_{false};
  std::vector<rmt_symbol_word_t> tx_syms_;
  int64_t last_tx_us_{-10000000}, tx_end_us_{0};
  uint64_t tx_fire_idx_{0}, tx_win_end_{0};
  uint32_t tx_expect_{0};
  float tx_base_{0};
  bool tx_in_{false};
  float tx_min_{0};
  uint32_t tx_len_{0};
  std::vector<float> tx_depths_;
  bool last_burst_board_{false};
  float us_per_sample_{10.0f};
  std::atomic<uint32_t> tx_count_{0};

  // poller (main loop)
  boiler::Poller poller_{boiler::PollerConfig(), boiler::PollScheduler::defaults()};
  uint32_t ping_delay_us_{3050000}, fast_gap_us_{46000};
  uint32_t published_seq_{0};
  int published_mode_{-1};
  int last_display_state_{-1};
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
