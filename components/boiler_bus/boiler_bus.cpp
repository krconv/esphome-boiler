#include "boiler_bus.h"

#include "driver/gpio.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace boiler_bus {

static const char *const TAG = "boiler_bus";

static constexpr uint32_t HEARTBEAT_MS = 60000;      // re-publish unchanged values this often
static constexpr uint32_t STATUS_STALE_MS = 30000;   // 0010 is polled every ~2.7 s; older = bus quiet
static constexpr uint32_t DIAG_MS = 10000;
static constexpr uint32_t SAVE_MS = 10 * 60 * 1000;
static constexpr size_t ADC_FRAME_BYTES = 1024;      // 5 ms of samples at 100 kS/s per DMA frame
static constexpr size_t ADC_POOL_BYTES = 16384;

// ---------------------------------------------------------------------------------------------
void BoilerBus::drive_tx_low_() {
  // Never transmit: this pin drives an NPN that pulls the bus down. Latch LOW before enabling the output driver.
  gpio_set_level((gpio_num_t) this->tx_pin_, 0);
  gpio_set_direction((gpio_num_t) this->tx_pin_, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t) this->tx_pin_, 0);
}

void BoilerBus::setup() {
  this->drive_tx_low_();

  this->pref_ = global_preferences->make_preference<float>(fnv1_hash("boiler_bus_propane_total"));
  float restored;
  if (this->pref_.load(&restored) && std::isfinite(restored) && restored >= 0) {
    this->propane_total_ = restored;
    this->saved_total_ = restored;
    this->total_restored_ = true;
  }

  this->queue_ = xQueueCreate(64, sizeof(QItem));
  if (!this->queue_ || !this->start_adc_()) {
    ESP_LOGE(TAG, "ADC sampling failed to start; component disabled");
    this->mark_failed();
    return;
  }
}

bool BoilerBus::start_adc_() {
  adc_unit_t unit;
  adc_channel_t chan;
  if (adc_continuous_io_to_channel(this->adc_pin_, &unit, &chan) != ESP_OK || unit != ADC_UNIT_1) {
    ESP_LOGE(TAG, "GPIO%u is not an ADC1 pin", this->adc_pin_);
    return false;
  }

  // calibration is only used to convert counts <-> mV for thresholds and diagnostics
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t cc = {};
  cc.unit_id = ADC_UNIT_1;
  cc.atten = ADC_ATTEN_DB_12;
  cc.bitwidth = ADC_BITWIDTH_12;
  if (adc_cali_create_scheme_line_fitting(&cc, &this->cali_) != ESP_OK) this->cali_ = nullptr;
#endif
  if (this->cali_) {
    int a = 0, b = 0;
    adc_cali_raw_to_voltage(this->cali_, 2000, &a);
    adc_cali_raw_to_voltage(this->cali_, 3000, &b);
    if (b > a) this->mv_per_count_ = float(b - a) / 1000.0f;
  }

  boiler::DipDetectorConfig dc;
  dc.nominal_depth = this->dip_depth_mv_ / this->mv_per_count_;
  dc.threshold_frac = this->dip_threshold_mv_ / this->dip_depth_mv_;
  dc.min_threshold = 0.4f * this->dip_threshold_mv_ / this->mv_per_count_;
  dc.auto_depth = this->auto_threshold_;
  dc.fixed_threshold = this->auto_threshold_ ? 0.0f : this->dip_threshold_mv_ / this->mv_per_count_;
  dc.warmup_samples = this->sample_rate_ / 20;  // 50 ms fast baseline convergence
  dc.baseline_alpha = 1.0f / (this->sample_rate_ * 0.04f);  // ~40 ms time constant
  dc.max_width = this->sample_rate_ / 330;                   // ~3 ms
  dc.min_width = 2;
  this->dips_ = new boiler::DipDetector(dc);
  this->bursts_ = new boiler::BurstDecoder((float) this->sample_rate_);

  adc_continuous_handle_cfg_t hc = {};
  hc.max_store_buf_size = ADC_POOL_BYTES;
  hc.conv_frame_size = ADC_FRAME_BYTES;
  if (adc_continuous_new_handle(&hc, &this->adc_) != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_new_handle failed");
    return false;
  }
  adc_digi_pattern_config_t pat = {};
  pat.atten = ADC_ATTEN_DB_12;
  pat.channel = chan;
  pat.unit = ADC_UNIT_1;
  pat.bit_width = ADC_BITWIDTH_12;
  adc_continuous_config_t cfg = {};
  cfg.pattern_num = 1;
  cfg.adc_pattern = &pat;
  cfg.sample_freq_hz = this->sample_rate_;
  cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
  if (adc_continuous_config(this->adc_, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_config failed (sample rate %u)", (unsigned) this->sample_rate_);
    return false;
  }
  adc_continuous_evt_cbs_t cbs = {};
  cbs.on_pool_ovf = &BoilerBus::on_pool_ovf_;
  adc_continuous_register_event_callbacks(this->adc_, &cbs, this);
  if (adc_continuous_start(this->adc_) != ESP_OK) {
    ESP_LOGE(TAG, "adc_continuous_start failed");
    return false;
  }
  if (xTaskCreatePinnedToCore(&BoilerBus::task_trampoline_, "boiler_adc", 8192, this, 5, &this->task_, 1) != pdPASS) {
    ESP_LOGE(TAG, "sampler task failed to start");
    return false;
  }
  return true;
}

bool IRAM_ATTR BoilerBus::on_pool_ovf_(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *user) {
  auto *self = static_cast<BoilerBus *>(user);
  self->overflow_.store(true, std::memory_order_relaxed);
  self->overruns_.fetch_add(1, std::memory_order_relaxed);
  return false;
}

void BoilerBus::task_trampoline_(void *arg) { static_cast<BoilerBus *>(arg)->sampler_loop_(); }

float BoilerBus::counts_to_mv_(float counts) const {
  if (this->cali_) {
    int mv = 0;
    if (adc_cali_raw_to_voltage(this->cali_, (int) counts, &mv) == ESP_OK) return (float) mv;
  }
  return counts * this->mv_per_count_;
}

// Runs in its own task: DMA samples -> dips -> bits -> frames. Only finished frames cross to the main loop.
void BoilerBus::sampler_loop_() {
  static uint8_t buf[ADC_FRAME_BYTES];
  uint32_t blocks = 0;
  while (true) {
    uint32_t n = 0;
    esp_err_t r = adc_continuous_read(this->adc_, buf, sizeof(buf), &n, 50);
    if (this->overflow_.exchange(false)) {
      this->dips_->reset_state();
      this->bursts_->abort();  // sample continuity lost: the burst in progress is invalid
    }
    if (r != ESP_OK) continue;  // ESP_ERR_TIMEOUT: nothing yet
    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= n; i += SOC_ADC_DIGI_RESULT_BYTES) {
      auto *p = reinterpret_cast<adc_digi_output_data_t *>(&buf[i]);
      uint16_t x = p->type1.data;
      boiler::Dip dip;
      boiler::BurstResult br;
      if (this->dips_->push(x, &dip) && this->bursts_->on_dip(dip.start, &br)) this->handle_burst_(br);
    }
    boiler::BurstResult br;
    if (this->bursts_->on_idle(this->dips_->index(), &br)) this->handle_burst_(br);
    if ((++blocks & 15) == 0) {
      this->baseline_counts_.store(this->dips_->baseline(), std::memory_order_relaxed);
      this->depth_counts_.store(this->dips_->depth(), std::memory_order_relaxed);
      this->zero_period_us_.store(this->bursts_->zero_period_us(), std::memory_order_relaxed);
    }
  }
}

void BoilerBus::handle_burst_(const boiler::BurstResult &br) {
  using boiler::ParseStatus;
  switch (br.status) {
    case ParseStatus::OK: {
      QItem it{br.frame, millis()};
      if (xQueueSend(this->queue_, &it, 0) != pdTRUE) this->queue_drops_.fetch_add(1, std::memory_order_relaxed);
      break;
    }
    case ParseStatus::CRC_ERROR: this->crc_errors_.fetch_add(1, std::memory_order_relaxed); break;
    case ParseStatus::NOISE: break;
    default: this->malformed_.fetch_add(1, std::memory_order_relaxed); break;
  }
}

// ---------------------------------------------------------------------------------------------
void BoilerBus::loop() {
  QItem it;
  int guard = 0;
  while (guard++ < 32 && xQueueReceive(this->queue_, &it, 0) == pdTRUE) this->process_frame_(it);
  const uint32_t now = millis();
  if (now - this->last_tick_ms_ >= 1000) {
    this->tick_(now);
    this->last_tick_ms_ = now;
  }
  if (now - this->last_diag_ms_ >= DIAG_MS) {
    this->publish_diagnostics_(now);
    this->last_diag_ms_ = now;
  }
}

void BoilerBus::process_frame_(const QItem &it) {
  const boiler::Frame &f = it.frame;
  this->counters_.frames_ok++;
  this->last_frame_ms_ = it.t_ms;
  this->any_frame_ = true;
  if (this->log_frames_)
    ESP_LOGD(TAG, "%s %s reg=%04X val=%04X", f.from_display() ? "disp>" : "boil>", f.is_request() ? (f.sub == 0x10 ? "WR" : "RD") : (f.sub == 0x10 ? "wr-ack" : "rsp"), f.reg, f.val);
  uint16_t reg;
  if (!this->pairer_.on_frame(f, &reg, &this->counters_.unpaired)) return;
  if (!boiler::is_known_register(reg)) {
    this->counters_.unknown_regs++;
    if (!this->table_.seen(reg)) ESP_LOGD(TAG, "first sight of unmapped register %04X = %04X", reg, f.val);
  }
  const bool first = !this->table_.seen(reg);
  this->table_.set(reg, f.val, it.t_ms);
  if (first && boiler::is_known_register(reg)) ESP_LOGI(TAG, "register %04X seen, value %04X", reg, f.val);
  this->on_register_(reg, f.val, it.t_ms);
}

void BoilerBus::pub_(sensor::Sensor *s, float v, uint32_t now) {
  if (!s) return;
  auto it = this->last_pub_.find(s);
  if (it != this->last_pub_.end()) {
    const bool same = (std::isnan(v) && std::isnan(it->second.first)) || v == it->second.first;
    if (same && now - it->second.second < HEARTBEAT_MS) return;
  }
  this->last_pub_[s] = {v, now};
  s->publish_state(v);
}

void BoilerBus::pub_text_(text_sensor::TextSensor *s, const std::string &v) {
  if (s) s->publish_state(v);
}

void BoilerBus::publish_counter_(uint16_t hi_reg, uint16_t lo_reg, sensor::Sensor *s) {
  if (!s) return;
  const auto *lo = this->table_.get(lo_reg);
  if (!lo) return;  // never publish until the low word has really been seen
  const auto *hi = this->table_.get(hi_reg);
  if (!hi && !this->assume_high_zero_) return;
  const uint32_t v = boiler::join32(hi ? hi->val : 0, lo->val);
  this->pub_(s, (float) v, millis());
}

void BoilerBus::publish_error_history_() {
  if (!this->error_history_text_sensor_) return;
  uint16_t v[5] = {};
  bool seen[5] = {};
  for (int i = 0; i < 5; i++) {
    const auto *e = this->table_.get(0x0142 + i);
    if (e) { v[i] = e->val; seen[i] = true; }
  }
  std::string s = boiler::format_error_history(v, seen);
  if (s != this->last_err_) {
    this->last_err_ = s;
    this->error_history_text_sensor_->publish_state(s);
  }
}

void BoilerBus::on_register_(uint16_t reg, uint16_t val, uint32_t now) {
  using namespace boiler;
  switch (reg) {
    case 0x0010: {
      if (this->pump_binary_sensor_) this->pump_binary_sensor_->publish_state(val & ST_PUMP);
      if (this->ch_demand_binary_sensor_) this->ch_demand_binary_sensor_->publish_state(val & ST_CH_DEMAND);
      if (this->spark_binary_sensor_) this->spark_binary_sensor_->publish_state(val & ST_SPARK);
      if (this->flame_binary_sensor_) this->flame_binary_sensor_->publish_state(val & ST_FLAME);
      if (this->dhw_demand_binary_sensor_) this->dhw_demand_binary_sensor_->publish_state(val & ST_DHW_DEMAND);
      std::string st = state_text(val);
      if (this->state_text_sensor_ && st != this->last_state_) {
        this->last_state_ = st;
        this->state_text_sensor_->publish_state(st);
      }
      if (this->status_raw_text_sensor_) {
        char raw[8];
        snprintf(raw, sizeof(raw), "0x%04X", val);
        if (this->last_raw_ != raw) {
          this->last_raw_ = raw;
          this->status_raw_text_sensor_->publish_state(raw);
        }
      }
      break;
    }
    case 0x0020: this->pub_(this->supply_temperature_sensor_, decode_temp_c(val), now); break;
    case 0x0130: this->pub_(this->return_temperature_sensor_, decode_temp_c(val), now); break;
    case 0x0021: this->pub_(this->cold_in_temperature_sensor_, decode_temp_c(val), now); break;
    case 0x0022: this->pub_(this->dhw_out_temperature_sensor_, decode_temp_c(val), now); break;
    case 0x0024: this->pub_(this->exhaust_temperature_sensor_, decode_temp_c(val), now); break;
    case 0x0023: this->pub_(this->unknown_temperature_sensor_, decode_temp_c(val), now); break;
    case 0x0101: this->pub_(this->ch_setpoint_sensor_, decode_temp_c(val), now); break;
    case 0x0102: this->pub_(this->dhw_setpoint_sensor_, decode_temp_c(val), now); break;
    case 0x0093: case 0x0094: this->publish_counter_(0x0093, 0x0094, this->power_on_hours_sensor_); break;
    case 0x0091: case 0x0092: this->publish_counter_(0x0091, 0x0092, this->ch_burner_hours_sensor_); break;
    case 0x0133: case 0x0134: this->publish_counter_(0x0133, 0x0134, this->dhw_burner_hours_sensor_); break;
    case 0x0095: case 0x0096: this->publish_counter_(0x0095, 0x0096, this->ch_ignitions_sensor_); break;
    case 0x0135: case 0x0136: this->publish_counter_(0x0135, 0x0136, this->dhw_counter_sensor_); break;
    case 0x0098: case 0x0099: this->publish_counter_(0x0098, 0x0099, this->pump_hours_sensor_); break;
    case 0x0027: this->pub_(this->flow_sensor_, val / 1000.0f, now); break;
    case 0x0033: this->pub_(this->fan_rpm_sensor_, (float) val, now); break;
    case 0x0132: this->pub_(this->water_pressure_sensor_, (float) val, now); break;
    case 0x0137: this->pub_(this->flame_sensor_sensor_, val / 10.0f, now); break;
    case 0x0142: case 0x0143: case 0x0144: case 0x0145: case 0x0146: this->publish_error_history_(); break;
    default: break;
  }
}

// 1 Hz: gas estimate + bus_active
void BoilerBus::tick_(uint32_t now) {
  using namespace boiler;
  const bool active = this->any_frame_ && now - this->last_frame_ms_ < STATUS_STALE_MS;
  if (this->bus_active_binary_sensor_) this->bus_active_binary_sensor_->publish_state(active);

  const auto *st = this->table_.get(0x0010);
  const bool status_fresh = st && now - st->t_ms < STATUS_STALE_MS;
  float btu = NAN;
  if (status_fresh) {
    if (!(st->val & ST_FLAME)) {
      btu = 0.0f;  // burner off: exactly zero
    } else {
      const auto *fan = this->table_.get(0x0033);
      // fan RPM is only on the bus while the display sits on a status screen
      if (fan && now - fan->t_ms < this->fan_stale_ms_) btu = gas_btu_per_hour((float) fan->val);
    }
  }
  const float gph = std::isnan(btu) ? NAN : propane_gal_per_hour(btu);
  if (this->gas_btu_h_sensor_ || this->propane_gal_h_sensor_ || this->propane_total_sensor_) {
    this->pub_(this->gas_btu_h_sensor_, btu, now);
    this->pub_(this->propane_gal_h_sensor_, gph, now);
    const uint32_t dt = now - this->last_tick_ms_;
    if (!std::isnan(gph) && dt > 0 && dt < 5000 && this->last_tick_ms_ != 0) this->propane_total_ += double(gph) * dt / 3600000.0;
    if (this->propane_total_sensor_ && (this->total_restored_ || this->propane_total_ > 0))
      this->pub_(this->propane_total_sensor_, (float) this->propane_total_, now);
  }
  if (now - this->last_save_ms_ >= SAVE_MS) {
    this->last_save_ms_ = now;
    if (std::fabs((float) this->propane_total_ - this->saved_total_) >= 0.001f) {
      float v = (float) this->propane_total_;
      if (this->pref_.save(&v)) this->saved_total_ = v;
    }
  }
}

void BoilerBus::on_shutdown() {
  float v = (float) this->propane_total_;
  if (std::fabs(v - this->saved_total_) >= 0.001f) this->pref_.save(&v);
}

void BoilerBus::publish_diagnostics_(uint32_t now) {
  const uint32_t crc = this->crc_errors_.load(), bad = this->malformed_.load();
  const uint32_t total = this->counters_.frames_ok + crc + bad;
  const float dt_s = DIAG_MS / 1000.0f;
  this->pub_(this->frames_per_second_sensor_, (total - this->last_diag_frames_) / dt_s, now);
  this->last_diag_frames_ = total;
  this->pub_(this->frames_ok_sensor_, (float) this->counters_.frames_ok, now);
  this->pub_(this->crc_errors_sensor_, (float) crc, now);
  this->pub_(this->malformed_frames_sensor_, (float) bad, now);
  this->pub_(this->unknown_frames_sensor_, (float) this->counters_.unknown_regs, now);
  this->pub_(this->adc_overruns_sensor_, (float) this->overruns_.load(), now);
  const float base = this->baseline_counts_.load();
  if (base > 0) {
    this->pub_(this->adc_baseline_sensor_, this->counts_to_mv_(base), now);
    this->pub_(this->dip_depth_sensor_, this->depth_counts_.load() * this->mv_per_count_, now);
  }
  const float zp = this->zero_period_us_.load();
  if (zp > 0) this->pub_(this->bit_period_sensor_, zp, now);
  if (this->any_frame_) this->pub_(this->last_frame_age_sensor_, (now - this->last_frame_ms_) / 1000.0f, now);
}

void BoilerBus::dump_config() {
  ESP_LOGCONFIG(TAG, "Boiler bus (passive display-link decoder):");
  ESP_LOGCONFIG(TAG, "  ADC pin: GPIO%u, %u samples/s, ~%.3f mV/count", this->adc_pin_, (unsigned) this->sample_rate_, this->mv_per_count_);
  ESP_LOGCONFIG(TAG, "  TX pin: GPIO%u (held LOW, never transmits)", this->tx_pin_);
  ESP_LOGCONFIG(TAG, "  Dip depth %.0f mV, threshold %.0f mV (%s)", this->dip_depth_mv_, this->dip_threshold_mv_,
                this->auto_threshold_ ? "auto-tracks depth" : "fixed");
  ESP_LOGCONFIG(TAG, "  High word of 32-bit counters assumed 0 when unseen: %s", YESNO(this->assume_high_zero_));
  if (this->total_restored_) ESP_LOGCONFIG(TAG, "  Restored propane total: %.3f gal", this->propane_total_);
}

}  // namespace boiler_bus
}  // namespace esphome
