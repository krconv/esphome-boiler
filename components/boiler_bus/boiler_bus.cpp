#include "boiler_bus.h"

#include <algorithm>
#include <cstdarg>
#include <cstring>
#include <functional>

#include "driver/gpio.h"
#include "esp_timer.h"
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
static constexpr size_t ADC_FRAME_BYTES = 256;       // 1.28 ms of samples at 100 kS/s per DMA frame (keeps the TX idle check fresh)
static constexpr size_t ADC_READ_BYTES = 1024;
static constexpr size_t ADC_POOL_BYTES = 16384;
static constexpr uint32_t RING_N = 8192;            // raw sample ring (power of two), 82 ms at 100 kS/s
static constexpr size_t TRACE_SAMPLES = 248;

// transmit guards (see boiler_core.h for the pulse limits)
// (the minimum spacing between transmissions is tx_min_spacing_us_, never below 120 ms: the display itself does ~7.7 requests/s)
static constexpr uint32_t PING_WINDOW_US = 6000;     // ping slot: fire within this long after the configured quiet time
static constexpr uint32_t FAST_WINDOW_US = 1000000;  // joined: fire any time up to 1 s after the board's reply
static constexpr uint32_t TX_PROBE_TAIL_US = 6000;        // sample window kept after a test pattern
static constexpr uint32_t TX_PROBE_TAIL_FRAME_US = 2500;  // ... after a frame (next display request is >= 44 ms after the reply)
static constexpr float TX_PROBE_MIN_DEPTH = 40.0f;        // counts (~32 mV): a dip must go this far below baseline
static constexpr uint32_t EXP_RECORD_US = 225000;         // experiment: bursts starting this long after our request are recorded
static constexpr uint32_t EXP_END_US = 275000;            // ... and the result is sent this long after it (lets the last burst finish)
static constexpr uint32_t EXP_MIN_SPACING_MS = 1500;      // at most one experiment frame per 1.5 s (sweep: 2 s)
static constexpr uint32_t EXP_SWEEP_SPACING_MS = 2000;
static constexpr uint32_t EXP_ARM_TIMEOUT_MS = 1500;     // armed but no usable gap came: give up on this try
static constexpr uint16_t EXP_REG = 0x0020;
static_assert(sizeof(boiler::ExpEvent) * boiler::EXP_MAX_EVENTS <= 256, "events must fit a QItem");
static constexpr float DIVIDER_RATIO = 10.0f / 60.0f;      // 10k / (50k + 10k): bus volts -> ADC volts

// ---------------------------------------------------------------------------------------------
void BoilerBus::drive_tx_low_() {
  // Never transmit: this pin drives an NPN that pulls the bus down. Latch LOW before enabling the output driver.
  gpio_set_level((gpio_num_t) this->tx_pin_, 0);
  gpio_set_direction((gpio_num_t) this->tx_pin_, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t) this->tx_pin_, 0);
}

void BoilerBus::setup() {
  this->drive_tx_low_();
  this->us_per_sample_ = 1e6f / (float) this->sample_rate_;
  // NB: no gpio_set_direction() after this: it would disconnect the RMT output from the pin
  this->tx_ok_ = this->tx_init_();
  if (this->poll_result_text_sensor_) this->poll_result_text_sensor_->publish_state("idle");

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

// RMT drives GPIO32 for requests: each symbol is {HIGH dip, LOW rest}, durations are fixed in hardware, and
// the channel idles and ends every transmission LOW. If it cannot be set up, the pin stays a plain LOW output.
bool BoilerBus::tx_init_() {
  rmt_tx_channel_config_t cc = {};
  cc.clk_src = RMT_CLK_SRC_DEFAULT;
  cc.gpio_num = (gpio_num_t) this->tx_pin_;
  cc.mem_block_symbols = 64;
  cc.resolution_hz = 1000000;  // 1 us ticks
  cc.trans_queue_depth = 1;
  cc.flags.invert_out = 0;
  cc.flags.with_dma = 0;
  cc.flags.init_level = 0;
  if (rmt_new_tx_channel(&cc, &this->tx_chan_) != ESP_OK) {
    ESP_LOGW(TAG, "RMT channel for GPIO%u failed; polling disabled", this->tx_pin_);
    this->tx_chan_ = nullptr;
    return false;
  }
  rmt_copy_encoder_config_t ec = {};
  if (rmt_new_copy_encoder(&ec, &this->tx_enc_) != ESP_OK || rmt_enable(this->tx_chan_) != ESP_OK) {
    ESP_LOGW(TAG, "RMT encoder/enable failed; polling disabled");
    return false;
  }
  this->tx_depths_.reserve(160);
  this->tx_syms_.reserve(160);
  return true;
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
  this->ring_ = (uint16_t *) calloc(RING_N, sizeof(uint16_t));

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
  if (xTaskCreatePinnedToCore(&BoilerBus::task_trampoline_, "boiler_adc", 12288, this, 5, &this->task_, 1) != pdPASS) {
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
  static uint8_t buf[ADC_READ_BYTES];
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
      if (this->ring_) this->ring_[this->dips_->index() & (RING_N - 1)] = x;
      if (this->tx_active_) this->tx_probe_push_(this->dips_->index(), x);
      if (this->dips_->push(x, &dip)) {
        // our own test-pattern dips are measured by the probe and must not look like a burst on the bus
        if (this->tx_active_ && this->tx_swallow_ && dip.start >= this->tx_fire_idx_) continue;
        if (this->bursts_->on_dip(dip.start, &br)) this->handle_burst_(br);
      }
    }
    boiler::BurstResult br;
    if (this->bursts_->on_idle(this->dips_->index(), &br)) this->handle_burst_(br);
    this->tx_service_(n < sizeof(buf));
    if ((++blocks & 15) == 0) {
      this->baseline_counts_.store(this->dips_->baseline(), std::memory_order_relaxed);
      this->depth_counts_.store(this->dips_->depth(), std::memory_order_relaxed);
      this->zero_period_us_.store(this->bursts_->zero_period_us(), std::memory_order_relaxed);
      this->width_samples_.store(this->dips_->width(), std::memory_order_relaxed);
      this->rejected_dips_.store((uint32_t) this->dips_->rejected(), std::memory_order_relaxed);
    }
  }
}

void BoilerBus::enqueue_(const QItem &it) {
  if (xQueueSend(this->queue_, &it, 0) != pdTRUE) this->queue_drops_.fetch_add(1, std::memory_order_relaxed);
}

void BoilerBus::handle_burst_(const boiler::BurstResult &br) {
  using boiler::ParseStatus;
  // a burst that began after we started transmitting is ours (the bus is idle >= 2 ms before we start)
  const bool own = this->tx_active_ && br.start_sample >= this->tx_fire_idx_;
  if (this->exp_active_ && br.start_sample >= this->exp_fire_idx_) this->exp_record_(br, own);
  const char *label = "";
  if (own && this->tx_is_frame_) {
    char dec[80];
    if (br.status == ParseStatus::OK)
      snprintf(dec, sizeof(dec), "dec=%02X%02X %02X/%02X reg=%04X val=%04X crc=%04X OK", br.frame.addr1, br.frame.addr2, br.frame.type,
               br.frame.sub, br.frame.reg, br.frame.val, br.frame.crc);
    else
      snprintf(dec, sizeof(dec), "dec=FAIL status=%d nbits=%u", (int) br.status, (unsigned) br.nbits);
    this->queue_compare_(br, 2, "OWN", dec);
  } else if (!own && br.ndips >= 100) {
    const int64_t t = esp_timer_get_time();
    if (br.status == ParseStatus::OK && br.frame.from_display() && br.frame.is_request() && t - this->last_req_cmp_us_ > 2000000) {
      this->last_req_cmp_us_ = t;
      char x[24];
      snprintf(x, sizeof(x), "reg=%04X", br.frame.reg);
      this->queue_compare_(br, 0, "DISP", x);
    } else if (br.status == ParseStatus::OK && br.frame.from_board() && br.frame.is_response() && t - this->last_resp_cmp_us_ > 2000000) {
      this->last_resp_cmp_us_ = t;
      char x[24];
      snprintf(x, sizeof(x), "reg=%04X", br.frame.reg);
      this->queue_compare_(br, 1, "BOARD", x);
    }
  }
  switch (br.status) {
    case ParseStatus::OK: {
      this->last_burst_board_ = !own && br.frame.from_board();
      if (!own && esp_timer_get_time() - this->last_ok_trace_us_ > 10000000) this->queue_trace_(br, false, 20);
      QItem it{};
      it.own = own;
      it.frame = br.frame;
      it.t_ms = millis();
      this->enqueue_(it);
      return;
    }
    case ParseStatus::NOISE: return;
    case ParseStatus::CRC_ERROR: label = "crc"; break;
    case ParseStatus::BAD_LENGTH: label = br.nbits < boiler::FRAME_BITS ? "short" : "long"; break;
    default: label = "format"; break;
  }
  this->last_burst_board_ = false;
  const int64_t now_us = esp_timer_get_time();
  if (!own) {
    if (br.status == ParseStatus::CRC_ERROR) {
      this->crc_errors_.fetch_add(1, std::memory_order_relaxed);
    } else {
      this->malformed_.fetch_add(1, std::memory_order_relaxed);
      if (br.status == ParseStatus::BAD_LENGTH)
        (br.nbits < boiler::FRAME_BITS ? this->malformed_short_ : this->malformed_long_).fetch_add(1, std::memory_order_relaxed);
    }
  } else if (!this->tx_is_frame_) {
    return;
  }
  // diagnostics: at most one description per second (our own failures always)
  if (!own && now_us - this->last_info_us_ < 1000000) return;
  if (!own) {
    this->last_info_us_ = now_us;
    this->queue_trace_(br, true, boiler::first_bad_bit(br.bits));
  }
  QItem it{};
  it.kind = Kind::MALFORMED;
  it.t_ms = millis();
  snprintf(it.info, sizeof(it.info), "%s%s n=%u dips=%u gap=%.1fms %s", own ? "OWN " : "", label, (unsigned) br.nbits,
           (unsigned) br.ndips, br.gap_samples * this->us_per_sample_ / 1000.0f, br.bits);
  this->enqueue_(it);
}

void BoilerBus::queue_compare_(const boiler::BurstResult &br, uint32_t sink, const char *tag, const char *extra) {
  if (!this->ring_ || br.ndips == 0) return;
  if (this->dips_->index() - br.start_sample > RING_N - 64) return;  // no longer buffered
  {  // raw counts of ~4 dips, starting 4 samples before dip 20
    const size_t k = std::min<size_t>(br.ndips - 1, 20);
    const uint64_t from = br.start_sample + br.dip_off[k] - 4;
    QItem rt{};
    rt.kind = Kind::TRACE;
    rt.found = 10 + sink;
    rt.t_ms = millis();
    int n = snprintf(rt.info, sizeof(rt.info), "%c:", sink == 2 ? 'T' : (sink == 1 ? 'B' : 'D'));
    boiler::render_raw(this->ring_, RING_N - 1, from, 80, rt.info + n);
    this->enqueue_(rt);
  }
  QItem it{};
  it.kind = Kind::COMPARE;
  it.found = sink;
  it.t_ms = millis();
  char s[200];
  boiler::summarize_burst(this->ring_, RING_N - 1, br.start_sample, br.dip_off, br.ndips, this->mv_per_count_, this->us_per_sample_, tag, s, sizeof(s));
  snprintf(it.info, sizeof(it.info), "%s | %s", s, extra);
  this->enqueue_(it);
}

// Raw ADC samples around the first damaged byte (bad) or from inside a good frame, as hex digits.
void BoilerBus::queue_trace_(const boiler::BurstResult &br, bool bad, int bitpos, uint32_t sink) {
  if (!this->ring_ || br.ndips == 0) return;
  size_t k;
  if (bad) {
    k = bitpos >= 0 ? (size_t) bitpos : (br.ndips > 14 ? br.ndips - 14 : 0);
  } else {
    this->last_ok_trace_us_ = esp_timer_get_time();
    k = 20;
  }
  if (k >= br.ndips) k = br.ndips - 1;
  if (k >= boiler::MAX_TRACK_DIPS) k = boiler::MAX_TRACK_DIPS - 1;
  const uint64_t at = br.start_sample + br.dip_off[k];
  const uint64_t from = at > 30 ? at - 30 : 0;
  const uint64_t now_idx = this->dips_->index();
  if (now_idx < from + TRACE_SAMPLES || now_idx - from > RING_N - 8) return;  // not buffered (any more)
  QItem it{};
  it.kind = Kind::TRACE;
  it.found = sink != 9 ? sink : (bad ? 1 : 0);
  it.t_ms = millis();
  int n = snprintf(it.info, sizeof(it.info), "%c%03d:", sink == 2 ? 't' : (bad ? 'b' : 'o'), bitpos);
  boiler::render_trace(this->ring_, RING_N - 1, from, TRACE_SAMPLES, this->dips_->baseline(), this->dips_->depth(), it.info + n);
  this->enqueue_(it);
}

// ---- transmit (sampler task) ----------------------------------------------------------
// Fire only when the bus has been quiet for a while, as seen by the sampler. The DMA delivers samples in 1.28 ms
// chunks and we only look after draining it, so the real quiet time is >= the measured one.
void BoilerBus::tx_service_(bool drained) {
  const int64_t now = esp_timer_get_time();
  this->exp_service_();
  if (this->tx_active_) {
    if (this->dips_->index() >= this->tx_win_end_ || now > this->tx_end_us_ + 50000) this->tx_finish_();
    return;
  }
  if (!this->tx_ok_ || !drained) return;
  const uint64_t word = this->tx_req_word_.load();
  if (!(word >> 63)) return;
  if (this->dips_->in_dip()) return;
  if (now - this->last_tx_us_ < (int64_t) this->tx_min_spacing_us_.load()) return;
  // quiet time as the sampler sees it; with no dip ever seen it is the time since sampling started
  const uint64_t since = this->bursts_->seen_dip() ? this->bursts_->last_dip_index() : 0;
  const float idle_us = float(this->dips_->index() - since) * this->us_per_sample_;
  // only ever in the gap after a board frame: a 31 ms frame has to end before the board's next one starts
  if (!this->last_burst_board_ || idle_us < (float) this->tx_req_min_us_.load() || idle_us > (float) this->tx_req_max_us_.load()) return;
  boiler::Frame f = (word & 0xFF00000000ull) == 0x1000000000ull ? boiler::make_write_request((uint16_t)(word >> 16), (uint16_t) word)
                                                                : boiler::make_read_request((uint16_t)(word >> 16));
  uint8_t bytes[boiler::FRAME_BYTES];
  boiler::encode_frame(f, bytes);
  auto p = boiler::tx_frame_pulses(bytes, this->tx_dip_us_);
  uint64_t exp = word;
  if (!this->tx_req_word_.compare_exchange_strong(exp, 0)) return;  // the main loop replaced or cancelled it meanwhile
  if (this->tx_fire_(p, true, (uint32_t) p.size())) {
    this->last_tx_us_ = now;
    this->tx_count_.fetch_add(1);
    QItem it{};
    it.kind = Kind::TX_SENT;
    it.frame = f;
    it.t_ms = millis();
    if (this->exp_pending_.exchange(false)) {  // this was the experiment's request, not the poller's
      it.own = true;
      this->exp_active_ = true;
      this->exp_have_own_ = false;
      this->exp_n_ = 0;
      this->exp_fire_idx_ = this->tx_fire_idx_;
      this->exp_fire_gap_us_ = idle_us;
      this->exp_end_idx_ = this->tx_fire_idx_ + (uint64_t)(EXP_END_US / this->us_per_sample_);
    }
    this->enqueue_(it);
  }
}

bool BoilerBus::tx_fire_(const std::vector<boiler::TxPulse> &p, bool is_frame, uint32_t expect_dips) {
  if (!this->tx_ok_ || this->tx_active_ || !boiler::tx_pulses_safe(p)) {  // the cap on HIGH time is enforced here
    ESP_LOGE(TAG, "refusing to transmit: pulse list not safe");
    return false;
  }
  this->tx_syms_.resize(p.size());
  for (size_t i = 0; i < p.size(); i++) {
    rmt_symbol_word_t s = {};
    s.duration0 = p[i].high_us;
    s.level0 = 1;
    s.duration1 = p[i].low_us;
    s.level1 = 0;
    this->tx_syms_[i] = s;
  }
  const uint32_t total_us = boiler::tx_pulses_total_us(p);
  this->tx_is_frame_ = is_frame;
  this->tx_swallow_ = !is_frame;
  this->tx_expect_ = expect_dips;
  this->tx_fire_idx_ = this->dips_->index();
  this->tx_base_ = this->dips_->baseline();
  this->tx_win_end_ = this->tx_fire_idx_ +
                      (uint64_t)((total_us + (is_frame ? TX_PROBE_TAIL_FRAME_US : TX_PROBE_TAIL_US)) / this->us_per_sample_);
  this->tx_in_ = false;
  this->tx_depths_.clear();
  this->tx_active_ = true;
  this->dips_->set_learn(false);
  rmt_transmit_config_t tc = {};
  tc.loop_count = 0;
  tc.flags.eot_level = 0;  // pin LOW when the list ends
  const int64_t now = esp_timer_get_time();
  this->tx_end_us_ = now + total_us;
  if (rmt_transmit(this->tx_chan_, this->tx_enc_, this->tx_syms_.data(), this->tx_syms_.size() * sizeof(rmt_symbol_word_t), &tc) !=
      ESP_OK) {
    ESP_LOGE(TAG, "rmt_transmit failed");
    this->tx_active_ = false;
    this->dips_->set_learn(true);
    return false;
  }
  return true;
}

void BoilerBus::tx_probe_push_(uint64_t idx, uint16_t x) {
  if (idx < this->tx_fire_idx_ || idx >= this->tx_win_end_) return;
  const bool low = float(x) < this->tx_base_ - TX_PROBE_MIN_DEPTH;
  if (low) {
    if (!this->tx_in_) {
      this->tx_in_ = true;
      this->tx_min_ = x;
      this->tx_len_ = 1;
    } else {
      this->tx_len_++;
      if (float(x) < this->tx_min_) this->tx_min_ = x;
    }
  } else if (this->tx_in_) {
    this->tx_in_ = false;
    if (this->tx_len_ >= 2 && this->tx_depths_.size() < this->tx_depths_.capacity()) this->tx_depths_.push_back(this->tx_base_ - this->tx_min_);
  }
}

void BoilerBus::tx_finish_() {
  if (this->tx_in_ && this->tx_len_ >= 2 && this->tx_depths_.size() < this->tx_depths_.capacity())
    this->tx_depths_.push_back(this->tx_base_ - this->tx_min_);
  this->tx_in_ = false;
  // depth = mean of the deepest `expected` dips (rejects the odd noise dip that scraped past the floor)
  std::sort(this->tx_depths_.begin(), this->tx_depths_.end(), std::greater<float>());
  const size_t k = std::min<size_t>(this->tx_depths_.size(), this->tx_expect_);
  float sum = 0;
  for (size_t i = 0; i < k; i++) sum += this->tx_depths_[i];
  QItem it{};
  it.kind = Kind::TX_DEPTH;
  it.t_ms = millis();
  it.found = (uint32_t) this->tx_depths_.size();
  it.expected = this->tx_expect_;
  it.depth_mv = k ? (sum / k) * this->mv_per_count_ : 0.0f;
  this->enqueue_(it);
  this->tx_active_ = false;
  this->tx_swallow_ = false;
  this->dips_->set_learn(true);
}

// ---------------------------------------------------------------------------------------------
void BoilerBus::loop() {
  QItem it;
  int guard = 0;
  while (guard++ < 32 && xQueueReceive(this->queue_, &it, 0) == pdTRUE) {
    switch (it.kind) {
      case Kind::FRAME: this->process_frame_(it); break;
      case Kind::MALFORMED:
        ESP_LOGD(TAG, "malformed burst: %s", it.info);
        if (this->malformed_info_text_sensor_) this->malformed_info_text_sensor_->publish_state(it.info);
        break;
      case Kind::TRACE: {
        text_sensor::TextSensor *s;
        if (it.found >= 10) s = it.found == 12 ? this->tx_trace_text_sensor_ : (it.found == 11 ? this->resp_trace_text_sensor_ : this->req_trace_text_sensor_);
        else s = it.found ? this->malformed_trace_text_sensor_ : this->ok_trace_text_sensor_;
        if (s) s->publish_state(it.info);
        break;
      }
      case Kind::COMPARE: {
        auto *s = it.found == 2 ? this->tx_compare_text_sensor_ : (it.found == 1 ? this->resp_compare_text_sensor_ : this->req_compare_text_sensor_);
        ESP_LOGI(TAG, "wave %s", it.info);
        if (s) s->publish_state(it.info);
        break;
      }
      case Kind::EXPERIMENT: this->on_experiment_result_(it); break;
      default: this->on_tx_event_(it); break;
    }
  }
  const uint32_t now = millis();
  this->poll_tick_(now);
  this->exp_tick_(now);
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
  if (it.own) {  // our own request, decoded back: not bus traffic and not a display
    counters_.frames_ok--;
    if (this->log_frames_) ESP_LOGD(TAG, "self> %s reg=%04X val=%04X", f.sub == 0x10 ? "WR" : "RD", f.reg, f.val);
    return;
  }
  if (f.from_display() && f.is_request()) this->poller_.on_display_request(it.t_ms);
  this->last_frame_ms_ = it.t_ms;
  this->any_frame_ = true;
  if (this->log_frames_)
    ESP_LOGD(TAG, "%s %s reg=%04X val=%04X", f.from_display() ? "disp>" : "boil>", f.is_request() ? (f.sub == 0x10 ? "WR" : "RD") : (f.sub == 0x10 ? "wr-ack" : "rsp"), f.reg, f.val);
  if (f.from_board() && f.is_response()) this->poller_.on_board_response(it.t_ms, f);
  uint16_t reg;
  if (!this->pairer_.on_frame(f, &reg, &this->counters_.unpaired)) return;
  if (f.sub == 0x10) return;  // write acknowledge (ours or the display's): the poller / read-back handle it, not a value read
  if (!boiler::is_known_register(reg)) {
    this->counters_.unknown_regs++;
    if (!this->table_.seen(reg)) {
      ESP_LOGD(TAG, "first sight of unmapped register %04X = %04X", reg, f.val);
    }
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
  if (!this->error_history_text_sensor_ && !this->last_error_text_sensor_) return;
  uint16_t v[5] = {};
  bool seen[5] = {};
  for (int i = 0; i < 5; i++) {
    const auto *e = this->table_.get(0x0142 + i);
    if (e) { v[i] = e->val; seen[i] = true; }
  }
  std::string s = boiler::format_error_history(v, seen);
  if (s != this->last_err_) {
    this->last_err_ = s;
    if (this->error_history_text_sensor_) this->error_history_text_sensor_->publish_state(s);
  }
  unsigned code;
  if (this->last_error_text_sensor_ && boiler::last_error_code(v, seen, &code)) {  // E0 = the most recent entry
    const std::string d = boiler::describe_error(code);
    if (d != this->last_err_text_) {
      this->last_err_text_ = d;
      this->last_error_text_sensor_->publish_state(d);
    }
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

// ---- bus master (main loop) -------------------------------------------------------------
void BoilerBus::set_polling_enabled(bool on) {
  this->poller_.set_enabled(on);
  ESP_LOGI(TAG, "bus polling %s", on ? "on" : "off");
}

float BoilerBus::ch_setpoint_c() const {
  const auto *e = this->table_.get(boiler::REG_CH_SETPOINT);
  return e ? boiler::decode_setpoint(e->val) : NAN;
}

bool BoilerBus::write_ch_setpoint(float c) {
  const uint32_t now = millis();
  if (!boiler::setpoint_in_range(c)) {
    ESP_LOGW(TAG, "CH setpoint %.1f C refused: outside %.0f..%.0f C", c, boiler::CH_SETPOINT_MIN_C, boiler::CH_SETPOINT_MAX_C);
    return false;
  }
  const uint16_t val = boiler::encode_setpoint(c);
  const auto r = this->poller_.request_write(now, val);
  switch (r) {
    case boiler::WriteResult::OK:
      ESP_LOGI(TAG, "CH setpoint write queued: %.1f C = %04X", boiler::decode_setpoint(val), val);
      if (this->poll_result_text_sensor_) this->poll_result_text_sensor_->publish_state("setpoint write queued");
      return true;
    case boiler::WriteResult::NOT_MASTER:
      ESP_LOGW(TAG, "CH setpoint write refused: %s", this->poller_.enabled() ? "a display is on the bus" : "bus polling is off");
      break;
    case boiler::WriteResult::BUSY: ESP_LOGW(TAG, "CH setpoint write refused: previous write still running"); break;
    default: ESP_LOGW(TAG, "CH setpoint write refused: value out of range"); break;
  }
  return false;
}

void BoilerBus::poll_arm_(const boiler::PollRequest &r, bool valid) {
  this->tx_req_word_.store(valid ? (1ull << 63) | ((uint64_t) r.sub << 32) | ((uint64_t) r.reg << 16) | r.val : 0);
}

// ping slot: long quiet time after a board frame, narrow window. joined: the display's short gap, wide window.
void BoilerBus::poll_timing_() {
  const bool fast = this->poller_.mode() == boiler::PollMode::FAST;
  const uint32_t lo = fast ? this->fast_gap_us_ : this->ping_delay_us_;
  this->tx_req_min_us_.store(lo);
  this->tx_req_max_us_.store(lo + (fast ? FAST_WINDOW_US : PING_WINDOW_US));
}

void BoilerBus::poll_tick_(uint32_t now) {
  if (!this->tx_ok_) return;
  this->poller_.update(now);
  const int disp = this->poller_.display_present(now) ? 1 : 0;
  if (disp != this->last_display_state_) {
    this->last_display_state_ = disp;
    if (this->display_connected_binary_sensor_) this->display_connected_binary_sensor_->publish_state(disp);
  }
  for (const auto &e : this->poller_.take_events()) {
    if (e.find("failed") != std::string::npos) ESP_LOGW(TAG, "poll: %s", e.c_str());
    else ESP_LOGI(TAG, "poll: %s", e.c_str());
    if (this->poll_result_text_sensor_) this->poll_result_text_sensor_->publish_state(e);
  }
  const int mode = (int) this->poller_.mode();
  if (mode != this->published_mode_) {
    this->published_mode_ = mode;
    this->poll_timing_();
  }
  if (this->poller_.armed_seq() != this->published_seq_) {
    this->published_seq_ = this->poller_.armed_seq();
    boiler::PollRequest r;
    const bool valid = this->poller_.armed(&r);
    this->poll_timing_();
    this->poll_arm_(r, valid);
  }
}

void BoilerBus::on_tx_event_(const QItem &it) {
  if (it.kind == Kind::TX_SENT) {
    if (it.own) {  // the experiment's frame: the poller did not send it
      this->exp_fired_ = true;
      this->exp_fired_ms_ = it.t_ms;
      return;
    }
    boiler::PollRequest r{it.frame.reg, it.frame.val, it.frame.sub};
    this->poller_.on_sent(it.t_ms, r);
    return;
  }
  if (it.kind == Kind::TX_DEPTH) {
    if (this->tx_dip_depth_sensor_) this->tx_dip_depth_sensor_->publish_state(it.depth_mv);
    if (this->tx_dip_depth_bus_sensor_) this->tx_dip_depth_bus_sensor_->publish_state(it.depth_mv / 1000.0f / DIVIDER_RATIO);
  }
}

// ---- experiment: transmit while the display is connected ------------------------------------
// Sampler task: after our request went out, record every burst for EXP_RECORD_US (time relative to the end of the
// board reply we followed), then hand the list to the main loop, which summarises it.
void BoilerBus::exp_record_(const boiler::BurstResult &br, bool own) {
  using boiler::ParseStatus;
  if (br.status == ParseStatus::NOISE || this->exp_n_ >= boiler::EXP_MAX_EVENTS) return;
  const float since_fire_us = float(br.start_sample - this->exp_fire_idx_) * this->us_per_sample_;
  if (since_fire_us > (float) EXP_RECORD_US) return;
  boiler::ExpEvent e{};
  e.t_ms = (since_fire_us + this->exp_fire_gap_us_) / 1000.0f;
  e.ok = br.status == ParseStatus::OK;
  e.reg = e.ok ? br.frame.reg : 0;
  if (own && !this->exp_have_own_) {
    e.kind = 'O';
    this->exp_have_own_ = true;
  } else if (e.ok && br.frame.from_board()) {
    e.kind = 'B';
  } else if (e.ok && br.frame.from_display() && br.frame.is_request()) {
    e.kind = 'D';
  } else {
    e.kind = 'X';
  }
  this->exp_ev_[this->exp_n_++] = e;
}

void BoilerBus::exp_service_() {
  if (!this->exp_active_ || this->dips_->index() < this->exp_end_idx_) return;
  QItem it{};
  it.kind = Kind::EXPERIMENT;
  it.t_ms = millis();
  it.found = (uint32_t) this->exp_n_;
  it.expected = (uint32_t) this->exp_fire_gap_us_;
  memcpy(it.info, this->exp_ev_, this->exp_n_ * sizeof(boiler::ExpEvent));
  this->enqueue_(it);
  this->exp_active_ = false;
}

// Main loop. Arms the request; the sampler fires it when the quiet time after a board reply reaches the gap.
bool BoilerBus::exp_begin_try_(uint32_t now, float gap_ms, bool sweep) {
  const char *refuse = nullptr;
  if (!this->tx_ok_) refuse = "transmitter not available";
  else if (!this->poller_.display_present(now)) refuse = "display not connected (the poller handles that case)";
  else if (this->exp_try_open_ || this->exp_active_) refuse = "previous try still running";
  else if (this->exp_ever_ && now - this->exp_last_ms_ < EXP_MIN_SPACING_MS) refuse = "too soon (max one frame per 1.5 s)";
  if (refuse) {
    ESP_LOGW(TAG, "experiment refused: %s", refuse);
    if (this->experiment_result_text_sensor_) this->experiment_result_text_sensor_->publish_state(std::string("refused: ") + refuse);
    return false;
  }
  const uint32_t lo = (uint32_t)(gap_ms * 1000.0f);
  this->exp_try_open_ = true;
  this->exp_fired_ = false;
  this->exp_try_sweep_ = sweep;
  this->exp_try_gap_ms_ = gap_ms;
  this->exp_try_ms_ = now;
  this->exp_last_ms_ = now;
  this->exp_ever_ = true;
  this->tx_req_min_us_.store(lo);
  this->tx_req_max_us_.store(lo + 4000);  // narrow window: a missed gap waits for the next display cycle
  this->exp_pending_.store(true);
  this->poll_arm_(boiler::PollRequest{EXP_REG, 0, 0x11}, true);
  ESP_LOGI(TAG, "experiment armed: read %04X at gap %.0f ms after the next board reply", EXP_REG, gap_ms);
  return true;
}

void BoilerBus::exp_cancel_try_(const char *why) {
  if (!this->exp_try_open_) return;
  if (this->exp_pending_.exchange(false)) {  // not fired yet
    this->tx_req_word_.store(0);
    this->exp_finish_try_("N--", std::string("not sent: ") + why);
  }
  // else the sampler already fired: its result arrives shortly
}

void BoilerBus::exp_finish_try_(const char *code, const std::string &line) {
  const bool sweep = this->exp_try_sweep_ && this->exp_sweep_on_;
  this->exp_try_open_ = false;
  this->exp_fired_ = false;
  this->poll_timing_();
  ESP_LOGI(TAG, "experiment: %s", line.c_str());
  if (!sweep) {
    if (this->experiment_result_text_sensor_) this->experiment_result_text_sensor_->publish_state(line);
    return;
  }
  this->sweep_.record(code);
  char p[40];
  snprintf(p, sizeof(p), "sweep %u/%u: gap %.0f %s", (unsigned) this->sweep_.completed(), (unsigned) this->sweep_.total(), this->exp_try_gap_ms_, code);
  if (this->experiment_result_text_sensor_) this->experiment_result_text_sensor_->publish_state(p);
  if (this->sweep_.done()) {
    this->exp_sweep_on_ = false;
    const std::string t = this->sweep_.table();
    ESP_LOGI(TAG, "experiment sweep done (%s): %s", boiler::SweepPlan::legend(), t.c_str());
    if (this->experiment_result_text_sensor_) this->experiment_result_text_sensor_->publish_state(t);
  }
}

void BoilerBus::on_experiment_result_(const QItem &it) {
  boiler::ExpEvent ev[boiler::EXP_MAX_EVENTS];
  const size_t n = std::min<size_t>(it.found, boiler::EXP_MAX_EVENTS);
  memcpy(ev, it.info, n * sizeof(boiler::ExpEvent));
  const boiler::ExpOutcome o = boiler::summarize_experiment(it.expected / 1000.0f, EXP_REG, ev, n);
  this->exp_finish_try_(o.code.c_str(), o.line);
}

void BoilerBus::experiment_send() {
  if (this->exp_sweep_on_) {
    ESP_LOGW(TAG, "experiment send refused: sweep running");
    return;
  }
  this->exp_begin_try_(millis(), this->exp_gap_ms_, false);
}

void BoilerBus::experiment_sweep(bool on) {
  if (!on) {
    if (this->exp_sweep_on_) ESP_LOGI(TAG, "experiment sweep stopped: %s", this->sweep_.table().c_str());
    this->exp_sweep_on_ = false;
    this->exp_cancel_try_("sweep stopped");
    return;
  }
  if (this->exp_sweep_on_ || this->exp_try_open_) return;
  if (!this->poller_.display_present(millis())) {
    ESP_LOGW(TAG, "experiment sweep refused: display not connected");
    if (this->experiment_result_text_sensor_) this->experiment_result_text_sensor_->publish_state("refused: display not connected");
    return;
  }
  this->sweep_ = boiler::SweepPlan();
  this->exp_sweep_on_ = true;
  ESP_LOGI(TAG, "experiment sweep started: gaps 5..40 ms, 3 tries each");
}

void BoilerBus::exp_tick_(uint32_t now) {
  if (this->exp_try_open_) {
    if (!this->poller_.display_present(now)) {
      this->exp_cancel_try_("display lost");
    } else if (!this->exp_fired_ && now - this->exp_try_ms_ > EXP_ARM_TIMEOUT_MS) {
      this->exp_cancel_try_("no usable gap within 1.5 s");
    } else if (this->exp_fired_ && now - this->exp_fired_ms_ > 1500) {
      this->exp_finish_try_("N--", "no result from the sampler");  // should not happen
    }
  }
  if (!this->exp_sweep_on_ || this->exp_try_open_) return;
  if (!this->poller_.display_present(now)) {
    ESP_LOGW(TAG, "experiment sweep aborted: display lost. Partial: %s", this->sweep_.table().c_str());
    if (this->experiment_result_text_sensor_)
      this->experiment_result_text_sensor_->publish_state("sweep aborted (display lost): " + this->sweep_.table());
    this->exp_sweep_on_ = false;
    return;
  }
  if (this->exp_ever_ && now - this->exp_last_ms_ < EXP_SWEEP_SPACING_MS) return;
  this->exp_begin_try_(now, (float) this->sweep_.current_gap_ms(), true);
}

void BoilerBus::on_shutdown() {
  this->tx_req_word_.store(0);
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
  this->pub_(this->tx_bursts_sensor_, (float) this->tx_count_.load(), now);
  this->pub_(this->malformed_short_sensor_, (float) this->malformed_short_.load(), now);
  this->pub_(this->malformed_long_sensor_, (float) this->malformed_long_.load(), now);
  this->pub_(this->rejected_dips_sensor_, (float) this->rejected_dips_.load(), now);
  const float wd = this->width_samples_.load();
  if (wd > 0) this->pub_(this->dip_width_sensor_, wd * this->us_per_sample_, now);
  const float zp = this->zero_period_us_.load();
  if (zp > 0) this->pub_(this->bit_period_sensor_, zp, now);
  if (this->any_frame_) this->pub_(this->last_frame_age_sensor_, (now - this->last_frame_ms_) / 1000.0f, now);
}

void BoilerBus::dump_config() {
  ESP_LOGCONFIG(TAG, "Boiler bus (display-link decoder and poller):");
  ESP_LOGCONFIG(TAG, "  ADC pin: GPIO%u, %u samples/s, ~%.3f mV/count", this->adc_pin_, (unsigned) this->sample_rate_, this->mv_per_count_);
  ESP_LOGCONFIG(TAG, "  TX pin: GPIO%u (LOW unless a request is sent; RMT %s; dip %u us, HIGH <= %u us per dip, <= %u us total)", this->tx_pin_,
                this->tx_ok_ ? "ready" : "FAILED", (unsigned) this->tx_dip_us_, (unsigned) boiler::TX_DIP_US_MAX, (unsigned) boiler::TX_MAX_TOTAL_US);
  ESP_LOGCONFIG(TAG, "  Poller: ping slot %u ms after a board frame, fast gap %u ms, min interval %u ms", (unsigned) (this->ping_delay_us_ / 1000),
                (unsigned) (this->fast_gap_us_ / 1000), (unsigned) (this->tx_min_spacing_us_.load() / 1000));
  ESP_LOGCONFIG(TAG, "  Dip depth %.0f mV, threshold %.0f mV (%s)", this->dip_depth_mv_, this->dip_threshold_mv_,
                this->auto_threshold_ ? "auto-tracks depth" : "fixed");
  ESP_LOGCONFIG(TAG, "  High word of 32-bit counters assumed 0 when unseen: %s", YESNO(this->assume_high_zero_));
  if (this->total_restored_) {
    ESP_LOGCONFIG(TAG, "  Restored propane total: %.3f gal", this->propane_total_);
  }
}

}  // namespace boiler_bus
}  // namespace esphome
