#include "boiler_bus.h"

#include <algorithm>
#include <cstdarg>
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
static constexpr uint32_t FAST_RETRY_US = 200000;     // joined: quiet time after our own unanswered request before we try again
static constexpr uint32_t FAST_WINDOW_US = 1000000;  // joined: fire any time up to 1 s after the board's reply
static constexpr uint32_t TX_PROBE_TAIL_US = 6000;        // sample window kept after a test pattern
static constexpr uint32_t TX_PROBE_TAIL_FRAME_US = 2500;  // ... after a frame (next display request is >= 44 ms after the reply)
static constexpr float TX_PROBE_MIN_DEPTH = 40.0f;        // counts (~32 mV): a dip must go this far below baseline
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
  if (this->last_write_text_sensor_) this->last_write_text_sensor_->publish_state("none since boot");

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
  // (a burst that started inside our transmit window is ours even if the window closed before the burst was finalised)
  const bool own = br.start_sample >= this->tx_fire_idx_ && (this->tx_active_ || br.start_sample < this->tx_win_end_);
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
  // joined: after our own request that drew no usable reply (none at all, or one the sampler lost bits of, e.g. while a 1-Wire
  // read held off interrupts) nothing else starts a burst (the board only talks when asked, apart from its idle frame every
  // ~3 s), so waiting for a board frame would stall polling until that idle frame and force a rejoin. The board answers
  // within ~60 ms; once the bus has been quiet for FAST_RETRY_US it is ours to try again.
  const bool retry = this->tx_fast_.load() && !this->last_burst_board_ && idle_us >= (float) FAST_RETRY_US;
  if (!retry && (!this->last_burst_board_ || idle_us < (float) this->tx_req_min_us_.load())) return;
  if (idle_us > (float) this->tx_req_max_us_.load()) return;
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
      default: this->on_tx_event_(it); break;
    }
  }
  const uint32_t now = millis();
  this->regchg_publish_(now);
  this->poll_tick_(now);
  if (now - this->last_tick_ms_ >= 1000) {
    this->tick_(now);
    this->last_tick_ms_ = now;
  }
  if (now - this->last_diag_ms_ >= DIAG_MS) {
    this->publish_diagnostics_(now);
    this->last_diag_ms_ = now;
  }
}

// Fast-changing measurements, counters and status: not worth a line when they change. Everything else (setpoints,
// unmapped / installer registers, error history) is reported. See register_map.md.
static bool regchg_skip_(uint16_t r) {
  switch (r) {
    case 0x0010:                                                          // status bit-field
    case 0x0020: case 0x0021: case 0x0022: case 0x0023: case 0x0024:      // temperatures
    case 0x0130:                                                          // temperature
    case 0x0027: case 0x0033: case 0x0132: case 0x0137:                   // flow, fan rpm, pressure, flame sensor
    case 0x0091: case 0x0092: case 0x0093: case 0x0094: case 0x0095: case 0x0096: case 0x0098: case 0x0099:  // hour / ignition counters
    case 0x0133: case 0x0134: case 0x0135: case 0x0136:
      return true;
    default:
      return false;
  }
}

static constexpr size_t REGCHG_MAX_QUEUE = 48;
static constexpr uint32_t REGCHG_INTERVAL_MS = 500;  // at most 2 published lines per second

void BoilerBus::regchg_event_(bool is_write, const char *fmt, ...) {
  RegEvent e;
  e.is_write = is_write;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(e.text, sizeof(e.text), fmt, ap);
  va_end(ap);
  ESP_LOGI("regchg", "%s", e.text);
  if (this->regchg_q_.size() >= REGCHG_MAX_QUEUE) {
    // overflow: drop the oldest change event; writes only go if nothing else is left
    auto victim = std::find_if(this->regchg_q_.begin(), this->regchg_q_.end(), [](const RegEvent &x) { return !x.is_write; });
    if (victim == this->regchg_q_.end()) victim = this->regchg_q_.begin();
    this->regchg_q_.erase(victim);
  }
  this->regchg_q_.push_back(e);
}

void BoilerBus::regchg_on_frame_(const boiler::Frame &f) {
  if (!this->register_changes_text_sensor_ && !this->registers_seen_text_sensor_) return;
  if (!boiler::is_known_register(f.reg) && this->regs_seen_.insert(f.reg).second) this->regs_seen_dirty_ = true;
  if (f.sub == 0x10) {  // write request (display) or its acknowledge / echo (board)
    if (f.reg == 0x004C || f.reg == 0x004D || f.reg == 0x004E) {  // rewritten constantly: only a changed value is news
      auto w = this->regchg_wlast_.find(f.reg);
      if (w != this->regchg_wlast_.end() && w->second == f.val) return;
      this->regchg_wlast_[f.reg] = f.val;
    }
    this->regchg_event_(true, "W %04X %04X (%s)", f.reg, f.val, f.from_display() ? "disp" : "brd");
    return;
  }
  if (!(f.sub == 0x11 && f.is_response() && f.from_board()) || regchg_skip_(f.reg)) return;
  auto it = this->regchg_last_.find(f.reg);
  if (it == this->regchg_last_.end()) {
    this->regchg_last_[f.reg] = f.val;
    this->regchg_event_(false, "N %04X %04X", f.reg, f.val);
  } else if (it->second != f.val) {
    const uint16_t old = it->second;
    it->second = f.val;
    this->regchg_event_(false, "R %04X %04X\xE2\x86\x92%04X", f.reg, old, f.val);
  }
}

void BoilerBus::regchg_publish_(uint32_t now) {
  if (!this->regchg_q_.empty() && now - this->regchg_last_pub_ms_ >= REGCHG_INTERVAL_MS) {
    this->regchg_last_pub_ms_ = now;
    char buf[64];
    snprintf(buf, sizeof(buf), "#%u %s", (unsigned) ++this->regchg_seq_, this->regchg_q_.front().text);
    this->regchg_q_.pop_front();
    if (this->register_changes_text_sensor_) this->register_changes_text_sensor_->publish_state(buf);
  }
  if (this->regs_seen_dirty_ && now - this->regs_seen_last_ms_ >= 1000) {
    this->regs_seen_last_ms_ = now;
    this->regs_seen_dirty_ = false;
    std::string s;
    for (uint16_t r : this->regs_seen_) {
      char b[8];
      snprintf(b, sizeof(b), "%s%04X", s.empty() ? "" : ",", r);
      if (s.size() + strlen(b) > 255) break;
      s += b;
    }
    if (this->registers_seen_text_sensor_) this->registers_seen_text_sensor_->publish_state(s);
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
  if (f.from_display() && f.is_request()) {
    const boiler::PollRequest seen{f.reg, f.val, f.sub};
    const auto ev = this->poller_.on_display_request(it.t_ms, &seen);
    if (ev == boiler::Poller::DispEvidence::CONFIRMED)
      ESP_LOGI(TAG, "display confirmed by repeated requests (last %s %04X)", f.sub == 0x10 ? "WR" : "RD", f.reg);
    else if (ev != boiler::Poller::DispEvidence::PRESENT)
      ESP_LOGD(TAG, "display-addressed %s %04X %s (%u echoes, %u strays so far)", f.sub == 0x10 ? "WR" : "RD", f.reg,
               ev == boiler::Poller::DispEvidence::ECHO ? "is our own echo, ignored" : "alone, not enough to call it a display",
               (unsigned) this->poller_.display_echoes(), (unsigned) this->poller_.display_strays());
  }
  this->regchg_on_frame_(f);
  this->last_frame_ms_ = it.t_ms;
  this->any_frame_ = true;
  if (this->log_frames_)
    ESP_LOGD(TAG, "%s %s reg=%04X val=%04X", f.from_display() ? "disp>" : "boil>", f.is_request() ? (f.sub == 0x10 ? "WR" : "RD") : (f.sub == 0x10 ? "wr-ack" : "rsp"), f.reg, f.val);
  if (f.from_board() && !(f.is_response() && this->poller_.on_board_response(it.t_ms, f))) this->poller_.on_board_unsolicited(it.t_ms, f);
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
    case 0x0131: this->pub_(this->heating_type_preset_sensor_, (float) val, now); break;
    case 0x0097:
      if (this->vent_material_text_sensor_) {
        char b[8];
        snprintf(b, sizeof(b), "%04X", val);
        const std::string t = val == 0 ? "PVC" : b;
        if (t != this->last_vent_) {
          this->last_vent_ = t;
          this->vent_material_text_sensor_->publish_state(t);
        }
      }
      break;
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

float BoilerBus::setpoint_c_(uint16_t reg) const {
  const auto *e = this->table_.get(reg);
  return e ? boiler::decode_setpoint(e->val) : NAN;
}

float BoilerBus::ch_setpoint_c() const { return this->setpoint_c_(boiler::REG_CH_SETPOINT); }
float BoilerBus::dhw_setpoint_c() const { return this->setpoint_c_(boiler::REG_DHW_SETPOINT); }
bool BoilerBus::write_ch_setpoint(float c) { return this->write_setpoint_(boiler::REG_CH_SETPOINT, c); }
bool BoilerBus::write_dhw_setpoint(float c) { return this->write_setpoint_(boiler::REG_DHW_SETPOINT, c); }

bool BoilerBus::write_setpoint_(uint16_t reg, float c) {
  const char *name = boiler::setpoint_name(reg);
  if (!boiler::setpoint_in_range(reg, c)) {
    ESP_LOGW(TAG, "%s setpoint %.1f C refused: outside %.0f..%.0f C", name, c, boiler::setpoint_min_c(reg), boiler::setpoint_max_c(reg));
    this->refuse_write_(reg, "outside the allowed range");
    return false;
  }
  const PendingWrite w{reg, 0xFFFF, boiler::encode_setpoint(reg, c)};
  return this->queue_writes_(&w, 1);  // queued like every other write, so one right behind another waits instead of being refused
}

float BoilerBus::read_field(uint16_t reg, uint16_t mask) const {
  const auto *e = this->table_.get(reg);
  if (!e || !mask) return NAN;
  uint16_t v = e->val & mask;
  while (!(mask & 1)) { mask >>= 1; v >>= 1; }
  return (float) v;
}

void BoilerBus::publish_last_write_(const std::string &s) {
  ESP_LOGI(TAG, "last write: %s", s.c_str());
  if (this->last_write_text_sensor_) this->last_write_text_sensor_->publish_state(s);
}

void BoilerBus::refuse_write_(uint16_t reg, const char *why) {
  ESP_LOGW(TAG, "write %04X refused: %s", reg, why);
  char b[64];
  snprintf(b, sizeof(b), "%04X refused: %s", reg, why);
  this->publish_last_write_(b);
}

const char *BoilerBus::not_master_reason_(uint32_t now) const {
  if (!this->poller_.enabled()) return "bus polling off";
  if (this->poller_.display_present(now)) return "display present";
  return "starting up";
}

bool BoilerBus::queue_writes_(const PendingWrite *w, size_t n) {
  const uint32_t now = millis();
  if (!this->poller_.is_master(now)) {
    this->refuse_write_(w[0].reg, this->not_master_reason_(now));
    return false;
  }
  if (this->pending_writes_.size() + n > 8) {
    this->refuse_write_(w[0].reg, "previous writes still queued");
    return false;
  }
  for (size_t i = 0; i < n; i++) this->pending_writes_.push_back(w[i]);
  ESP_LOGI(TAG, "write queued: %04X mask %04X value %04X (%u pending)", w[0].reg, w[0].mask, w[0].value, (unsigned) this->pending_writes_.size());
  {
    char b[48];
    snprintf(b, sizeof(b), "%04X queued", w[0].reg);
    this->publish_last_write_(b);
  }
  return true;
}

bool BoilerBus::write_field(uint16_t reg, uint16_t mask, uint16_t value) {
  if (!mask) return false;
  uint16_t shift = 0;
  for (uint16_t t = mask; !(t & 1); t >>= 1) shift++;
  const uint16_t v = uint16_t(value << shift);
  if (!boiler::is_writable_reg(reg) || !boiler::write_value_ok(reg, mask, v)) {
    ESP_LOGW(TAG, "write %04X mask %04X value %u", reg, mask, (unsigned) value);
    this->refuse_write_(reg, "value not allowed");
    return false;
  }
  if (mask != 0xFFFF && !this->table_.seen(reg)) {
    this->refuse_write_(reg, "register not read yet");
    return false;
  }
  const PendingWrite w{reg, mask, v};
  return this->queue_writes_(&w, 1);
}

bool BoilerBus::write_temp_(uint16_t reg, float c) {
  const uint16_t v = boiler::encode_temp_c(c);
  if (!std::isfinite(c) || !boiler::write_value_ok(reg, 0xFFFF, v)) {
    this->refuse_write_(reg, "outside the allowed range");
    return false;
  }
  const auto *other = this->table_.get(reg == boiler::REG_CH_MIN_TEMP ? boiler::REG_CH_MAX_TEMP : boiler::REG_CH_MIN_TEMP);
  if (!other) {
    this->refuse_write_(reg, "other limit not read yet");
    return false;
  }
  const bool ok = reg == boiler::REG_CH_MIN_TEMP ? boiler::decode_temp_c(v) < boiler::decode_temp_c(other->val)
                                                 : boiler::decode_temp_c(v) > boiler::decode_temp_c(other->val);
  if (!ok) {
    this->refuse_write_(reg, "minimum must stay below maximum");
    return false;
  }
  const PendingWrite w{reg, 0xFFFF, v};
  return this->queue_writes_(&w, 1);
}

bool BoilerBus::write_min_temp(float c) { return this->write_temp_(boiler::REG_CH_MIN_TEMP, c); }
bool BoilerBus::write_max_temp(float c) { return this->write_temp_(boiler::REG_CH_MAX_TEMP, c); }

bool BoilerBus::clear_error_history() {
  PendingWrite w[5];
  for (uint16_t i = 0; i < 5; i++) w[i] = {uint16_t(boiler::REG_ERROR_HISTORY_FIRST + i), 0xFFFF, 0};
  return this->queue_writes_(w, 5);
}

// one queued write at a time through the poller's own write path (double write + read-back)
void BoilerBus::drain_writes_() {
  for (const auto &r : this->poller_.take_write_results()) this->publish_last_write_(r);
  if (this->pending_writes_.empty() || this->poller_.write_active()) return;
  const PendingWrite w = this->pending_writes_.front();
  this->pending_writes_.pop_front();
  const auto *e = this->table_.get(w.reg);
  uint16_t val = w.value;
  const char *why = nullptr;
  if (w.mask != 0xFFFF) {
    if (!e) why = "register not read yet";
    else val = uint16_t((e->val & ~w.mask) | w.value);
  }
  if (!why) {
    const uint32_t now = millis();
    const auto r = this->poller_.request_write(now, w.reg, val);
    if (r == boiler::WriteResult::NOT_MASTER) why = this->not_master_reason_(now);
    else if (r == boiler::WriteResult::BUSY) why = "previous write still running";
    else if (r != boiler::WriteResult::OK) why = "value out of range";
  }
  if (why) {
    // the rest of a multi-register write (clear history) only makes sense as a whole, so it goes too, but loudly
    char b[80];
    const size_t more = this->pending_writes_.size();
    if (more) snprintf(b, sizeof(b), "%04X dropped: %s (+%u queued dropped)", w.reg, why, (unsigned) more);
    else snprintf(b, sizeof(b), "%04X dropped: %s", w.reg, why);
    ESP_LOGW(TAG, "queued write %04X=%04X: %s", w.reg, val, b);
    this->publish_last_write_(b);
    this->pending_writes_.clear();
  }
}

void BoilerBus::poll_arm_(const boiler::PollRequest &r, bool valid) {
  this->tx_req_word_.store(valid ? (1ull << 63) | ((uint64_t) r.sub << 32) | ((uint64_t) r.reg << 16) | r.val : 0);
}

// ping slot: long quiet time after a board frame, narrow window. joined: the display's short gap, wide window.
void BoilerBus::poll_timing_() {
  const bool fast = this->poller_.mode() == boiler::PollMode::FAST;
  const uint32_t lo = fast ? this->fast_gap_us_ : this->ping_delay_us_;
  this->tx_req_min_us_.store(lo);
  this->tx_fast_.store(fast);
  this->tx_req_max_us_.store(lo + (fast ? FAST_WINDOW_US : PING_WINDOW_US));
}

void BoilerBus::poll_tick_(uint32_t now) {
  if (!this->tx_ok_) return;
  this->poller_.update(now);
  this->drain_writes_();
  const int disp = this->poller_.display_present(now) ? 1 : 0;
  if (disp != this->last_display_state_) {
    this->last_display_state_ = disp;
    if (this->display_connected_binary_sensor_) this->display_connected_binary_sensor_->publish_state(disp);
  }
  for (const auto &e : this->poller_.take_events()) {
    if (e.find("failed") != std::string::npos || e.find("lost") != std::string::npos) ESP_LOGW(TAG, "poll: %s", e.c_str());
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
    boiler::PollRequest r{it.frame.reg, it.frame.val, it.frame.sub};
    this->poller_.on_sent(it.t_ms, r);
    return;
  }
  if (it.kind == Kind::TX_DEPTH) {
    if (this->tx_dip_depth_sensor_) this->tx_dip_depth_sensor_->publish_state(it.depth_mv);
    if (this->tx_dip_depth_bus_sensor_) this->tx_dip_depth_bus_sensor_->publish_state(it.depth_mv / 1000.0f / DIVIDER_RATIO);
  }
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
