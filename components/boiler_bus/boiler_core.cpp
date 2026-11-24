#include "boiler_core.h"

#include <algorithm>
#include <cstdio>

namespace boiler {

uint16_t crc16_xmodem(const uint8_t *data, size_t len, uint16_t crc) {
  for (size_t i = 0; i < len; i++) {
    crc ^= uint16_t(data[i]) << 8;
    for (int k = 0; k < 8; k++)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}

static int hexval(uint8_t c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

static bool hex4(const uint8_t *p, uint16_t *out) {
  uint16_t v = 0;
  for (int i = 0; i < 4; i++) {
    int h = hexval(p[i]);
    if (h < 0) return false;
    v = uint16_t((v << 4) | h);
  }
  *out = v;
  return true;
}

ParseStatus parse_bits(const uint8_t *bits, size_t nbits, Frame *out) {
  if (nbits < MIN_BURST_DIPS - 1) return ParseStatus::NOISE;
  if (nbits != FRAME_BITS) return ParseStatus::BAD_LENGTH;
  uint8_t b[FRAME_BYTES];
  for (size_t i = 0; i < FRAME_BYTES; i++) {
    uint8_t v = 0;
    for (int k = 0; k < 8; k++)
      if (bits[i * 8 + k]) v |= uint8_t(1u << k);  // LSB first
    b[i] = v;
  }
  if (b[0] != '\r' || b[17] != '\n') return ParseStatus::BAD_FORMAT;
  Frame f;
  if (!hex4(b + 5, &f.reg) || !hex4(b + 9, &f.val) || !hex4(b + 13, &f.crc)) return ParseStatus::BAD_FORMAT;
  f.addr1 = b[1];
  f.addr2 = b[2];
  f.type = b[3];
  f.sub = b[4];
  if (crc16_xmodem(b + 1, 12) != f.crc) return ParseStatus::CRC_ERROR;
  if (out) *out = f;
  return ParseStatus::OK;
}

ParseStatus parse_bit_string(const std::string &s, Frame *out) {
  std::vector<uint8_t> bits;
  for (char c : s) bits.push_back(c == '1');
  return parse_bits(bits.data(), bits.size(), out);
}

// ---- DipDetector ------------------------------------------------------------
float DipDetector::threshold_now() const {
  if (cfg_.fixed_threshold > 0) return cfg_.fixed_threshold;
  float d = cfg_.auto_depth && depth_ > 0 ? depth_ : cfg_.nominal_depth;
  float t = cfg_.threshold_frac * d;
  return std::max(cfg_.min_threshold, std::min(t, cfg_.threshold_frac * cfg_.nominal_depth * 1.5f));
}

void DipDetector::reset_state() {
  in_dip_ = false;
  above_run_ = 0;
  half_count_ = 0;
  init_ = false;
  holdoff_ = 0;
}

bool DipDetector::push(uint16_t xi, Dip *dip) {
  const float x = float(xi);
  const uint64_t idx = n_++;
  if (!init_) {
    baseline_ = x;
    if (depth_ <= 0) depth_ = cfg_.nominal_depth;
    init_ = true;
    return false;
  }
  const float thr = threshold_now();
  if (!in_dip_) {
    if (x < baseline_ - thr) {
      in_dip_ = true;
      dip_start_ = idx;
      dip_len_ = 1;
      dip_sum_ = x;  // here: running minimum
      half_count_ = 1;
      above_run_ = 0;
      last_low_len_ = 1;
      return false;
    }
    if (holdoff_) {
      holdoff_--;
    } else {
      float a = idx < cfg_.warmup_samples ? cfg_.baseline_alpha_fast : cfg_.baseline_alpha;
      baseline_ += a * (x - baseline_);
    }
    return false;
  }
  // in a dip: stay while below the (hysteresis) exit level. The ADC sometimes reads the idle level for a sample
  // or two in the middle of a dip, so a dip only ends after exit_samples consecutive samples above the exit level;
  // the dip start (which is all the bit timing uses) is unaffected.
  if (x < baseline_ - 0.5f * thr) {
    dip_len_ += above_run_ + 1;  // a notch counts as part of the dip
    above_run_ = 0;
    last_low_len_ = dip_len_;
    if (x < baseline_ - thr) half_count_++;
    if (x < dip_sum_) dip_sum_ = x;
    if (dip_len_ > cfg_.max_width) {  // the bus level moved, not a data dip
      in_dip_ = false;
      baseline_ = x;
      rejected_++;
    }
    return false;
  }
  if (++above_run_ < cfg_.exit_samples) return false;
  in_dip_ = false;
  above_run_ = 0;
  holdoff_ = 0;
  dip_len_ = last_low_len_;
  const float d = baseline_ - dip_sum_;
  // a 1-sample dip is normally noise, but a full-depth one is a real dip whose other samples read idle (ADC dropout)
  if (dip_len_ < cfg_.min_width && !(d >= 0.8f * (depth_ > 0 ? depth_ : cfg_.nominal_depth))) {
    rejected_++;
    return false;
  }
  if (learn_) {
    if (cfg_.auto_depth && d > 0.5f * cfg_.nominal_depth && d < 2.0f * cfg_.nominal_depth) depth_ += (d - depth_) / 32.0f;
    width_ += (float(half_count_) - width_) * (width_ > 0 ? 1.0f / 256 : 1.0f);
  }
  dips_++;
  if (dip) {
    dip->start = dip_start_;
    dip->width = dip_len_;
    dip->depth = d;
  }
  return true;
}

// ---- BurstDecoder -------------------------------------------------------------
void BurstDecoder::finish(BurstResult *res) {
  if (res) {
    res->start_sample = first_;
    res->nbits = bits_.size();
    res->ndips = dips_in_burst_;
    {
      const size_t n = std::min(bits_.size(), sizeof(res->bits) - 1);
      for (size_t i = 0; i < n; i++) res->bits[i] = bits_[i] ? '1' : '0';
      res->bits[n] = 0;
    }
    for (size_t i = 0; i < MAX_TRACK_DIPS; i++) res->dip_off[i] = i < offs_.size() ? offs_[i] : 0;
    res->gap_samples = have_prev_ ? first_ - prev_burst_end_ : 0;
    res->status = dips_in_burst_ < MIN_BURST_DIPS ? ParseStatus::NOISE : parse_bits(bits_.data(), bits_.size(), &res->frame);
  }
  prev_burst_end_ = last_;
  have_prev_ = true;
  bits_.clear();
  offs_.clear();
  have_last_ = false;
  dips_in_burst_ = 0;
}

bool BurstDecoder::on_dip(uint64_t idx, BurstResult *res) {
  bool finished = false;
  if (have_last_ && float(idx - last_) * us_per_sample_ > ONE_MAX_US) {
    finish(res);
    finished = true;
  }
  if (!have_last_) {
    first_ = idx;
    dips_in_burst_ = 1;
    bits_.clear();
    offs_.clear();
  } else {
    const float period = float(idx - last_) * us_per_sample_;
    const bool bit = period >= ZERO_MAX_US;
    bits_.push_back(bit);
    dips_in_burst_++;
    if (!bit && period > 100.0f) zero_period_us_ += (period - zero_period_us_) * (zero_period_us_ > 0 ? 1.0f / 64 : 1.0f);
    if (bit) one_period_us_ += (period - one_period_us_) * (one_period_us_ > 0 ? 1.0f / 64 : 1.0f);
  }
  if (offs_.size() < MAX_TRACK_DIPS) offs_.push_back((uint16_t) std::min<uint64_t>(idx - first_, 65535));
  have_last_ = true;
  last_ = idx;
  last_dip_ = idx;
  seen_dip_ = true;
  return finished;
}

bool BurstDecoder::on_idle(uint64_t idx, BurstResult *res) {
  if (have_last_ && float(idx - last_) * us_per_sample_ > ONE_MAX_US) {
    finish(res);
    return true;
  }
  return false;
}

int first_bad_bit(const char *bits) {
  const size_t n = std::strlen(bits);
  for (size_t i = 0; i < FRAME_BYTES; i++) {
    if ((i + 1) * 8 > n) return int(i * 8);
    uint8_t v = 0;
    for (int k = 0; k < 8; k++)
      if (bits[i * 8 + k] == '1') v |= uint8_t(1u << k);
    bool ok;
    if (i == 0) ok = v == '\r';
    else if (i == 1) ok = v == 0x51 || v == 0xA1;
    else if (i == 2) ok = v == 0x51 || v == 0xA1;
    else if (i == 3) ok = v == 0x10 || v == 0x11;
    else if (i == 4) ok = v == 0x10 || v == 0x11;
    else if (i < 17) ok = (v >= '0' && v <= '9') || (v >= 'A' && v <= 'F');
    else ok = v == '\n';
    if (!ok) return int(i * 8);
  }
  return -1;
}

void render_trace(const uint16_t *ring, uint32_t mask, uint64_t from, size_t n, float baseline, float depth, char *out) {
  static const char H[] = "0123456789ABCDEF";
  const float step = std::max(depth, 20.0f) / 10.0f;
  for (size_t i = 0; i < n; i++) {
    int v = int((baseline - float(ring[(from + i) & mask])) / step + 0.5f);
    out[i] = H[v < 0 ? 0 : (v > 15 ? 15 : v)];
  }
  out[n] = 0;
}

void summarize_burst(const uint16_t *ring, uint32_t mask, uint64_t start, const uint16_t *dip_off, size_t ndips,
                     float mv_per_count, float us_per_sample, const char *tag, char *out, size_t cap) {
  auto at = [&](int64_t i) { return float(ring[uint64_t(int64_t(start) + i) & mask]); };
  if (ndips < 3 || start < 60) { snprintf(out, cap, "%s dips=%u (too few)", tag, (unsigned) ndips); return; }
  float base = 0;
  for (int i = -40; i < -6; i++) base += at(i);
  base /= 34.0f;
  const size_t n = std::min<size_t>(ndips, MAX_TRACK_DIPS);
  double w_sum = 0, d_sum = 0, d_min = 1e9, d_max = 0, p0_sum = 0, p1_sum = 0, p0_min = 1e9, p0_max = 0, p1_min = 1e9, p1_max = 0;
  double r0_sum = 0, r1_sum = 0;
  int w_n = 0, p0_n = 0, p1_n = 0, r0_n = 0, r1_n = 0;
  float over = 0;
  for (size_t k = 0; k < n; k++) {
    const int64_t s = int64_t(dip_off[k]);
    float mn = 1e9f, mn2 = 1e9f;  // two lowest samples (one stray low sample must not set the depth)
    for (int j = -1; j <= 12; j++) {
      const float v = at(s + j);
      if (v < mn) { mn2 = mn; mn = v; } else if (v < mn2) { mn2 = v; }
    }
    const float depth = base - (mn + mn2) / 2;
    d_sum += depth; d_min = std::min<double>(d_min, depth); d_max = std::max<double>(d_max, depth);
    const float half = base - depth / 2;
    int first = -100, last = -100;
    for (int j = -2; j < 14; j++)
      if (at(s + j) < half) { if (first == -100) first = j; last = j; }
    if (first != -100) { w_sum += last - first + 1; w_n++; }
    for (int j = -2; j < 20; j++) over = std::max(over, at(s + j) - base);
    if (k + 1 < n) {
      const int64_t per = int64_t(dip_off[k + 1]) - s;
      const double pu = per * us_per_sample;
      if (pu < ZERO_MAX_US) {
        p0_sum += pu; p0_n++; p0_min = std::min(p0_min, pu); p0_max = std::max(p0_max, pu);
        if (per >= 12) { for (int j = 8; j < per - 1 && j < 14; j++) { r0_sum += base - at(s + j); r0_n++; } }
      } else if (pu < ONE_MAX_US) {
        p1_sum += pu; p1_n++; p1_min = std::min(p1_min, pu); p1_max = std::max(p1_max, pu);
        for (int j = 8; j < per - 1 && j < 28; j++) { r1_sum += base - at(s + j); r1_n++; }
      }
    }
  }
  auto avg = [](double s, int c) { return c ? s / c : 0.0; };
  snprintf(out, cap, "%s dips=%u w=%.0fus dep=%.0f(%.0f-%.0f)mV p0=%.0f(%.0f-%.0f) p1=%.0f(%.0f-%.0f) r0=%.0f r1=%.0f ov=%.0f",
           tag, (unsigned) ndips, avg(w_sum, w_n) * us_per_sample, avg(d_sum, int(n)) * mv_per_count, d_min * mv_per_count, d_max * mv_per_count,
           avg(p0_sum, p0_n), p0_n ? p0_min : 0.0, p0_n ? p0_max : 0.0, avg(p1_sum, p1_n), p1_n ? p1_min : 0.0, p1_n ? p1_max : 0.0,
           avg(r0_sum, r0_n) * mv_per_count, avg(r1_sum, r1_n) * mv_per_count, over * mv_per_count);
}

void render_raw(const uint16_t *ring, uint32_t mask, uint64_t from, size_t n, char *out) {
  static const char H[] = "0123456789ABCDEF";
  for (size_t i = 0; i < n; i++) {
    const uint16_t v = ring[(from + i) & mask] & 0xFFF;
    out[3 * i] = H[v >> 8];
    out[3 * i + 1] = H[(v >> 4) & 15];
    out[3 * i + 2] = H[v & 15];
  }
  out[3 * n] = 0;
}

// ---- pairing / table ------------------------------------------------------------
bool Pairer::on_frame(const Frame &f, uint16_t *reg, uint32_t *unpaired_counter) {
  if (f.from_display() && f.is_request()) {
    pending_ = true;
    pending_reg_ = f.reg;
    pending_sub_ = f.sub;
    return false;
  }
  if (f.from_board() && f.is_response()) {
    if (!(pending_ && pending_reg_ == f.reg && pending_sub_ == f.sub) && unpaired_counter) (*unpaired_counter)++;
    pending_ = false;
    *reg = f.reg;  // the response carries its own register; it is authoritative
    return true;
  }
  return false;
}

void RegisterTable::set(uint16_t reg, uint16_t val, uint32_t now_ms) {
  RegEntry &e = regs_[reg];
  e.val = val;
  e.t_ms = now_ms;
  e.count++;
}

const RegEntry *RegisterTable::get(uint16_t reg) const {
  auto it = regs_.find(reg);
  return it == regs_.end() ? nullptr : &it->second;
}

uint32_t RegisterTable::age_ms(uint16_t reg, uint32_t now_ms) const {
  const RegEntry *e = get(reg);
  return e ? now_ms - e->t_ms : UINT32_MAX;
}

bool is_known_register(uint16_t r) {
  switch (r) {
    case 0x0010: case 0x0020: case 0x0130: case 0x0021: case 0x0022: case 0x0023: case 0x0024:
    case 0x0101: case 0x0102:
    case 0x0091: case 0x0092: case 0x0093: case 0x0094: case 0x0095: case 0x0096: case 0x0098: case 0x0099:
    case 0x0133: case 0x0134: case 0x0135: case 0x0136:
    case 0x0027: case 0x0033: case 0x0132: case 0x0137:
    case 0x0142: case 0x0143: case 0x0144: case 0x0145: case 0x0146:
      return true;
    default:
      return false;
  }
}

const char *state_text(uint16_t s) {
  const bool dhw = s & ST_DHW_DEMAND;
  if (s & ST_FLAME) return dhw ? "hot water" : "heating";
  if (s & (ST_SPARK | ST_CH_DEMAND)) return "igniting";
  if (dhw) return "hot water";
  if (s & ST_PUMP) return "pump";
  return "idle";
}

std::string format_error_history(const uint16_t *vals, const bool *seen) {
  std::string out;
  for (int i = 0; i < 5; i++) {
    for (int half = 0; half < 2; half++) {
      if (!out.empty()) out += ',';
      if (!seen[i]) {
        out += '?';
      } else {
        unsigned code = half == 0 ? (vals[i] >> 8) : (vals[i] & 0xFF);
        out += std::to_string(code);
      }
    }
  }
  return out;
}

float gas_btu_per_hour(float rpm) {
  float r = std::max(GAS_RPM_LO, std::min(rpm, GAS_RPM_HI));
  return GAS_BTU_LO + (r - GAS_RPM_LO) * (GAS_BTU_HI - GAS_BTU_LO) / (GAS_RPM_HI - GAS_RPM_LO);
}

// ---- transmit helpers ---------------------------------------------------------------
uint32_t clamp_tx_dip_us(float us) {
  if (!std::isfinite(us)) return TX_DIP_US_DEFAULT;
  if (us < float(TX_DIP_US_MIN)) return TX_DIP_US_MIN;
  if (us > float(TX_DIP_US_MAX)) return TX_DIP_US_MAX;
  return uint32_t(us + 0.5f);
}

std::vector<TxPulse> tx_test_pattern(uint32_t n, uint32_t dip_us) {
  dip_us = clamp_tx_dip_us(float(dip_us));
  if (n < 1) n = 1;
  if (n > 32) n = 32;
  std::vector<TxPulse> p;
  for (uint32_t i = 0; i < n; i++)
    p.push_back({uint16_t(dip_us), uint16_t(i + 1 < n ? TX_BIT0_PERIOD_US - dip_us : TX_TAIL_LOW_US)});
  return p;
}

static void put_hex4(uint8_t *p, uint16_t v) {
  static const char H[] = "0123456789ABCDEF";
  for (int i = 0; i < 4; i++) p[i] = (uint8_t) H[(v >> (12 - 4 * i)) & 0xF];
}

void encode_frame(const Frame &f, uint8_t out[FRAME_BYTES]) {
  out[0] = '\r';
  out[1] = f.addr1;
  out[2] = f.addr2;
  out[3] = f.type;
  out[4] = f.sub;
  put_hex4(out + 5, f.reg);
  put_hex4(out + 9, f.val);
  put_hex4(out + 13, crc16_xmodem(out + 1, 12));
  out[17] = '\n';
}

Frame make_read_request(uint16_t reg) {
  Frame f;
  f.addr1 = 0xA1;
  f.addr2 = 0x51;
  f.type = 0x10;
  f.sub = 0x11;
  f.reg = reg;
  f.val = 0;
  return f;
}

Frame make_write_request(uint16_t reg, uint16_t val) {
  Frame f = make_read_request(reg);
  f.sub = 0x10;
  f.val = val;
  return f;
}

bool is_setpoint_reg(uint16_t reg) { return reg == REG_CH_SETPOINT || reg == REG_DHW_SETPOINT; }
const char *setpoint_name(uint16_t reg) { return reg == REG_DHW_SETPOINT ? "DHW" : "CH"; }
float setpoint_min_c(uint16_t reg) { return reg == REG_DHW_SETPOINT ? DHW_SETPOINT_MIN_C : CH_SETPOINT_MIN_C; }
float setpoint_max_c(uint16_t reg) { return reg == REG_DHW_SETPOINT ? DHW_SETPOINT_MAX_C : CH_SETPOINT_MAX_C; }

bool setpoint_in_range(uint16_t reg, float c) {
  return std::isfinite(c) && c >= setpoint_min_c(reg) && c <= setpoint_max_c(reg);
}

uint16_t encode_setpoint(uint16_t reg, float c) {
  const float lo = setpoint_min_c(reg), hi = setpoint_max_c(reg);
  if (!std::isfinite(c) || c < lo) c = lo;
  if (c > hi) c = hi;
  const int half = (int) std::floor(c * 2.0f + 0.5f);  // half degrees
  return uint16_t(((half / 2) << 8) | ((half & 1) ? 0x05 : 0x00));
}

float decode_setpoint(uint16_t v) { return decode_temp_c(v); }

std::vector<TxPulse> tx_frame_pulses(const uint8_t bytes[FRAME_BYTES], uint32_t dip_us) {
  dip_us = clamp_tx_dip_us(float(dip_us));
  std::vector<TxPulse> p;
  // dip 0 starts the burst; dip k+1 follows bit k
  for (size_t i = 0; i < FRAME_BITS; i++) {
    const bool bit = (bytes[i / 8] >> (i % 8)) & 1;
    p.push_back({uint16_t(dip_us), uint16_t((bit ? TX_BIT1_PERIOD_US : TX_BIT0_PERIOD_US) - dip_us)});
  }
  p.push_back({uint16_t(dip_us), uint16_t(TX_TAIL_LOW_US)});
  return p;
}

uint32_t tx_pulses_total_us(const std::vector<TxPulse> &p) {
  uint32_t t = 0;
  for (auto &x : p) t += x.high_us + x.low_us;
  return t;
}

bool tx_pulses_safe(const std::vector<TxPulse> &p) {
  if (p.empty()) return false;
  for (auto &x : p)
    if (x.high_us == 0 || x.high_us > TX_MAX_HIGH_US || x.low_us < TX_MIN_LOW_US) return false;
  return tx_pulses_total_us(p) <= TX_MAX_TOTAL_US;
}

bool parse_hex_reg(const std::string &s, uint16_t *out) {
  if (s.size() != 4) return false;
  uint16_t v = 0;
  for (char c : s) {
    int h = hexval((uint8_t) c);
    if (h < 0) return false;
    v = uint16_t((v << 4) | h);
  }
  *out = v;
  return true;
}

}  // namespace boiler
