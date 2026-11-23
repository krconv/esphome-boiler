#include "boiler_core.h"

#include <algorithm>

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
  // in a dip: stay while below the (hysteresis) exit level
  if (x < baseline_ - 0.5f * thr) {
    dip_len_++;
    if (x < dip_sum_) dip_sum_ = x;
    if (dip_len_ > cfg_.max_width) {  // the bus level moved, not a data dip
      in_dip_ = false;
      baseline_ = x;
      rejected_++;
    }
    return false;
  }
  in_dip_ = false;
  holdoff_ = 2;  // the rising edge samples must not drag the baseline
  if (dip_len_ < cfg_.min_width) {
    rejected_++;
    return false;
  }
  const float d = baseline_ - dip_sum_;
  if (cfg_.auto_depth && d > 0.5f * cfg_.nominal_depth && d < 2.0f * cfg_.nominal_depth) depth_ += (d - depth_) / 32.0f;
  dips_++;
  if (dip) {
    dip->start = dip_start_;
    dip->width = dip_len_;
  }
  return true;
}

// ---- BurstDecoder -------------------------------------------------------------
void BurstDecoder::finish(BurstResult *res) {
  if (res) {
    res->start_sample = first_;
    res->nbits = bits_.size();
    res->status = dips_in_burst_ < MIN_BURST_DIPS ? ParseStatus::NOISE : parse_bits(bits_.data(), bits_.size(), &res->frame);
  }
  bits_.clear();
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
  } else {
    const float period = float(idx - last_) * us_per_sample_;
    const bool bit = period >= ZERO_MAX_US;
    bits_.push_back(bit);
    dips_in_burst_++;
    if (!bit && period > 100.0f) zero_period_us_ += (period - zero_period_us_) * (zero_period_us_ > 0 ? 1.0f / 64 : 1.0f);
    if (bit) one_period_us_ += (period - one_period_us_) * (one_period_us_ > 0 ? 1.0f / 64 : 1.0f);
  }
  have_last_ = true;
  last_ = idx;
  return finished;
}

bool BurstDecoder::on_idle(uint64_t idx, BurstResult *res) {
  if (have_last_ && float(idx - last_) * us_per_sample_ > ONE_MAX_US) {
    finish(res);
    return true;
  }
  return false;
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

}  // namespace boiler
