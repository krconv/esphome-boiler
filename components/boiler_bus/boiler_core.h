// Plain C++ core of the boiler display-bus decoder. No ESPHome / ESP-IDF dependency,
// so it is unit-tested on the host (esphome/test/test_core.cpp).
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace boiler {

// ---- protocol constants ---------------------------------------------------
constexpr float ZERO_MAX_US = 230.0f;   // dip-to-dip period below this = bit 0 (nominal 160)
constexpr float ONE_MAX_US = 450.0f;    // below this = bit 1 (nominal 320); longer = end of burst
constexpr size_t FRAME_BITS = 144;      // 18 bytes
constexpr size_t FRAME_BYTES = 18;
constexpr size_t MIN_BURST_DIPS = 8;    // fewer dips than this = noise, ignored (as boiler_sniff.py)

uint16_t crc16_xmodem(const uint8_t *data, size_t len, uint16_t crc = 0);

// ---- frames ---------------------------------------------------------------
struct Frame {
  uint8_t addr1{0}, addr2{0};  // A1 51 = display->board, 51 A1 = board->display
  uint8_t type{0};             // 0x10 request, 0x11 response
  uint8_t sub{0};              // 0x11 read, 0x10 write
  uint16_t reg{0};
  uint16_t val{0};
  uint16_t crc{0};
  bool is_request() const { return type == 0x10; }
  bool is_response() const { return type == 0x11; }
  bool from_board() const { return addr1 == 0x51 && addr2 == 0xA1; }
  bool from_display() const { return addr1 == 0xA1 && addr2 == 0x51; }
};

enum class ParseStatus : uint8_t {
  OK,
  CRC_ERROR,   // structurally fine, CRC mismatch
  BAD_FORMAT,  // wrong framing bytes or non-hex characters
  BAD_LENGTH,  // not 144 bits
  NOISE,       // < MIN_BURST_DIPS dips
};

// bits[i] is 0/1 in transmission order. Each byte is LSB first.
ParseStatus parse_bits(const uint8_t *bits, size_t nbits, Frame *out);
ParseStatus parse_bit_string(const std::string &bits, Frame *out);  // "0101..." convenience for tests

// ---- dip detector ---------------------------------------------------------
// Input: ADC samples (raw counts) at a fixed rate. Output: completed dips (start sample index).
struct DipDetectorConfig {
  float nominal_depth{210.0f};  // expected dip depth in counts (0.17 V divider-side ~ 210 counts at 0.8 mV/count)
  float threshold_frac{0.5f};   // dip when below baseline - frac*depth
  float min_threshold{20.0f};   // floor for the threshold (counts)
  float fixed_threshold{0.0f};  // > 0: use this threshold in counts and disable auto tracking
  bool auto_depth{true};        // track dip depth and set threshold = frac * depth
  uint32_t min_width{2};        // dips narrower than this many samples are noise
  uint32_t max_width{300};      // wider than this = bus level step, re-learn baseline
  float baseline_alpha_fast{1.0f / 64};
  float baseline_alpha{1.0f / 4096};
  uint32_t warmup_samples{3000};
};

struct Dip {
  uint64_t start;  // sample index of the first sample below threshold
  uint32_t width;  // samples
  float depth;     // counts: baseline minus the lowest sample
};

class DipDetector {
 public:
  explicit DipDetector(const DipDetectorConfig &cfg = DipDetectorConfig()) : cfg_(cfg) {}
  void set_config(const DipDetectorConfig &cfg) { cfg_ = cfg; }
  // Feed one sample. Returns true and fills *dip when a dip has just completed.
  bool push(uint16_t x, Dip *dip);
  uint64_t index() const { return n_; }
  float baseline() const { return baseline_; }
  float depth() const { return depth_; }          // tracked dip depth (counts)
  float threshold() const { return threshold_now(); }
  uint64_t dips() const { return dips_; }
  uint64_t rejected() const { return rejected_; }  // too narrow / too wide
  bool in_dip() const { return in_dip_; }
  void reset_state();                              // after sample loss
  // false: dips still come out, but do not move the learned depth / width (used while we transmit ourselves)
  void set_learn(bool v) { learn_ = v; }
  // EMA of the dip width in samples, measured at half depth (0 until a dip was seen)
  float width() const { return width_; }

 private:
  float threshold_now() const;
  DipDetectorConfig cfg_;
  uint64_t n_{0};
  float baseline_{0};
  float depth_{0};
  bool init_{false};
  bool in_dip_{false};
  uint64_t dip_start_{0};
  uint32_t dip_len_{0};
  float dip_sum_{0};
  uint32_t holdoff_{0};
  uint64_t dips_{0}, rejected_{0};
  bool learn_{true};
  uint32_t half_count_{0};  // samples of the current dip below baseline - 1 x threshold (~half depth)
  float width_{0};
};

// ---- dips -> bursts -> frames ---------------------------------------------
constexpr size_t MAX_TRACK_DIPS = 160;

struct BurstResult {
  ParseStatus status;
  Frame frame;
  size_t nbits;
  uint64_t start_sample;
  size_t ndips;            // dips in the burst
  uint64_t gap_samples;    // quiet time between the previous burst's last dip and this burst's first dip
  uint16_t dip_off[MAX_TRACK_DIPS];  // dip start sample minus start_sample, first MAX_TRACK_DIPS dips
  char bits[FRAME_BITS + 17];  // the decoded bits as '0'/'1' (truncated), NUL-terminated; for diagnostics
};

class BurstDecoder {
 public:
  explicit BurstDecoder(float sample_rate_hz) : us_per_sample_(1e6f / sample_rate_hz) {}
  // Report a dip. May finish the previous burst (returns true and fills *res).
  bool on_dip(uint64_t sample_idx, BurstResult *res);
  // Call periodically with the current sample index. Finishes a burst once the quiet time exceeds ONE_MAX_US.
  bool on_idle(uint64_t sample_idx, BurstResult *res);
  void abort() { bits_.clear(); offs_.clear(); have_last_ = false; }  // sample loss
  float zero_period_us() const { return zero_period_us_; }  // EMA of bit-0 periods: shows real sample-clock error
  float one_period_us() const { return one_period_us_; }
  uint64_t last_dip_index() const { return last_dip_; }  // most recent dip start, kept after a burst finishes
  bool seen_dip() const { return seen_dip_; }

 private:
  void finish(BurstResult *res);
  float us_per_sample_;
  bool have_last_{false};
  uint64_t last_{0}, first_{0};
  uint64_t last_dip_{0}, prev_burst_end_{0};
  bool seen_dip_{false}, have_prev_{false};
  std::vector<uint8_t> bits_;
  std::vector<uint16_t> offs_;
  size_t dips_in_burst_{0};
  float zero_period_us_{0}, one_period_us_{0};
};

// Bit index (in '0'/'1' text, LSB-first bytes) of the first byte that breaks the frame layout, or -1 when the
// layout is intact (CR, addresses, type bytes, 12 uppercase hex digits, LF). Used to locate decoding damage.
int first_bad_bit(const char *bits);
// Render ADC samples ring[(from+i) & mask], i in [0,n), as one hex digit each: (baseline - x) / (depth / 10),
// clamped to 0..F. Writes n chars and a NUL.
void render_trace(const uint16_t *ring, uint32_t mask, uint64_t from, size_t n, float baseline, float depth, char *out);

// ---- register table and request/response pairing ---------------------------
struct RegEntry {
  uint16_t val{0};
  uint32_t t_ms{0};  // time of last response
  uint32_t count{0};
};

struct Counters {
  uint32_t frames_ok{0};
  uint32_t crc_errors{0};
  uint32_t malformed{0};     // bad length / format
  uint32_t unknown_regs{0};  // valid frames for registers outside the known list
  uint32_t unpaired{0};      // response without a matching preceding request
};

class Pairer {
 public:
  // Feed every CRC-valid frame. Returns true if it is a board response whose value should be stored; *reg is
  // the register (taken from the response, cross-checked with the pending request).
  bool on_frame(const Frame &f, uint16_t *reg, uint32_t *unpaired_counter);

 private:
  bool pending_{false};
  uint16_t pending_reg_{0};
  uint8_t pending_sub_{0};
};

class RegisterTable {
 public:
  void set(uint16_t reg, uint16_t val, uint32_t now_ms);
  const RegEntry *get(uint16_t reg) const;
  bool seen(uint16_t reg) const { return get(reg) != nullptr; }
  size_t size() const { return regs_.size(); }
  // age of the last update in ms, UINT32_MAX when never seen
  uint32_t age_ms(uint16_t reg, uint32_t now_ms) const;

 private:
  std::map<uint16_t, RegEntry> regs_;
};

bool is_known_register(uint16_t reg);

// ---- value decoding ---------------------------------------------------------
inline float decode_temp_c(uint16_t v) { return float(v >> 8) + float(v & 0xFF) / 10.0f; }
// 32-bit counter whose low word is the even register, high word the odd one.
inline uint32_t join32(uint16_t hi, uint16_t lo) { return (uint32_t(hi) << 16) | lo; }

// register 0010 bits
constexpr uint16_t ST_CH_DEMAND = 0x0002;
constexpr uint16_t ST_PUMP = 0x0008;
constexpr uint16_t ST_SPARK = 0x0020;
constexpr uint16_t ST_FLAME = 0x00C0;  // either bit
constexpr uint16_t ST_DHW_DEMAND = 0x0100;

const char *state_text(uint16_t status);  // idle / pump / igniting / heating / hot water

// error history: 0142..0146, two codes per register, high byte first -> 10 codes
std::string format_error_history(const uint16_t *vals, const bool *seen);  // 5 entries each

// ---- gas estimate -----------------------------------------------------------
constexpr float GAS_RPM_LO = 1965.0f, GAS_BTU_LO = 18000.0f;
constexpr float GAS_RPM_HI = 6300.0f, GAS_BTU_HI = 199000.0f;
constexpr float PROPANE_BTU_PER_GAL = 91500.0f;
float gas_btu_per_hour(float rpm);  // linear, clamped to the calibrated range
inline float propane_gal_per_hour(float btu_h) { return btu_h / PROPANE_BTU_PER_GAL; }

// ---- transmit (GPIO32 -> NPN -> bus) ------------------------------------------------
// Each pulse is one dip: pin HIGH for high_us (bus pulled down), then LOW for low_us. The pin is LOW whenever a
// pulse list is not running, and no pulse list may hold it HIGH longer than TX_MAX_HIGH_US.
constexpr uint32_t TX_MAX_HIGH_US = 500;       // hard ceiling, enforced by tx_pulses_safe()
constexpr uint32_t TX_BIT0_PERIOD_US = 160;    // dip-to-dip, display's bit 0
constexpr uint32_t TX_BIT1_PERIOD_US = 320;    // dip-to-dip, display's bit 1
constexpr uint32_t TX_MIN_LOW_US = 20;         // keeps the dips distinct and the pin low between them
constexpr uint32_t TX_DIP_US_DEFAULT = 40;     // display dip width (boiler_sniff.py: "fixed ~40 us dips")
constexpr uint32_t TX_DIP_US_MIN = 10;
constexpr uint32_t TX_DIP_US_MAX = TX_BIT0_PERIOD_US - TX_MIN_LOW_US;  // 140: stricter than TX_MAX_HIGH_US
constexpr uint32_t TX_TAIL_LOW_US = 100;       // pin LOW after the last dip
constexpr uint32_t TX_MAX_TOTAL_US = 40000;    // longest pulse list we will ever send (a frame is ~31 ms)

struct TxPulse {
  uint16_t high_us;
  uint16_t low_us;
};

uint32_t clamp_tx_dip_us(float us);  // into [TX_DIP_US_MIN, TX_DIP_US_MAX]; NaN -> default
// n dips, bit-0 spacing (a short non-frame burst)
std::vector<TxPulse> tx_test_pattern(uint32_t n_dips, uint32_t dip_us);
// 18 raw frame bytes (\r addr addr type sub REG VAL CRC \n) with the CRC filled in
void encode_frame(const Frame &f, uint8_t out[FRAME_BYTES]);
Frame make_read_request(uint16_t reg);  // A1 51, type 0x10, sub 0x11, value 0000
// bits LSB-first per byte; first dip is the burst start, every following dip is one bit (160 us = 0, 320 us = 1)
std::vector<TxPulse> tx_frame_pulses(const uint8_t bytes[FRAME_BYTES], uint32_t dip_us);
uint32_t tx_pulses_total_us(const std::vector<TxPulse> &p);
bool tx_pulses_safe(const std::vector<TxPulse> &p);  // every HIGH <= TX_MAX_HIGH_US, every LOW >= TX_MIN_LOW_US, bounded total
bool parse_hex_reg(const std::string &s, uint16_t *out);  // exactly 4 hex digits

}  // namespace boiler
