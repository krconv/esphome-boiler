#pragma once

// Schedule for the 1-Wire temperature probes (pure logic, no hardware). The bit-banged 1-Wire transactions hold
// interrupts off and corrupt any boiler reply that arrives meanwhile, so they only run in "quiet slots" chosen by the
// sampler: one transaction per slot, right after a board frame has been decoded and before our next request goes out.
//   - one skip-ROM "convert all" (a few ms) starts a conversion on every probe at once,
//   - then, once the conversion is done, one probe's scratchpad is read per slot (~11 ms each).
// A new cycle starts `period` after the previous convert.
#include <cstddef>
#include <cstdint>

namespace boiler {

class ProbeCycle {
 public:
  enum class Op : uint8_t { NONE, CONVERT, READ };

  ProbeCycle(size_t n, uint32_t period_ms, uint32_t conversion_ms = 800) : n_(n), period_(period_ms), conv_(conversion_ms) {}

  // What to do with the quiet slot that is available at now_ms. For READ, *idx is the probe to read.
  Op next(uint32_t now_ms, size_t *idx) const {
    if (n_ == 0) return Op::NONE;
    if (phase_ == Phase::IDLE) return (!started_ || now_ms - start_ms_ >= period_) ? Op::CONVERT : Op::NONE;
    if (now_ms - start_ms_ < conv_) return Op::NONE;
    if (idx) *idx = idx_;
    return Op::READ;
  }

  // Report the outcome of what next() returned. A failed convert (no presence pulse) is retried after RETRY_MS.
  void done(Op op, bool ok, uint32_t now_ms) {
    if (op == Op::CONVERT) {
      started_ = true;
      if (ok) {
        phase_ = Phase::READING;
        start_ms_ = now_ms;
        idx_ = 0;
      } else {
        start_ms_ = now_ms - period_ + RETRY_MS;  // try again in RETRY_MS
      }
    } else if (op == Op::READ) {
      if (++idx_ >= n_) phase_ = Phase::IDLE;
    }
  }

  static constexpr uint32_t RETRY_MS = 500;

 private:
  enum class Phase : uint8_t { IDLE, READING };
  size_t n_;
  uint32_t period_, conv_;
  Phase phase_{Phase::IDLE};
  bool started_{false};
  uint32_t start_ms_{0};
  size_t idx_{0};
};

// DS18B20 scratchpad -> deg C (NAN if the CRC fails). The resolution bits of byte 4 say which low bits are undefined.
inline float probe_temp_c(const uint8_t sp[9], bool *crc_ok) {
  uint8_t crc = 0;
  for (int i = 0; i < 8; i++) {
    uint8_t b = sp[i];
    for (int k = 0; k < 8; k++) {
      const bool mix = (crc ^ b) & 1;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      b >>= 1;
    }
  }
  if (crc_ok) *crc_ok = crc == sp[8];
  if (crc != sp[8]) return 0.0f / 0.0f;
  int16_t t = (int16_t)((sp[1] << 8) | sp[0]);
  switch ((sp[4] >> 5) & 3) {  // 0 = 9 bit .. 3 = 12 bit
    case 0: t &= (int16_t) 0xfff8; break;
    case 1: t &= (int16_t) 0xfffc; break;
    case 2: t &= (int16_t) 0xfffe; break;
    default: break;
  }
  return t / 16.0f;
}

}  // namespace boiler
