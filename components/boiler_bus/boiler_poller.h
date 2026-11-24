// Bus-master poll logic: pure C++, no ESPHome / ESP-IDF dependency (unit-tested on the host).
// The ESP only acts as master while no display is on the bus.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "boiler_core.h"

namespace boiler {

struct PollRequest {
  uint16_t reg{0};
  uint16_t val{0};
  uint8_t sub{0x11};  // 0x11 read, 0x10 write
  bool is_write() const { return sub == 0x10; }
  bool operator==(const PollRequest &o) const { return reg == o.reg && val == o.val && sub == o.sub; }
  bool operator!=(const PollRequest &o) const { return !(*this == o); }
};

// Fast registers every pass, with one slow register interleaved after each fast one (round-robin).
class PollScheduler {
 public:
  PollScheduler(std::vector<uint16_t> fast, std::vector<uint16_t> slow) : fast_(std::move(fast)), slow_(std::move(slow)) {}
  static PollScheduler defaults();
  uint16_t next();
  const std::vector<uint16_t> &fast() const { return fast_; }
  const std::vector<uint16_t> &slow() const { return slow_; }

 private:
  std::vector<uint16_t> fast_, slow_;
  size_t step_{0}, slow_pos_{0};
};

struct PollerConfig {
  uint32_t display_timeout_ms{5000};  // a display request this recent = display present
  uint32_t answer_timeout_ms{250};    // from the moment the request is fired (frame 31 ms + board ~52 ms + reply 31 ms)
  uint32_t max_misses{3};             // consecutive unanswered requests in FAST mode before falling back to PING
  uint32_t write_repeat_ms{13500};    // second write this long after the first ack (the display repeats at ~13.45 s)
  uint32_t write_tries{3};            // unanswered write / read-back attempts per stage before giving up
};

enum class PollMode : uint8_t { PING, FAST };
enum class WriteResult : uint8_t { OK, NOT_MASTER, OUT_OF_RANGE, BUSY };

class Poller {
 public:
  Poller(PollerConfig cfg, PollScheduler sched) : cfg_(cfg), sched_(std::move(sched)) {}

  void set_enabled(bool on) { enabled_ = on; }
  bool enabled() const { return enabled_; }
  void on_display_request(uint32_t now) { last_disp_ms_ = now; seen_disp_ = true; }
  bool display_present(uint32_t now) const { return seen_disp_ && now - last_disp_ms_ < cfg_.display_timeout_ms; }
  // master only when enabled, no display seen recently, and (after boot) long enough to have heard one
  bool is_master(uint32_t now) const { return enabled_ && !display_present(now) && now >= cfg_.display_timeout_ms; }
  PollMode mode() const { return mode_; }

  // the request that should be armed in the sampler (valid=false: none). Changes whenever armed_seq() changes.
  bool armed(PollRequest *out) const { *out = armed_; return armed_valid_; }
  uint32_t armed_seq() const { return armed_seq_; }

  // events
  void on_sent(uint32_t now, const PollRequest &fired);          // the sampler transmitted this
  bool on_board_response(uint32_t now, const Frame &f);          // a CRC-valid board response; true if it answered us
  void update(uint32_t now);                                     // timeouts, arming; call every loop

  // write val to a setpoint register (REG_CH_SETPOINT / REG_DHW_SETPOINT); OUT_OF_RANGE for any other register
  WriteResult request_write(uint32_t now, uint16_t reg, uint16_t val);
  bool write_active() const { return wstate_ != WState::NONE; }

  // human-readable events since the last call (joined, lost, write results, ...)
  std::vector<std::string> take_events() { std::vector<std::string> e; e.swap(events_); return e; }

 private:
  enum class WState : uint8_t { NONE, SEND1, WAIT_REPEAT, SEND2, READBACK };
  void arm_(const PollRequest &r);
  void disarm_();
  void miss_(uint32_t now);
  void fail_write_(const char *why);
  void event_(const std::string &s) { if (events_.size() < 16) events_.push_back(s); }

  PollerConfig cfg_;
  PollScheduler sched_;
  bool enabled_{true}, seen_disp_{false};
  uint32_t last_disp_ms_{0};
  PollMode mode_{PollMode::PING};
  uint32_t misses_{0};
  bool armed_valid_{false}, inflight_valid_{false};
  PollRequest armed_, inflight_;
  uint32_t armed_seq_{0}, sent_ms_{0};
  bool had_master_{false};

  WState wstate_{WState::NONE};
  uint16_t wreg_{REG_CH_SETPOINT}, wval_{0};
  uint32_t wtries_{0}, wrepeat_at_{0};
  std::vector<std::string> events_;
};

}  // namespace boiler
