#include "boiler_poller.h"

#include <cstdio>

namespace boiler {

PollScheduler PollScheduler::defaults() {
  return PollScheduler(
      {0x0010, 0x0020, 0x0033, 0x0027, 0x0137},  // status, supply, fan rpm, flow, flame volts x10
      {0x0130, 0x0021, 0x0022, 0x0023, 0x0024, 0x0132, 0x0101, 0x0102, 0x0091, 0x0092, 0x0093, 0x0094,
       0x0095, 0x0096, 0x0098, 0x0099, 0x0133, 0x0134, 0x0142, 0x0143, 0x0144, 0x0145, 0x0146, 0x0009});
}

uint16_t PollScheduler::next() {
  if (fast_.empty() && slow_.empty()) return 0;
  if (slow_.empty()) return fast_[step_++ % fast_.size()];
  if (fast_.empty()) return slow_[slow_pos_++ % slow_.size()];
  const size_t k = step_++ % (2 * fast_.size());
  if (k % 2 == 0) return fast_[k / 2];
  return slow_[slow_pos_++ % slow_.size()];
}

void Poller::arm_(const PollRequest &r) {
  armed_ms_ = now_ms_;
  armed_ = r;
  armed_valid_ = true;
  armed_seq_++;
}

void Poller::disarm_() {
  if (armed_valid_) armed_seq_++;
  armed_valid_ = false;
}

void Poller::fail_write_(const char *why) {
  char b[112];
  snprintf(b, sizeof(b), "%s setpoint write %04X failed: %s", setpoint_name(wreg_), wval_, why);
  event_(b);
  wstate_ = WState::NONE;
}

WriteResult Poller::request_write(uint32_t now, uint16_t reg, uint16_t val) {
  if (!is_master(now)) return WriteResult::NOT_MASTER;
  if (wstate_ != WState::NONE) return WriteResult::BUSY;
  if (!is_setpoint_reg(reg) || !setpoint_in_range(reg, decode_setpoint(val))) return WriteResult::OUT_OF_RANGE;
  wreg_ = reg;
  wval_ = val;
  wtries_ = 0;
  wstate_ = WState::SEND1;
  return WriteResult::OK;
}

void Poller::on_sent(uint32_t now, const PollRequest &fired) {
  inflight_ = fired;
  inflight_valid_ = true;
  sent_ms_ = now;
  if (armed_valid_ && armed_ == fired) disarm_();
}

bool Poller::on_board_response(uint32_t now, const Frame &f) {
  if (!inflight_valid_) return false;
  const bool ack = f.sub == 0x10;
  if (f.reg != inflight_.reg || ack != inflight_.is_write()) return false;
  inflight_valid_ = false;
  misses_ = 0;
  answered_ms_ = now;
  if (mode_ != PollMode::FAST) {
    mode_ = PollMode::FAST;
    event_("joined: board answers, fast polling");
  }
  if (inflight_.is_write()) {
    if (f.val != inflight_.val) {
      char b[80];
      snprintf(b, sizeof(b), "ack mismatch (sent %04X, got %04X)", inflight_.val, f.val);
      fail_write_(b);
    } else if (wstate_ == WState::SEND1) {
      wstate_ = WState::WAIT_REPEAT;
      wtries_ = 0;
      wrepeat_at_ = now + cfg_.write_repeat_ms;
      char b[80];
      snprintf(b, sizeof(b), "%s setpoint write %04X acked, repeating in %u ms", setpoint_name(wreg_), wval_, (unsigned) cfg_.write_repeat_ms);
      event_(b);
    } else if (wstate_ == WState::SEND2) {
      wstate_ = WState::READBACK;
      wtries_ = 0;
      event_(std::string(setpoint_name(wreg_)) + " setpoint write acked twice, reading back");
    }
  } else if (wstate_ == WState::READBACK && inflight_.reg == wreg_) {
    char b[80];
    if (f.val == wval_) {
      snprintf(b, sizeof(b), "%s setpoint %04X confirmed by read-back", setpoint_name(wreg_), f.val);
      event_(b);
      wstate_ = WState::NONE;
    } else {
      snprintf(b, sizeof(b), "read-back %04X, expected %04X", f.val, wval_);
      fail_write_(b);
    }
  }
  return true;
}

// Drop to the ping slot and rejoin. A write in progress stays queued and is retried once rejoined,
// but each fallback uses up one of its tries so it cannot wait forever.
void Poller::lose_(uint32_t, const char *why, bool count_write) {
  mode_ = PollMode::PING;
  misses_ = 0;
  inflight_valid_ = false;
  disarm_();
  event_(std::string("lost board, rejoining: ") + why);
  if (count_write && wstate_ != WState::NONE && ++wtries_ >= cfg_.write_tries) fail_write_("board lost, not acknowledged");
}

void Poller::miss_(uint32_t now) {
  inflight_valid_ = false;
  misses_++;
  const bool was_write = inflight_.is_write();
  const bool was_readback = !was_write && wstate_ == WState::READBACK && inflight_.reg == wreg_;
  if ((was_write || was_readback) && wstate_ != WState::NONE && ++wtries_ >= cfg_.write_tries)
    fail_write_(was_write ? "not acknowledged (no ack)" : "no read-back");
  if (mode_ == PollMode::FAST && misses_ >= cfg_.max_misses) lose_(now, "no answers", false);
}

void Poller::on_board_unsolicited(uint32_t now, const Frame &f) {
  // the board only sends its 0020 presence frame when no master is talking to it
  if (mode_ != PollMode::FAST || f.reg != ST_SPARK || !f.from_board()) return;
  if (now - answered_ms_ < cfg_.presence_guard_ms) return;
  lose_(now, "board idle (presence frame)", true);
}

void Poller::update(uint32_t now) {
  now_ms_ = now;
  if (!is_master(now)) {
    if (had_master_) {
      if (display_present(now)) event_("display present: polling paused");
      had_master_ = false;
    }
    disarm_();
    inflight_valid_ = false;
    mode_ = PollMode::PING;
    misses_ = 0;
    if (wstate_ != WState::NONE) fail_write_(display_present(now) ? "display appeared" : "polling off");
    return;
  }
  if (!had_master_) {
    had_master_ = true;
    event_("no display: polling active");
  }
  if (inflight_valid_ && now - sent_ms_ > cfg_.answer_timeout_ms) miss_(now);
  if (mode_ == PollMode::FAST && now - answered_ms_ > cfg_.watchdog_ms) lose_(now, "watchdog, no answer", true);
  // an armed request the sampler never sent (no gap found, transmit failed, event dropped): nothing else would re-arm it
  if (armed_valid_ && now - armed_ms_ > (mode_ == PollMode::FAST ? cfg_.armed_timeout_ms : cfg_.ping_rearm_ms)) {
    disarm_();
    if (mode_ == PollMode::FAST) miss_(now);
  }
  if (wstate_ == WState::WAIT_REPEAT && (int32_t)(now - wrepeat_at_) >= 0) {
    wstate_ = WState::SEND2;
    wtries_ = 0;
  }
  // what goes out next: a due write, the read-back, else the next scheduled read
  const bool fast = mode_ == PollMode::FAST;
  // no second write / read-back while the first one is still in flight
  const bool busy_reg = inflight_valid_ && inflight_.reg == wreg_;
  PollRequest want;
  bool is_prio = false;
  if (fast && !busy_reg && (wstate_ == WState::SEND1 || wstate_ == WState::SEND2)) {
    want = {wreg_, wval_, 0x10};
    is_prio = true;
  } else if (fast && !busy_reg && wstate_ == WState::READBACK) {
    want = {wreg_, 0, 0x11};
    is_prio = true;
  }
  if (is_prio) {
    if (!armed_valid_ || armed_ != want) arm_(want);
  } else if (!armed_valid_) {
    arm_({sched_.next(), 0, 0x11});
  }
}

}  // namespace boiler
