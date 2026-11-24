// "Can we transmit while the display is connected?" experiment: pure summary logic (host-tested).
// Times are ms after the END of the board reply that preceded our request.
#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace boiler {

constexpr float EXP_FRAME_MS = 31.0f;       // one frame on the bus
constexpr float EXP_DISPLAY_NOMINAL_MS = 46.0f;  // display request start after the board reply, when it owns the bus
constexpr float EXP_ON_TIME_TOL_MS = 5.0f;
constexpr float EXP_SKIP_AFTER_MS = 130.0f;  // first display request later than this (a full display cycle) = it skipped its slot
constexpr size_t EXP_MAX_EVENTS = 10;

// One burst seen after we fired. kind: 'O' our own request, 'B' board frame, 'D' display request,
// 'X' anything that did not decode (collision debris).
struct ExpEvent {
  float t_ms;
  char kind;
  bool ok;
  uint16_t reg;
};

struct ExpOutcome {
  bool own_ok{false};
  bool answered{false};  // a board reply to our register arrived before the display's next request
  float answer_ms{NAN};
  bool disp_seen{false};
  float disp_ms{NAN};
  bool disp_ok{false};   // the display request decoded
  char disp_class{'x'};  // 'o' on time, 'd' delayed, 's' skipped (none in the window), 'x' unknown
  bool collision{false};
  std::string code;  // 3 chars: answered A/-, collision C/., display o/d/s
  std::string line;  // one-line description
};

ExpOutcome summarize_experiment(float gap_ms, uint16_t reg, const ExpEvent *ev, size_t n);

// 8 gaps x 3 tries; record() each try's 3-char code (or "N--" when we never got to fire).
class SweepPlan {
 public:
  SweepPlan() : gaps_({5, 10, 15, 20, 25, 30, 35, 40}) {}
  explicit SweepPlan(std::vector<uint32_t> gaps, size_t tries = 3) : gaps_(std::move(gaps)), tries_(tries) {}
  bool done() const { return gi_ >= gaps_.size(); }
  uint32_t current_gap_ms() const { return done() ? 0 : gaps_[gi_]; }
  void record(const std::string &code);
  size_t completed() const { return completed_; }
  size_t total() const { return gaps_.size() * tries_; }
  // "5:A.o,A.o,A.o 10:..." (fits an HA state)
  std::string table() const;
  static const char *legend() { return "A=board answered, C=collision, o/d/s=display request on time/delayed/skipped, N=not sent"; }

 private:
  std::vector<uint32_t> gaps_;
  size_t tries_{3}, gi_{0}, ti_{0}, completed_{0};
  std::vector<std::vector<std::string>> rows_;
};

}  // namespace boiler
