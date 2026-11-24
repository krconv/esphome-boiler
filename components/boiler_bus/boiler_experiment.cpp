#include "boiler_experiment.h"

#include <cstdio>

namespace boiler {

ExpOutcome summarize_experiment(float gap_ms, uint16_t reg, const ExpEvent *ev, size_t n) {
  ExpOutcome o;
  // our own frame
  const ExpEvent *own = nullptr;
  for (size_t i = 0; i < n; i++)
    if (ev[i].kind == 'O') { own = &ev[i]; break; }
  o.own_ok = own && own->ok;
  // the display's next request: first D, or X debris, that is not ours
  for (size_t i = 0; i < n; i++) {
    if (ev[i].kind == 'D' || ev[i].kind == 'X') {
      o.disp_seen = true;
      o.disp_ms = ev[i].t_ms;
      o.disp_ok = ev[i].kind == 'D';
      break;
    }
  }
  // board reply to our register before that request
  for (size_t i = 0; i < n; i++) {
    if (ev[i].kind != 'B' || !ev[i].ok || ev[i].reg != reg) continue;
    if (own && ev[i].t_ms < own->t_ms) continue;
    if (o.disp_seen && ev[i].t_ms > o.disp_ms) continue;
    o.answered = true;
    o.answer_ms = ev[i].t_ms;
    break;
  }
  bool debris = false;
  for (size_t i = 0; i < n; i++) debris = debris || ev[i].kind == 'X';
  o.collision = !own || !o.own_ok || debris;
  if (!o.disp_seen || o.disp_ms > EXP_SKIP_AFTER_MS) o.disp_class = 's';
  else if (std::fabs(o.disp_ms - EXP_DISPLAY_NOMINAL_MS) <= EXP_ON_TIME_TOL_MS) o.disp_class = 'o';
  else o.disp_class = 'd';
  o.code = std::string(1, o.answered ? 'A' : '-') + (o.collision ? 'C' : '.') + o.disp_class;

  char b[320];
  int k = snprintf(b, sizeof(b), "gap=%.0fms reg=%04X ours=%s answered=%s", gap_ms, reg, !own ? "not seen" : (own->ok ? "ok" : "garbled"),
                   o.answered ? "yes" : "no");
  if (o.answered) k += snprintf(b + k, sizeof(b) - k, "@%.0f", o.answer_ms);
  if (o.disp_seen) {
    const char *cls = o.disp_class == 'o' ? "on time" : (o.disp_class == 'd' ? "delayed" : "late/skipped");
    k += snprintf(b + k, sizeof(b) - k, " display=%s@%.0f(%+.0f vs 46) %s", o.disp_ok ? "ok" : "MALFORMED", o.disp_ms,
                  o.disp_ms - EXP_DISPLAY_NOMINAL_MS, cls);
  } else {
    k += snprintf(b + k, sizeof(b) - k, " display=none in window (skipped)");
  }
  k += snprintf(b + k, sizeof(b) - k, " collision=%s | ", o.collision ? "yes" : "no");
  for (size_t i = 0; i < n && k < (int) sizeof(b) - 24; i++)
    k += snprintf(b + k, sizeof(b) - k, "%c%s%.0f ", ev[i].kind, ev[i].ok ? "" : "!", ev[i].t_ms);
  o.line = b;
  return o;
}

void SweepPlan::record(const std::string &code) {
  if (done()) return;
  if (rows_.size() <= gi_) rows_.resize(gi_ + 1);
  rows_[gi_].push_back(code);
  completed_++;
  if (++ti_ >= tries_) {
    ti_ = 0;
    gi_++;
  }
}

std::string SweepPlan::table() const {
  std::string s;
  for (size_t g = 0; g < rows_.size(); g++) {
    if (!s.empty()) s += ' ';
    s += std::to_string(gaps_[g]) + ':';
    for (size_t t = 0; t < rows_[g].size(); t++) {
      if (t) s += ',';
      s += rows_[g][t];
    }
  }
  return s;
}

}  // namespace boiler
