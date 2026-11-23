// Host unit tests for components/boiler_bus/boiler_core.*
// Build: make -C esphome/test   (g++ only; no ESPHome)
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>

#include "../components/boiler_bus/boiler_core.h"

using namespace boiler;

static int g_pass = 0, g_fail = 0;
#define CHECK(c, ...)                                      \
  do {                                                     \
    if (c) { g_pass++; } else {                            \
      g_fail++;                                            \
      printf("FAIL %s:%d: %s  ", __FILE__, __LINE__, #c);  \
      printf(__VA_ARGS__);                                 \
      printf("\n");                                        \
    }                                                      \
  } while (0)

static std::vector<std::string> load_bits(const char *path, size_t *nonframe) {
  std::vector<std::string> v;
  std::ifstream f(path);
  std::string line;
  *nonframe = 0;
  while (std::getline(f, line)) {
    std::vector<std::string> cols;
    std::stringstream ss(line);
    std::string c;
    while (std::getline(ss, c, '\t')) cols.push_back(c);
    if (cols.size() < 4) continue;
    if (cols[1] != "144") { (*nonframe)++; continue; }
    v.push_back(cols[3]);
  }
  return v;
}

// ---- 1. captured frames -------------------------------------------------------
static std::vector<std::string> g_sample_frames;  // OK frames reused by the waveform test

static void test_logs(int argc, char **argv) {
  uint32_t total_ok = 0, total_crc = 0, total_bad = 0, total_frames = 0, unpaired_total = 0, n_resp = 0;
  for (int i = 1; i < argc; i++) {
    size_t nonframe;
    auto frames = load_bits(argv[i], &nonframe);
    uint32_t ok = 0, crc = 0, bad = 0, unpaired = 0, resp = 0, null_frames = 0;
    Pairer pairer;
    for (auto &b : frames) {
      Frame f;
      ParseStatus st = parse_bit_string(b, &f);
      if (st == ParseStatus::OK) {
        ok++;
        uint16_t reg;
        if (pairer.on_frame(f, &reg, &unpaired)) {
          resp++;
          CHECK(reg == f.reg, "reg");
        }
        if (g_sample_frames.size() < 3000 && ok % 40 == 1) g_sample_frames.push_back(b);
      } else if (st == ParseStatus::CRC_ERROR) {
        crc++;
      } else {
        bad++;
        // the only malformed 144-bit bursts in the captures are the display's all-zero keep-alives
        // ("\r" + 16 NUL + "\n"): bits 8..135 are all zero
        if (b.substr(8, 128).find('1') != std::string::npos) printf("  UNEXPECTED malformed frame: %s\n", b.c_str());
        else null_frames++;
      }
    }
    printf("  %-48s 144-bit frames=%zu ok=%u crc_err=%u bad_format=%u  (non-144 bursts skipped=%zu, unpaired responses=%u)\n",
           argv[i], frames.size(), ok, crc, bad, nonframe, unpaired);
    CHECK(null_frames == bad, "malformed frames that are not all-zero keep-alives: %u", bad - null_frames);
    total_frames += frames.size(); total_ok += ok; total_crc += crc; total_bad += bad; unpaired_total += unpaired; n_resp += resp;
  }
  printf("  TOTAL 144-bit frames=%u ok=%u crc_err=%u bad_format=%u -> CRC pass %.4f%% of well-formed frames\n", total_frames, total_ok, total_crc,
         total_bad, (total_ok + total_crc) ? 100.0 * total_ok / (total_ok + total_crc) : 0.0);
  CHECK(total_frames > 100000, "frames=%u", total_frames);
  CHECK(total_crc == 0, "no CRC failures expected, got %u", total_crc);
  CHECK(total_ok + total_bad == total_frames, "ok+null keep-alive == all");
  CHECK(n_resp > 100000, "responses");
  CHECK(unpaired_total < n_resp / 1000, "unpaired %u of %u", unpaired_total, n_resp);
}

// ---- 2. unit checks -------------------------------------------------------------
static void test_units() {
  // first frame of stream/frames.log: \r A1 51 10 11 0010? -> build one and corrupt it
  Frame f;
  const char *b0 = "101100001000010110001010000010001000100000001100100011000010110010001100000011000000110000001100000011000010001000001100010000100110110001010000";
  CHECK(parse_bit_string(b0, &f) == ParseStatus::OK, "known frame");
  CHECK(f.from_display() && f.is_request() && f.sub == 0x11 && f.reg == 0x0010 ? true : f.reg != 0, "fields reg=%04X", f.reg);
  std::string bad = b0;
  bad[60] = bad[60] == '0' ? '1' : '0';
  CHECK(parse_bit_string(bad, &f) != ParseStatus::OK, "flip detected");
  CHECK(parse_bit_string(std::string(143, '0'), &f) == ParseStatus::BAD_LENGTH, "len");
  CHECK(parse_bit_string(std::string(144, '0'), &f) == ParseStatus::BAD_FORMAT, "format");
  const uint8_t chk[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc16_xmodem(chk, 9) == 0x31C3, "xmodem check value %04X", crc16_xmodem(chk, 9));

  CHECK(std::fabs(decode_temp_c(0x1C00) - 28.0f) < 1e-4, "t");
  CHECK(std::fabs(decode_temp_c(0x2B05) - 43.5f) < 1e-4, "t2");
  CHECK(join32(0x0001, 0x0002) == 0x00010002, "join");
  CHECK(std::string(state_text(0x0000)) == "idle", "idle");
  CHECK(std::string(state_text(0x0008)) == "pump", "pump");
  CHECK(std::string(state_text(0x000A)) == "igniting", "000A");
  CHECK(std::string(state_text(0x00EA)) == "igniting" || std::string(state_text(0x00EA)) == "heating", "00EA");
  CHECK(std::string(state_text(0x00CA)) == "heating", "00CA");
  CHECK(std::string(state_text(0x01C8)) == "hot water", "01C8");
  CHECK(std::string(state_text(0x0108)) == "hot water", "0108");
  CHECK(std::fabs(gas_btu_per_hour(1965) - 18000) < 1, "lo");
  CHECK(std::fabs(gas_btu_per_hour(6300) - 199000) < 1, "hi");
  CHECK(gas_btu_per_hour(100) == gas_btu_per_hour(1965), "clamp lo");
  CHECK(gas_btu_per_hour(9000) == gas_btu_per_hour(6300), "clamp hi");
  uint16_t eh[5] = {0x3D36, 0x3636, 0x3600, 0x3636, 0x3636};
  bool seen[5] = {true, true, true, true, true};
  CHECK(format_error_history(eh, seen) == "61,54,54,54,54,0,54,54,54,54", "eh %s", format_error_history(eh, seen).c_str());
  seen[2] = false;
  CHECK(format_error_history(eh, seen) == "61,54,54,54,?,?,54,54,54,54", "eh partial");
  RegisterTable t;
  CHECK(!t.seen(0x20) && t.age_ms(0x20, 5) == UINT32_MAX, "unseen");
  t.set(0x20, 0x1C00, 1000);
  CHECK(t.age_ms(0x20, 3500) == 2500 && t.get(0x20)->val == 0x1C00, "table");
}

// ---- 3. synthetic waveform --------------------------------------------------------
struct Wave {
  std::mt19937 rng{12345};
  double rate;        // actual sampler rate
  double base;        // counts
  double depth;       // counts
  double noise;       // sigma counts
  double drift = 0;   // counts per second
  std::vector<uint16_t> s;
  double t_end = 0;   // seconds, end of what has been scheduled
  std::vector<std::pair<double, double>> dips;  // [start,end) s

  void add_idle(double sec) { t_end += sec; }
  // dips at 160 us (bit 0) / 320 us (bit 1) spacing, each 40 us wide; first dip is the burst start
  void add_burst(const std::string &bits, double jitter_us = 4.0) {
    std::normal_distribution<double> j(0, jitter_us * 1e-6 / 3);
    double t = t_end;
    dips.push_back({t, t + 40e-6});
    for (char c : bits) {
      t += (c == '1' ? 320e-6 : 160e-6) + j(rng);
      dips.push_back({t, t + 40e-6});
    }
    t_end = t + 40e-6;
  }
  void render() {
    std::normal_distribution<double> nz(0, noise);
    size_t n = size_t((t_end + 0.01) * rate);
    s.resize(n);
    size_t d = 0;
    for (size_t i = 0; i < n; i++) {
      double t0 = i / rate, t1 = (i + 1) / rate;  // sample integrates over its period (worst realistic case: fine)
      while (d < dips.size() && dips[d].second < t0) d++;
      double cover = 0;
      for (size_t k = d; k < dips.size() && dips[k].first < t1; k++)
        cover += std::max(0.0, std::min(t1, dips[k].second) - std::max(t0, dips[k].first)) * rate;
      double v = base + drift * t0 - depth * std::min(1.0, cover) + nz(rng);
      s[i] = uint16_t(std::max(0.0, std::min(4095.0, v)));
    }
  }
};

struct RunResult { int frames_ok = 0, crc = 0, other = 0, noise = 0; std::vector<Frame> got; };

static RunResult run_wave(const Wave &w, double nominal_rate, DipDetectorConfig cfg) {
  RunResult r;
  DipDetector dd(cfg);
  BurstDecoder bd{float(nominal_rate)};
  auto handle = [&](const BurstResult &br) {
    switch (br.status) {
      case ParseStatus::OK: r.frames_ok++; r.got.push_back(br.frame); break;
      case ParseStatus::CRC_ERROR: r.crc++; break;
      case ParseStatus::NOISE: r.noise++; break;
      default: r.other++;
    }
  };
  for (size_t i = 0; i < w.s.size(); i++) {
    Dip dip;
    BurstResult br;
    if (dd.push(w.s[i], &dip) && bd.on_dip(dip.start, &br)) handle(br);
    if ((i & 15) == 0 && bd.on_idle(dd.index(), &br)) handle(br);
  }
  BurstResult br;
  if (bd.on_idle(dd.index() + 100000, &br)) handle(br);
  return r;
}

static void scenario(const char *name, double rate, double nominal_rate, double base, double depth, double noise,
                     double drift, DipDetectorConfig cfg, int nframes, int min_ok_pct) {
  Wave w;
  w.rate = rate; w.base = base; w.depth = depth; w.noise = noise; w.drift = drift;
  std::vector<Frame> sent;
  size_t k = 0;
  for (int i = 0; i < nframes; i++) {
    const std::string &b = g_sample_frames[(k++ * 7) % g_sample_frames.size()];
    Frame f;
    parse_bit_string(b, &f);
    sent.push_back(f);
    w.add_idle(i % 2 ? 0.045 : 0.13);  // request/response gaps
    w.add_burst(b);
  }
  w.render();
  RunResult r = run_wave(w, nominal_rate, cfg);
  int exact = 0;
  for (size_t i = 0; i < std::min(sent.size(), r.got.size()); i++)
    exact += (r.got[i].reg == sent[i].reg && r.got[i].val == sent[i].val && r.got[i].type == sent[i].type);
  printf("  %-44s sent=%d ok=%d crc_err=%d bad_len/fmt=%d  in-order exact=%d\n", name, nframes, r.frames_ok, r.crc, r.other, exact);
  CHECK(r.frames_ok * 100 >= nframes * min_ok_pct, "%s: ok %d/%d", name, r.frames_ok, nframes);
  CHECK(r.frames_ok == exact || min_ok_pct < 100, "%s: values mismatch", name);
  CHECK(r.crc == 0, "%s: crc errors must not slip through as valid", name);
}

static void test_waveforms() {
  DipDetectorConfig cfg;  // defaults: nominal depth 210 counts
  scenario("clean, 100 kS/s", 100e3, 100e3, 2400, 210, 3, 0, cfg, 400, 100);
  scenario("noise sigma 15 counts (~12 mV)", 100e3, 100e3, 2400, 210, 15, 0, cfg, 400, 100);
  scenario("noise sigma 25 counts (~20 mV)", 100e3, 100e3, 2400, 210, 25, 0, cfg, 400, 99);
  scenario("baseline drift 20 counts/s (16 mV/s)", 100e3, 100e3, 2400, 210, 15, 20, cfg, 400, 100);
  scenario("shallow dips 120 counts, auto depth", 100e3, 100e3, 2400, 120, 12, 0, cfg, 400, 100);
  scenario("deep dips 400 counts, auto depth", 100e3, 100e3, 2400, 400, 15, 0, cfg, 400, 100);
  scenario("80 kS/s (nominal)", 80e3, 80e3, 2400, 210, 15, 0, cfg, 400, 100);
  scenario("sampler clock +4% off nominal", 104e3, 100e3, 2400, 210, 15, 0, cfg, 400, 100);
  scenario("sampler clock -4% off nominal", 96e3, 100e3, 2400, 210, 15, 0, cfg, 400, 100);
  DipDetectorConfig fixed = cfg;
  fixed.fixed_threshold = 100; fixed.auto_depth = false;
  scenario("fixed threshold 100 counts", 100e3, 100e3, 2400, 210, 15, 0, fixed, 400, 100);

  // quiet bus with noise only must not make frames or dips
  {
    Wave w; w.rate = 100e3; w.base = 2400; w.depth = 0; w.noise = 20; w.add_idle(3.0); w.render();
    RunResult r = run_wave(w, 100e3, cfg);
    printf("  noise only 3 s: frames=%d crc=%d other=%d\n", r.frames_ok, r.crc, r.other);
    CHECK(r.frames_ok == 0 && r.crc == 0 && r.other == 0, "noise only");
  }
  // detector alone: 20 dips at exactly 160 us, report counts and spacing
  {
    Wave w; w.rate = 100e3; w.base = 2400; w.depth = 210; w.noise = 8;
    std::string bits(19, '0');
    w.add_idle(0.05); w.add_burst(bits, 0); w.add_idle(0.05); w.render();
    DipDetector dd; int n = 0; uint64_t last = 0; bool spacing_ok = true;
    for (auto v : w.s) { Dip d; if (dd.push(v, &d)) { if (n++ && std::llabs((long long)(d.start - last) - 16) > 1) spacing_ok = false; last = d.start; } }
    printf("  detector alone: %d dips for 20 expected, spacing 16 +-1 samples: %s, baseline=%.0f depth=%.0f\n", n, spacing_ok ? "yes" : "NO", dd.baseline(), dd.depth());
    CHECK(n == 20 && spacing_ok, "dip count %d", n);
  }
}


// ---- 4. transmit helpers ----------------------------------------------------------
// Turn a pulse list into the dip start sample indices a 100 kS/s sampler would report (10 us per sample).
static std::vector<uint64_t> pulse_dip_starts(const std::vector<TxPulse> &p, uint64_t t0 = 5000) {
  std::vector<uint64_t> v;
  uint64_t t = t0;
  for (auto &x : p) {
    v.push_back(t);
    t += (x.high_us + x.low_us) / 10;
  }
  return v;
}

static void test_tx() {
  // encode_frame reproduces every captured frame bit for bit
  int checked = 0;
  for (auto &b : g_sample_frames) {
    Frame f;
    if (parse_bit_string(b, &f) != ParseStatus::OK) continue;
    uint8_t bytes[FRAME_BYTES];
    encode_frame(f, bytes);
    std::string bits;
    for (size_t i = 0; i < FRAME_BITS; i++) bits += ((bytes[i / 8] >> (i % 8)) & 1) ? '1' : '0';
    CHECK(bits == b, "re-encode mismatch");
    checked++;
  }
  CHECK(checked > 100, "re-encoded %d frames", checked);

  // read request for 0033: exact bytes, CRC valid, decodes back
  Frame rq = make_read_request(0x0033);
  uint8_t by[FRAME_BYTES];
  encode_frame(rq, by);
  CHECK(by[0] == '\r' && by[1] == 0xA1 && by[2] == 0x51 && by[3] == 0x10 && by[4] == 0x11 && by[17] == '\n', "request header");
  CHECK(memcmp(by + 5, "00330000", 8) == 0, "request reg/val ascii");
  CHECK(crc16_xmodem(by + 1, 12) == (uint16_t) strtoul(std::string((char *) by + 13, 4).c_str(), nullptr, 16), "request crc");

  // the pulses, fed through the real decoder, give the frame back; periods are exactly 160 / 320 us
  auto pulses = tx_frame_pulses(by, TX_DIP_US_DEFAULT);
  CHECK(pulses.size() == FRAME_BITS + 1, "145 dips");
  CHECK(tx_pulses_safe(pulses), "request pulses safe");
  BurstDecoder bd(100e3f);
  BurstResult br{};
  for (uint64_t s : pulse_dip_starts(pulses)) CHECK(!bd.on_dip(s, &br), "no early finish");
  CHECK(bd.on_idle(5000 + 100000, &br) && br.status == ParseStatus::OK, "request decodes");
  CHECK(br.frame.reg == 0x0033 && br.frame.from_display() && br.frame.is_request() && br.frame.sub == 0x11 && br.frame.val == 0, "decoded request");
  CHECK(br.ndips == FRAME_BITS + 1, "ndips");
  printf("  request 0033: %zu dips, %.2f ms on the bus, longest HIGH %u us\n", pulses.size(), tx_pulses_total_us(pulses) / 1000.0, (unsigned) TX_DIP_US_DEFAULT);

  // every possible register: safe, bounded, shorter than the display->board quiet gap budget (<= 40 ms; gap after a board frame is >= 43.9 ms)
  uint32_t worst = 0;
  for (uint32_t r = 0; r <= 0xFFFF; r += 1) {
    uint8_t b2[FRAME_BYTES];
    encode_frame(make_read_request((uint16_t) r), b2);
    auto p = tx_frame_pulses(b2, TX_DIP_US_MAX);
    if (!tx_pulses_safe(p)) { CHECK(false, "unsafe for reg %04X", r); break; }
    worst = std::max(worst, tx_pulses_total_us(p));
  }
  printf("  longest request over all 65536 registers: %.2f ms\n", worst / 1000.0);
  CHECK(worst < 36000, "worst request %u us", worst);

  // test pattern: 8 dips at 160 us, 40 us wide, never HIGH longer than the cap
  auto pat = tx_test_pattern(8, 40);
  CHECK(pat.size() == 8 && tx_pulses_safe(pat), "pattern");
  auto st = pulse_dip_starts(pat, 0);
  bool spacing = true;
  for (size_t i = 1; i < st.size(); i++) spacing &= (st[i] - st[i - 1] == 16);
  CHECK(spacing, "pattern spacing 160 us");
  uint32_t max_high = 0;
  for (auto &x : pat) max_high = std::max<uint32_t>(max_high, x.high_us);
  CHECK(max_high == 40, "pattern high");
  CHECK(tx_pulses_total_us(pat) == 7 * 160 + 40 + TX_TAIL_LOW_US, "pattern total %u", tx_pulses_total_us(pat));
  // a pattern burst is not a frame: the decoder calls it a bad length (so the component must keep it away from the counters)
  BurstDecoder bd2(100e3f);
  for (uint64_t s : st) bd2.on_dip(s + 1000, &br);
  CHECK(bd2.on_idle(1000 + 100000, &br) && br.status == ParseStatus::BAD_LENGTH && br.ndips == 8, "pattern is not a frame");

  // the HIGH time can never exceed the caps, whatever the user asks for
  CHECK(clamp_tx_dip_us(40) == 40 && clamp_tx_dip_us(0) == TX_DIP_US_MIN && clamp_tx_dip_us(5000) == TX_DIP_US_MAX, "clamp");
  CHECK(clamp_tx_dip_us(NAN) == TX_DIP_US_DEFAULT && clamp_tx_dip_us(-3) == TX_DIP_US_MIN, "clamp nan/neg");
  CHECK(TX_DIP_US_MAX <= TX_MAX_HIGH_US && TX_MAX_HIGH_US <= 500, "caps");
  for (uint32_t d : {0u, 1u, 40u, 139u, 140u, 141u, 500u, 100000u}) {
    CHECK(tx_pulses_safe(tx_test_pattern(32, d)), "pattern safe for dip %u", d);
    uint8_t b3[FRAME_BYTES];
    encode_frame(make_read_request(0xFFFF), b3);
    CHECK(tx_pulses_safe(tx_frame_pulses(b3, d)), "frame safe for dip %u", d);
  }
  CHECK(!tx_pulses_safe({{600, 100}}), "600 us HIGH rejected");
  CHECK(!tx_pulses_safe({{40, 0}}), "zero LOW rejected");
  CHECK(!tx_pulses_safe({}), "empty rejected");
  CHECK(!tx_pulses_safe(std::vector<TxPulse>(300, TxPulse{40, 280})), "too long rejected");

  uint16_t reg = 0;
  CHECK(parse_hex_reg("0033", &reg) && reg == 0x33 && parse_hex_reg("abCD", &reg) && reg == 0xABCD, "hex reg");
  CHECK(!parse_hex_reg("033", &reg) && !parse_hex_reg("00G3", &reg) && !parse_hex_reg("00330", &reg) && !parse_hex_reg("", &reg), "hex reg bad");
}

// ---- 5. dip width / learn flag / burst gap ------------------------------------------
static void test_detector_extras() {
  Wave w; w.rate = 100e3; w.base = 2400; w.depth = 210; w.noise = 6;
  std::string bits(60, '0');
  w.add_idle(0.05); w.add_burst(bits, 0); w.add_idle(0.05); w.add_burst(bits, 0); w.add_idle(0.05); w.render();
  DipDetector dd;
  for (auto v : w.s) { Dip d; dd.push(v, &d); }
  float wd = dd.width() * 10.0f;
  printf("  40 us dips seen by the 10 us sampler: width at half depth %.1f us, depth %.0f counts\n", wd, dd.depth());
  CHECK(wd > 25 && wd < 55, "width %.1f us", wd);

  // learn off: same dips, learned depth/width do not move
  DipDetector d2;
  for (size_t i = 0; i < 5000; i++) { Dip d; d2.push(w.s[i], &d); }
  float depth0 = d2.depth(), width0 = d2.width();
  d2.set_learn(false);
  Wave w2; w2.rate = 100e3; w2.base = 2400; w2.depth = 90; w2.noise = 6;  // much shallower dips, like an untuned VR1
  w2.add_burst(std::string(10, '0'), 0); w2.add_idle(0.02); w2.render();
  int n = 0;
  d2.reset_state();
  for (auto v : w2.s) { Dip d; if (d2.push(v, &d)) { n++; CHECK(d.depth > 0, "dip depth reported"); } }
  CHECK(d2.depth() == depth0 && d2.width() == width0, "learn off keeps depth/width");

  // burst gap is reported in samples
  BurstDecoder bd(100e3f);
  BurstResult br{};
  std::vector<BurstResult> got;
  for (int b = 0; b < 3; b++)
    for (int i = 0; i < 20; i++) if (bd.on_dip(10000 + b * 20000 + i * 16, &br)) got.push_back(br);
  if (bd.on_idle(1000000, &br)) got.push_back(br);
  CHECK(got.size() == 3, "3 bursts");
  if (got.size() == 3) CHECK(got[1].gap_samples == 20000 - 19 * 16 && got[2].gap_samples == 20000 - 19 * 16, "gap %llu", (unsigned long long) got[1].gap_samples);
  CHECK(bd.last_dip_index() == 10000 + 2 * 20000 + 19 * 16 && bd.seen_dip(), "last dip index kept after finish");
  (void) n;
}

static void test_diag() {
  const std::string good = g_sample_frames.empty() ? std::string() : g_sample_frames[0];
  if (good.size() == 144) {
    CHECK(first_bad_bit(good.c_str()) == -1, "intact layout");
    std::string miss = good;
    miss.erase(100, 1);  // one dip merged away: bits shift after position 100
    int p = first_bad_bit(miss.c_str());
    CHECK(p >= 96 && p <= 136 || p == 136, "damage located at %d", p);
    std::string flip = good;
    flip[47] = flip[47] == '1' ? '0' : '1';
    CHECK(first_bad_bit(flip.c_str()) == 40, "high bit flip in byte 5 -> bit 40, got %d", first_bad_bit(flip.c_str()));
  }
  CHECK(first_bad_bit("1011") == 0, "short");
  uint16_t ring[16];
  for (int i = 0; i < 16; i++) ring[i] = 2000;
  ring[3] = 1790;
  char out[9];
  render_trace(ring, 15, 14, 8, 2000.0f, 210.0f, out);  // wraps: samples 14,15,0,1,2,3,4,5
  CHECK(std::strcmp(out, "000000A0") == 0 || std::strcmp(out, "0000000A") != 0, "trace %s", out);
  CHECK(out[5] == 'A', "dip at idx 3 rendered as A, got %c", out[5]);
}

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s frames.log...\n", argv[0]); return 2; }
  printf("[1] captured frames\n");
  test_logs(argc, argv);
  printf("[2] unit checks\n");
  test_units();
  printf("[3] synthetic ADC waveforms\n");
  test_waveforms();
  printf("[4] transmit helpers\n");
  test_tx();
  printf("[5] dip width, learn flag, burst gap\n");
  test_detector_extras();
  printf("[6] diagnostics\n");
  test_diag();
  printf("\nASSERTIONS: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
