#include "boiler_errors.h"

namespace boiler {

namespace {
struct Entry {
  unsigned code;
  const char *text;
};
// Sources: Noritz CB199-DV Owner's Guide p.34 (11, 29, 40, 45, 54) and Install manual (54 shown at the start of auto-fill);
// help.noritz.com error-code pages (12, 14, 16, 20, 61, 63, 67, 71, 73, 76, 90, 91, 94) for the tankless/Pro line, which
// shares the control family. The other codes are not published for the CB199.
const Entry TABLE[] = {
    {11, "Ignition failure"},
    {12, "Flame loss"},
    {14, "Heat exchanger overheat"},
    {16, "Outlet water too hot"},
    {20, "High-limit switch tripped"},
    {29, "Air pressure / condensate drain blocked"},
    {40, "Gas leak detected"},
    {45, "Water leak inside unit"},
    {54, "Low heating water pressure"},
    {61, "Fan motor abnormality"},
    {63, "Recirculation abnormality"},
    {67, "AGM abnormality"},
    {71, "Gas valve abnormality"},
    {73, "Circuit board setting error"},
    {76, "Display communication error"},
    {90, "Combustion abnormality"},
    {91, "Wrong gas type setting"},
    {94, "Exhaust temperature abnormality"},
};
}  // namespace

const char *error_description(unsigned code) {
  for (const Entry &e : TABLE)
    if (e.code == code) return e.text;
  return nullptr;
}

std::string describe_error(unsigned code) {
  if (code == 0) return "None";
  const char *d = error_description(code);
  if (!d) return "Code " + std::to_string(code);
  return std::to_string(code) + " \xE2\x80\x93 " + d;  // en dash
}

bool last_error_code(const uint16_t *vals, const bool *seen, unsigned *code) {
  if (!seen[0]) return false;
  *code = vals[0] >> 8;
  return true;
}

}  // namespace boiler
