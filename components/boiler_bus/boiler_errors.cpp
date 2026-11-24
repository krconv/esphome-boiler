#include "boiler_errors.h"

namespace boiler {

namespace {
struct Entry {
  unsigned code;
  const char *text;
};
// Source: the error-code label inside this boiler's cabinet (photo, 2026-11-24).
const Entry TABLE[] = {
    {10, "Flame went out 8 times"},
    {11, "Ignition failed 10 times"},
    {16, "Overheat during hot water"},
    {20, "High-limit switch tripped"},
    {29, "Air pressure switch open (check condensate trap)"},
    {30, "Outdoor sensor fault"},
    {31, "Inlet water sensor fault"},
    {32, "Hot water sensor fault"},
    {33, "Outlet water sensor fault"},
    {34, "Return water sensor fault"},
    {35, "Exhaust sensor fault"},
    {38, "Memory (EEPROM) fault"},
    {39, "Flame after shutdown"},
    {40, "Gas leak detected"},
    {41, "Flue blocked"},
    {43, "Burner overheat switch tripped"},
    {45, "Water leak detected"},
    {54, "Low heating water pressure"},
    {55, "Refill stopped (pressure not rising)"},
    {56, "High heating water pressure"},
    {58, "Low water cutoff tripped"},
    {59, "Pressure sensor fault"},
    {61, "Fan speed sensor fault"},
    {63, "Overheat during heating"},
    {65, "Water valve fault"},
    {67, "Gas/air mixer fault"},
    {70, "Control board self-check fault"},
    {72, "Flame signal before ignition"},
    {73, "DIP switch setting fault"},
    {76, "Control panel communication fault"},
    {94, "Exhaust overheat"},
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
