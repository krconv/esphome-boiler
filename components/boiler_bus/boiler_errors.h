// Error-code table and "Last error" text. Pure C++ (host-tested).
// Codes are the decimal numbers the display prints (Er:NN); the history registers 0142..0146 hold two codes each,
// hi byte first, E0 (the most recent) first.
#pragma once
#include <cstdint>
#include <string>

namespace boiler {

// Short description, or nullptr when the code is not in the table.
const char *error_description(unsigned code);
// "None" for 0, "76 – Display communication error", or "Code 99" for an unknown code.
std::string describe_error(unsigned code);
// E0 = hi byte of register 0142. false while 0142 has not been seen.
bool last_error_code(const uint16_t *vals, const bool *seen, unsigned *code);

}  // namespace boiler
