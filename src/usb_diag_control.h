#pragma once
#include <stdint.h>
#include <string.h>
namespace usb_diag {
enum class Command { None, Enable, Disable, Status, Download, Unknown };
class Parser {
 public:
  Command feed(char c) {
    if (c == '\r') return Command::None;
    if (c != '\n') {
      if (length_ < sizeof(line_) - 1) line_[length_++] = c;
      else overflow_ = true;
      return Command::None;
    }
    line_[length_] = 0;
    const Command result = overflow_ ? Command::Unknown :
        !strcmp(line_, "DIAG ON") ? Command::Enable :
        !strcmp(line_, "DIAG OFF") ? Command::Disable :
        !strcmp(line_, "DIAG STATUS") ? Command::Status :
        !strcmp(line_, "ENTER DOWNLOAD") ? Command::Download :
        length_ ? Command::Unknown : Command::None;
    length_ = 0; overflow_ = false; return result;
  }
 private:
  char line_[32]{};
  uint8_t length_ = 0;
  bool overflow_ = false;
};
}
