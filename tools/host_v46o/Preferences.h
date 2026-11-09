#pragma once
#include <string>
#include <cstring>
// Fake NVS used only by host persistence tests.
class Preferences {
 public:
  inline static std::string blob;
  inline static bool fail_write = false;
  bool begin(const char*, bool = false) { return true; }
  void end() {}
  size_t getBytesLength(const char*) { return blob.size(); }
  size_t getBytes(const char*, void* target, size_t size) {
    if (blob.size() != size) return 0;
    std::memcpy(target, blob.data(), size); return size;
  }
  size_t putBytes(const char*, const void* data, size_t size) {
    if (fail_write) return 0;
    blob.assign(static_cast<const char*>(data), size); return size;
  }
};
