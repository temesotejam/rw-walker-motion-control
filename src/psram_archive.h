#pragma once
#include <stddef.h>
#include <new>

// Lifetime-long diagnostic storage. Allocate once during startup, never from
// a control update. The live control/completion state stays outside this archive.
template <typename T, size_t Capacity>
class PsramArchive {
 public:
  bool begin(void* (*allocate)(size_t)) {
    if (storage_) return true;
    void* memory = allocate(sizeof(Storage));
    if (!memory) return false;
    storage_ = new (memory) Storage();
    return true;
  }
  bool available() const { return storage_ != nullptr; }
  T& operator[](size_t index) { return storage_->rows[index]; }
  const T& operator[](size_t index) const { return storage_->rows[index]; }
  static constexpr size_t bytes() { return sizeof(Storage); }
 private:
  struct Storage { T rows[Capacity]; };
  Storage* storage_ = nullptr;
};
