#pragma once
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <utility>

// Large exports must never fall back to scarce internal/DMA RAM. Fixed capacity,
// sticky failure, move-only ownership: a partial JSON can never be published.
class PsramString {
 public:
  PsramString() = default;
  ~PsramString() { free(data_); }
  PsramString(const PsramString&) = delete;
  PsramString& operator=(const PsramString&) = delete;
  PsramString(PsramString&& s) noexcept { swap(s); }
  PsramString& operator=(PsramString&& s) noexcept { swap(s); return *this; }
  bool reserve(size_t capacity) {
    if (data_) return capacity <= capacity_;
    data_ = static_cast<char*>(heap_caps_malloc(capacity + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (!data_) { failed_ = true; return false; }
    capacity_ = capacity; data_[0] = 0; return true;
  }
  void clear() {
    length_ = 0; failed_ = false;
    if (data_) data_[0] = 0;
  }
  PsramString& operator+=(const char* s) { append(s, strlen(s)); return *this; }
  PsramString& operator+=(const String& s) { append(s.c_str(), s.length()); return *this; }
  void append(const char* s, size_t n) {
    if (failed_ || !data_ || n > capacity_ - length_) { failed_ = true; return; }
    memcpy(data_ + length_, s, n); length_ += n; data_[length_] = 0;
  }
  size_t length() const { return length_; }
  const char* c_str() const { return data_ ? data_ : ""; }
  bool ok() const { return data_ && !failed_; }
  void fail() { failed_ = true; }
 private:
  void swap(PsramString& s) {
    std::swap(data_, s.data_); std::swap(length_, s.length_);
    std::swap(capacity_, s.capacity_); std::swap(failed_, s.failed_);
  }
  char* data_ = nullptr;
  size_t length_ = 0, capacity_ = 0;
  bool failed_ = false;
};
