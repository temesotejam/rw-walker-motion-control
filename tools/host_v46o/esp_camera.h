#pragma once
#include <stdint.h>
#include <sys/time.h>
enum pixformat_t {PIXFORMAT_GRAYSCALE};
struct camera_fb_t {uint8_t* buf;size_t len;size_t width,height;pixformat_t format;timeval timestamp;};
