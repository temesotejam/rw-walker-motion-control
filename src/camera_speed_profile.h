#pragma once
#include <stdint.h>

// GC0308 page 0 timing; no crop/subsample/clock changes. Applied only at boot
// while the camera owns temporary SCCB I2C0. No runtime register polling.
namespace camera_speed_profile {
struct Reg { uint8_t address, value; };
constexpr Reg expected[] = {{0x01,0x32},{0x02,0x0c},{0x0f,0x01},
    {0x09,0x01},{0x0a,0xe8},{0x0b,0x02},{0x0c,0x88},{0x12,0x2a},{0x28,0x00}};
// HB 306 -> 64 (Linux GC0308 default; above its minimum 33).
// Existing window height 488 + VB 12 is retained. Shorter AEC limits keep
// exposure below the frame length. Anti-flicker step is ~10ms at the estimated
// 75.8us row time; physical timing and lighting still require board validation.
constexpr Reg settings[] = {{0x01,0x40},{0x0f,0x00},
    {0xe2,0x00},{0xe3,0x84}, // 132 rows
    {0xe4,0x01},{0xe5,0x08}, // 264 rows
    {0xe6,0x01},{0xe7,0x8c},{0xe8,0x01},{0xe9,0x8c},{0xea,0x01},{0xeb,0x8c}};
template<class Read, class Write> bool apply(Read read, Write write) {
    if (!write(0xfe,0)) return false;
    for (auto r:expected) if (read(r.address)!=r.value) return false;
    const int aec=read(0xd2);
    if (aec<0 || !write(0xd2,aec & ~0x80)) return false;
    bool ok=true;
    for(auto r:settings) if(!write(r.address,r.value) || read(r.address)!=r.value){ok=false;break;}
    if(ok) ok=write(0x03,0x01) && write(0x04,0x08); // Seed exposure while AEC is frozen.
    const bool restored=write(0xd2,aec) && read(0xd2)==aec;
    return ok && restored; // Caller deinitializes camera on any failure.
}
}
