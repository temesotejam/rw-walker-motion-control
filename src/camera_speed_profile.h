#pragma once
#include <stdint.h>

// GC0308 page 0 timing; no crop/subsample/clock changes. Applied only at boot
// while the camera owns temporary SCCB I2C0. No runtime register polling.
namespace camera_speed_profile {
struct Report {
    const char* stage = "not_attempted";
    int reg = -1, expected = -1, actual = -1;
    bool applied = false;
    uint16_t settle_wait_ms = 0;
};
struct Reg { uint8_t address, value; };
constexpr Reg expected[] = {{0x01,0x32},{0x02,0x0c},{0x0f,0x01},
    {0x09,0x01},{0x0a,0xe8},{0x0b,0x02},{0x0c,0x88},{0x12,0x2a},{0x28,0x00}};
#if defined(FOOT_SPEED_FIX1) && FOOT_SPEED_FIX1
// HB64 washed out even with the SDK AEC unchanged in the camera-only bench.
// HB128 keeps margin with this board's sample-hold delay42 at XCLK20MHz.
// Preserve the SDK's complete AEC/anti-flicker/exposure setup.
// Change only line blanking, with the shared high bits written first.
constexpr Reg settings[] = {{0x0f,0x00},{0x01,0x80}};
#else
// Original v22 experimental settings, retained for comparison builds.
// Existing window height 488 + VB 12 is retained. Shorter AEC limits keep
// exposure below the frame length. Anti-flicker step is ~10ms at the estimated
// 75.8us row time; physical timing and lighting still require board validation.
constexpr Reg settings[] = {{0x01,0x40},{0x0f,0x00},
    {0xe2,0x00},{0xe3,0x84}, // 132 rows
    {0xe4,0x01},{0xe5,0x08}, // 264 rows
    {0xe6,0x01},{0xe7,0x8c},{0xe8,0x01},{0xe9,0x8c},{0xea,0x01},{0xeb,0x8c}};
#endif
template<class Read, class Write, class Wait> bool apply(Read read, Write write, Wait wait, Report* report=nullptr) {
    Report local;
    Report& out=report?*report:local;
    out=Report{};
    auto fail=[&](const char* stage,int reg,int wanted,int actual){
        out.stage=stage;out.reg=reg;out.expected=wanted;out.actual=actual;return false;
    };
    auto verify=[&](const char* stage,int reg,int wanted){
        // On the board, an acknowledged timing write still read its old value
        // immediately afterwards. Wait for visibility at boot only. A bus read
        // error remains fatal; a permanently wrong value cannot pass this test.
        int actual=-1;
        for(unsigned i=0;i<=20;++i){
            actual=read(reg);
            if(actual==wanted)return true;
            if(actual<0 || i==20)break;
            wait(5);out.settle_wait_ms+=5;
        }
        return fail(stage,reg,wanted,actual);
    };
    if (!write(0xfe,0)) return fail("page_write",0xfe,0,-1);
    for (auto r:expected) {
        const int actual=read(r.address);
        if(actual!=r.value)return fail("baseline_readback",r.address,r.value,actual);
    }
#if defined(FOOT_SPEED_FIX1) && FOOT_SPEED_FIX1
    bool ok=true;
#else
    const int aec=read(0xd2);
    if (aec<0 || !write(0xd2,aec & ~0x80)) return fail("aec_freeze",0xd2,aec,-1);
    bool ok=verify("aec_freeze_readback",0xd2,aec & ~0x80);
#endif
    for(auto r:settings) { if(!ok)break;
        if(!write(r.address,r.value)){ok=fail("timing_write",r.address,r.value,-1);break;}
        ok=verify("timing_readback",r.address,r.value);
    }
#if !defined(FOOT_SPEED_FIX1) || !FOOT_SPEED_FIX1
    if(ok) {
        if(!write(0x03,0x01))ok=fail("exposure_write",0x03,0x01,-1);
        else if(!write(0x04,0x08))ok=fail("exposure_write",0x04,0x08,-1);
        else ok=verify("exposure_readback",0x03,0x01) && verify("exposure_readback",0x04,0x08);
    }
    if(!write(0xd2,aec))return fail("aec_restore_write",0xd2,aec,-1);
    if(!verify("aec_restore_readback",0xd2,aec))return false;
#endif
    if(ok){out.stage="applied";out.applied=true;}
    return ok; // Caller deinitializes camera on any failure.
}
template<class Read,class Write> bool apply(Read read,Write write){
    return apply(read,write,[](unsigned){},nullptr);
}
}
