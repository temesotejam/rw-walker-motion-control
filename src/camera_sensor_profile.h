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

#if defined(FOOT_NATIVE_HB80_V33) && FOOT_NATIVE_HB80_V33
#undef CAMERA_NATIVE_HB
#define CAMERA_NATIVE_HB 80
#endif

#ifndef CAMERA_NATIVE_HB
#define CAMERA_NATIVE_HB 96
#endif

// Boot-only GC0308 transaction after camera_speed_profile's HB128 setup.
// Caller must stop reception, discard old frames and deinitialize on failure.
// No exposure/gain writes: live AEC may continue updating its result registers.
namespace camera_native_skip_profile {
// HB64 developed a washed-out image during the 90-second native-skip bench.
// HB96 is the current candidate, subject to sustained board verification.
// Lower explicit overrides are experimental; the range check is not a claim
// that every representable setting has acceptable image quality.
constexpr uint16_t kHorizontalBlank = CAMERA_NATIVE_HB;
static_assert(CAMERA_NATIVE_HB >= 64 && CAMERA_NATIVE_HB <= 128,
              "Native HB must stay within the isolated bench range");
constexpr uint16_t kNativeHeight = 488;
constexpr uint16_t kOutputWidth = 320, kOutputHeight = 236;
// Stopped-board image comparison identifies 0x10 as the actual legacy v24
// orientation. 0x12 vertically reverses the calibrated upper/lower marker lanes;
// the old SDK setter's intended flip value is not the empirical image state.
constexpr uint8_t kOrientation = 0x10;
// Bench image alignment: logical old row y corresponds to native row y-2.
// The caller must use this mapping, not relabel the received image as QVGA.
constexpr int kLogicalRowOrigin = 2;
struct Report {
    const char* stage = "not_attempted";
    int reg = -1, expected = -1, actual = -1;
    bool applied = false, page0_restored = false;
    uint16_t settle_wait_ms = 0;
};
struct Reg { uint8_t address, value; };
constexpr Reg baseline0[] = {
    {0x00,0x9b},{0x01,0x80},{0x02,0x0c},{0x0f,0x00},
    {0x05,0x00},{0x06,0x00},{0x07,0x00},{0x08,0x00},
    {0x09,0x01},{0x0a,0xe8},{0x0b,0x02},{0x0c,0x88},
    {0x0d,0x02},{0x0e,0x02},{0x12,0x2a},{0x13,0x00},
    {0x14,kOrientation},{0x15,0x0a},{0x24,0xb1},{0x28,0x00},{0x46,0x00}};
constexpr Reg baseline1[] = {
    {0x50,0x00},{0x53,0x82},{0x54,0x22},
    {0x56,0x00},{0x57,0x00},{0x58,0x00},{0x59,0x00}};
// Configured AEC/gain controls must remain unchanged. Do not compare live
// exposure03/04 or auto-gain51/52/AWB5a..5c, which can change with AEC enabled.
constexpr uint8_t preserved0[] = {
    0x50,0xd0,0xd1,0xd2,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,
    0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xef};

template<class Read, class Write, class Wait>
bool apply(Read read, Write write, Wait wait, Report* report=nullptr) {
    Report local;
    Report& out=report?*report:local;
    out=Report{};
    auto fail=[&](const char* stage,int reg,int expected,int actual) {
        out.stage=stage;out.reg=reg;out.expected=expected;out.actual=actual;
        return false;
    };
    auto verify=[&](int reg,int expected,int mask) {
        int actual=-1;
        for(unsigned i=0;i<=20;++i) {
            actual=read(reg&0xff);
            if(actual>=0 && (actual&mask)==expected)return true;
            if(actual<0 || i==20)break;
            wait(5);out.settle_wait_ms+=5;
        }
        out.actual=actual;
        return false;
    };
    auto page=[&](int selected) {
        if(!write(0xfe,selected))return fail("page_write",0xfe,selected,-1);
        if(!verify(0xfe,selected,1))return fail("page_readback",0xfe,selected,out.actual);
        return true;
    };
    auto exact=[&](const char* stage,int reg,int expected) {
        const int actual=read(reg&0xff);
        return actual==expected || fail(stage,reg,expected,actual);
    };
    auto setting=[&](int reg,int value) {
        if(!write(reg&0xff,value))return fail("setting_write",reg,value,-1);
        return verify(reg,value,0xff) || fail("setting_readback",reg,value,out.actual);
    };
    int saved[sizeof(preserved0)] = {};
    int sub_mode=-1, pixel_count_limit=-1;
    bool ok=page(0);
    if(ok)for(auto r:baseline0) {
        if(r.address==0x14) {
            const int actual=read(0x14);
            // Legacy flip/mirror setters separately RMW this register and
            // can overwrite a not-yet-visible earlier write. Admit only the
            // known upper bits, then explicitly establish the empirically
            // verified legacy image orientation, including both low bits.
            if(actual<0 || (actual&0xfc)!=0x10) {
                ok=fail("baseline_orientation_bits",0x14,0x10,actual);break;
            }
            continue;
        }
        if(!exact("baseline_readback",r.address,r.value)){ok=false;break;}
    }
    if(ok)for(unsigned i=0;i<sizeof(preserved0);++i) {
        saved[i]=read(preserved0[i]);
        if(saved[i]<0){ok=fail("preserve_read",preserved0[i],-1,saved[i]);break;}
    }
    if(ok)ok=page(1);
    if(ok)for(auto r:baseline1) {
        if(r.address==0x53) {
            pixel_count_limit=read(0x53);
            // GC0308 datasheet p.28: P1:53[4:0] is the row-tail pixel
            // count limit (at least 2). SDK initialization has two verified
            // defaults,82/83; retain the exact byte, including upper bits.
            // Other values are not admitted merely by masking the low bits.
            if(pixel_count_limit!=0x82 && pixel_count_limit!=0x83) {
                ok=fail("baseline_readback",0x153,0x82,pixel_count_limit);break;
            }
            continue;
        }
        if(!exact("baseline_readback",0x100|r.address,r.value)){ok=false;break;}
    }
    if(ok) {
        sub_mode=read(0x55);
        // SDK sets extension bit0 and preserves optional neighbor averaging.
        if(sub_mode!=0x01 && sub_mode!=0x03)
            ok=fail("baseline_sub_mode",0x155,0x01,sub_mode);
    }
    // Every baseline check completes before any sensor timing/mode mutation.
    if(ok)ok=page(0) && setting(0x14,kOrientation) && setting(0x15,0x4a) && setting(0x01,kHorizontalBlank);
    if(ok)ok=page(1) && setting(0x154,0x12);
    if(ok)for(auto r:baseline1) {
        const int wanted=r.address==0x54?0x12:r.address==0x53?pixel_count_limit:r.value;
        if(!exact("final_readback",0x100|r.address,wanted)){ok=false;break;}
    }
    if(ok)ok=exact("final_readback",0x155,sub_mode);
    if(ok)ok=page(0);
    if(ok)for(auto r:baseline0) {
        const int wanted=r.address==0x15?0x4a:r.address==0x01?kHorizontalBlank:r.value;
        if(!exact("final_readback",r.address,wanted)){ok=false;break;}
    }
    if(ok)for(unsigned i=0;i<sizeof(preserved0);++i) {
        if(!exact("preserved_readback",preserved0[i],saved[i])){ok=false;break;}
    }
    // Cleanup is attempted on every path, including failure on page1. Preserve
    // the original failure diagnostic if page restoration itself also fails.
    const Report original=out;
    const bool restored=write(0xfe,0) && verify(0xfe,0,1);
    const uint16_t total_wait=out.settle_wait_ms;
    if(!ok)out=original;
    out.settle_wait_ms=total_wait;
    out.page0_restored=restored;
    if(!restored && ok)ok=fail("page0_restore",0xfe,0,out.actual);
    if(ok){out.stage="applied";out.reg=-1;out.expected=-1;out.actual=-1;out.applied=true;}
    return ok;
}
template<class Read,class Write>
bool apply(Read read,Write write) {
    return apply(read,write,[](unsigned){},nullptr);
}
}

// ESP32-S3 LCD_CAM source PLL160. Integer divider7 gives about22.857MHz.
// The24MHz fractional-divider bench produced incomplete DMA frames/overflows;
// it is not a production option. Start at20MHz, switch while gated at boot.
namespace camera_capture_clock {
constexpr uint32_t kSourceHz=160000000;
constexpr unsigned kDivider=7,kDenominator=0,kNumerator=0;
constexpr uint32_t kTargetHz=kSourceHz/kDivider;
static_assert(kTargetHz>20000000 && kTargetHz<=24000000,"Within sensor nominal24MHz specification");
template<class Register> bool apply(Register& r) {
    // Fail closed on a different SDK/source clock setup; preserve other bits.
    if(r.cam_clk_sel!=3 || r.cam_clkm_div_num!=8 ||
       r.cam_clkm_div_a!=0 || r.cam_clkm_div_b!=0)return false;
    r.cam_clkm_div_num=kDivider;
    r.cam_clkm_div_a=kDenominator;
    r.cam_clkm_div_b=kNumerator;
    r.cam_update=1;
    return r.cam_clk_sel==3 && r.cam_clkm_div_num==kDivider &&
        r.cam_clkm_div_a==kDenominator && r.cam_clkm_div_b==kNumerator;
}
}
