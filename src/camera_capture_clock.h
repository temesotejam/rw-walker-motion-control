#pragma once
#include <stdint.h>

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
