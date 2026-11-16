// Only the isolated v21 environment replaces the pinned SDK's cam_hal.o.
#if defined(CAMERA_FRAME_READY_V21) && CAMERA_FRAME_READY_V21
#include "../third_party/esp32_camera_f0bb429/cam_hal.c"
#endif
