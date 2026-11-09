#if !defined(FOOT_EARLY_TRIAL) || FOOT_EARLY_TRIAL != 1
#error FOOT_EARLY_TRIAL must be enabled in the trial firmware
#endif
#if !defined(FOOT_GEOMETRY_PROVISIONAL_EXPOSURE) || FOOT_GEOMETRY_PROVISIONAL_EXPOSURE != 1
#error Provisional exposure assumption must be explicitly enabled in the trial firmware
#endif
