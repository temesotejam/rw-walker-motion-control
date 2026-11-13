#pragma once
#if defined(FOOT_SUPPORT_LOCAL) && FOOT_SUPPORT_LOCAL
#include "foot_support_assist_v12.h"
namespace foot_support_policy=foot_support_assist_v12;
#else
#include "foot_support_assist.h"
namespace foot_support_policy=foot_support_assist;
#endif
