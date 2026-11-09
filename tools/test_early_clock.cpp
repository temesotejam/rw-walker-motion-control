#include "early_clock.h"
#include <cassert>
int main(){uint32_t x=99;
 assert(!early_clock::relativeMs(99,100,x));
 assert(early_clock::relativeMs(1001000,1000000,x)&&x==1);
 assert(!early_clock::relativeMs(600001001,1000,x));
 assert(!early_clock::epochSupported((static_cast<uint64_t>(UINT32_MAX)+1)*1000));
 assert(early_clock::epochSupported(static_cast<uint64_t>(UINT32_MAX)*1000));
}
