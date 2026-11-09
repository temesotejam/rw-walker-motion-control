#include "early_geometry_diagnostics.h"
early_diag::Counters g_early_diag;
int main(){g_early_diag.reset(1);return g_early_diag.snapshot().run==1?0:1;}
