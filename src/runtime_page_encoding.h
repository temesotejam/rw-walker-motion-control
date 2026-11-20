#pragma once

#include <cstring>
#include <strings.h>

namespace runtime_page {
enum class Encoding { Identity, Gzip, Unacceptable };

inline const char* skipSpace(const char* p) { return p + std::strspn(p, " \t"); }

// A qvalue in thousandths. An invalid weight must never enable that coding.
inline int weight(const char* p, const char* end) {
  p = skipSpace(p);
  if (p == end) return 1000;
  if (*p++ != ';') return 0;
  p = skipSpace(p);
  if (*p != 'q' && *p != 'Q') return 0;
  p = skipSpace(p + 1);
  if (*p++ != '=') return 0;
  p = skipSpace(p);
  if (*p != '0' && *p != '1') return 0;
  int result = (*p++ - '0') * 1000;
  if (*p == '.') {
    ++p;
    for (int place = 100; place && *p >= '0' && *p <= '9'; place /= 10) {
      result += (*p++ - '0') * place;
    }
  }
  return skipSpace(p) == end && result <= 1000 ? result : 0;
}

inline Encoding selectEncoding(const char* header) {
  int gzip = -1, identity = -1, wildcard = -1;
  const char* item = header ? header : "";
  while (*item) {
    const char* end = item + std::strcspn(item, ",");
    item = skipSpace(item);
    const size_t n = std::strcspn(item, ";, \t");
    const int q = weight(item + n, end);
    if ((n == 4 && strncasecmp(item, "gzip", n) == 0) ||
        (n == 6 && strncasecmp(item, "x-gzip", n) == 0)) gzip = q;
    else if (n == 8 && strncasecmp(item, "identity", n) == 0) identity = q;
    else if (n == 1 && *item == '*') wildcard = q;
    item = *end ? end + 1 : end;
  }
  if (gzip < 0) gzip = wildcard;
  if (gzip > 0 && gzip >= identity) return Encoding::Gzip;
  if (identity > 0 || (identity < 0 && wildcard != 0)) return Encoding::Identity;
  return Encoding::Unacceptable;
}
}  // namespace runtime_page
