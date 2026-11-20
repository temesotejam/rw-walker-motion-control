#include "runtime_page_encoding.h"
#include <cassert>

int main() {
  using runtime_page::Encoding;
  using runtime_page::selectEncoding;
  assert(selectEncoding(nullptr) == Encoding::Identity);
  assert(selectEncoding("") == Encoding::Identity);
  assert(selectEncoding("br") == Encoding::Identity);
  assert(selectEncoding("deflate, gzip, br") == Encoding::Gzip);
  assert(selectEncoding("  X-GZIP  ; Q=0.001 , br") == Encoding::Gzip);
  assert(selectEncoding("*") == Encoding::Gzip);
  assert(selectEncoding("identity;q=0.8, gzip;q=0.5") == Encoding::Identity);
  assert(selectEncoding("identity;q=0.8, gzip;q=0.800") == Encoding::Gzip);
  assert(selectEncoding("gzip;q=0, *;q=1") == Encoding::Identity);
  assert(selectEncoding("gzip;q=1, *;q=0") == Encoding::Gzip);
  assert(selectEncoding("gzip;q=0.000, identity;q=0") == Encoding::Unacceptable);
  assert(selectEncoding("br, *;q=0.") == Encoding::Unacceptable);
  assert(selectEncoding("br, *;q=0, identity") == Encoding::Identity);
  assert(selectEncoding("not-gzip, gzip2, gzip;q=1.001") == Encoding::Identity);
  assert(selectEncoding("gzip;q=0.0001") == Encoding::Identity);
}
