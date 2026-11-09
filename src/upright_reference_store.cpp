#include "upright_reference_store.h"
#include <Preferences.h>
#include <cstring>

namespace upright_reference {
bool load(Record& record) {
  record = Record{};
  Preferences prefs;
  if (!prefs.begin("lateral-ref", true)) return false;
  Record read;
  const bool ok = prefs.getBytesLength("record") == sizeof(read) &&
      prefs.getBytes("record", &read, sizeof(read)) == sizeof(read) && valid(read);
  prefs.end();
  if (ok) record = read;
  return ok;
}
bool save(const Record& record) {
  if (!valid(record)) return false;
  Preferences prefs;
  if (!prefs.begin("lateral-ref", false)) return false;
  const bool written = prefs.putBytes("record", &record, sizeof(record)) == sizeof(record);
  prefs.end();
  Record read;
  return written && load(read) && std::memcmp(&record, &read, sizeof(record)) == 0;
}
}
