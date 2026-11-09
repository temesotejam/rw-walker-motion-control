#include "lateral_cg_energy.h"
#include <Preferences.h>
#include <cstring>
namespace lateral_cg {
bool load(Record& record) {
  record=Record{};Preferences p;
  if (!p.begin("lateral-cg",true)) return false;
  Record read;
  const bool ok=p.getBytesLength("record")==sizeof(read) &&
      p.getBytes("record",&read,sizeof(read))==sizeof(read) && valid(read);
  p.end();if(ok) record=read;return ok;
}
bool save(const Record& record) {
  if(!valid(record)) return false;
  Preferences p;if(!p.begin("lateral-cg",false)) return false;
  const bool written=p.putBytes("record",&record,sizeof(record))==sizeof(record);
  p.end();Record read;
  return written && load(read) && std::memcmp(&record,&read,sizeof(read))==0;
}
}
