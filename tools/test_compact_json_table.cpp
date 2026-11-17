#include "compact_json_table.h"
#include <cassert>
#include <iostream>
struct FailingOutput {
  std::string bytes;
  unsigned calls=0, fail_at=0;
  size_t length()const{return bytes.size();}
  const char* c_str()const{return bytes.c_str();}
  FailingOutput& operator+=(const char* text){
    if(++calls!=fail_at)bytes+=text; // Arduino append failure preserves old bytes.
    return *this;
  }
};
int main() {
  String out; CompactJsonTable<String> t(out);
  assert(t.append("{\"a\":null,\"b\":[1,2,{\"x\":3}],\"c\":\"comma,quote\\\"end\"}"));
  assert(t.append("{\"a\":0,\"b\":[],\"c\":false}"));
  assert(t.finish());
  assert(!strcmp(out.c_str(),"{\"fields\":[\"a\",\"b\",\"c\"],\"rows\":[[null,[1,2,{\"x\":3}],\"comma,quote\\\"end\"],[0,[],false]]}"));
  String empty; CompactJsonTable<String> e(empty); assert(e.finish());
  assert(!strcmp(empty.c_str(),"{\"fields\":[],\"rows\":[]}"));
  for(const char* bad : {"{}tail", "{\"a\":}", "{\"a\":[1}", "{\"a\":\"x}", "{\"z\":1}"}) {
    String ignored; CompactJsonTable<String> broken(ignored);
    assert(broken.append("{\"a\":0}")); assert(!broken.append(bad)); assert(!broken.finish());
  }
  FailingOutput good; CompactJsonTable<FailingOutput> baseline(good);
  const String row("{\"a\":123,\"b\":\"quoted,\\\"value\"}");
  assert(baseline.append(row));assert(baseline.append(row));assert(baseline.finish());
  for(unsigned failure=1;failure<=good.calls;++failure){
    FailingOutput out;out.fail_at=failure;CompactJsonTable<FailingOutput> broken(out);
    const bool first=broken.append(row),second=broken.append(row),end=broken.finish();
    assert(!(first&&second&&end));assert(!end); // Never publish partial output.
    const auto before=out.bytes;assert(!broken.append(row));assert(!broken.finish());assert(out.bytes==before);
  }
  // Compare full schema, including removals, reordering and trailing separators.
  for(const char* bad:{"{\"a\":1}","{\"b\":1,\"a\":2}","{\"a\":1,\"b\":2,}","{}"}){
    String out;CompactJsonTable<String> table(out);assert(table.append(row));assert(!table.append(bad));
  }
  std::cout << "Lossless compact tables: strings, escapes, nested arrays, empty, schema drift and malformed input PASS\n";
}
