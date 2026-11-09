#include "compact_json_table.h"
#include <cassert>
#include <iostream>
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
  std::cout << "Lossless compact tables: strings, escapes, nested arrays, empty, schema drift and malformed input PASS\n";
}
