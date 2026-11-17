#pragma once
#include <Arduino.h>
#include <cstring>

// Stopped-export only. Schema lives in the output; stream values without
// allocating per-row fields/values Strings or a separate schema copy.
template<class Output> class CompactJsonTable {
 public:
  explicit CompactJsonTable(Output& out):out_(out) {}
  bool append(const String& object) { return appendObject(object); }
  template<class Row> bool appendObject(const Row& object) {
    if (!ok_ || finished_) return ok_=false;
    const char* source=object.c_str();
    if (!started_) {
      if (!write("{\"fields\":[")) return false;
      schema_start_=out_.length();
    }
    size_t schema_pos=0, count=0;
    const bool parsed=members(source,[&](const char* key,size_t n,const char*,size_t) {
      if (count++) {
        if (started_) {
          if (schema_pos>=schema_length_ || out_.c_str()[schema_start_+schema_pos++]!=',') return false;
        } else if (!write(",")) return false;
      }
      if (!started_) return range(key,n);
      if (n>schema_length_-schema_pos || memcmp(out_.c_str()+schema_start_+schema_pos,key,n)) return false;
      schema_pos+=n; return true;
    });
    if (!parsed || (started_ && schema_pos!=schema_length_)) return ok_=false;
    if (!started_) {
      schema_length_=out_.length()-schema_start_;
      if (!write("],\"rows\":[")) return false;
      started_=true;
    } else if (!write(",")) return false;
    if (!write("[")) return false;
    count=0;
    if (!members(source,[&](const char*,size_t,const char* value,size_t n) {
      if (count++ && !write(",")) return false;
      return range(value,n);
    })) return ok_=false;
    return write("]");
  }
  bool finish() {
    if (!ok_ || finished_) return ok_=false;
    finished_=true;
    return write(started_?"]}":"{\"fields\":[],\"rows\":[]}");
  }
 private:
  template<class Visitor> static bool members(const char* p,Visitor visit) {
    if (*p++!='{') return false;
    if (*p=='}') return !p[1];
    for (;;) {
      if (*p!='"') return false;
      const char* key=p++;
      while (*p && *p!='"') { if (*p=='\\' && p[1]) ++p; ++p; }
      if (!*p) return false;
      const size_t key_size=++p-key;
      if (*p++!=':') return false;
      const char* value=p;
      int depth=0; bool quoted=false,escaped=false;
      for (;*p;++p) {
        const char c=*p;
        if (quoted) {
          if (escaped) escaped=false;
          else if (c=='\\') escaped=true;
          else if (c=='"') quoted=false;
        } else {
          if (c=='"') quoted=true;
          else if (c=='[' || c=='{') ++depth;
          else if (c==']' || c=='}') { if (!depth) break; --depth; }
          else if (c==',' && !depth) break;
        }
      }
      if (p==value || quoted || depth || !*p || !visit(key,key_size,value,p-value)) return false;
      if (*p=='}') return !p[1];
      if (*p++!=',' || *p!='"') return false;
    }
  }
  bool write(const char* text) {
    if (!ok_) return false;
    const size_t before=out_.length(),n=strlen(text);
    out_+=text;
    return ok_=(out_.length()==before+n);
  }
  bool range(const char* p,size_t size) {
    char buffer[128];
    while (size) {
      const size_t n=size<sizeof(buffer)-1?size:sizeof(buffer)-1;
      memcpy(buffer,p,n); buffer[n]=0;
      if (!write(buffer)) return false;
      p+=n; size-=n;
    }
    return true;
  }
  Output& out_;
  size_t schema_start_=0,schema_length_=0;
  bool started_=false,ok_=true,finished_=false;
};
