#include <cassert>
#include <vector>
#include <iostream>
#include <algorithm>
#include "../src/export_protocol.h"
#include "../src/psram_string.h"
int main(){
  using namespace export_protocol;
  const uint8_t check[]="123456789";assert(crc32(0,check,9)==0xcbf43926);
  assert(crc32(crc32(0,check,4),check+4,5)==0xcbf43926);
  std::vector<uint8_t> source(7*1024*1024+113);
  for(size_t i=0;i<source.size();++i)source[i]=(i*137+29)&255;
  const Span spans[]={{source.data(),110},{source.data()+110,25791},
      {source.data()+25901,source.size()-25905},{source.data()+source.size()-4,4}};
  std::vector<uint8_t> assembled(source.size()),chunk(kChunkBytes),retry(kChunkBytes);
  // Copy the real scatter reader in reverse order (resume/out-of-order requests).
  const size_t pieces=(source.size()+kChunkBytes-1)/kChunkBytes;
  for(size_t i=pieces;i>0;--i){size_t offset=(i-1)*kChunkBytes,n=std::min<size_t>(kChunkBytes,source.size()-offset);
    assert(copy(spans,4,source.size(),offset,chunk.data(),n));
    assert(copy(spans,4,source.size(),offset,retry.data(),n));
    assert(std::equal(chunk.begin(),chunk.begin()+n,retry.begin()));
    std::copy(chunk.begin(),chunk.begin()+n,assembled.begin()+offset);
  }
  assert(source==assembled);
  assert(!copy(spans,4,source.size(),SIZE_MAX,chunk.data(),1));
  assert(!copy(spans,4,source.size(),source.size(),chunk.data(),1));
  assert(!copy(spans,4,source.size(),0,chunk.data(),kChunkBytes+1));
  assert(!validRange(source.size(),0,0,kChunkBytes));
  PsramString s;assert(s.reserve(4));s+="1234";assert(s.ok()&&s.length()==4);s+="5";
  assert(!s.ok()&&s.length()==4);s+="";assert(!s.ok());
  host_psram_failure=true;PsramString failed;assert(!failed.reserve(20)&&!failed.ok());
  std::cout<<"7 MiB resume/retry, scatter boundaries, CRC, overflow ranges and PSRAM failures PASS\n";
}
