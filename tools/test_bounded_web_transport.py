"""Compile and exercise the production socket-write loop with a paced peer."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
fixture = r'''
#include "bounded_web_server.h"
#include <cassert>
#include <cerrno>
#include <cstdint>
#include <iostream>
#include <vector>
uint32_t clock_ms = 0;
int socket_fd = 1, stops = 0, mode = 0, calls = 0;
std::vector<char> received;
uint32_t millis() { return clock_ms; }
void vTaskDelay(uint32_t n) { clock_ms += n; }
void Client::stop() { ++stops; }
int Client::fd() { return socket_fd; }
int send(int, const void* buffer, size_t length, int) {
  ++calls;
  if (mode == 1) { errno = EAGAIN; return -1; }
  if (mode == 2) { errno = EPIPE; return -1; }
  if (mode == 3) return 0;
  if (mode == 4) { received.push_back(*static_cast<const char*>(buffer)); return 1; }
  if (mode == 5 && calls % 5 != 0) {
    const int errors[] = {EINTR, ENOMEM, EAGAIN, EWOULDBLOCK};
    errno = errors[(calls - 1) % 4]; return -1;
  }
  if (mode == 6) clock_ms += 80; // About 12 kB/s; 96 kB needs over 1.2 s.
  const size_t maximum = mode == 6 ? 1024 : 37;
  const auto count = length < maximum ? length : maximum;
  const auto data = static_cast<const char*>(buffer);
  received.insert(received.end(),data,data+count);
  return static_cast<int>(count);
}
class Server : public BoundedWriteWebServer {
 public:
  size_t write(bool flash, const char* p, size_t n) {
    return flash ? _currentClientWrite_P(p,n) : _currentClientWrite(p,n);
  }
};
void reset(int m, uint32_t start = 0) {
  mode=m; clock_ms=start; stops=0; calls=0; socket_fd=1; received.clear();
}
int main() {
  Server server; std::vector<char> page(95778,'x');
  for (size_t i=0;i<page.size();++i) page[i]=static_cast<char>(i%251);
  for (bool flash : {false,true}) {
    for (int m : {0,5,6}) {
      reset(m);
      assert(server.write(flash,page.data(),page.size()) == page.size());
      assert(received == page);
      assert(clock_ms > 1200 && clock_ms < 15000 && stops == 0);
    }
    reset(0,UINT32_MAX-255);
    assert(server.write(flash,page.data(),page.size()) == page.size());
    assert(received == page);
    assert(stops == 0); // millis() rollover does not terminate active progress.
    reset(1);
    assert(server.write(flash,page.data(),page.size()) == 0);
    assert(clock_ms == 3000 && stops == 1);
    reset(4);
    const auto partial = server.write(flash,page.data(),page.size());
    assert(partial > 0 && partial < page.size() && clock_ms == 15000 && stops == 1);
    for (int m : {2,3}) {
      reset(m); assert(server.write(flash,page.data(),page.size()) == 0);
      assert(stops == 1 && clock_ms == 0);
    }
    reset(0); socket_fd=-1;
    assert(server.write(flash,page.data(),page.size()) == 0 && calls == 0 && stops == 0);
    reset(0); assert(server.write(flash,page.data(),0) == 0 && calls == 0 && stops == 0);
  }
  std::cout << "PASS: production RAM/flash transport: 96kB slow/partial/retry, rollover, idle/total limits, fatal/closed/empty\n";
}
'''

with tempfile.TemporaryDirectory(prefix='bounded_web_transport_') as directory:
    tmp = Path(directory)
    (tmp / 'lwip').mkdir()
    (tmp / 'Arduino.h').write_text('#pragma once\n#include <cstdint>\nuint32_t millis();\nvoid vTaskDelay(uint32_t);\n', encoding='utf8')
    (tmp / 'WebServer.h').write_text('''#pragma once
#include <Arduino.h>
#include <cstddef>
using PGM_P = const char*;
class Client { public: int fd(); void stop(); };
class WebServer {
 public: explicit WebServer(int) {} virtual ~WebServer() = default;
 protected: bool _server = true; Client _currentClient;
 virtual size_t _currentClientWrite(const char*, size_t) { return 0; }
 virtual size_t _currentClientWrite_P(PGM_P, size_t) { return 0; }
};
''', encoding='utf8')
    (tmp / 'lwip/sockets.h').write_text('#pragma once\n#include <cstddef>\n#define MSG_DONTWAIT 0x40\nint send(int,const void*,size_t,int);\n', encoding='utf8')
    (tmp / 'test.cpp').write_text(fixture, encoding='utf8')
    executable = tmp / 'test.exe'
    subprocess.run(['g++', '-std=c++11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I'+str(tmp), '-I'+str(ROOT/'src'), str(tmp/'test.cpp'),
                    str(ROOT/'src/bounded_web_server.cpp'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
