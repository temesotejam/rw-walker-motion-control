'use strict';
const assert = require('node:assert/strict');
const {UsbSerialMonitor} = require('../site/serial-monitor.js');
const turn = () => new Promise(resolve => setImmediate(resolve));
async function until(condition) {
  for (let i = 0; i < 200 && !condition(); ++i) await turn();
  assert.ok(condition(), 'async session did not reach expected state');
}
function port(text) {
  let finish;
  const state = {opens: 0, closes: 0, releases: 0, cancels: 0, writes:[], writeReleases:0};
  return {state, getInfo: () => ({usbVendorId: 0x303a, usbProductId: 0x1001}),
    async open() { ++state.opens; }, async close() { ++state.closes; },
    writable: {getWriter() { return {
      async write(bytes) { state.writes.push(new TextDecoder().decode(bytes)); },
      releaseLock() { ++state.writeReleases; }
    }; }},
    readable: {getReader() {
      let sent = false;
      return {
        async read() {
          if (!sent) { sent = true; return {value: new TextEncoder().encode(text), done: false}; }
          return new Promise(resolve => { finish = resolve; });
        },
        async cancel() { ++state.cancels; if (finish) finish({done: true}); },
        releaseLock() { ++state.releases; }
      };
    }}
  };
}
(async () => {
  const a = port('boot=1\n'), b = port('boot=2\n'), c = port('unrelated');
  assert.equal(UsbSerialMonitor.select([a, b], a, a.getInfo()).port, a);
  assert.equal(UsbSerialMonitor.select([b], a, a.getInfo()).port, b);
  assert.equal(UsbSerialMonitor.select([b, c], a, a.getInfo()).ambiguous, true);
  assert.equal(UsbSerialMonitor.select([b], a, {}).port, undefined);
  let devices = [a], listener, chooserCalls = 0, log = '', retries = 0;
  const serial = {
    async requestPort() { ++chooserCalls; return a; },
    async getPorts() { return devices; },
    addEventListener(name, fn) { assert.equal(name, 'disconnect'); listener = fn; }
  };
  const monitor = new UsbSerialMonitor(serial, {data: s => { log += s; }, sleep: async () => { ++retries; await turn(); }});
  const session = monitor.connect();
  await until(() => log.includes('boot=1'));
  assert.deepEqual(a.state.writes,['DIAG OFF\n']);
  await monitor.setDiagnostics(true);
  assert.equal(a.state.writes[1],'DIAG ON\n');
  monitor.diagnosticsEnabled=()=>true;
  devices = [b]; listener({target: a});
  await until(() => log.includes('boot=2'));
  assert.equal(log, 'boot=1\nboot=2\n'); // Reset does not clear previous evidence.
  assert.equal(chooserCalls, 1); // Only the explicit diagnostic selection is reapplied.
  assert.deepEqual(b.state.writes,['DIAG ON\n']);
  assert.equal(a.state.closes, 1); assert.equal(a.state.releases, 1);
  await monitor.disconnect(); await session;
  assert.equal(b.state.closes, 1); assert.equal(b.state.releases, 1);
  assert.deepEqual(b.state.writes,['DIAG ON\n','DIAG OFF\n']);
  assert.equal(b.state.writeReleases,2);
  const opens = b.state.opens; await turn(); assert.equal(b.state.opens, opens);
  assert.ok(retries >= 1);
  // Auto reconnect off: a disconnect completes instead of reopening the port.
  const d = port('single\n');
  serial.requestPort = async () => d;
  const once = new UsbSerialMonitor(serial, {autoReconnect: () => false});
  const oneSession = once.connect(); await until(() => once.reader !== null); await turn();
  listener({port: d}); await oneSession;
  assert.equal(d.state.opens, 1); assert.equal(d.state.releases, 1);
  // Open failure retries using only previously authorized ports, then allows stop.
  let failures = 0;
  const missing = port(''); missing.open = async () => { ++failures; throw new Error('device gone'); };
  serial.requestPort = async () => missing; devices = [missing];
  const retry = new UsbSerialMonitor(serial, {sleep: turn});
  const failedSession = retry.connect(); await until(() => failures >= 2);
  await retry.disconnect(); await failedSession;
  assert.equal(missing.state.closes, 0);
  console.log('USB monitor: disconnect/reconnect, log retention, cleanup, opt-out and open failure passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
