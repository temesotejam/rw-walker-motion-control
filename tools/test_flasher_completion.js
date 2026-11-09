'use strict';
const assert = require('assert');
const {patchFlasherCompletion, patchFlasherAutoStart, installFlasherPatches} = require('../site/flasher-completion.js');
class Dialog { _renderInstall() { return this._client; } }
assert(patchFlasherCompletion(Dialog));
const d = new Dialog();
d._manifest = {new_install_improv_wait_time: 0};
for (const state of ['initializing', 'preparing', 'erasing', 'writing', 'error']) {
  d._client = undefined; d._installState = {state};
  assert.strictEqual(d._renderInstall(), undefined);
}
d._installState = {state:'finished'};
assert.strictEqual(d._renderInstall(), null);
d._client = undefined; d._manifest.new_install_improv_wait_time = 10;
assert.strictEqual(d._renderInstall(), undefined);
assert.strictEqual(patchFlasherCompletion(class {}), false);
console.log('Flasher completion waits for FINISHED and only skips disabled Improv PASS');

// Emulate the pinned 10.1.1 lifecycle, including its willUpdate reset when
// INSTALL is entered. No serial hardware or actual flash is touched.
class Installer {
  constructor() {
    this.port = {readable: {}, writable: {}};
    this._state = 'DASHBOARD';
    this._manifest = {new_install_improv_wait_time: 0};
    this._client = undefined;
    this.calls = [];
    this.logger = {error: () => {}};
  }
  async _initialize() { this._client = null; this.calls.push('initialized'); }
  _startInstall(erase) {
    this.calls.push(['start', erase]); this._state = 'INSTALL';
    this.updateComplete = Promise.resolve().then(() => {
      this._installConfirmed = false; this.calls.push('updated');
    });
  }
  async _confirmInstall() {
    assert.strictEqual(this._installConfirmed, false);
    this.calls.push('confirm'); this._installConfirmed = true;
    if (this.fail) throw new Error('port close failed');
  }
  _renderInstall() { return this._client; }
}
async function testAutomaticInstall() {
  assert(patchFlasherAutoStart(Installer));
  assert(patchFlasherAutoStart(Installer)); // no duplicate wrappers
  const i = new Installer();
  await i._initialize();
  assert.deepStrictEqual(i.calls, ['initialized', ['start', false], 'updated', 'confirm']);
  await i._initialize(true); // completion does not flash again
  await i._initialize(false); // retry/error path remains under user control
  assert.strictEqual(i.calls.filter(x => x === 'confirm').length, 1);
  const badPort = new Installer(); badPort.port.readable = null;
  await badPort._initialize(); assert.deepStrictEqual(badPort.calls, ['initialized']);
  const other = new Installer(); other._manifest.new_install_improv_wait_time = 1;
  await other._initialize(); assert.deepStrictEqual(other.calls, ['initialized']);
  const broken = new Installer(); broken.fail = true;
  await broken._initialize();
  assert.strictEqual(broken._state, 'ERROR'); assert(broken._error);
  await broken._initialize(false);
  assert.strictEqual(broken.calls.filter(x => x === 'confirm').length, 1);

  class Delayed extends Installer {}
  const definitions = new Map();
  const registry = {get: name => definitions.get(name),
    define(name, ctor) { definitions.set(name, ctor); }};
  const define = registry.define;
  installFlasherPatches(registry);
  registry.define('other-widget', class {});
  registry.define('ewt-install-dialog', Delayed);
  assert.strictEqual(registry.define, define);
  const delayed = new Delayed(); await delayed._initialize();
  assert.strictEqual(delayed.calls.filter(x => x === 'confirm').length, 1);
  assert.strictEqual(patchFlasherAutoStart(class {}), false);
  console.log('Automatic install: port selected, no full-chip erase, one confirmation PASS');
}
testAutomaticInstall().catch(error => { console.error(error); process.exitCode = 1; });
