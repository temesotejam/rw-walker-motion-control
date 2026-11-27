'use strict';
// ESP Web Tools 10.1.1 reports FINISHED only after writeFlash, reset and
// transport disconnect succeed. Improv is disabled for this firmware, so
// do not keep its completion screen waiting for a serial port to reopen.
function patchFlasherCompletion(ctor) {
  const p = ctor && ctor.prototype;
  if (!p || typeof p._renderInstall !== 'function') return false;
  const original = p._renderInstall;
  p._renderInstall = function (...args) {
    if (this._installState && this._installState.state === 'finished' &&
        this._client === undefined && this._manifest &&
        this._manifest.new_install_improv_wait_time === 0) {
      this._client = null;
    }
    return original.apply(this, args);
  };
  return true;
}
// The version pinned in index.html initializes the dialog after port selection,
// then normally asks for Install and a second confirmation. Start just once,
// after initialization and Lit's state update, without erasing the entire chip.
function patchFlasherAutoStart(ctor) {
  const p = ctor && ctor.prototype;
  if (!p || typeof p._initialize !== 'function' ||
      typeof p._startInstall !== 'function' ||
      typeof p._confirmInstall !== 'function') return false;
  if (p.__atomAutoStartPatched) return true;
  const original = p._initialize;
  p._initialize = async function (justInstalled = false) {
    const result = await original.call(this, justInstalled);
    if (justInstalled || this.__atomAutoStarted || this._state !== 'DASHBOARD' ||
        this._client !== null || !this._manifest ||
        this._manifest.new_install_improv_wait_time !== 0 ||
        !this.port || !this.port.readable || !this.port.writable) return result;
    this.__atomAutoStarted = true;
    try {
      this._startInstall(false);
      // willUpdate resets confirmation when entering INSTALL; wait for it.
      await this.updateComplete;
      if (this._state === 'INSTALL') await this._confirmInstall();
    } catch (error) {
      this._state = 'ERROR';
      this._error = '書き込みを開始できませんでした。ポートを確認し、もう一度お試しください。';
      if (this.logger && typeof this.logger.error === 'function') this.logger.error(error);
    }
    return result;
  };
  p.__atomAutoStartPatched = true;
  return true;
}
function installFlasherPatches(registry) {
  const apply = (ctor) => {
    const completion = patchFlasherCompletion(ctor);
    const automatic = patchFlasherAutoStart(ctor);
    if (!completion || !automatic) console.warn('ESP Web Tools compatibility patch unavailable');
  };
  const existing = registry.get('ewt-install-dialog');
  if (existing) { apply(existing); return; }
  // Patch before the element is defined/upgraded, even if the lazy import
  // finishes immediately after the port chooser returns.
  const define = registry.define;
  const wrapped = function (name, ctor, ...rest) {
    if (name === 'ewt-install-dialog') {
      apply(ctor);
      if (registry.define === wrapped) registry.define = define;
    }
    return define.call(this, name, ctor, ...rest);
  };
  registry.define = wrapped;
}
if (typeof module !== 'undefined') module.exports = {patchFlasherCompletion, patchFlasherAutoStart, installFlasherPatches};
if (typeof customElements !== 'undefined') {
  installFlasherPatches(customElements);
}

