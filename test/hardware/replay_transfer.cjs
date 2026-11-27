'use strict';

// Run against the isolated --measure fixture only. All measurements share one
// boot, one browser tab and one IndexedDB, as in consecutive operator trials.
const assert = require('node:assert/strict');
const { createHash } = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const { crc32 } = require('node:zlib');

const origin = 'http://192.168.4.1';
const build = 'walking-support-transfer-v39-walk7-measure-bench';
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const allowedGets = new Set(['/', '/status.json', '/export/manifest', '/export/chunk']);
const allowedPosts = new Set(['/bench/replay', '/clear', '/export/prepare', '/export/pause', '/export/complete']);

async function main() {
  const [source, directory, countText = '3', extra] = process.argv.slice(2);
  const count = Number(countText);
  if (!source || !directory || !Number.isInteger(count) || count < 1 || count > 50 ||
      (extra !== undefined && extra !== '--interrupt')) {
    throw Error('Usage: node replay_transfer.cjs <source.rwlog> <new-output-directory> [clean-run-count] [--interrupt]');
  }
  const sourceBytes = fs.readFileSync(source);
  assert.equal(sourceBytes.subarray(0, 8).toString(), 'RWLOG01\0');
  assert.equal(sourceBytes.readUInt16LE(8), 56);
  assert.equal(crc32(sourceBytes.subarray(0, -4)), sourceBytes.readUInt32LE(sourceBytes.length - 4));
  const firstArchiveId = sourceBytes.readUInt32LE(12);
  const sampleCount = sourceBytes.readUInt32LE(28);
  const metadataAt = sourceBytes.readUInt16LE(10);
  const metadata = JSON.parse(sourceBytes.subarray(metadataAt, metadataAt + sourceBytes.readUInt32LE(24)).toString());
  const footCount = metadata.foot_frames.rows.length;
  fs.mkdirSync(directory);  // Never overwrite a previous experiment.

  const req = name => require(name);
  let chromium;
  try {
    ({ chromium } = req('playwright'));
  } catch (error) {
    if (error.code !== 'MODULE_NOT_FOUND') throw error;
    ({ chromium } = req(path.join(process.env.LOCALAPPDATA, 'ms-playwright-go', '1.57.0', 'package')));
  }
  const browser = await chromium.launch({
    channel: 'msedge', headless: true,
    args: ['--disable-background-networking', '--no-proxy-server', '--no-first-run']
  });
  const report = { build, startedAt: new Date().toISOString(), archiveSha256: sha(sourceBytes),
    archiveBytes: sourceBytes.length, browser: browser.version(), runs: [], pageErrors: [], rejected: [] };
  const reportPath = path.join(directory, 'report.json');
  const pendingPath = reportPath + '.next';
  function save() {
    fs.writeFileSync(pendingPath, JSON.stringify(report, null, 2));
    // A Windows reader can briefly deny replacement. Keep the old valid
    // report plus the new checkpoint in that case and continue measuring.
    for (let attempt = 0; attempt < 3; ++attempt) {
      try {
        fs.renameSync(pendingPath, reportPath);
        return;
      } catch (err) {
        if (!['EPERM', 'EBUSY', 'EACCES'].includes(err.code)) throw err;
        if (attempt < 2) Atomics.wait(new Int32Array(new SharedArrayBuffer(4)), 0, 0, 25);
      }
    }
    console.warn('Report is open elsewhere; the latest complete checkpoint is at', pendingPath);
  }
  const ctx = await browser.newContext({ acceptDownloads: true, serviceWorkers: 'block' });
  const page = await ctx.newPage();
  let run = null, identified = false;
  try {
    page.on('pageerror', err => report.pageErrors.push(String(err)));
    page.on('requestfailed', req => {
      if (run) run.httpFailures.push({ path: new URL(req.url()).pathname, error: req.failure()?.errorText });
    });
    await ctx.route('**/*', async route => {
      const req = route.request(), url = new URL(req.url()), p = url.pathname;
      if (url.origin === origin && p === '/favicon.ico') return route.fulfill({ status: 204, body: '' });
      if (url.origin !== origin || !(req.method() === 'GET' && allowedGets.has(p) ||
          identified && req.method() === 'POST' && allowedPosts.has(p))) {
        report.rejected.push(req.method() + ' ' + url.href);
        return route.abort('blockedbyclient');
      }
      if (p === '/export/chunk' && run) {
        const offset = Number(url.searchParams.get('offset'));
        if (run.interrupt && run.injectedAt == null && offset >= sourceBytes.length / 2) {
          run.injectedAt = offset;
          return route.abort('connectionreset');
        }
        if (run.injectedAt != null && run.resumedAt == null) run.resumedAt = offset;
      }
      return route.continue();
    });
    await page.goto(origin, { waitUntil: 'domcontentloaded', timeout: 25000 });
    await page.waitForFunction(() => typeof latest !== 'undefined' && latest?.diagnostic_build_id, null, { timeout: 30000 });
    const initial = await page.evaluate(() => ({ build: latest.diagnostic_build_id, boot: latest.boot_id, motor: latest.motor_mA }));
    assert.equal(initial.build, build, 'Wrong board/firmware; no requests to change state were allowed');
    assert.equal(initial.motor, 0);
    assert.ok(Number.isInteger(initial.boot));
    identified = true;
    report.boot = initial.boot;
    save();
    const states = async () => page.evaluate(() => ({
      state: latest.state, run: latest.run_id, boot: latest.boot_id, build: latest.diagnostic_build_id,
      samples: latest.samples, motor: latest.motor_mA, actual: latest.actual_mA,
      parked: latest.export_camera_parked, priority: latest.export_priority_active, phase: latest.export_phase,
      cameraFrames: latest.foot?.captured_frames, feed: latest.foot_periodic?.count
    }));
    const ready = () => page.waitForFunction(({ build, boot }) => latest?.state === 'READY_TO_MEASURE' &&
      latest.diagnostic_build_id === build && latest.boot_id === boot && latest.motor_mA === 0 &&
      !latest.running && !latest.downloadable && !transferRunning && !commandInFlight && latest.samples === 0,
    { build, boot: initial.boot }, { timeout: 35000 });
    for (let n = 0; n < count + Number(extra === '--interrupt'); ++n) {
      await ready();
      run = { cycle: n + 1, interrupt: n === count, httpFailures: [], before: await states() };
      report.runs.push(run);
      const expectedId = run.before.run + 1;
      if (n === 0 && run.before.run === 0) assert.equal(expectedId, firstArchiveId);
      run.stage = 'ready';
      save();
      const startTime = Date.now();
      const reply = await page.evaluate(async () => {
        const res = await fetch('/bench/replay', { method: 'POST', signal: AbortSignal.timeout(6000) });
        return { status: res.status, text: await res.text() };
      });
      assert.equal(reply.status, 202, JSON.stringify(reply));
      run.stage = 'requested';
      save();
      await page.waitForFunction(() => latest?.running && latest.motor_mA === 0, null, { timeout: 20000 });
      run.running = await states();
      assert.equal(run.running.boot, initial.boot);
      assert.equal(run.running.build, build);
      assert.equal(run.running.run, expectedId);
      if (n > 0) assert.equal(run.running.run, report.runs[n - 1].finished.run + 1);
      run.stage = 'recording';
      save();
      await page.waitForFunction(snapshot => latest?.running && latest.run_id === snapshot.run &&
        latest.samples > snapshot.samples && latest.motor_mA === 0, run.running, { timeout: 15000 });
      run.acquiring = await states();
      save();
      await page.waitForFunction(id => latest?.run_id === id && latest.state === 'FINISHED' &&
        !latest.running && latest.downloadable && !document.querySelector('#download').disabled,
      run.running.run, { timeout: 55000 });
      run.finished = await states();
      assert.equal(run.finished.samples, sampleCount);
      assert.equal(run.finished.boot, initial.boot);
      assert.equal(run.finished.motor, 0);
      run.startToObservedFinishSeconds = (Date.now() - startTime) / 1000;
      run.stage = 'finished';
      save();
      const downloaded = page.waitForEvent('download', { timeout: 480000 });
      downloaded.catch(() => {}); // A UI error should be reported before the download timeout.
      await page.locator('#download').click();
      run.stage = 'downloading';
      save();
      await page.waitForFunction(id => transferDiagnostic?.run_id === id && !transferRunning &&
        ['complete', 'error', 'paused'].includes(transferDiagnostic.phase), run.finished.run, { timeout: 480000 });
      const { diag, feet } = await page.evaluate(() => ({ diag: transferDiagnostic, feet: completedFoot?.length }));
      run.transfer = diag;
      save();
      assert.equal(diag.phase, 'complete', diag.error);
      const item = await downloaded;
      const destination = path.join(directory, 'cycle-' + run.cycle + '.rwlog');
      await item.saveAs(destination);
      const actual = fs.readFileSync(destination);
      const expected = Buffer.from(sourceBytes);
      expected.writeUInt32LE(run.finished.run, 12);
      const crc = crc32(expected.subarray(0, -4));
      expected.writeUInt32LE(crc, expected.length - 4);
      run.sha256 = sha(actual);
      run.bytes = actual.length;
      run.crc32 = crc.toString(16).padStart(8, '0');
      run.footRows = feet;
      assert.ok(expected.equals(actual), 'File differs from this measurement’s expected header, archived rows, or CRC');
      assert.equal(footCount, feet);
      assert.equal(diag.manifest.crc32, crc);
      assert.equal(diag.manifest.run_id, run.finished.run);
      assert.equal(diag.manifest.boot_id, initial.boot);
      assert.equal(diag.cached_bytes, 0, 'A new trial reused data from another download');
      assert.equal(diag.confirmed_bytes, expected.length);
      assert.equal(diag.received_bytes, expected.length);
      assert.equal(diag.completion_ack, true);
      if (n > 0) assert.ok(report.runs.slice(0, n).every(prev => prev.transfer.manifest.token !== diag.manifest.token));
      if (run.interrupt) {
        assert.ok(run.injectedAt > 0);
        assert.equal(run.resumedAt, run.injectedAt);
      }
      run.transferSeconds = (Date.parse(diag.updated_at) - Date.parse(diag.started_at)) / 1000;
      run.stage = 'download_verified';
      save();
      const ackAt = Date.parse(diag.updated_at);
      await page.waitForFunction(({id, ackAt}) => lastSeen >= ackAt && latest?.run_id === id && latest.state === 'FINISHED' &&
        latest.export_priority_active === false && latest.export_camera_parked === false &&
        !document.querySelector('#clear').disabled, {id: run.finished.run, ackAt}, { timeout: 30000 });
      run.afterTransfer = await states();
      run.stage = 'export_released';
      save();
      await page.locator('#clear').click();
      await ready();
      run.afterClear = await states();
      assert.equal(run.afterClear.priority, false);
      assert.equal(run.afterClear.parked, false);
      if (Number.isFinite(run.afterClear.cameraFrames)) {
        await page.waitForFunction(prev => latest.foot?.captured_frames > prev, run.afterClear.cameraFrames,
          { timeout: 20000 });
        run.resumedCamera = await states();
      }
      run.stage = 'ready_for_next';
      save();
      console.log(JSON.stringify({ cycle: run.cycle, run: run.finished.run, bytes: run.bytes,
        seconds: run.transferSeconds, retries: diag.retries, fallbacks: diag.stream_fallbacks || 0,
        cut: run.injectedAt, resumed: run.resumedAt, phase: run.afterClear.phase }));
    }
    assert.deepEqual(report.pageErrors, []);
    assert.deepEqual(report.rejected, []);
    report.ok = true;
  } catch (err) {
    report.error = String(err.stack || err);
    report.lastUI = await page.evaluate(() => ({
      build: typeof latest !== 'undefined' && latest?.diagnostic_build_id,
      state: typeof latest !== 'undefined' && latest?.state,
      detail: document.querySelector('#transfer')?.textContent,
      diag: typeof transferDiagnostic !== 'undefined' && transferDiagnostic,
      command: typeof latest !== 'undefined' && latest?.command
    })).catch(() => null);
    throw err;
  } finally {
    save();
    await browser.close();
  }
}

main().catch(err => { console.error(err); process.exitCode = 1; });
