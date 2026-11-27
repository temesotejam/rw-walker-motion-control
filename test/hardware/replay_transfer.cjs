'use strict';

// Run against the isolated --measure fixture only. All measurements share one
// boot, one browser tab and one IndexedDB, as in consecutive operator trials.
const assert = require('node:assert/strict');
const { createHash } = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');
const { crc32 } = require('node:zlib');

const origin = 'http://192.168.4.1';
const optimizedBuild = 'walking-support-transfer-v39-walk7-logfix1-measure-bench';
const build = process.env.RWLOG_BENCH_BUILD || optimizedBuild;
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const allowedGets = new Set(['/', '/status.json', '/export/manifest', '/export/chunk']);
const allowedPosts = new Set(['/bench/replay', '/clear', '/export/prepare', '/export/pause', '/export/complete']);

async function main() {
  const [source, directory, countText = '3', ...extra] = process.argv.slice(2);
  const count = Number(countText);
  assert.match(build, /^walking-support-transfer-v39-walk7(?:-logfix[0-9]+)?-measure-bench$/);
  if (!source || !directory || !Number.isInteger(count) || count < 0 || count > 50 || (!count && !extra.length) ||
      new Set(extra).size !== extra.length || extra.some(s => !['--interrupt', '--reload', '--observer'].includes(s))) {
    throw Error('Usage: node replay_transfer.cjs <source.rwlog> <new-output-directory> [clean-run-count] [--interrupt] [--reload] [--observer]');
  }
  const scenarios = [...Array(count).fill('clean'), ...extra.map(flag => flag.slice(2))];
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
  await page.addInitScript(() => {
    window.benchTransactions = { readonly: 0, readwrite: 0 };
    window.benchStatusSentWhileDownloading = 0;
    const fetch = window.fetch;
    window.fetch = function (input, ...options) {
      if (typeof input === 'string' && input.startsWith('/status.json') &&
          typeof transferRunning !== 'undefined' && transferRunning) ++window.benchStatusSentWhileDownloading;
      return fetch.call(this, input, ...options);
    };
    const transaction = IDBDatabase.prototype.transaction;
    IDBDatabase.prototype.transaction = function (store, mode = 'readonly', ...options) {
      const result = transaction.call(this, store, mode, ...options);
      if (this.name === 'freefoot-rwlog-v2' && mode in window.benchTransactions) ++window.benchTransactions[mode];
      return result;
    };
  });
  let run = null, identified = false, transferPrepared = false, readingChunks = false, resumingReload = false;
  let observer = null, allowObserverClear = false;
  try {
    page.on('pageerror', err => report.pageErrors.push(String(err)));
    page.on('response', response => {
      if (run?.startedMs && !run.finished && response.ok() && new URL(response.url()).pathname === '/status.json') {
        const now = Date.now();
        run.maxStatusGapMs = Math.max(run.maxStatusGapMs, now - (run.lastStatusMs || run.startedMs));
        run.lastStatusMs = now;
        ++run.statusResponsesDuringAcquisition;
      }
    });
    page.on('requestfailed', req => {
      if (run) run.httpFailures.push({ at: new Date().toISOString(),
        path: new URL(req.url()).pathname, error: req.failure()?.errorText });
    });
    await ctx.route('**/*', async route => {
      const req = route.request(), url = new URL(req.url()), p = url.pathname;
      if (url.origin === origin && p === '/favicon.ico') return route.fulfill({ status: 204, body: '' });
      if (observer && req.frame().page() === observer) {
        if (url.origin === origin && req.method() === 'GET' && (p === '/' || p === '/status.json')) {
          if (p === '/status.json' && readingChunks) ++run.observerStatusRequests;
          return route.continue();
        }
        if (url.origin === origin && req.method() === 'POST' && p === '/clear' &&
            allowObserverClear && identified && build === optimizedBuild && readingChunks) {
          allowObserverClear = false;
          return route.continue();
        }
        report.rejected.push('observer ' + req.method() + ' ' + url.href);
        return route.abort('blockedbyclient');
      }
      if (url.origin !== origin || !(req.method() === 'GET' && allowedGets.has(p) ||
          identified && req.method() === 'POST' && allowedPosts.has(p))) {
        report.rejected.push(req.method() + ' ' + url.href);
        return route.abort('blockedbyclient');
      }
      if (p === '/export/prepare') transferPrepared = true;
      if (p === '/' || p === '/export/complete' || p === '/export/pause') transferPrepared = readingChunks = false;
      if (p === '/status.json' && readingChunks) run.statusRoutesBetweenChunks.push({stage: run.stage, at: Date.now()});
      if (p === '/export/chunk' && run) {
        readingChunks = transferPrepared;
        const offset = Number(url.searchParams.get('offset'));
        if (resumingReload && run.resumedAt == null) run.resumedAt = offset;
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
    const states = async (target = page) => target.evaluate(() => ({
      state: latest.state, run: latest.run_id, boot: latest.boot_id, build: latest.diagnostic_build_id,
      samples: latest.samples, motor: latest.motor_mA, actual: latest.actual_mA,
      parked: latest.export_camera_parked, priority: latest.export_priority_active, phase: latest.export_phase,
      seenAt: lastSeen, cameraFrames: latest.foot?.captured_frames, cameraFps: latest.foot?.fps,
      frameValid: latest.foot?.frame_valid, timestampValid: latest.foot?.frame_timestamp_valid,
      driverToReadyMs: latest.foot?.frame_to_ready_ms, driverToControlMs: latest.foot?.driver_to_control_ms,
      rowCacheReady: latest.foot?.row_cache_ready,
      rowCacheInternal: latest.foot?.row_cache_internal, hotScratchInternal: latest.foot?.hot_scratch_internal,
      feed: latest.foot_periodic?.count
    }));
    const ready = () => page.waitForFunction(({ build, boot }) => latest?.state === 'READY_TO_MEASURE' &&
      latest.diagnostic_build_id === build && latest.boot_id === boot && latest.motor_mA === 0 &&
      !latest.running && !latest.downloadable && !transferRunning && !commandInFlight && latest.samples === 0,
    { build, boot: initial.boot }, { timeout: 35000 });
    for (const [n, scenario] of scenarios.entries()) {
      await ready();
      resumingReload = false;
      run = { cycle: n + 1, scenario, interrupt: scenario === 'interrupt',
        httpFailures: [], statusRoutesBetweenChunks: [], before: await states() };
      const firstBootRun = build === optimizedBuild && run.before.run === 0;
      if (firstBootRun) assert.equal(run.before.hotScratchInternal, true, 'Expected internal scratch before the first run');
      if (scenario === 'observer') {
        run.observerStatusRequests = 0;
        observer = await ctx.newPage();
        observer.on('response', response => {
          if (readingChunks && new URL(response.url()).pathname === '/status.json')
            (run.observerStatusCodes ??= []).push(response.status());
        });
        await observer.goto(origin, { waitUntil: 'domcontentloaded', timeout: 25000 });
        await observer.waitForFunction(({build, boot}) => typeof latest !== 'undefined' &&
          latest?.diagnostic_build_id === build && latest.boot_id === boot && latest.motor_mA === 0,
        { build, boot: initial.boot }, { timeout: 30000 });
      }
      report.runs.push(run);
      const expectedId = run.before.run + 1;
      if (n === 0 && run.before.run === 0) assert.equal(expectedId, firstArchiveId);
      run.stage = 'ready';
      save();
      const startTime = Date.now();
      run.startedMs = startTime;
      run.maxStatusGapMs = run.statusResponsesDuringAcquisition = 0;
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
      assert.ok(run.acquiring.cameraFrames > run.running.cameraFrames);
      assert.ok(run.acquiring.frameValid && run.acquiring.timestampValid);
      save();
      await page.waitForFunction(id => latest?.run_id === id && latest.state === 'FINISHED' &&
        !latest.running && latest.downloadable && !document.querySelector('#download').disabled,
      run.running.run, { timeout: 115000 });
      run.finished = await states();
      run.maxStatusGapMs = Math.max(run.maxStatusGapMs, Date.now() - (run.lastStatusMs || startTime));
      assert.equal(run.finished.samples, sampleCount);
      assert.equal(run.finished.boot, initial.boot);
      assert.equal(run.finished.motor, 0);
      run.startToObservedFinishSeconds = (Date.now() - startTime) / 1000;
      run.stage = 'finished';
      save();
      if (observer) {
        run.stage = 'loading_finished_page';
        save();
        const started = Date.now();
        const document = await observer.reload({ waitUntil: 'domcontentloaded', timeout: 45000 });
        assert.equal(document.status(), 200);
        const html = await document.body();
        assert.match(html.subarray(-128).toString().trimEnd(), /<\/script>\s*<\/html>$/i);
        run.finishedPage = { seconds: (Date.now() - started) / 1000, htmlBytes: html.length,
          responseBytes: Number(document.headers()['content-length']), encoding: document.headers()['content-encoding'] };
        await observer.waitForFunction(({ build, boot, id }) => typeof latest !== 'undefined' &&
          latest?.diagnostic_build_id === build && latest.boot_id === boot && latest.run_id === id &&
          latest.state === 'FINISHED' && latest.motor_mA === 0 && latest.downloadable &&
          typeof window.document.querySelector('#download')?.onclick === 'function',
        { build, boot: initial.boot, id: expectedId }, { timeout: 35000 });
        run.finishedPage.statusReadySeconds = (Date.now() - started) / 1000;
        run.finishedPage.state = await states(observer);
        if (build === optimizedBuild) {
          assert.equal(run.finishedPage.state.rowCacheReady, true);
          assert.equal(run.finishedPage.state.rowCacheInternal, false);
          assert.equal(run.finishedPage.state.hotScratchInternal, false);
          assert.equal(run.finishedPage.state.priority, false);
        }
        save();
      }
      const downloaded = page.waitForEvent('download', { timeout: 480000 });
      downloaded.catch(() => {}); // A UI error should be reported before the download timeout.
      await page.evaluate(() => {
        window.benchTransactions.readonly = window.benchTransactions.readwrite = window.benchStatusSentWhileDownloading = 0;
      });
      await page.locator('#download').click();
      run.stage = 'downloading';
      save();
      if (scenario === 'observer' && build === optimizedBuild) {
        // Protect a live log only on the patched, motor-free fixture. Baseline
        // firmware would accept Clear and destroy the unfinished trial.
        await page.waitForFunction(bytes => transferDiagnostic?.confirmed_bytes >= bytes / 4,
          sourceBytes.length, { timeout: 240000 });
        allowObserverClear = true;
        run.observerClear = await observer.evaluate(async () => {
          const r = await fetch('/clear', { method: 'POST', signal: AbortSignal.timeout(10000) });
          return { status: r.status, body: await r.text() };
        });
        assert.equal(run.observerClear.status, 409, JSON.stringify(run.observerClear));
        save();
      }
      if (scenario === 'reload') {
        // Abrupt document loss without Pause. Only durable browser data will
        // be available when the new document opens the same sealed log.
        await page.waitForFunction(bytes => transferDiagnostic?.confirmed_bytes >= bytes / 2,
          sourceBytes.length, { timeout: 240000 });
        run.beforeReload = await page.evaluate(() => ({ confirmed: transferDiagnostic.confirmed_bytes,
          token: transferDiagnostic.manifest.token, startedAt: transferDiagnostic.started_at,
          storageTransactions: window.benchTransactions, statusWhileDownloading: window.benchStatusSentWhileDownloading }));
        assert.ok(run.beforeReload.confirmed < sourceBytes.length);
        run.stage = 'reloading';
        save();
        identified = transferPrepared = readingChunks = false;
        await page.reload({ waitUntil: 'domcontentloaded', timeout: 25000 });
        await page.waitForFunction(({build, boot, id}) => typeof latest !== 'undefined' &&
          latest?.diagnostic_build_id === build && latest.boot_id === boot && latest.run_id === id &&
          latest.downloadable && latest.state === 'FINISHED' && latest.motor_mA === 0 &&
          !document.querySelector('#download').disabled, {build, boot: initial.boot, id: expectedId}, { timeout: 35000 });
        assert.equal(await page.evaluate(() => performance.getEntriesByType('navigation')[0]?.type), 'reload');
        identified = true;
        resumingReload = true;
        await page.locator('#download').click();
        run.stage = 'resuming_after_reload';
        save();
      }
      await page.waitForFunction(id => transferDiagnostic?.run_id === id && !transferRunning &&
        ['complete', 'error', 'paused'].includes(transferDiagnostic.phase), run.finished.run, { timeout: 480000 });
      const { diag, feet } = await page.evaluate(() => ({ diag: transferDiagnostic, feet: completedFoot?.length }));
      run.transfer = diag;
      run.storageTransactions = await page.evaluate(() => window.benchTransactions);
      // Playwright may dispatch a callback from the previous document after
      // navigation. Attribute actual fetches to each document at issuance.
      run.statusRequestsDuringChunks = (run.beforeReload?.statusWhileDownloading || 0) +
        await page.evaluate(() => window.benchStatusSentWhileDownloading);
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
      if (firstBootRun) assert.equal(diag.manifest.export_camera_released_bytes, 12800);
      if (scenario === 'reload') {
        assert.equal(diag.manifest.token, run.beforeReload.token);
        assert.ok(run.resumedAt >= run.beforeReload.confirmed && run.resumedAt < actual.length);
        assert.equal(diag.cached_bytes, run.resumedAt, 'Persistent cache must cover precisely the prefix skipped on the wire');
      } else {
        assert.equal(diag.cached_bytes, 0, 'A new trial reused data from another download');
      }
      assert.equal(diag.confirmed_bytes, expected.length);
      assert.equal(diag.received_bytes, expected.length - diag.cached_bytes);
      assert.equal(diag.completion_ack, true);
      assert.equal(run.statusRequestsDuringChunks, 0, 'This page issued status requests during chunk transfer');
      if (scenario === 'observer') assert.ok(run.observerStatusRequests > 0, 'Observer did not poll during transfer');
      if (n > 0) assert.ok(report.runs.slice(0, n).every(prev => prev.transfer.manifest.token !== diag.manifest.token));
      if (run.interrupt) {
        assert.ok(run.injectedAt > 0);
        assert.equal(run.resumedAt, run.injectedAt);
      }
      const completeAt = Date.parse(diag.updated_at);
      run.transferSeconds = (completeAt - Date.parse(run.beforeReload?.startedAt || diag.started_at)) / 1000;
      if (scenario === 'reload') run.resumeSeconds = (completeAt - Date.parse(diag.started_at)) / 1000;
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
      run.stage = 'waiting_for_camera_restore';
      save();
      assert.ok(Number.isFinite(run.afterClear.cameraFrames), 'No camera counter after Clear');
      const observations = run.cameraAfterClear = [];
      let previous = run.afterClear;
      for (let i = 0; i < 3; ++i) {
        await page.waitForFunction(frames => latest.foot?.captured_frames > frames,
          previous.cameraFrames, { timeout: 20000 });
        const fresh = await states();
        observations.push(fresh);
        assert.equal(fresh.state, 'READY_TO_MEASURE');
        assert.equal(fresh.boot, initial.boot);
        assert.equal(fresh.motor, 0);
        assert.equal(fresh.actual, 0);
        assert.ok(fresh.frameValid && fresh.timestampValid && fresh.seenAt > previous.seenAt);
        previous = fresh;
      }
      run.resumedCamera = previous;
      const first = observations[0];
      run.resumedCameraAverageHz = 1000 * (previous.cameraFrames - first.cameraFrames) /
        (previous.seenAt - first.seenAt);
      assert.ok(run.resumedCameraAverageHz >= 40, 'Camera frame count after Clear averaged ' +
        run.resumedCameraAverageHz.toFixed(2) + ' Hz; expected at least 40 Hz');
      run.stage = 'ready_for_next';
      if (observer) {
        await observer.close();
        observer = null;
      }
      save();
      console.log(JSON.stringify({ cycle: run.cycle, run: run.finished.run, bytes: run.bytes,
        seconds: run.transferSeconds, retries: diag.retries, fallbacks: diag.stream_fallbacks || 0,
        cut: run.injectedAt, resumed: run.resumedAt, cached: diag.cached_bytes,
        resumeSeconds: run.resumeSeconds, statusDuringChunks: run.statusRequestsDuringChunks, phase: run.afterClear.phase }));
    }
    assert.deepEqual(report.pageErrors, []);
    assert.deepEqual(report.rejected, []);
    report.ok = true;
  } catch (err) {
    report.error = String(err.stack || err);
    report.lastUI = await page.evaluate(() => ({
      build: typeof latest !== 'undefined' && latest?.diagnostic_build_id,
      state: typeof latest !== 'undefined' && latest?.state,
      samples: typeof latest !== 'undefined' && latest?.samples,
      cameraFrames: typeof latest !== 'undefined' && latest?.foot?.captured_frames,
      rowCacheInternal: typeof latest !== 'undefined' && latest?.foot?.row_cache_internal,
      hotScratchInternal: typeof latest !== 'undefined' && latest?.foot?.hot_scratch_internal,
      statusAgeMs: typeof lastSeen !== 'undefined' && lastSeen ? Date.now() - lastSeen : null,
      statusFailure: typeof lastStatusFailure !== 'undefined' && lastStatusFailure,
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
