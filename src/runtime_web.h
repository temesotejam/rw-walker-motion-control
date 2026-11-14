#pragma once
#include <Arduino.h>
static const char RUNTIME_HTML[] PROGMEM = R"FREEFOOT(<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AtomS3R Free-foot</title>
<style>
:root{font-family:system-ui,sans-serif;color:#1d293d;background:#eef2f5;font-size:16px}*{box-sizing:border-box}body{max-width:950px;margin:auto;padding:20px}h1{font-size:1.65rem;margin-bottom:4px}h2{font-size:1.08rem}p{line-height:1.6}.muted{color:#546477;font-size:.88rem}.card{background:white;border-radius:14px;padding:20px;margin:16px 0;border:1px solid #d9e1e8}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:14px}.value{font-size:2rem;font-variant-numeric:tabular-nums;margin:4px 0}.label{font-size:.85rem;color:#546477}#state{font-size:1.35rem;overflow-wrap:anywhere}button{padding:13px 18px;border:0;border-radius:8px;background:#174b8e;color:white;font:inherit;cursor:pointer;margin:4px 4px 4px 0}button:disabled{opacity:.4;cursor:default}#stop{background:#b62032}#clear,#cancel{background:#58677a}code,pre{font-family:ui-monospace,monospace}pre{white-space:pre-wrap;font-size:.78rem;overflow-wrap:anywhere}#connection{font-weight:600}progress{width:100%;height:24px}table{width:100%;border-collapse:collapse}td,th{text-align:left;padding:9px 4px;border-bottom:1px solid #e1e6eb}canvas{width:100%;height:120px;background:#f3f6fa;border-radius:8px}#message{min-height:26px;color:#9c2636}a{color:#174b8e}
</style>
<h1>AtomS3R Free-foot</h1><div class="muted">左右揺動のエネルギー制御 · MEKF姿勢推定 · ZEROクロスで必要な入力時間を計算</div>
<p id="connection">接続を確認中…</p>
<p class="muted" id="firmware-revision">本体プログラムの版を確認中…</p>
<section class="card"><div class="grid"><div><div class="label">状態</div><div class="value" id="state">—</div></div><div><div class="label">残り時間</div><div class="value" id="remaining">—</div></div><div><div class="label">胴体の左右揺動 · MEKF</div><div class="value" id="pitch">—</div></div><div><div class="label">指令 / 実測電流</div><div class="value" style="font-size:1.5rem" id="current">—</div></div></div>
<p id="guide">起動後は静止させてください。LEDが点灯したら左右のpitchを基準に合わせ、機体を静止させます。足のマーカー認識は測定開始に不要です。</p>
<h2>比較用の保存姿勢（従来制御用）</h2>
<p>機体の左右の中心線が鉛直になるように保持し、下のボタンを押してください。約2秒間の左右角の平均を保存します。重心の偏りで自然に傾く姿勢とは区別してください。測定開始・ログ消去・再起動では基準を変えません。</p>
<button id="upright-reference-capture" disabled>今の姿勢を直立基準として保存</button>
<p id="upright-reference-status" role="status">保存した基準を確認中…</p>
  <p class="muted">1.0.40：12°では、入力前の回転数・電池電圧などから入力中と入力後の仕事を予測し、目標エネルギーに近づく電流と時間を選びます。負側の補助入力上限は回転数と電圧に応じて調整します。絶対角0°基準、直立基準の保存不要、各30秒測定です。まず同じ固定脚条件の12°で確認してください。</p>
<p class="muted">仕事に基づく制御は絶対角0°を原点とし、選んだ目標角を左右それぞれに適用します。この保存操作では原点を変更しません。保存姿勢は比較と従来方式で使用します。</p>
<h2>左右の偏りを動作中に自動補正</h2>
<p>「仕事に基づく制御」を開始すると、左右の実測ピークと仕事モデルの予測を比較し、左右別のエネルギー予測と等価な重心ずれを更新します。選んだ目標角に対応する左右のエネルギー計算に反映します。重心ずれの手動設定は不要です。表示するずれは損失やモデル誤差も含み、実際の重心位置そのものとは限りません。</p>
<p id="cg-status" role="status">自動補正の状態を確認中…</p>
<details><summary>初期値を手動で指定する場合</summary>
<p>通常は設定不要です。手動値は自動補正の初期値として使い、動作中は観測結果から更新します。＋の値は、画面の角度が＋に傾くほど重心が高くなる側へのずれです。</p>
<label>重心ずれ（mm、−2～＋2） <input id="cg-shift" type="number" min="-2" max="2" step="0.01" placeholder="測定値" disabled></label>
<button id="cg-save" type="button" disabled>重心ずれを保存</button>
<details><summary>自然な静止姿勢から推定する</summary>
<p>MEKF絶対角0°を機体の基準として扱います。機体を左右に支えず、ホイールを止めて自然に静止させ、下のボタンを押します。足裏の円弧上で釣り合う条件に限って推定します。中央の接地範囲内や固定具で支えている場合は、静止角だけでは重心ずれを特定できません。取得中は機体を動かさないでください。</p>
<button id="cg-capture" type="button" disabled>自然な静止姿勢を2秒測って推定</button>
</details>
<p class="muted">新しい手動初期値は絶対角基準で保存します。旧基準の手動値は引き継ぎません。未指定の場合は補正ゼロから自動更新します。</p>
</details>
<div id="work-energy-control">
  <h2 id="work-title">固定脚・ホイールの仕事を使う制御（30秒）</h2>
  <p id="work-description" class="muted">脚の前後移動を固定し、各ボタンで30秒測定します。正側は300・350・400 mA、最大100 msです。10°・12°の負側は、不足が実測でも続いた場合にエネルギーを加える補助入力を使います。補助入力は300 mA・上限25 msから段階的に最大100 ms、ブレーキは最大70 msです。補助入力の効き方は正側の仕事モデルから仮定した試験用で、負側での精度は未確認です。8°は従来の入力選択を使います。</p>
  <button id="work-start" type="button" disabled>目標±8°で30秒測定</button>
  <button id="work-start-10" type="button" disabled>目標±10°で30秒測定</button>
  <button id="work-start-12" type="button" disabled>目標±12°で30秒測定</button>
  <p id="work-start-status" class="muted" role="status" aria-live="polite">本体の開始条件を確認中…</p>
</div>
<div id="negative-energy-control">
  <h2>固定脚・新しいエネルギー制御（15秒）</h2>
  <p class="muted">脚の前後移動を固定し、目標8°・300 mAで測ります。正側は従来制御、負側は次の最大角度に必要なエネルギーから入力時間を選びます。今の姿勢で直立基準を保存済みなら、そのMEKF絶対角を学習時の数値に合わせる必要はありません。</p>
  <button id="inverse-start" type="button" disabled>新しい制御で15秒測定を開始</button>
</div>
<div id="negative-probe-extended-package">
  <h2>負側30・35・40 msの一括測定（30秒）</h2>
  <p class="muted">脚の前後移動を固定します。目標8°・300 mAで負側の入力時間を3条件で順序を変えて試します。正側は従来制御を続けます。負側ピークと次の正側ピークまで記録します。終了後はRWLOGを保存してください。</p>
  <button id="probe-extended" type="button" disabled>負側3条件の一括測定を開始</button>
</div>
<div id="positive-current-package">
  <h2>正側の電流3条件の一括測定（30秒）</h2>
  <p class="muted">脚の前後移動を固定します。目標8°、始動キックと負側は300 mA、負側の減速は40 msに固定します。正側だけ300・350・400 mAを3回ごとに順序を変えて試し、両側のピークと電圧・ホイールの動きを記録します。終了後はRWLOGを保存してください。</p>
  <button id="probe-positive-current" type="button" disabled>正側の電流3条件を測定</button>
</div>
<details id="negative-probe-packages"><summary>負側5条件の一括実験を行う（30秒）</summary>
  <p class="muted">入力時間0・10・15・20・25 msを5条件ずつ順序を変えて試す測定です。正側の制御は従来どおりです。毎回RWLOGを保存し、「ログを消去・次の測定へ」を押してから次の回を始めてください。</p>
  <button id="probe-batch" type="button" disabled>負側5条件の一括実験を開始</button>
</details>
<details><summary>従来制御や比較用の方式を測定する（15秒）</summary>
<p><label for="target">目標とする左右の最大角度 </label><select id="target" disabled aria-describedby="target-help" style="font:inherit;padding:10px 14px;border:1px solid #aab8c6;border-radius:8px;background:white"><option value="8" selected>8°</option><option value="10">10°</option><option value="12">12°</option></select></p>
<p class="muted" id="target-help">ZEROクロスの角速度から次の最大角度を予測し、選んだ目標に近づくよう電流の入力時間を0〜100 msで決めます。</p>
<p class="muted" id="run-target" role="status">目標角度は測定開始時に確定します。</p>
<p><label for="energy-model">ZEROクロス後の予測式 </label><select id="energy-model" disabled style="font:inherit;padding:10px 14px;border:1px solid #aab8c6;border-radius:8px;background:white"><option value="bridge" selected>旧到達角予測をエネルギーに換算</option><option value="work_control">ホイールの仕事から両側の入力を選ぶ（固定脚で試験）</option><option value="bridge_inverse">次ピークのエネルギーから負側入力を選ぶ（固定脚で試験）</option><option value="quadratic">現行の二次エネルギー式（比較用）</option></select></p>
<p class="muted">前者は旧版0.47.31の次ピーク角を位置エネルギーに換算し、不足分だけ正方向へ入力します。過剰時は入力を止めます。二次式を選ぶと従来の逆入力設定が使えます。</p>
<p class="muted">新しい方式は目標8°・300 mAで固定します。正側は従来制御、負側は速度・直前のピーク・電圧・ホイール回転数から次ピークのエネルギーを予測して入力時間を選びます。実測条件外では負側に入力しません。25 msを超える入力は実測ピークを確認しながら最大30 msまで1 msずつ増やします。脚を前後に動かさない条件で試してください。</p>
<p class="muted" id="run-model" role="status">予測式は測定開始時に確定します。</p>
<p><label for="input-current">指令電流 </label><input id="input-current" type="number" min="100" max="1200" step="10" value="300" disabled aria-describedby="input-current-help" style="font:inherit;padding:10px 14px;border:1px solid #aab8c6;border-radius:8px;width:110px"> mA</p>
<p class="muted" id="input-current-help">100〜1200 mA、10 mA刻み。初期値300 mAは予備データに近い条件です。始動キックと通常入力に同じ指令値を使います。</p>
<p><label for="brake-max">負側ブレーキ時間の上限 </label><select id="brake-max" disabled aria-describedby="brake-max-help" style="font:inherit;padding:10px 14px;border:1px solid #aab8c6;border-radius:8px;background:white"><option value="0">0 ms（ブレーキなし）</option><option value="20">20 ms</option><option value="50" selected>50 ms（従来条件）</option></select></p>
<p class="muted" id="brake-max-help">二次エネルギー式でのみ使用します。旧到達角予測の逆入力は同定されていないため、そちらではブレーキを使用しません。</p>
<p class="muted" id="run-brake" role="status">負側ブレーキ上限は測定開始時に確定します。</p>
<p><label for="peak-adaptation">予測誤差の学習補正 </label><select id="peak-adaptation" disabled style="font:inherit;padding:10px 14px;border:1px solid #aab8c6;border-radius:8px;background:white"><option value="0" selected>使用しない（式の比較）</option><option value="1">使用する</option></select></p>
<p class="muted">まず学習補正なし、目標8°・300 mAで式を比較してください。次ピーク角、入力時間、実電流とホイール速度をRWLOGで確認します。</p>
<p class="muted" id="run-adaptation" role="status">学習補正の設定は測定開始時に確定します。</p>
<input id="input-percent" type="hidden" value="0">
<p class="muted" id="run-percent" role="status">通常入力はZEROクロスで行います。</p>
<button id="start" disabled>通常制御の15秒測定を開始</button>
</details>
<p><button id="reconnect" style="display:none">終了・Web復帰を確認</button><button id="clear" disabled>ログを消去・次の測定へ</button></p>
<div id="message" role="status"></div><p class="muted">開始・終了のLED同期はそれぞれ5秒です。始動キックの後、MEKFの左右角と角速度でZEROクロスを検出します。</p></section>
<p class="muted">運転中もWi-Fiを維持し、Web更新を休止して本体で制御・観測・記録します。前後90°以上の傾斜で停止します。横倒しは姿勢STOPの対象にしません。姿勢を戻しても再始動しません。Wi-Fi接続と画面を保ったままお待ちください。終了後にWeb表示が復帰したらログを保存してください。Web復帰だけではログは消えません。本体の電源断・再起動では未取得のログが失われます。</p>
<section class="card"><h2>左右足角度 · 胴体に対する相対角</h2><div class="grid"><div><div class="label">右足 · 上段マーカー A</div><div class="value" id="right">—</div></div><div><div class="label">左足 · 下段マーカー B</div><div class="value" id="left">—</div></div><div><div class="label">カメラ実測 / 目標</div><div class="value" style="font-size:1.5rem" id="fps">— / 15 fps</div></div></div>
<canvas id="markers" width="640" height="120" aria-label="マーカー検出位置。上段が右足、下段が左足。"></canvas>
<p class="muted" id="foot-status">足角度は任意です。マーカーが見えれば起動ごとにゼロ点を確定します。</p><p class="muted">角度の正方向はマーカーが左へ動く方向です。検出失敗・古い画像・設定範囲外を区別して表示し、その状態もログに保存します。候補を区別できないときは角度を無効にします。足を地面に固定して胴体を前後に傾けると、この相対角は変わります。</p><details><summary>検出画像を確認</summary><p class="muted">待機中に更新される1枚の画像です。測定完了後は測定前の画像を保持します。ログ保存後に「次の測定へ」進むと画像更新を再開します。線は検出に使った帯、丸は選択位置、黄色は未確定の位置・別候補です。</p><button id="preview" disabled>検出画像を取得</button><button id="preview-save" disabled>画像付き診断を保存</button><p class="muted" id="preview-status">画像はまだ取得していません。</p><canvas id="preview-image" width="640" height="480" style="height:auto;display:none" aria-label="カメラ画像と同じフレームの検出位置"></canvas></details></section>
<section class="card"><h2>前後傾斜と足角度の比較</h2>
<p id="mekf-axes" class="muted">MEKFの更新を待っています。</p>
<p>両足を地面に固定して、胴体と両足が直立した姿勢を基準にします。基準の後、胴体を前後の片方向へ約5°・10°・15°・20°と傾け、10°・直立へ戻す順に、設定範囲内で記録してください。各姿勢で約3秒間静止します。</p>
<button id="pose-base" disabled>比較用の直立姿勢を取得</button><button id="pose-add" disabled>この姿勢を追加</button><button id="pose-cancel" disabled>姿勢取得を中止</button><button id="pose-save" disabled>比較JSONを保存</button><button id="pose-reset" disabled>保存後に比較をやり直す</button>
<p class="muted">この姿勢比較は記録用です。仕事に基づく制御の原点はMEKF絶対角0°に固定しています。</p>
<p id="pose-status" role="status">胴体と両足を直立させ、基準を取得してください。</p>
<div style="overflow-x:auto"><table><thead><tr><th>姿勢</th><th>胴体前後変化</th><th>右の差</th><th>左の差</th><th>確認</th></tr></thead><tbody id="pose-rows"></tbody></table></div>
<p class="muted">「差」は基準からの足相対角の変化＋胴体前後角の変化です。足を固定した平面運動では0°が目安になります。左右傾斜が大きい場合は参考値とします。各姿勢の取得では角度換算係数を変更しません。現在は固定2姿勢から求めた係数を使用しています。最大12姿勢の画像と診断をまとめて保存します。再読み込みする前に比較JSONを保存してください。</p>
</section>
<section class="card"><h2>測定ログ</h2><p>測定終了後にログを確定します。中断した場合は「取得・再開」で続きから取得できます。画面を再読み込みしても、端末に保存済みの部分を再利用します。</p>
<button id="download" disabled>RWLOGを取得・再開</button><button id="cancel" disabled>取得を一時停止</button><button id="csv" disabled>足角度CSVを保存</button>
<progress id="progress" value="0" max="1"></progress><div id="transfer" role="status">測定待ち</div><p class="muted">RWLOGにIMU・制御イベント・電流・LED同期・足角度をまとめて保存します。USB診断ログは書き込みページから保存できます。</p></section>
<section class="card"><h2>ホイールが機体に与えた仕事</h2>
<p>RWLOG取得後、ホイールの位置変化から求めた速度と、同じ時刻の機体角速度を使って計算します。正なら加振、負なら制動です。位置の読み取りは5 ms間隔を目標にします。慣性と軸の符号は暫定値です。</p>
<p><label>慣性モーメント［kg·m²］ <input id="work-inertia" type="number" value="0.000028" min="0.000000001" step="any"></label>
<label>生の正回転と機体の正方向 <select id="work-sign"><option value="-1" selected>逆向き（暫定）</option><option value="1">同じ向き</option></select></label></p>
<button id="work-calculate">取得したRWLOGから再計算</button><button id="work-pulses">入力別CSVを保存</button><button id="work-intervals">全区間CSVを保存</button><button id="work-kinematics">位置由来の速度CSVを保存</button>
<pre id="work-result" role="status">RWLOGを取得すると計算結果を表示します。</pre></section>
<details class="card"><summary>診断情報</summary><label><input type="checkbox" id="usb-diag">USB診断を有効にする</label><p class="muted">起動ごとにOFFです。運転中の変更はUSB側から行えます。メモリ・スタックの調査は有効時の待機中だけ行い、診断JSONには測定時刻付きで保持します。未測定のメモリ値は0です。</p><p class="muted">MEKFのroll・pitch、姿勢確認用クォータニオン、有効性・更新時刻を保存します。方位には独立したジャイロ推定を使用します。</p><button id="diagnostics">診断JSONを保存</button><pre id="diagnostic-view">—</pre></details>
<script>
'use strict';
// Pure analysis of frozen frame-delivery snapshots. No device writes or fits.
const PoseComparison = (() => {
  const limits = Object.freeze({samples:5, interval_ms:800, min_span_us:2500000,
    max_span_us:12000000, max_age_ms:300, max_sensor_age_us:300000,
    max_gyro_dps:2, max_accel_error_g:0.08, max_tilt_spread_deg:0.8,
    max_marker_spread_px:3, max_poses:12, max_trace:600});
  const wrap = x => ((x + 180) % 360 + 360) % 360 - 180;
  const mean = a => a.reduce((s,x) => s + x, 0) / a.length;
  const vector = v => Array.isArray(v) && v.length === 3 && v.every(Number.isFinite);
  function orientation(q) {
    if (!q || ![q.w,q.x,q.y,q.z].every(Number.isFinite)) throw Error('姿勢データが不完全です');
    const n = Math.hypot(q.w,q.x,q.y,q.z);
    if (Math.abs(n - 1) > 0.01) throw Error('姿勢データの長さが不正です');
    const [w,x,y,z] = [q.w,q.x,q.y,q.z].map(v => v / n);
    const up = [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)];
    const deg = 180 / Math.PI;
    return {roll_deg:Math.atan2(up[1],up[2])*deg,
      pitch_deg:Math.atan2(-up[0],Math.hypot(up[1],up[2]))*deg};
  }
  function sameSession(a,b) {
    return a.boot_id === b.boot_id && a.run_id === b.run_id && a.revision === b.revision &&
      a.right_zero_x === b.right_zero_x && a.left_zero_x === b.left_zero_x;
  }
  function validateFrame(m) {
    if (!m || !Number.isInteger(m.boot_id) || !Number.isInteger(m.run_id) ||
        typeof m.revision !== 'string' || m.state_id !== 2)
      throw Error('測定前の待機状態で取得してください');
    if (!m.timestamp_valid || !Number.isFinite(m.age_ms) || m.age_ms < 0 || m.age_ms > limits.max_age_ms ||
        !Number.isSafeInteger(m.frame_us) || !Number.isSafeInteger(m.delivered_us) || m.frame_us <= 0 ||
        m.delivered_us < m.frame_us || m.delivered_us - m.frame_us > limits.max_sensor_age_us ||
        !Number.isInteger(m.sequence) || !m.zero_ready || !m.right_valid || !m.left_valid ||
        !m.right?.valid || !m.left?.valid ||
        ![m.right_deg,m.left_deg,m.right_zero_x,m.left_zero_x,m.right.x,m.left.x].every(Number.isFinite))
      throw Error('左右マーカーと新しい画像を確認してください');
    const k = m.mekf, input = k?.inputs;
    if (!k?.valid || !k.fresh || k.frame !== 'mekf' || k.euler_order !== 'ZYX' ||
        !Number.isFinite(k.age_us) || k.age_us < 0 || k.age_us > limits.max_sensor_age_us ||
        !input?.valid || input.frame !== 'mekf' || input.axis_order !== 'xyz' ||
        !Number.isFinite(input.accel_age_us) || input.accel_age_us < 0 || input.accel_age_us > limits.max_sensor_age_us ||
        !vector(input.accel_g) || !vector(input.gyro_dps) || !vector(input.gyro_bias_dps))
      throw Error('新しいIMU診断が必要です。ファームウェアと接続を確認してください');
    orientation(k.quaternion);
    const rate = Math.hypot(...input.gyro_dps.map((v,i) => v - input.gyro_bias_dps[i]));
    if (rate > limits.max_gyro_dps || Math.abs(Math.hypot(...input.accel_g) - 1) > limits.max_accel_error_g)
      throw Error('動きを検出しました。姿勢を保って再取得してください');
  }
  function summarize(frames, baseline = false) {
    if (frames.length !== limits.samples) throw Error('静止姿勢の試料が不足しています');
    frames.forEach(validateFrame);
    const first = frames[0], last = frames[frames.length - 1];
    if (!frames.every(m => sameSession(m,first))) throw Error('再起動・測定・ゼロ点の変更を検出しました。基準を取り直してください');
    for (let i=1;i<frames.length;i++) {
      if (frames[i].sequence === frames[i-1].sequence || frames[i].frame_us <= frames[i-1].frame_us ||
          frames[i].delivered_us <= frames[i-1].delivered_us)
        throw Error('画像が更新されていません');
    }
    const span = last.delivered_us - first.delivered_us;
    if (span < limits.min_span_us || span > limits.max_span_us) throw Error('取得間隔が不適切です。再取得してください');
    const angles = frames.map(m => orientation(m.mekf.quaternion));
    const summary = {boot_id:first.boot_id, run_id:first.run_id, revision:first.revision,
      right_zero_x:first.right_zero_x, left_zero_x:first.left_zero_x,
      first_frame_us:first.frame_us, last_frame_us:last.frame_us, span_us:span,
      right_in_range:frames.every(m => m.right_in_range === true),
      left_in_range:frames.every(m => m.left_in_range === true), spread:{}};
    for (const [key,values,maxSpread] of [
      ...['roll_deg','pitch_deg'].map(key => [key,
        angles.map(a => angles[0][key] + wrap(a[key]-angles[0][key])),
        limits.max_tilt_spread_deg]),
      ['right_x',frames.map(m => m.right.x),limits.max_marker_spread_px],
      ['left_x',frames.map(m => m.left.x),limits.max_marker_spread_px]]) {
      summary[key] = mean(values); summary.spread[key] = Math.max(...values) - Math.min(...values);
      if ( summary.spread[key] > maxSpread) throw Error('姿勢または足の位置が動いています。静止させて再取得してください');
    }
    for (const side of ['right','left']) summary[side+'_deg'] = mean(frames.map(m => m[side+'_deg']));
    const accelRoll = frames.map(m => Math.atan2(m.mekf.inputs.accel_g[1],m.mekf.inputs.accel_g[2])*180/Math.PI);
    summary.accel_roll_deg = mean(accelRoll.map(v => accelRoll[0] + wrap(v-accelRoll[0])));
    if (baseline && (Math.abs(summary.roll_deg)>6 || Math.abs(summary.pitch_deg)>6 ||
        Math.abs(summary.right_deg)>3 || Math.abs(summary.left_deg)>3))
      throw Error('基準は胴体と両足を直立させて取得してください');
    return summary;
  }
  function compare(base,pose) {
    if (!sameSession(base,pose) || pose.first_frame_us <= base.last_frame_us)
      throw Error('基準との取得条件が変わりました。記録を保存し、比較をやり直してください');
    const delta = {roll_deg:wrap(pose.roll_deg-base.roll_deg), pitch_deg:wrap(pose.pitch_deg-base.pitch_deg),
      accel_roll_deg:wrap(pose.accel_roll_deg-base.accel_roll_deg),
      right_deg:pose.right_deg-base.right_deg, left_deg:pose.left_deg-base.left_deg};
    const flags = [];
    if (Math.abs(delta.pitch_deg)>2) flags.push('sideways_changed');
    if (!base.right_in_range || !pose.right_in_range || !base.left_in_range || !pose.left_in_range) flags.push('outside_range');
    if (Math.abs(delta.roll_deg)>30) flags.push('large_tilt');
    return {delta, right_residual_deg:delta.right_deg+delta.roll_deg,
      left_residual_deg:delta.left_deg+delta.roll_deg, planar_check:flags.every(f => f === 'outside_range'), flags};
  }
  return {limits, wrap, orientation, sameSession, validateFrame, summarize, compare};
})();
if (typeof module !== 'undefined') module.exports = PoseComparison;

/* Offline reaction-couple work. The same implementation runs on the device's
 * browser page, the public file reader, and in Node tests/CLI. No motor control. */
(function(root) {
  'use strict';
  const PI = Math.PI, DEG = PI / 180, RPM = 2 * PI / 60;
  function crc32(bytes) {
    let c = 0xffffffff;
    for (const b of bytes) {
      c ^= b;
      for (let j = 0; j < 8; ++j) c = (c >>> 1) ^ ((c & 1) ? 0xedb88320 : 0);
    }
    return (c ^ 0xffffffff) >>> 0;
  }
  function parse(bytes) {
    if (!(bytes instanceof Uint8Array) || bytes.length < 114) throw Error('RWLOGが短すぎます');
    const v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    if (new TextDecoder().decode(bytes.subarray(0, 7)) !== 'RWLOG01' || v.getUint16(10, true) !== 110)
      throw Error('RWLOGヘッダーが一致しません');
    const version=v.getUint16(8,true), rowSize=v.getUint16(40,true);
    if (!((version===55 && rowSize===151) || (version===56 && rowSize===172)))
      throw Error('RWLOG v55または位置を記録したv56が必要です。旧ログからは計算できません。');
    const count = v.getUint32(28, true), offset = v.getUint32(62, true);
    const crcOffset = v.getUint32(74, true), metaLength = v.getUint32(24, true);
    if (crcOffset + 4 !== bytes.length || offset !== 110 + metaLength ||
        offset + count * rowSize > crcOffset || offset + count * rowSize !== v.getUint32(66, true))
      throw Error('RWLOGの長さ・オフセットが不正です');
    if (!(v.getUint32(58, true) & 1) || crc32(bytes.subarray(0, crcOffset)) !== v.getUint32(crcOffset, true))
      throw Error('RWLOGのCRCが一致しません');
    const metadata = JSON.parse(new TextDecoder().decode(bytes.subarray(110, offset)));
    const origin = v.getUint32(16, true), rows = [];
    const time = x => (x - origin) | 0; // signed modular difference; runs are <35 min
    let previousTime = 0;
    for (let i = 0; i < count; ++i) {
      const p = offset + i * rowSize, u32 = j => v.getUint32(p + j, true);
      const rate = v.getInt16(p + 135, true), speed = v.getInt32(p + 114, true);
      const bodyTime = time(u32(137)), speedTime = time(u32(122));
      // The legacy measurement-boundary marker has log_time_us=0. Its sensor
      // clocks remain continuous; do not mistake that marker for a clock reset.
      const positionTime=version===56 ? time(u32(159)) : 0;
      const rowTime = Math.max(previousTime, u32(0), bodyTime, speedTime, positionTime);
      previousTime = rowTime;
      rows.push({t:rowTime, state:v.getUint8(p + 8), pulse_id:u32(9),
        body_t:bodyTime, body_stamp_valid:u32(137) !== 0,
        body_rate_dps:rate === -32768 ? NaN : rate / 100,
        speed_t:speedTime, speed_start:time(u32(118)), speed_end:time(u32(126)),
        speed_seq:u32(130), speed_valid:v.getUint8(p + 134) === 1,
        speed_rpm:speed === -2147483648 ? NaN : speed / 100,
        command_seq:u32(143), command_t:time(u32(147)),
        applied_mA:v.getInt16(p + 141, true),
        ...(version===56 ? {position_raw_cdeg:v.getInt32(p+151,true),
          position_start:time(u32(155)),position_t:positionTime,position_end:time(u32(163)),
          position_seq:u32(167),position_valid:v.getUint8(p+171)===1} : {})});
    }
    return {rows, metadata};
  }
  // Centered offline fit. Time is scaled to 10 ms and position is centered to
  // preserve numerical accuracy for long cumulative-angle records.
  function solve3(a) {
    for(let k=0;k<3;++k) {
      let pivot=k;
      for(let i=k+1;i<3;++i) if(Math.abs(a[i][k])>Math.abs(a[pivot][k])) pivot=i;
      if(Math.abs(a[pivot][k])<1e-12) return null;
      [a[k],a[pivot]]=[a[pivot],a[k]];
      const divisor=a[k][k];for(let j=k;j<4;++j)a[k][j]/=divisor;
      for(let i=0;i<3;++i) if(i!==k) {
        const f=a[i][k];for(let j=k;j<4;++j)a[i][j]-=f*a[k][j];
      }
    }
    return a.map(r=>r[3]);
  }
  function positionVelocity(positions, fitPoints=5) {
    if(![5,9].includes(fitPoints)) throw Error('位置近似の点数は5または9にしてください');
    const half=(fitPoints-1)/2, nodes=[], rejected={};
    let angle=0;
    const pp=positions.map((p,i)=>{
      if(i) angle+=((p.raw-positions[i-1].raw)|0)/100;
      return {...p,angle};
    });
    for(let k=half;k<pp.length-half;++k) {
      const p=pp[k], window=pp.slice(k-half,k+half+1);
      let reason='';
      for(let j=0;j<window.length;++j) {
        const w=window[j];
        if(!w.valid || !Number.isFinite(w.raw)) {reason='position_read_invalid';break;}
        if(w.duration<0 || w.duration>3000) {reason='position_read_too_long';break;}
        if(j) {
          const prev=window[j-1],dt=w.t-prev.t;
          if(((w.seq-prev.seq)>>>0)!==1) {reason='position_sequence_gap';break;}
          if(dt<=0 || dt>15000) {reason='position_time_gap';break;}
          // Refuse a per-revolution reset or implausible jump; signed-int32
          // counter rollover itself is handled above, never a 360-deg unwrap.
          if(Math.abs(w.angle-prev.angle)>6000*6*dt/1e6+1) {reason='position_discontinuity';break;}
        }
      }
      const sums=Array(5).fill(0),rhs=Array(3).fill(0);
      if(!reason) for(const w of window) {
        const x=(w.t-p.t)/10000,y=w.angle-p.angle;
        for(let i=0;i<5;++i)sums[i]+=x**i;
        for(let i=0;i<3;++i)rhs[i]+=y*x**i;
      }
      const c=reason ? null : solve3(Array.from({length:3},(_,i)=>[sums[i],sums[i+1],sums[i+2],rhs[i]]));
      if(!reason && !c) reason='position_fit_singular';
      const residual=c ? Math.sqrt(window.reduce((sum,w)=>{
        const x=(w.t-p.t)/10000;
        return sum+(w.angle-p.angle-c[0]-c[1]*x-c[2]*x*x)**2;
      },0)/window.length) : null;
      if(!reason && residual>.5) reason='position_fit_residual';
      if(reason) rejected[reason]=(rejected[reason]||0)+1;
      nodes.push({t:p.t,seq:p.seq,valid:!reason,reason,duration:p.duration,
        rpm:reason ? NaN : c[1]*100/6,position_deg:p.angle,
        fit_span_us:window[window.length-1].t-window[0].t,fit_rmse_deg:residual});
    }
    return {nodes,rejected};
  }
  function analyze(rows, metadata = {}, options = {}) {
    const inertia = options.inertia ?? metadata.wheel_work_inertia_kg_m2 ?? 2.8e-5;
    const sign = options.sign ?? metadata.wheel_speed_to_body_sign ?? -1;
    if (!Number.isFinite(inertia) || inertia <= 0 || (sign !== 1 && sign !== -1))
      throw Error('慣性モーメントは正の数、軸の符号は+1または−1にしてください');
    const maxSpeedGap = 30000, maxBodyGap = 25000, maxReadTime = 3000;
    const usePosition=metadata.wheel_work_velocity_source==='position_local_quadratic_5_points' ||
      rows.some(r=>Object.hasOwn(r,'position_seq'));
    const positionSign=metadata.wheel_position_to_speed_sign ?? 1;
    if(positionSign!==1 && positionSign!==-1) throw Error('位置と回転速度の符号が不正です');
    const fitPoints=options.positionFitPoints ?? 5;
    const gyros = new Map(), positions=[], speeds=[], commands = [], states = [];
    let lastSpeed, lastPosition, lastCommand, lastState;
    for (const r of rows) {
      if (r.body_stamp_valid && Number.isFinite(r.body_rate_dps) &&
          r.body_t <= r.t && r.t - r.body_t <= 10000)
        gyros.set(r.body_t, r.body_rate_dps * DEG);
      if (r.speed_seq !== lastSpeed && r.speed_seq !== 0) {
        speeds.push({t:r.speed_valid ? r.speed_t : (r.speed_start + r.speed_end)/2,
          rpm:r.speed_rpm, seq:r.speed_seq, valid:r.speed_valid,
          duration:r.speed_end-r.speed_start});
        lastSpeed = r.speed_seq;
      }
      if(usePosition && r.position_seq!==lastPosition && r.position_seq!==0 && r.position_seq!==undefined) {
        positions.push({t:r.position_valid ? r.position_t : (r.position_start+r.position_end)/2,
          raw:r.position_raw_cdeg,seq:r.position_seq,valid:r.position_valid,
          duration:r.position_end-r.position_start});
        lastPosition=r.position_seq;
      }
      if (r.command_seq !== lastCommand && r.command_seq !== 0) {
        commands.push({t:r.command_t, seq:r.command_seq, current:r.applied_mA, pulse_id:r.pulse_id});
        lastCommand = r.command_seq;
      }
      if (r.state !== lastState) { states.push({t:r.t, state:r.state}); lastState = r.state; }
    }
    const positionResult=usePosition ? positionVelocity(positions,fitPoints) : {nodes:[],rejected:{}};
    const wheels=usePosition ? positionResult.nodes.map(n=>({...n,rpm:n.rpm*positionSign})) : speeds;
    const body = [...gyros].sort((a,b) => a[0]-b[0]);
    commands.sort((a,b) => a.t-b.t);
    function bodyAt(t) {
      let lo=0, hi=body.length;
      while (lo<hi) {const m=(lo+hi)>>>1; if (body[m][0]<t) lo=m+1; else hi=m;}
      if (lo<body.length && body[lo][0]===t) return body[lo][1];
      if (!lo || lo===body.length) return null; // no extrapolation
      const a=body[lo-1], b=body[lo];
      if (b[0]-a[0]>maxBodyGap) return null;
      return a[1]+(b[1]-a[1])*(t-a[0])/(b[0]-a[0]);
    }
    const endTime = rows.length ? rows[rows.length-1].t : 0;
    const pulses=[];
    for (let i=0; i<commands.length; ++i) {
      const c=commands[i];
      if (!c.current) continue;
      const next=commands[i+1];
      pulses.push({index:pulses.length+1, pulse_id:c.pulse_id, command_sequence:c.seq,
        start_us:c.t, end_us:next ? next.t : endTime, current_mA:c.current,
        end_observed:!!next, observed_net_j:0, added_j:0, removed_j:0, covered_us:0,
        boundary_interpolated:false});
    }
    const intervals=[], rejected={};
    const total={observed_net_j:0, added_j:0, removed_j:0, covered_us:0, missing_us:0};
    const coast={observed_net_j:0, added_j:0, removed_j:0, covered_us:0};
    const active={observed_net_j:0, added_j:0, removed_j:0, covered_us:0};
    function stateAt(t) {
      let state=0;
      for (const s of states) {if (s.t>t) break; state=s.state;}
      return state;
    }
    function add(target,w,dt) {
      target.observed_net_j+=w; target.added_j+=Math.max(0,w);
      target.removed_j+=Math.max(0,-w); target.covered_us+=dt;
    }
    for (let i=1; i<wheels.length; ++i) {
      const a=wheels[i-1], b=wheels[i], dt=b.t-a.t;
      const oa=bodyAt(a.t), ob=bodyAt(b.t);
      let reason='';
      if (dt<=0) reason='nonmonotonic_speed_time';
      else if (!a.valid || !b.valid || !Number.isFinite(a.rpm) || !Number.isFinite(b.rpm))
        reason=usePosition ? 'position_velocity_unavailable' : 'speed_read_invalid';
      else if (((b.seq-a.seq)>>>0)!==1) reason='speed_sequence_gap';
      else if (dt>maxSpeedGap) reason='speed_time_gap';
      else if (a.duration<0 || b.duration<0 || a.duration>maxReadTime || b.duration>maxReadTime) reason='speed_read_too_long';
      else if (oa===null || ob===null) reason='body_rate_not_bracketed';
      const cuts=[a.t,b.t];
      for (const c of commands) if(c.t>a.t && c.t<b.t) cuts.push(c.t);
      for (const s of states) if(s.t>a.t && s.t<b.t) cuts.push(s.t);
      if (!reason && oa*ob<0) cuts.push(a.t+dt*(-oa)/(ob-oa));
      cuts.sort((x,y)=>x-y);
      const wa=sign*RPM*a.rpm+oa, wb=sign*RPM*b.rpm+ob;
      if (reason) rejected[reason]=(rejected[reason]||0)+1;
      for (let j=1; j<cuts.length; ++j) {
        const l=cuts[j-1], r=cuts[j];
        if (r<=l) continue;
        const midpoint=(l+r)/2, measuring=stateAt(midpoint)===3;
        const pulse=pulses.find(p=>midpoint>=p.start_us && midpoint<p.end_us);
        const fraction=(r-l)/dt;
        const omegaMean = reason ? null : oa+(ob-oa)*((midpoint-a.t)/dt);
        const delta = reason ? null : (wb-wa)*fraction;
        const w = reason ? null : -inertia*omegaMean*delta;
        intervals.push({start_us:l,end_us:r,measurement:measuring,
          pulse_index:pulse?.index ?? 0,command_current_mA:pulse?.current_mA ?? 0,
          valid:!reason,reason,body_mean_rad_s:omegaMean,wheel_abs_delta_rad_s:delta,
          body_torque_nm:reason ? null : -inertia*delta/((r-l)*1e-6),
          work_j:w,boundary_interpolated:l!==a.t || r!==b.t});
        if (measuring) {
          if (reason) total.missing_us+=r-l;
          else { add(total,w,r-l); add(pulse ? active : coast,w,r-l); }
        }
        if (pulse && !reason) {
          add(pulse,w,r-l);
          pulse.boundary_interpolated ||= l!==a.t || r!==b.t;
        }
      }
    }
    for (const p of pulses) {
      const duration=p.end_us-p.start_us;
      p.coverage=duration>0 ? Math.min(1,p.covered_us/duration) : 0;
      p.complete=p.end_observed && p.coverage>=0.999999;
      if (!p.covered_us) p.observed_net_j=p.added_j=p.removed_j=null;
    }
    // Include uncovered measurement edges, not only holes between wheel reads.
    let measurementUs=0;
    for (let i=0;i<states.length;++i)
      if(states[i].state===3) measurementUs+=Math.max(0,(states[i+1]?.t ?? endTime)-states[i].t);
    total.missing_us=Math.max(total.missing_us,measurementUs-total.covered_us,0);
    total.coverage=measurementUs>0 ? Math.min(1,total.covered_us/measurementUs) : 0;
    for (const result of [total,active,coast])
      if (!result.covered_us) result.observed_net_j=result.added_j=result.removed_j=null;
    const admissionV19=metadata.energy_model==='walking_energy_admission_v19' ||
      metadata.walking_energy?.revision==='walking_energy_admission_v19' || metadata.walking_admission?.revision==='walking_energy_admission_v19';
    const transferV18=metadata.energy_model==='walking_energy_transfer_v18' ||
      metadata.walking_energy?.revision==='walking_energy_transfer_v18' || metadata.walking_transfer?.revision==='walking_energy_transfer_v18';
    return {revision:usePosition ? 'reaction_couple_position_work_v2_20261016' : 'reaction_couple_work_v1_20261016',inertia_kg_m2:inertia,
      walking_control_revision:admissionV19 ? 'walking_energy_admission_v19' : transferV18 ? 'walking_energy_transfer_v18' : null,
      wheel_speed_to_body_sign:sign,provisional:true,readback_delay_compensated:false,
      velocity_source:usePosition ? 'position_local_quadratic' : 'legacy_filtered_rpm',
      legacy_filtered_rpm_warning:!usePosition,position_to_speed_sign:positionSign,
      position_fit_points:usePosition ? fitPoints : null,
      position_fit_rejected:positionResult.rejected,
      position_attempts:positions.length,position_velocity_samples:positionResult.nodes.filter(n=>n.valid).length,
      kinematics:usePosition ? wheels.map(n=>({time_us:n.t,sequence:n.seq,valid:n.valid,reason:n.reason,
        relative_position_deg:n.position_deg,relative_speed_rpm:n.valid ? n.rpm : null,
        fit_span_us:n.fit_span_us,fit_rmse_deg:n.fit_rmse_deg})) : [],
      body_energy_definition:'reaction_couple_work_excluding_rotor_absolute_axial_spin',
      measurement:total,active,coast,pulses,intervals,rejected,
      speed_attempts:speeds.length,body_samples:body.length,
      limits:{max_speed_gap_us:maxSpeedGap,max_body_bracket_gap_us:maxBodyGap,max_read_us:maxReadTime,
        max_position_fit_adjacent_gap_us:15000,position_fit_max_rmse_deg:.5}};
  }
  function fromBytes(bytes,options={}) {
    const {rows,metadata}=parse(bytes); return analyze(rows,metadata,options);
  }
  function csv(rows, report) {
    if (!rows.length) return '';
    if (report) rows=rows.map(r=>({inertia_kg_m2:report.inertia_kg_m2,
      wheel_speed_to_body_sign:report.wheel_speed_to_body_sign,velocity_source:report.velocity_source,
      position_fit_points:report.position_fit_points,provisional:true,...r}));
    const keys=Object.keys(rows[0]);
    const cell=v=>v==null?'': '"'+String(v).replace(/"/g,'""')+'"';
    return keys.map(cell).join(',')+'\r\n'+rows.map(r=>keys.map(k=>cell(r[k])).join(',')).join('\r\n');
  }
  function describe(r) {
    const mj=x=>x==null?'算出不可':(x*1000).toFixed(4)+' mJ';
    const lines=[`暫定の仕事：J=${r.inertia_kg_m2} kg·m²、軸の符号=${r.wheel_speed_to_body_sign}`,
      `測定中の観測できた正味の仕事：${mj(r.measurement.observed_net_j)}`,
      `加えた分 ${mj(r.measurement.added_j)} / 取り除いた分 ${mj(r.measurement.removed_j)}`,
      `指令適用中 ${mj(r.active.observed_net_j)} / 入力を切った区間 ${mj(r.coast.observed_net_j)}`,
      `測定区間の計算対象 ${(r.measurement.coverage*100).toFixed(1)}% / 欠測 ${(r.measurement.missing_us/1000).toFixed(1)} ms`,
      '入力後の区間も軸受などの反作用を含む仕事です。次のピークに残るエネルギーとは異なります。',
      r.velocity_source==='position_local_quadratic'
        ? `位置の${r.position_fit_points}点近似から速度を算出しています。位置が欠測した区間は速度読み値で補いません。`
        : '旧ログの速度読み値による参考値です。読み値の遅れは未補正です。入力中と入力後の配分は結論に使えません。',
      '指令の境界は線形補間しています。取得時刻は内部センサ時刻ではなく、慣性・軸の符号・平面近似にも依存します。',
      '', '各入力（指令を書き込めた時刻から、次の指令を書き込めた時刻まで）：'];
    if(r.walking_control_revision==='walking_energy_transfer_v18') lines.splice(6,0,
      'v18の救済完了予測は、戻り通過の要求に対する充足です。次ピークの到達エネルギーを表す値ではありません。',
      '主入力は実ZEROクロスで改めて計算し、救済と合わせて半周期80ms以内に制限します。');
    if(r.walking_control_revision==='walking_energy_admission_v19') lines.splice(6,0,
      'v19の救済完了予測は、戻り通過の要求に対する充足です。次ピークの到達エネルギーを表す値ではありません。',
      '主入力は実ZEROクロスで改めて計算し、救済と合わせて半周期80ms以内に制限します。',
      '前倒し候補のモデル支持を確認し、範囲外はZEROで再評価します。前後の持続逸脱では入力を停止します。');
    for(const p of r.pulses) lines.push(`${p.index}. ${p.current_mA} mA / ${((p.end_us-p.start_us)/1000).toFixed(1)} ms：${mj(p.observed_net_j)} / 計算対象 ${(p.coverage*100).toFixed(1)}%${p.complete?'':'（区間の一部が未取得）'}`);
    return lines.join('\n');
  }
  root.WheelWorkAnalyzer={parse,analyze,fromBytes,csv,describe,positionVelocity};
  if (typeof module!=='undefined') module.exports=root.WheelWorkAnalyzer;
})(typeof globalThis!=='undefined' ? globalThis : this);

'use strict';
const $ = id => document.getElementById(id);
let previewRunning = false, previewEvidence = null;
let poseRunning = false, poseCancelled = false, poseSession = null, poseSaved = false;
let latest = null, lastSeen = 0, refreshInFlight = false, commandInFlight = false;
let transferRunning = false, cancelTransfer = false, completedFile = null, completedName = '', completedFoot = null;
let wheelWorkReport = null;
function calculateWheelWork() {
  wheelWorkReport = null;
  try {
    if (!completedFile) throw Error('先にRWLOGを取得してください。');
    wheelWorkReport = WheelWorkAnalyzer.fromBytes(completedFile, {
      inertia:Number($('work-inertia').value), sign:Number($('work-sign').value)});
    $('work-result').textContent = completedName + '\n' + WheelWorkAnalyzer.describe(wheelWorkReport);
  } catch (error) { $('work-result').textContent = error.message; }
}
function saveWheelWorkCsv(kind) {
  if (!wheelWorkReport) { $('work-result').textContent = '先に仕事を計算してください。'; return; }
  const csv = WheelWorkAnalyzer.csv(wheelWorkReport[kind], wheelWorkReport);
  saveBlob(new Blob(['\ufeff', csv], {type:'text/csv;charset=utf-8'}),
    completedName.replace(/\.rwlog$/, `_wheel_work_${kind}.csv`));
}
let offlineMode = false, offlineUntil = 0;
let offlineGeneration = 0;
const offlineKey = 'freefoot-offline-run-until';
const targetChoices = [8, 10, 12];
let targetInitialized = false, targetBootId = null, requestedTargetDeg = null;
let angleInitialized = false, angleBootId = null, requestedInputPercent = null;
let currentInitialized = false, currentBootId = null, requestedInputCurrentMa = null;
let brakeInitialized = false, brakeBootId = null, requestedBrakeMaxMs = null;
const brakeChoices = [0, 20, 50];
const negativeWidthChoices = [0, 15, 20, 25, 30, 40, 50];
let adaptationInitialized = false, adaptationBootId = null, requestedAdaptation = null;
let modelInitialized = false, modelBootId = null, requestedEnergyModel = null;
const validInputCurrent = value => Number.isInteger(value) && value >= 100 && value <= 1200 && value % 10 === 0;
const validInputPercent = value => value === 0;
const nap = ms => new Promise(resolve => setTimeout(resolve, ms));
function crc32(data, crc = 0) {
  crc = ~crc;
  for (const b of data) {
    crc ^= b;
    for (let bit = 0; bit < 8; ++bit) crc = (crc >>> 1) ^ (0xedb88320 & -(crc & 1));
  }
  return (~crc) >>> 0;
}
async function request(path, {method = 'GET', kind = 'json', timeout = 3000} = {}) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeout);
  try {
    const response = await fetch(path, {method, signal: controller.signal, cache: 'no-store'});
    if (!response.ok) throw new Error(`${response.status}: ${await response.text()}`);
    // Abort remains armed through body consumption, including response.json().
    if (kind === 'bytes') return new Uint8Array(await response.arrayBuffer());
    if (kind === 'text') return await response.text();
    return await response.json();
  } finally { clearTimeout(timer); }
}
function walkingEnergyTrial(s = latest) {
  return walkingTransferTrial(s) || /^walking-return-energy-v17(?:-|$)/.test(s?.diagnostic_build_id || '');
}
function walkingTransferTrial(s = latest) {
  return walkingAdmissionTrial(s) || /^walking-energy-transfer-v18(?:-|$)/.test(s?.diagnostic_build_id || '');
}
function walkingAdmissionTrial(s = latest) {
  return /^walking-energy-admission-v19(?:-|$)/.test(s?.diagnostic_build_id || '');
}
function foreAftStopText(reason) {
  return ({
    fore_aft_outward_divergence:'前後の傾きが外向きに持続して増えたため、入力を停止しました。',
    fore_aft_invalid_sample:'前後の姿勢監視に必要なIMUデータが無効なため、入力を停止しました。',
    fore_aft_stale_sample:'前後の姿勢監視に必要なIMUの更新が古くなったため、入力を停止しました。',
    fore_aft_unobservable:'前後の姿勢を確認できないため、入力を見合わせています。',
    fore_aft_outward_candidate:'前後の外向きの動きを確認しているため、入力を見合わせています。'
  })[reason] || '';
}
function footStartBlockReason(s) {
  if (s.foot?.control_extension_deg && ['left_out_of_range','right_out_of_range'].includes(s.foot_control_reason)) {
    const side=s.foot_control_reason.startsWith('left')?'左足':'右足';
    return side + 'の位置が、校正端からの許容' + s.foot.control_extension_deg + '°を超えています。';
  }
  const messages = {
    camera_unavailable:'カメラの準備ができていません。',
    zero_not_ready:'足は検出されていても、足角度のゼロ点が未確定です。機体と足を静止させてください。',
    frame_zero_not_ready:'ゼロ点確定後の新しい画像を待っています。',
    frame_invalid:'足の画像を正常に取得できていません。',
    frame_timestamp_invalid:'画像時刻が無効です。角度の表示だけでは開始できません。',
    left_not_detected:'左足の有効な検出を待っています。',
    right_not_detected:'右足の有効な検出を待っています。',
    left_out_of_range:'左足は検出されていますが、角度が校正範囲外です。',
    right_out_of_range:'右足は検出されていますが、角度が校正範囲外です。',
    frame_stale:'足の画像が古いため、200ms以内の新しい画像を待っています。',
    angle_nonfinite:'足の角度を有効な数値として取得できていません。'
  };
  if (messages[s.foot_control_reason]) return messages[s.foot_control_reason];
  const f = s.foot || {};
  if (f.zero_ready === false) return messages.zero_not_ready;
  if (f.frame_valid === false) return messages.frame_invalid;
  if (f.frame_timestamp_valid === false) return messages.frame_timestamp_invalid;
  if (f.left_valid === false) return messages.left_not_detected;
  if (f.right_valid === false) return messages.right_not_detected;
  if (f.left_in_range === false && f.left_control_range_ok !== true) return messages.left_out_of_range;
  if (f.right_in_range === false && f.right_control_range_ok !== true) return messages.right_out_of_range;
  return '足の検出以外に、ゼロ点・校正範囲・画像時刻・鮮度の開始条件を確認しています。';
}
function controls() {
  const fresh = !offlineMode && latest && Date.now() - lastSeen < 3500;
  const busy = commandInFlight || previewRunning || poseRunning || !!latest?.command?.pending;
  const building = latest?.export_phase === 'building';
  $('start').disabled = !fresh || busy || transferRunning || !latest.ready || !latest.upright_reference?.valid || latest.running || latest.export_phase !== 'empty';
  $('inverse-start').disabled = $('start').disabled;
  $('work-start').disabled = !fresh || busy || transferRunning || !latest.ready || latest.running || latest.export_phase !== 'empty' || !!latest.upright_reference?.capturing;
  $('work-start-10').disabled = $('work-start').disabled;
  $('work-start-12').disabled = $('work-start').disabled;
  if (latest?.foot_control_required && !latest.foot_control_ready) $('work-start').disabled = true;
  $('work-start-status').textContent = !fresh ? '±8°を開始できません：本体との通信を確認してください。' :
    latest.running ? '±8°を開始できません：測定中です。' :
    busy || transferRunning ? '±8°を開始できません：処理の完了を待っています。' :
    latest.export_phase !== 'empty' ? '±8°を開始できません：ログ保存後に「次の測定へ」進んでください。' :
    latest.upright_reference?.capturing ? '±8°を開始できません：基準姿勢を取得中です。' :
    !latest.ready ? '±8°を開始できません：IMUとモータの準備を待っています。' :
    latest.foot_control_required && !latest.foot_control_ready ? '±8°を開始できません：' + footStartBlockReason(latest) :
    '±8°を開始できます。機体と足を静止させてボタンを押してください。';
  $('probe-batch').disabled = $('start').disabled;
  $('probe-extended').disabled = $('start').disabled;
  $('probe-positive-current').disabled = $('start').disabled;
  $('upright-reference-capture').disabled = !fresh || busy || transferRunning || building ||
    !latest.controller_fresh || !latest.imu_ok || latest.running || latest.pulse_active || latest.motor_mA !== 0 ||
    !['READY_TO_MEASURE', 'FINISHED', 'ESTOP'].includes(latest.state);
  $('cg-save').disabled = $('upright-reference-capture').disabled;
  $('cg-capture').disabled = $('cg-save').disabled;
  $('cg-shift').disabled = $('cg-save').disabled;
  const dedicatedSelected = ['bridge_inverse','work_control'].includes($('energy-model').value);
  $('target').disabled = $('start').disabled || dedicatedSelected;
  $('input-percent').disabled = $('start').disabled;
  $('input-current').disabled = $('start').disabled || dedicatedSelected;
  $('energy-model').disabled = $('start').disabled;
  $('brake-max').disabled = $('start').disabled || ['bridge','bridge_inverse','work_control'].includes($('energy-model').value);
  $('peak-adaptation').disabled = $('start').disabled || dedicatedSelected;
  $('clear').disabled = !fresh || busy || transferRunning || building || latest.running || !['FINISHED', 'ESTOP'].includes(latest.state);
  $('download').disabled = !fresh || busy || transferRunning || !latest.downloadable || latest.running;
  $('cancel').disabled = !transferRunning;
  $('csv').disabled = !completedFoot;
  $('preview').disabled = !fresh || busy || transferRunning || building || latest.running || !latest.foot?.preview_available;
  $('preview-save').disabled = !previewEvidence || previewRunning;
  const poseReady = fresh && latest.controller_fresh && !busy && !transferRunning && !building && !latest.running &&
    latest.state === 'READY_TO_MEASURE' && latest.foot?.zero_ready && latest.foot?.preview_available && latest.mekf?.inputs?.valid;
  $('pose-base').disabled = !poseReady || !!poseSession;
  $('pose-add').disabled = !poseReady || !poseSession?.baseline || poseSession.poses.length >= PoseComparison.limits.max_poses;
  $('pose-cancel').disabled = !poseRunning;
  $('pose-save').disabled = !poseSession?.baseline || poseRunning;
  $('pose-reset').disabled = !poseSession || poseRunning || (!!poseSession.baseline && !poseSaved);
  if ($('usb-diag')) $('usb-diag').disabled = !fresh || commandInFlight;
  if ($('reconnect')) $('reconnect').style.display = offlineMode ? 'inline-block' : 'none';
}
function setOffline(waitMs) {
  ++offlineGeneration;
  offlineMode = true; offlineUntil = Date.now() + Math.max(0, Math.min(30000, waitMs));
  try { sessionStorage.setItem(offlineKey, String(offlineUntil)); } catch (_) {}
  renderOffline();
}
function clearOffline() {
  ++offlineGeneration;
  offlineMode = false; offlineUntil = 0;
  try { sessionStorage.removeItem(offlineKey); } catch (_) {}
}
function renderOffline() {
  const remaining = Math.max(0, Math.ceil((offlineUntil - Date.now()) / 1000));
  $('connection').textContent = remaining ? 'Web更新を休止中 · Wi-Fi接続を維持してください' : 'Webの復帰待ち · 自動再試行';
  $('state').textContent = 'Web休止中';
  $('run-target').textContent = requestedTargetDeg === null
    ? '目標角度はWeb復帰後に確認します。'
    : `目標角度：${requestedTargetDeg}°（ZEROクロスで入力時間を計算）`;
  $('run-percent').textContent = requestedInputPercent === null
    ? '通常入力はZEROクロスで行います。'
    : requestedEnergyModel === 'work_control' && requestedTargetDeg === 8 && walkingEnergyTrial()
      ? walkingTransferTrial() ? '開始要求：戻りを救済し、実ZEROクロスで主入力を再計算' : '開始要求：左右の不足エネルギーから入力時間を選択'
    : requestedEnergyModel === 'work_control'
      ? '開始要求：正側300・350・400 mAから選択、負側300 mA'
      : `開始要求：ZEROクロスで${requestedInputCurrentMa} mA、入力時間はエネルギーから計算`;
  $('run-brake').textContent = requestedBrakeMaxMs === null
    ? '負側ブレーキ上限はWeb復帰後に確認します。'
    : requestedEnergyModel === 'work_control' && requestedTargetDeg === 8 && walkingEnergyTrial()
      ? walkingTransferTrial() ? '開始要求：救済と主入力を合わせて半周期80ms以内の試験範囲' : '開始要求：主入力・救済とも最大80msの試験範囲'
    : requestedEnergyModel === 'work_control' ? '開始要求：負側はエネルギー差から選択。上限は25 msから段階的に調整'
    : requestedEnergyModel === 'bridge_probe_positive_current' ? '開始要求：負側40 ms固定、正側300・350・400 mAを順序を変えて測定'
    : requestedEnergyModel === 'bridge_probe_extended' ? '開始要求：負側30・35・40 msを順序を変えて測定'
    : requestedEnergyModel === 'bridge_probe_batch' ? '開始要求：負側5条件を順序を変えて測定'
      : requestedEnergyModel === 'bridge_inverse' ? '開始要求：次ピークのエネルギーから負側入力を選択'
      : `開始要求：負側ブレーキ上限 ${requestedBrakeMaxMs} ms`;
  $('run-model').textContent = requestedEnergyModel === 'bridge'
    ? '開始要求：旧到達角予測をエネルギーに換算（逆入力なし）'
    : requestedEnergyModel === 'work_control' && requestedTargetDeg === 8 && walkingTransferTrial()
      ? walkingAdmissionTrial() ? '開始要求：前倒し候補のモデル支持を確認し、範囲外はZEROで再評価。前後の持続逸脱では入力停止'
        : '開始要求：救済は戻りの不足分、主入力は次ピークの不足分から選択'
    : requestedEnergyModel === 'work_control' ? '開始要求：ホイールの仕事から次ピークの位置エネルギーを予測して入力を選択'
    : requestedEnergyModel === 'bridge_probe_positive_current' ? '開始要求：固定脚・正側電流3条件の一括実験'
    : requestedEnergyModel === 'bridge_probe_extended' ? '開始要求：固定脚・負側3条件の一括実験'
    : requestedEnergyModel === 'bridge_probe_batch' ? '開始要求：固定脚・負側5条件の一括実験'
    : requestedEnergyModel === 'bridge_inverse' ? '開始要求：次ピークの位置エネルギーから負側入力を選択'
    : requestedEnergyModel === 'quadratic' ? '開始要求：二次エネルギー式' : '予測式はWeb復帰後に確認します。';
  $('remaining').textContent = remaining ? `${remaining} s（Web復帰目安）` : '復帰待ち';
  $('run-adaptation').textContent = requestedAdaptation === null
    ? '学習補正の設定はWeb復帰後に確認します。'
    : `開始要求：予測誤差の学習補正 ${requestedAdaptation ? '使用する' : '使用しない'}`;
  for (const id of ['pitch', 'current', 'right', 'left', 'fps']) $(id).textContent = '—';
  $('guide').textContent = `本体で制御・観測・記録を行います。開始5秒＋測定${['work_control','bridge_probe_batch','bridge_probe_extended','bridge_probe_positive_current'].includes(requestedEnergyModel) ? 30 : 15}秒＋終了5秒が予定時間です。前後90°以上の傾斜でSTOPします。横倒しは姿勢STOPの対象にしません。表示時間はPC側の目安で、実際の進行・終了を確認した値ではありません。` +
    (walkingAdmissionTrial() ? ' 前後の持続逸脱でも入力を停止します。' : '');
  $('foot-status').textContent = '足角度の画面更新を停止。本体内の記録は継続します。';
  $('mekf-axes').textContent = '運転中の姿勢表示を停止しています。';
  controls();
}
const format = (n, digits = 2) => Number.isFinite(n) ? n.toFixed(digits) : '—';
function drawMarkers(f) {
  const ctx = $('markers').getContext('2d'); ctx.clearRect(0, 0, 640, 120);
  for (const [name, x, y, range] of [['A / 右', f.right_x, 35, f.range?.right_support_x ?? [42, 173]],
      ['B / 左', f.left_x, 90, f.range?.left_support_x ?? [43.5, 177.5]]]) {
    const [lo, hi] = range;
    ctx.fillStyle = '#dce8e5'; ctx.fillRect(lo * 2, y - 13, (hi - lo) * 2, 26);
    ctx.strokeStyle = '#aab8c6'; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(640, y); ctx.stroke();
    ctx.fillStyle = '#304962'; ctx.font = '12px system-ui'; ctx.fillText(name, 5, y - 16);
    if (Number.isFinite(x)) { ctx.fillStyle = '#145fad'; ctx.beginPath(); ctx.arc(x * 2, y, 7, 0, 2 * Math.PI); ctx.fill(); }
  }
}
function footIssues(f, stale, terminal) {
  const issues = [];
  if (terminal) issues.push('最終フレームを表示');
  else if (stale) issues.push('画像更新なし');
  if (f.frame_valid === false) issues.push('画像取得失敗');
  else if (f.frame_timestamp_valid === false) issues.push('画像時刻が無効');
  const reasons = {low_contrast: '未検出（明暗差不足）', low_weight: '未検出（白領域不足）',
    bad_shape: '白領域の幅が不適合', ambiguous: '候補を区別できません',
    track_jump: '検出位置が急変', reacquiring: '再捕捉中'};
  for (const [side, label] of [['right', '右'], ['left', '左']]) {
    if (f[side + '_valid']) {
      if (f[side + '_in_range'] === false) {
        if (f[side + '_control_range_ok'] === true)
          issues.push(label + '：許容範囲内（校正端から' + format(f[side + '_extrapolation_deg'],1) + '°の外挿）');
        else issues.push(label + (f.control_extension_deg ? '：許容' + f.control_extension_deg + '°を超える範囲外' : '：設定範囲外'));
      }
    } else {
      const reason = f[side + '_reason'];
      if (reasons[reason]) issues.push(label + '：' + reasons[reason]);
      else if (reason === 'no_frame' && f.frame_valid !== false) issues.push(label + '：画像なし');
      else if (!reason) issues.push(label + '：未検出または未校正'); // Older status data.
    }
  }
  if (f.overflow) issues.push('記録容量超過');
  return issues.length ? ' · ' + issues.join(' · ') : '';
}
function render(s) {
  if (walkingAdmissionTrial(s)) {
    $('work-title').textContent = '歩行のエネルギー制御（v19・30秒）';
    $('work-description').textContent = '今回のv19は±8°を使用します。戻りの救済後、実ZEROクロスで主入力を再計算します。救済と主入力を合わせて半周期80ms以内の試験範囲です。前倒し候補のモデル支持を確認し、範囲外はZEROで再評価します。前後の持続逸脱では入力を停止します。開始できない理由は±8°ボタンの下に表示します。±10°・±12°は別の比較方式です。' +
      (s.foot?.control_extension_deg ? ' 校正端から' + s.foot.control_extension_deg + '°以内のずれは許容し、外挿として記録します。' : '');
  } else if (walkingTransferTrial(s)) {
    $('work-title').textContent = '歩行のエネルギー制御（v18・30秒）';
    $('work-description').textContent = '今回のv18は±8°を使用します。戻りの不足分を救済した後、実ZEROクロスで主入力を再計算します。救済と主入力を合わせて半周期80ms以内の試験範囲です。足の認識が一時的に欠けたときは、新しい画像と機体の減速条件を確認して救済します。開始できない理由は±8°ボタンの下に表示します。±10°・±12°は別の比較方式です。' +
      (s.foot?.control_extension_deg ? ' 校正端から' + s.foot.control_extension_deg + '°以内のずれは許容し、外挿として記録します。' : '');
  } else if (walkingEnergyTrial(s)) {
    $('work-title').textContent = '歩行のエネルギー制御（v17・30秒）';
    $('work-description').textContent = '今回のv17は±8°を使用します。左右の到達エネルギーの予測誤差と、足が前に出た状態の戻り不足を考慮して入力時間を選びます。主入力・救済は最大80msの試験範囲です。開始できない理由は±8°ボタンの下に表示します。±10°・±12°は別の比較方式です。' +
      (s.foot?.control_extension_deg ? ' 校正端から' + s.foot.control_extension_deg + '°以内のずれは許容し、外挿として記録します。' : '');
  }
  $('connection').textContent = s.controller_fresh ? '接続中' : '接続中 · 制御状態の更新が停止';
  $('firmware-revision').textContent = `本体プログラム：${s.revision || '不明'}`;
  if (s.diagnostic_build_id) $('firmware-revision').textContent += ` (${s.diagnostic_build_id})`;
  $('state').textContent = s.state; $('remaining').textContent = format(s.remaining_ms / 1000, 1) + ' s';
  $('pitch').textContent = format(s.pitch_deg) + '°';
  $('current').textContent = `${s.motor_mA} / ${s.actual_mA} mA`;
  const f = s.foot, stale = !Number.isFinite(f.age_ms) || f.age_ms < 0 || f.age_ms > 500;
  const terminal = ['FINISHED', 'ESTOP'].includes(s.state);
  if (targetChoices.includes(s.target_deg)) {
    if (!targetInitialized || targetBootId !== s.boot_id || s.running || terminal) {
      $('target').value = String(s.target_deg);
      targetInitialized = true; targetBootId = s.boot_id;
    }
  }
  if (validInputPercent(s.input_peak_percent) &&
      (!angleInitialized || angleBootId !== s.boot_id || s.running || terminal)) {
    $('input-percent').value = String(s.input_peak_percent);
    angleInitialized = true; angleBootId = s.boot_id;
  }
  if (validInputCurrent(s.input_current_mA) &&
      (!currentInitialized || currentBootId !== s.boot_id || s.running || terminal)) {
    $('input-current').value = String(s.input_current_mA);
    currentInitialized = true; currentBootId = s.boot_id;
  }
  if (brakeChoices.includes(s.brake_max_ms) &&
      (!brakeInitialized || brakeBootId !== s.boot_id || s.running || terminal)) {
    $('brake-max').value = String(s.brake_max_ms);
    brakeInitialized = true; brakeBootId = s.boot_id;
  }
  if (['bridge','bridge_inverse','work_control','bridge_probe','bridge_probe_batch','bridge_probe_extended','bridge_probe_positive_current','quadratic'].includes(s.energy_model) &&
      (!modelInitialized || modelBootId !== s.boot_id || s.running || terminal)) {
    $('energy-model').value = s.energy_model.startsWith('bridge_probe') ? 'bridge' : s.energy_model;
    modelInitialized = true; modelBootId = s.boot_id;
  }
  if (!adaptationInitialized || adaptationBootId !== s.boot_id || s.running || terminal) {
    $('peak-adaptation').value = s.peak_adaptation_enabled ? '1' : '0';
    adaptationInitialized = true; adaptationBootId = s.boot_id;
  }
  $('run-adaptation').textContent = (s.running || terminal) && walkingTransferTrial(s) && s.target_deg === 8 && s.energy_model === 'work_control'
    ? '到達予測の補正：救済のない周期だけで更新'
    : s.running || terminal
    ? `今回の予測誤差の学習補正：${s.peak_adaptation_enabled ? '使用する' : '使用しない'}`
    : '学習補正の設定は測定開始時に確定します。';
  $('run-percent').textContent = (s.running || terminal) && walkingEnergyTrial(s) && s.target_deg === 8 && s.energy_model === 'work_control'
    ? walkingTransferTrial(s) ? '今回の入力：戻りを救済し、実ZEROクロスで主入力を再計算' : '今回の入力：左右の不足エネルギーから入力時間を選択'
    : s.running || terminal
    ? s.energy_model === 'work_control'
      ? '今回の入力：正側300・350・400 mAから選択、負側300 mA'
      : s.energy_model === 'bridge_probe_positive_current'
      ? '今回の入力：正側300・350・400 mA、負側300 mA。正側の入力時間は従来の計算で可変'
      : `今回の入力：ZEROクロスで${s.input_current_mA} mA、入力時間は可変`
    : '通常入力はZEROクロスで行います。';
  $('run-target').textContent = s.running || terminal
    ? `今回の目標角度：${format(s.target_deg, 0)}°`
    : '目標角度は測定開始時に確定します。';
  $('run-brake').textContent = (s.running || terminal) && walkingEnergyTrial(s) && s.target_deg === 8 && s.energy_model === 'work_control'
    ? walkingTransferTrial(s) ? '今回の入力時間：救済と主入力を合わせて半周期80ms以内。保護後の残不足も記録します。' : '今回の入力時間：主入力・救済とも最大80ms。保護後の残不足も記録します。'
    : s.running || terminal
    ? s.energy_model === 'bridge_probe_positive_current' ? '今回の負側入力：300 mA・40 msに固定'
      : s.energy_model === 'bridge_probe_extended' ? '今回の負側入力：30・35・40 msを3条件ずつ順序を変更'
      : s.energy_model === 'bridge_probe_batch' ? '今回の負側入力：0・10・15・20・25 msを5条件ずつ順序を変更'
      : s.energy_model === 'work_control' ? (s.target_deg>8 ? '今回の負側入力：エネルギー差から無入力・ブレーキ・不足時の補助入力を選択。補助入力は実測結果に応じて25 msから段階的に拡張' : `今回の負側入力：エネルギー差から0または10～70 msを選択。40 ms超は正側${format(s.target_deg-.2,1)}°以上で段階的に許可`)
      : s.energy_model === 'bridge_inverse' ? '今回の負側入力：実測0〜25 ms、ピークを確認して最大30 ms'
      : s.energy_model === 'bridge' ? '今回の旧式換算：ブレーキなし' : `今回の負側ブレーキ上限：${s.brake_max_ms} ms`
    : '負側ブレーキ上限は測定開始時に確定します。';
  $('run-model').textContent = (s.running || terminal) && walkingTransferTrial(s) && s.target_deg === 8 && s.energy_model === 'work_control'
    ? walkingAdmissionTrial(s) ? '今回の候補：前倒し候補のモデル支持を確認し、範囲外はZEROで再評価'
      : '今回の予測：救済は戻り通過の不足分、主入力は次ピークの不足分から選択'
    : s.running || terminal
    ? s.energy_model === 'bridge_probe_positive_current' ? '今回の方式：正側電流300・350・400 mAを順序変更、負側40 ms固定'
      : s.energy_model === 'bridge_probe_extended' ? '今回の方式：正側は従来制御、負側は30・35・40 msの一括実験'
      : s.energy_model === 'bridge_probe_batch' ? '今回の方式：正側は従来制御、負側は5条件の一括実験'
      : s.energy_model === 'work_control' ? '今回の予測式：ホイールの入力後の仕事を含めて次ピークを予測'
      : s.energy_model === 'bridge_inverse' ? '今回の予測式：次ピークの位置エネルギーから負側入力時間を選択'
      : s.energy_model === 'bridge' ? '今回の予測式：旧到達角予測をエネルギーに換算' : '今回の予測式：二次エネルギー式'
    : '予測式は測定開始時に確定します。';
  $('right').textContent = f.right_valid && (!stale || terminal) ? format(f.right_deg) + '°' : '—';
  $('left').textContent = f.left_valid && (!stale || terminal) ? format(f.left_deg) + '°' : '—';
  $('fps').textContent = terminal ? '停止中' : `${format(stale ? 0 : f.fps, 1)} / 15 fps`;
  $('foot-status').textContent = `足角度（${s.foot_control_required ? '8°試験の制御に使用' : '任意'}） · ${f.zero_ready ? 'ゼロ点確定' : 'ゼロ点未確定（' + (f.zero_samples ?? 0) + '枚）'} · 記録${f.frames ?? 0}枚 · 画像取得失敗${f.failures ?? 0}回` +
    footIssues(f, stale, terminal);
  const reference = s.upright_reference;
  const cg = s.lateral_cg;
  $('cg-status').textContent = cg?.capturing ? '自然な静止姿勢を測定中です。機体を支えず、動かさないでください。' :
    cg?.automatic ? `自動補正 · 等価重心ずれ ${format(cg.shift_mm, 3)} mm · 観測${cg.pairs ?? 0}往復／更新${cg.updates ?? 0}回 · ＋${s.target_deg}° ${format(cg.positive_target_mj, 3)} mJ／−${s.target_deg}° ${format(cg.negative_target_mj, 3)} mJ` :
    cg?.valid ? `重心ずれ ${format(cg.shift_mm, 3)} mmを適用 · ＋${s.target_deg}°の目標 ${format(cg.positive_target_mj, 3)} mJ · −${s.target_deg}°の目標 ${format(cg.negative_target_mj, 3)} mJ` :
      '絶対角基準の補正は未設定です。仕事に基づく制御では補正ゼロから自動更新します。';
  $('upright-reference-status').textContent = reference.capturing
    ? '基準を取得中です。約2秒間、今の姿勢で静止させてください。'
    : reference.valid
      ? `比較用の姿勢を保存済み（更新${reference.revision}回） · 保存姿勢との差 ${format(reference.relative_pitch_deg)}° · 保存時のMEKF角 ${format(reference.abs_pitch_deg)}°。仕事制御の原点は絶対角0°です。`
      : '比較用の姿勢は未保存です。仕事に基づく制御の開始には保存不要です。';
  const motionStop = walkingAdmissionTrial(s) && s.state === 'ESTOP' ? foreAftStopText(s.last_error) : '';
  if (motionStop) $('guide').textContent = motionStop + ' ログを保存して状態を確認してください。';
  else if (s.foot_control_required && !s.running && s.ready && !reference.capturing) $('guide').textContent = s.foot_control_ready
    ? '足のゼロ点と左右の検出を確認しました。機体を静止させて8°試験を開始できます。'
    : '8°試験を開始できません：' + footStartBlockReason(s);
  else if (!s.running && s.ready && !reference.capturing) $('guide').textContent = '仕事に基づく制御はMEKF絶対角0°が基準です。直立基準の保存は不要です。機体を静止させて開始してください。';
  else if (s.running) $('guide').textContent = s.foot_control_required ? walkingTransferTrial(s)
    ? walkingAdmissionTrial(s) ? '測定中。前倒し候補のモデル支持を確認し、範囲外はZEROで再評価します。前後の持続逸脱では入力を停止します。'
      : '測定中。足の認識が欠けたときは、新しい画像と機体の減速条件を確認して戻りを補助します。'
    : '測定中。足角度と機体の揺れを使って支持移動を補助します。' : '測定中。画面は更新されます。足角度は制御入力に使用しません。';
  else if (reference.capturing) $('guide').textContent = '基準を取得しています。機体を動かさずお待ちください。';
  else if (s.downloadable) $('guide').textContent = 'ログを保存してください。次の測定には「ログを消去・次の測定へ」を使います。';
  else if (s.ready) $('guide').textContent = f.zero_ready && f.right_valid && f.left_valid && !stale
    ? '直立姿勢を保ち、測定を開始してください。'
    : '測定を開始できます。カメラ・足マーカーは任意です。足角度が取得できない区間は無効として記録します。';
  else $('guide').textContent = `IMUとモータの準備、直立姿勢を確認してください。足マーカーは開始条件ではありません。pitch基準差 ${format(s.upright?.pitch_error_deg, 1)}°（5°以内）/ 角速度 ${format(s.upright?.gyro_dps, 1)}°/s`;
  $('diagnostic-view').textContent = JSON.stringify(s, null, 2);
  if ($('usb-diag')) $('usb-diag').checked = s.usb_diagnostics === true;
  $('mekf-axes').textContent = s.mekf?.valid && s.mekf.fresh
    ? `前後 roll ${format(s.mekf.roll_deg)}° · 左右 pitch ${format(s.mekf.pitch_deg)}°`
    : 'MEKFの更新を待っています。';
  drawMarkers(f); controls();
}
async function refresh(force = false) {
  if (offlineMode && Date.now() < offlineUntil && !force) { renderOffline(); return; }
  if (refreshInFlight) return;
  refreshInFlight = true;
  const generation = offlineGeneration;
  let received = false;
  try {
    const s = await request('/status.json', {timeout: 2500}); received = true;
    // A pre-START poll can arrive after START entered quiet mode. Do not let
    // that old READY response restart automatic polling during the run.
    if (generation !== offlineGeneration) { if (offlineMode) renderOffline(); return; }
    if (s?.offline_run === true && Number.isFinite(s.wait_ms)) { setOffline(s.wait_ms); return; }
    const wasOffline = offlineMode;
    adoptStatus(s); clearOffline(); render(latest);
    if (wasOffline) {
      const message = s.network?.last_error || s.last_error ||
        (s.downloadable ? 'Web表示が復帰しました。ログを保存してください。' : s.command.result || 'Web表示が復帰しました。');
      $('message').textContent = foreAftStopText(message) || message;
    }
  } catch (error) {
    lastSeen = 0;
    if (offlineMode) { renderOffline(); return; }
    $('connection').textContent = received || error instanceof SyntaxError
      ? '状態データ・画面更新のエラー（自動再試行）'
      : '装置から応答がありません（自動再試行）';
    controls();
  } finally { refreshInFlight = false; }
}
function adoptStatus(s) {
  if (!s || typeof s.state !== 'string' || typeof s.export_phase !== 'string' ||
      !validInputPercent(s.input_peak_percent) || !validInputCurrent(s.input_current_mA) ||
      !negativeWidthChoices.includes(s.brake_max_ms) ||
      typeof s.peak_adaptation_enabled !== 'boolean' ||
      !s.upright_reference || typeof s.upright_reference.valid !== 'boolean' ||
      typeof s.upright_reference.capturing !== 'boolean' ||
      (s.upright_reference.valid && (!Number.isFinite(s.upright_reference.abs_pitch_deg) ||
        !Number.isInteger(s.upright_reference.revision) || s.upright_reference.revision <= 0)) ||
      !s.foot || !s.upright || !s.command ||
      typeof s.command.pending !== 'boolean' ||
      !Number.isInteger(s.command.completed) || !Number.isInteger(s.command.submitted) ||
      ['running', 'ready', 'downloadable', 'controller_fresh'].some(key => typeof s[key] !== 'boolean'))
    throw new Error('装置の状態データが不完全です');
  latest = s; lastSeen = Date.now();
  if (poseSession && !poseSaved) {
    poseSession.trace.push(JSON.parse(JSON.stringify({client_time_ms:Date.now(), boot_id:s.boot_id,
      run_id:s.run_id, revision:s.revision, state:s.state, mekf:s.mekf, controller_fresh:s.controller_fresh,
      foot:{sequence:s.foot.sequence, age_ms:s.foot.age_ms, right_deg:s.foot.right_deg, left_deg:s.foot.left_deg,
        right_x:s.foot.right_x, left_x:s.foot.left_x, right_valid:s.foot.right_valid, left_valid:s.foot.left_valid}})));
    if (poseSession.trace.length > PoseComparison.limits.max_trace) { poseSession.trace.shift(); ++poseSession.trace_dropped; }
  }
}
async function poll() {
  try { await refresh(); }
  catch (error) { $('connection').textContent = '画面更新のエラー（自動再試行）'; }
  finally { setTimeout(poll, 800); }
}
async function startOfflineRun(isBatchPackage = false, forcedModel = null, workTarget = 8) {
  if (commandInFlight || offlineMode) return;
  const extendedPackage = forcedModel === 'bridge_probe_extended';
  const positiveCurrentPackage = forcedModel === 'bridge_probe_positive_current';
  const batchPackage = isBatchPackage || extendedPackage || positiveCurrentPackage;
  const workPackage = (forcedModel || $('energy-model').value) === 'work_control';
  const inversePackage = !isBatchPackage && (forcedModel || $('energy-model').value) === 'bridge_inverse';
  const target = workPackage ? Number(workTarget) : batchPackage || inversePackage ? 8 : Number($('target').value);
  if (!targetChoices.includes(target)) {
    $('message').textContent = '目標角度を8°・10°・12°から選択してください。'; return;
  }
  const angleText = batchPackage || workPackage ? '0' : $('input-percent').value.trim();
  const inputPercent = Number(angleText);
  if (!angleText || !validInputPercent(inputPercent)) {
    $('message').textContent = 'ZEROクロス設定を確認してください。'; return;
  }
  const currentText = $('input-current').value.trim();
  const inputCurrentMa = batchPackage || inversePackage || workPackage ? 300 : Number(currentText);
  if ((!batchPackage && !inversePackage && !workPackage && !currentText) || !validInputCurrent(inputCurrentMa)) {
    $('message').textContent = '指令電流を100〜1200 mAの範囲で10 mA刻みで設定してください。'; return;
  }
  const energyModel = isBatchPackage ? 'bridge_probe_batch' : (forcedModel || $('energy-model').value);
  if (!['bridge', 'bridge_inverse', 'work_control', 'bridge_probe_batch', 'bridge_probe_extended', 'bridge_probe_positive_current', 'quadratic'].includes(energyModel)) {
    $('message').textContent = '予測式を選択してください。'; return;
  }
  const brakeMaxMs = positiveCurrentPackage ? 50 : extendedPackage ? 40 : workPackage ? 40 : isBatchPackage || inversePackage ? 25 : energyModel === 'bridge' ? 0 : Number($('brake-max').value);
  if (!(positiveCurrentPackage ? [50] : extendedPackage ? [40] : workPackage ? [40] : isBatchPackage || inversePackage ? [25] : brakeChoices).includes(brakeMaxMs)) {
    $('message').textContent = '負側ブレーキ上限を0・20・50 msから選択してください。'; return;
  }
  const adaptationValue = batchPackage || inversePackage || workPackage ? '0' : $('peak-adaptation').value;
  if (!['0', '1'].includes(adaptationValue)) {
    $('message').textContent = '予測誤差の学習補正の設定を確認してください。'; return;
  }
  requestedAdaptation = adaptationValue === '1';
  requestedTargetDeg = target;
  requestedInputPercent = inputPercent;
  requestedInputCurrentMa = inputCurrentMa;
  requestedBrakeMaxMs = brakeMaxMs;
  requestedEnergyModel = energyModel;
  const runLabel = positiveCurrentPackage ? '正側300・350・400 mAの一括実験' : extendedPackage ? '負側30・35・40 msの一括実験' : isBatchPackage ? '負側5条件の一括実験' :
    workPackage ? '固定脚・ホイール仕事に基づく制御' : inversePackage ? '固定脚・負側のエネルギー制御' : '左右揺動エネルギー制御';
  commandInFlight = true; setOffline(batchPackage || workPackage ? 50000 : 30000); $('message').textContent = `${runLabel}（目標${target}°・${inputCurrentMa} mA）の開始要求を送信中…`;
  try {
    await request(`/start-energy-control-autonomous?target_deg=${target}&input_peak_percent=${inputPercent}&input_current_mA=${inputCurrentMa}&brake_max_ms=${brakeMaxMs}&peak_adaptation=${adaptationValue}&energy_model=${energyModel}`, {method:'POST', kind:'text'});
    $('message').textContent = `${runLabel}（目標${target}°・${inputCurrentMa} mA）の開始要求を受け付けました。Web休止後に本体が開始条件を確認します。Wi-Fi接続と画面をそのまま保ってお待ちください。`;
  } catch (error) {
    if (/^\d{3}:/.test(error.message)) { clearOffline(); await refresh(); }
    $('message').textContent = `開始結果の確認: ${error.message}。Web復帰後に本体の結果を確認します。`;
  } finally { commandInFlight = false; controls(); }
}
async function command(path) {
  const referenceCapture = path === '/upright-reference/capture' || path.startsWith('/lateral-cg/');
  commandInFlight = true; controls(); $('message').textContent = referenceCapture
    ? '今の姿勢を約2秒間保ってください。基準を取得して保存します。' : '要求を送信中…';
  const before = latest?.command?.submitted ?? 0;
  try {
    await request(path, {method: 'POST', kind: 'text'});
    for (let i = 0; i < (referenceCapture ? 30 : 12); ++i) {
      const s = await request('/status.json'); adoptStatus(s); render(s);
      if (!s.command.pending && (s.command.completed > before || path === '/stop')) {
        $('message').textContent = referenceCapture ? referenceResultMessage(s.command.result) :
          s.command.result || '要求を処理しました'; return;
      }
      await nap(150);
    }
    $('message').textContent = '処理結果を確認中です。状態表示を確認してください。';
  } catch (error) {
    $('message').textContent = `通信結果が未確認です。状態を確認してください: ${error.message}`;
  } finally { commandInFlight = false; controls(); }
}
function referenceResultMessage(result) {
  const messages = {
    lateral_cg_saved:'重心ずれの初期値を保存しました。仕事に基づく制御では、この値から動作中の自動補正を始めます。',
    lateral_cg_save_failed:'重心ずれを保存できませんでした。停止状態と直立基準を確認してください。',
    lateral_cg_shift_invalid:'重心ずれは−2～＋2 mmの数値で入力してください。',
    equilibrium_contact_not_identifiable:'この静止角からは重心ずれを特定できません。中央の接地範囲内では静止角だけで求められないため、測定した重心ずれをmmで入力してください。元の設定を保っています。',
    upright_reference_saved:'比較用の姿勢を本体に保存しました。仕事に基づく制御の原点はMEKF絶対角0°のままです。',
    upright_reference_requires_stopped:'測定を停止してから基準を取得してください。',
    upright_reference_hold_still:'静止を確認できませんでした。元の基準を保っています。機体を静止させてやり直してください。',
    upright_reference_pose_moved:'取得中に角度が変わりました。元の基準を保っています。姿勢を保ってやり直してください。',
    upright_reference_imu_not_fresh:'姿勢の更新を確認できませんでした。元の基準を保っています。準備完了後にやり直してください。',
    upright_reference_save_failed:'本体への保存を確認できませんでした。基準を再取得してください。',
    upright_reference_timeout:'基準の取得が完了しませんでした。機体を静止させてやり直してください。',
    upright_reference_cancelled:'基準の取得を中止しました。元の基準を保っています。',
    stopped:'基準の取得を中止しました。元の基準を保っています。'
  };
  return messages[result] || `基準の取得結果：${result || '状態を確認してください'}`;
}
// A cache is optional for one-session downloads. IndexedDB additionally allows
// reload/reconnect resume; every cached chunk is revalidated before reuse.
let cachePromise;
const memoryCache = new Map();
function openCache() {
  if (!cachePromise) cachePromise = new Promise(resolve => {
    try {
      const r = indexedDB.open('freefoot-rwlog-v2', 1);
      r.onupgradeneeded = () => r.result.createObjectStore('chunks');
      r.onsuccess = () => resolve(r.result); r.onerror = () => resolve(null); r.onblocked = () => resolve(null);
    } catch { resolve(null); }
  });
  return cachePromise;
}
async function cacheGet(key) {
  const db = await openCache();
  if (!db) return memoryCache.get(key);
  return new Promise(resolve => {
    const tx = db.transaction('chunks', 'readonly'), r = tx.objectStore('chunks').get(key);
    r.onsuccess = () => resolve(r.result); r.onerror = () => resolve(undefined);
  });
}
async function cachePut(key, value) {
  memoryCache.set(key, value);
  const db = await openCache(); if (!db) return;
  await new Promise(resolve => {
    const tx = db.transaction('chunks', 'readwrite');
    tx.objectStore('chunks').put(value, key);
    tx.oncomplete = resolve; tx.onabort = resolve; tx.onerror = resolve;
  });
}
async function pruneCache(token) {
  const prefix = token + ':';
  for (const key of memoryCache.keys()) if (!key.startsWith(prefix)) memoryCache.delete(key);
  const db = await openCache(); if (!db) return;
  await new Promise(resolve => {
    try {
      const tx = db.transaction('chunks', 'readwrite'), r = tx.objectStore('chunks').openCursor();
      r.onsuccess = () => {
        const cursor = r.result;
        if (cursor) { if (!String(cursor.key).startsWith(prefix)) cursor.delete(); cursor.continue(); }
      };
      tx.oncomplete = resolve; tx.onabort = resolve; tx.onerror = resolve;
    } catch { resolve(); }
  });
}
function validateChunk(bytes, offset, length) {
  if (!(bytes instanceof Uint8Array) || bytes.byteLength !== length + 16) throw new Error('チャンク長の不一致');
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (view.getUint32(0, true) !== 0x31484346 || view.getUint32(4, true) !== offset || view.getUint32(8, true) !== length)
    throw new Error('チャンク識別情報の不一致');
  if (crc32(bytes.subarray(16)) !== view.getUint32(12, true)) throw new Error('チャンクCRC不一致');
  return bytes.subarray(16);
}
function saveBlob(blob, name) {
  const url = URL.createObjectURL(blob), a = document.createElement('a');
  a.href = url; a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(url), 60000);
}
function expandFootTable(value, observations = null) {
  if (value == null) return value;
  if (Array.isArray(value)) return value;
  const {fields, rows} = value;
  if (!Array.isArray(fields) || !fields.every(f => typeof f === 'string') ||
      new Set(fields).size !== fields.length || !Array.isArray(rows) ||
      rows.some(row => !Array.isArray(row) || row.length !== fields.length))
    throw new Error('足角度テーブルの形式が不正です');
  const frames = rows.map(row => Object.fromEntries(fields.map((field,i) => [field,row[i]])));
  if (observations != null) {
    if (observations.version !== 1 || !Array.isArray(observations.rows)) throw Error('Invalid foot observations');
    for (const f of frames) f.control_observed_us = null;
    const seen = new Set();
    for (const row of observations.rows) {
      if (!Array.isArray(row) || row.length !== 2 || !Number.isSafeInteger(row[0]) ||
          row[0] < 0 || row[0] >= frames.length || seen.has(row[0]) ||
          !Number.isSafeInteger(row[1]) || row[1] < 0 || !Number.isSafeInteger(frames[row[0]].delivered_us) ||
          !Number.isSafeInteger(frames[row[0]].delivered_us + row[1])) throw Error('Invalid foot observation row');
      frames[row[0]].control_observed_us = frames[row[0]].delivered_us + row[1];
      seen.add(row[0]);
    }
  }
  return frames;
}
async function download() {
  transferRunning = true; cancelTransfer = false; controls();
  try {
    $('transfer').textContent = 'ログを確定中…';
    await request('/export/prepare', {method: 'POST', kind: 'text'});
    let m;
    do {
      if (cancelTransfer) throw new Error('一時停止しました。取得・再開で続けられます。');
      m = await request('/export/manifest');
      if (m.phase === 'error') throw new Error(m.error);
      if (m.phase !== 'ready') { $('transfer').textContent = `ログ確定中 ${m.hashed_bytes} / ${m.bytes || '?'} bytes`; await nap(400); }
    } while (m.phase !== 'ready');
    if (!/^[a-f0-9]{16}$/.test(m.token) || !Number.isInteger(m.bytes) || m.bytes < 114 || m.bytes > 9 * 1024 * 1024 || m.chunk_bytes !== 4096)
      throw new Error('ログ仕様が一致しません。ページを更新してください。');
    await pruneCache(m.token);
    const file = new Uint8Array(m.bytes); $('progress').max = m.bytes; $('progress').value = 0;
    for (let offset = 0; offset < m.bytes;) {
      if (cancelTransfer) throw new Error('一時停止しました。取得・再開で続けられます。');
      const length = Math.min(m.chunk_bytes, m.bytes - offset), key = `${m.token}:${offset}`;
      let payload, packet = await cacheGet(key);
      if (packet) { try { payload = validateChunk(packet, offset, length); } catch { packet = null; } }
      for (let retry = 0; !payload && retry < 5; ++retry) {
        if (cancelTransfer) throw new Error('一時停止しました。');
        try {
          packet = await request(`/export/chunk?token=${m.token}&offset=${offset}&length=${length}`, {kind: 'bytes'});
          payload = validateChunk(packet, offset, length); await cachePut(key, packet);
        } catch (error) {
          if (retry === 4 || error.message.startsWith('409:')) throw error;
          $('transfer').textContent = `${offset} bytesまで取得済み · 再試行 ${retry + 1}/5`;
          await nap(250 * (retry + 1));
        }
      }
      file.set(payload, offset); offset += length; $('progress').value = offset;
      $('transfer').textContent = `${(100 * offset / m.bytes).toFixed(1)}% · ${offset} / ${m.bytes} bytes`;
    }
    const v = new DataView(file.buffer), crc = crc32(file.subarray(0, file.length - 4));
    if (crc !== m.crc32 || crc !== v.getUint32(file.length - 4, true)) throw new Error('ファイル全体のCRCが一致しません');
    const headerSize = v.getUint16(10, true), metadataLength = v.getUint32(24, true);
    if (headerSize !== 110 || metadataLength > file.length - headerSize - 4) throw new Error('RWLOGヘッダー不一致');
    const metadata = JSON.parse(new TextDecoder().decode(file.subarray(headerSize, headerSize + metadataLength)));
    completedFoot = expandFootTable(metadata.foot_frames ?? null, metadata.foot_control_observations ?? null); completedFile = file; completedName = m.filename;
    saveBlob(new Blob([file], {type: 'application/octet-stream'}), m.filename);
    $('transfer').textContent = `CRC検証完了 · ${m.bytes} bytes · 足角度 ${completedFoot?.length ?? 0}行`;
    calculateWheelWork();
  } catch (error) {
    $('transfer').textContent = `${error.message}（保存済みの部分は保持しています）`;
  } finally { transferRunning = false; controls(); }
}
function footCsv(rows) {
  if (!rows.length) return '';
  const keys = Object.keys(rows[0]);
  const esc = v => v == null ? '' : '"' + String(v).replaceAll('"', '""') + '"';
  return keys.map(esc).join(',') + '\r\n' + rows.map(row => keys.map(k => esc(row[k])).join(',')).join('\r\n') + '\r\n';
}
function validatePreviewManifest(m) {
  if (!m || m.format !== 'gray8' || m.width !== 160 || m.height !== 120 ||
      m.source_width !== 320 || m.source_height !== 240 || m.bytes !== 19200 ||
      !Number.isInteger(m.token) || m.token <= 0 || m.token > 0xffffffff ||
      !Number.isInteger(m.crc32) || m.crc32 < 0 || m.crc32 > 0xffffffff ||
      !Number.isInteger(m.sequence) || !m.right || !m.left)
    throw new Error('画像診断の形式が不正です');
}
function previewReason(reason) {
  return ({detected:'検出', no_frame:'画像なし', low_contrast:'明暗差不足', low_weight:'白領域不足',
    bad_shape:'幅が不適合', ambiguous:'候補が競合', track_jump:'位置が急変', reacquiring:'再捕捉中',
    waiting:'待機中', body_moving:'胴体の静止待ち', marker_invalid:'マーカー確認待ち',
    position_mismatch:'基準位置から外れています', marker_moving:'足の静止待ち',
    collecting:'取得中', ready:'確定'})[reason] ?? '未確認';
}
function drawPreview(m, pixels) {
  const small = document.createElement('canvas'); small.width = m.width; small.height = m.height;
  const smallCtx = small.getContext('2d'), bitmap = smallCtx.createImageData(m.width, m.height);
  for (let i = 0; i < pixels.length; ++i) {
    bitmap.data[i * 4] = bitmap.data[i * 4 + 1] = bitmap.data[i * 4 + 2] = pixels[i];
    bitmap.data[i * 4 + 3] = 255;
  }
  smallCtx.putImageData(bitmap, 0, 0);
  const canvas = $('preview-image'), ctx = canvas.getContext('2d');
  canvas.style.display = 'block'; ctx.imageSmoothingEnabled = false;
  ctx.drawImage(small, 0, 0, canvas.width, canvas.height);
  for (const [side, label] of [['right', 'A / 右足'], ['left', 'B / 左足']]) {
    const c = m[side];
    if (!c.candidates) continue;
    const x = c.x * 2, y = c.scan_y * 2;
    ctx.strokeStyle = ctx.fillStyle = c.valid ? '#43ff9c' : '#ffdc55'; ctx.lineWidth = 2;
    ctx.strokeRect(x - c.width, y - 16, c.width * 2, 32);
    ctx.beginPath(); ctx.arc(x, y, 6, 0, Math.PI * 2); ctx.stroke();
    ctx.font = 'bold 16px system-ui'; ctx.fillText(label, Math.max(4, Math.min(x + 10, 540)), Math.max(20, y - 22));
    if (c.alternate_x >= 0 && c.alternate_y >= 0) {
      ctx.strokeStyle = '#ffdc55'; ctx.beginPath(); ctx.arc(c.alternate_x * 2, c.alternate_y * 2, 9, 0, Math.PI * 2); ctx.stroke();
    }
  }
}
async function readPreviewPixels(m, cancelled = () => false) {
    const pixels = new Uint8Array(m.bytes);
    for (let offset = 0; offset < m.bytes; offset += 4096) {
      if (cancelled()) throw Error('取得を中止しました');
      const length = Math.min(4096, m.bytes - offset);
      const packet = await request(`/vision/chunk?token=${m.token}&offset=${offset}&length=${length}`, {kind:'bytes'});
      pixels.set(validateChunk(packet, offset, length), offset);
      await nap(20);
    }
    if (crc32(pixels) !== m.crc32) throw new Error('画像全体のCRCが不一致です');
    // Store exact gray pixels plus matching metadata in one downloadable JSON.
    // Never substitute the separately polled live status for this frame.
    let binary = ''; for (const value of pixels) binary += String.fromCharCode(value);
    return {...m, pixels_gray8_base64: btoa(binary)};
}
function showEvidence(evidence) {
    const {pixels_gray8_base64:binary,...m} = evidence;
    const pixels = Uint8Array.from(atob(binary), c => c.charCodeAt(0));
    drawPreview(m, pixels); previewEvidence = evidence;
    $('preview-status').textContent = `画像 #${m.sequence} · 取得要求の${format(m.age_ms, 0)} ms前のフレーム · 右 ${previewReason(m.right.reason)} / 左 ${previewReason(m.left.reason)} · ゼロ点 ${previewReason(m.zero_reason)}。姿勢を変えたら再取得してください。`;
}
async function capturePreview() {
  if (previewRunning || poseRunning || transferRunning || latest?.running) return;
  previewRunning = true; controls(); $('preview-status').textContent = '画像を取得中…';
  try {
    const m = await request('/vision/capture', {method:'POST'});
    validatePreviewManifest(m);
    showEvidence(await readPreviewPixels(m));
  } catch (error) {
    $('preview-status').textContent = `画像取得に失敗しました。${previewEvidence ? '表示と保存の対象は前回の画像です。' : ''}再取得してください: ${error.message}`;
  } finally { previewRunning = false; controls(); }
}
function renderPoses() {
  const tbody = $('pose-rows'); tbody.replaceChildren();
  if (!poseSession?.baseline) return;
  const rows = [{label:'直立基準',comparison:null},...poseSession.poses];
  for (const [i,row] of rows.entries()) {
    const c = row.comparison, tr = document.createElement('tr');
    const notes = {sideways_changed:'左右傾斜あり',outside_range:'設定範囲外',large_tilt:'傾斜大'};
    for (const value of [row.label || `姿勢 ${i}`,
      ...[c?.delta.roll_deg,c?.right_residual_deg,c?.left_residual_deg].map(v => c ? format(v) + '°' : '0.00°'),
      c ? (c.planar_check ? '比較用' : '参考：' + c.flags.map(f => notes[f]).join('・'))
        : '基準']) {
      const td = document.createElement('td'); td.textContent = value; tr.appendChild(td);
    }
    tbody.appendChild(tr);
  }
}
async function capturePose(baseline) {
  controls();
  if ($(baseline ? 'pose-base' : 'pose-add').disabled) return;
  if (baseline) poseSession = {schema:'freefoot-static-poses-v1', created_at:new Date().toISOString(),
    assumptions:['feet_fixed_to_ground','fore_aft_rotation_about_mekf_x'],
    residual_semantics:'delta_foot_body_relative_plus_delta_body_roll; planar approximation, not absolute foot attitude',
    timing_semantics:'camera exposure and delivery-time control snapshot differ; hold each pose stationary',
    trace_semantics:'existing status polls; not a high-rate IMU integration log',
    limits:PoseComparison.limits, baseline:null, poses:[], trace:[], trace_dropped:0};
  poseRunning = true; poseCancelled = false; poseSaved = false; controls();
  const frames = [];
  try {
    for (let i=0;i<PoseComparison.limits.samples;i++) {
      if (i) await nap(PoseComparison.limits.interval_ms);
      if (poseCancelled) throw Error('取得を中止しました');
      if (Date.now()-lastSeen >= 3500 || latest?.running || latest?.state !== 'READY_TO_MEASURE' ||
          !latest.controller_fresh || latest.command.pending) throw Error('接続と待機状態を確認してください');
      $('pose-status').textContent = `その姿勢を保ってください… ${i+1}/${PoseComparison.limits.samples}`;
      const m = await request('/vision/capture', {method:'POST'});
      validatePreviewManifest(m); PoseComparison.validateFrame(m); frames.push(m);
    }
    const summary = PoseComparison.summarize(frames, baseline);
    const comparison = baseline ? null : PoseComparison.compare(poseSession.baseline.summary, summary);
    const last = frames[frames.length-1];
    $('pose-status').textContent = '静止を確認しました。最後の画像を保存しています…';
    const image = await readPreviewPixels(last, () => poseCancelled);
    if (poseCancelled) throw Error('取得を中止しました');
    const record = {label:baseline ? '直立基準' : `姿勢 ${poseSession.poses.length+1}`, summary, comparison, frames, image};
    if (baseline) poseSession.baseline = record; else poseSession.poses.push(record);
    showEvidence(image); renderPoses();
    $('pose-status').textContent = baseline ? '基準を記録しました。足を固定したまま本体を傾け、「この姿勢を追加」を押してください。'
      : `姿勢 ${poseSession.poses.length} を記録しました。次の姿勢を追加するか、比較JSONを保存してください。`;
  } catch (error) {
    $('pose-status').textContent = error.message + ' 取得済みの姿勢は保持しています。';
    if (baseline && !poseSession.baseline) poseSession = null;
  } finally { poseRunning = false; controls(); }
}
$('start').onclick = () => startOfflineRun();
$('inverse-start').onclick = () => startOfflineRun(false, 'bridge_inverse');
$('work-start').onclick = () => startOfflineRun(false, 'work_control', 8);
$('work-start-10').onclick = () => startOfflineRun(false, 'work_control', 10);
$('work-start-12').onclick = () => startOfflineRun(false, 'work_control', 12);
$('probe-batch').onclick = () => startOfflineRun(true);
$('probe-extended').onclick = () => startOfflineRun(false, 'bridge_probe_extended');
$('probe-positive-current').onclick = () => startOfflineRun(false, 'bridge_probe_positive_current');
$('energy-model').onchange = () => {
  if ($('energy-model').value === 'bridge_inverse') {
    $('target').value='8'; $('input-current').value='300';
    $('brake-max').value='0'; $('peak-adaptation').value='0';
  }
  controls();
};
$('upright-reference-capture').onclick = () => command('/upright-reference/capture');
$('cg-capture').onclick = () => command('/lateral-cg/capture');
$('cg-save').onclick = () => {
  const raw = $('cg-shift').value.trim(), mm = Number(raw);
  if (!raw || !Number.isFinite(mm) || Math.abs(mm) > 2) {
    $('message').textContent = referenceResultMessage('lateral_cg_shift_invalid'); return;
  }
  return command('/lateral-cg/set?shift_mm=' + encodeURIComponent(raw));
};
if ($('stop')) $('stop').onclick = () => { poseCancelled = true; return command('/stop'); };
if ($('reconnect')) $('reconnect').onclick = () => refresh(true);
if ($('usb-diag')) $('usb-diag').onchange = async () => {
  const enabled = $('usb-diag').checked;
  commandInFlight = true; controls();
  try {
    await request(`/diagnostics/usb?enabled=${enabled ? 1 : 0}`, {method:'POST', kind:'text'});
    $('message').textContent = enabled ? 'USB診断を有効にしました。' : 'USB診断を停止しました。';
  } catch (error) { $('message').textContent = error.message; }
  finally { commandInFlight = false; await refresh(); }
};
$('clear').onclick = () => command('/clear');
$('download').onclick = download;
$('cancel').onclick = () => { cancelTransfer = true; };
$('work-calculate').onclick = calculateWheelWork;
$('work-pulses').onclick = () => saveWheelWorkCsv('pulses');
$('work-intervals').onclick = () => saveWheelWorkCsv('intervals');
$('work-kinematics').onclick = () => saveWheelWorkCsv('kinematics');
$('csv').onclick = () => saveBlob(new Blob(['\ufeff', footCsv(completedFoot)], {type: 'text/csv;charset=utf-8'}), completedName.replace(/\.rwlog$/, '_foot.csv'));
$('preview').onclick = capturePreview;
$('preview-save').onclick = () => saveBlob(new Blob([JSON.stringify(previewEvidence, null, 2)], {type:'application/json'}), `freefoot-vision-diagnostics-${previewEvidence.sequence}.json`);
$('diagnostics').onclick = () => saveBlob(new Blob([JSON.stringify(latest, null, 2)], {type: 'application/json'}), 'freefoot-diagnostics.json');
$('pose-base').onclick = () => capturePose(true);
$('pose-add').onclick = () => capturePose(false);
$('pose-cancel').onclick = () => { poseCancelled = true; };
$('pose-save').onclick = () => {
  saveBlob(new Blob([JSON.stringify(poseSession, null, 2)], {type:'application/json'}), 'freefoot-pose-comparison.json');
  poseSaved = true; controls();
};
$('pose-reset').onclick = () => {
  if ($('pose-reset').disabled) return;
  poseSession = null; poseSaved = false; renderPoses(); controls();
  $('pose-status').textContent = '胴体と両足を直立させ、基準を取得してください。';
};
try {
  const until = Number(sessionStorage.getItem(offlineKey));
  if (until > Date.now() && until <= Date.now() + 30000) setOffline(until - Date.now());
} catch (_) {}
poll();

</script></html>
)FREEFOOT";
