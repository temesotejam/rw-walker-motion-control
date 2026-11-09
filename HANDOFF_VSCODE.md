# リアクションホイール歩行器：VS Code / Codex 引き継ぎ

更新日：2026-11-09（日本時間）

> **この文書が引き継ぎ元です。** VS Code 側の Codex には元のチャットが見えません。会話履歴がある前提で推測せず、このファイルと作業ツリー、実機・ログの確認結果を基に作業してください。ログやビルド成果物が見当たらない場合は、存在を仮定せず実際の状態を報告してください。

## 最初にやってほしいこと

このフォルダを開いた VS Code の Codex で、現在の変更を確認し、下記の `atoms3cam-angle-smooth` 試験版をこの PC の通常の PlatformIO 環境でビルドしてください。成功したら今回のバイナリをローカル Web 書き込みページに反映し、ユーザーが BOOT ボタンを長押しせず、従来の Web ページのように書き込めるところまで進めてください。Web 書き込み後は実機ログを取得し、歩行改善を続けます。

この作業フォルダにはユーザーとの作業中の未コミット変更があります。削除、リセット、上書きで消さず、まず `git status` と差分を確認してください。`.pio` はビルド環境・成果物で Git 管理外です。必要なキャッシュやユーザーの実験データを勝手に消さないでください。

## 研究の目的とユーザーの希望

- AtomS3R-CAM、Unit Roller485、リアクションホイールと円弧足を使った準受動歩行器。自然な左右揺動を活かし、必要なタイミングで短時間入力して歩行を継続する。
- 目標は、足を交互に自然に振り出し、長く、できるだけ真っすぐ歩くこと。少し足を擦る歩行も許容する。計測上の平均角が目標に近いだけで成功とはしない。
- 実際の問題は、良い動きが最初に出ても途中で止まったり、左右差・旋回が出たりすること。失速してから対処するだけでは遅く、足の位置も活用したい。ただし現状では現在の傾き・角速度を主に使い、カメラは補助でもよい。
- 初めの直立からの1歩目は短く、その次は概ね倍の移動幅となりうる。歩行中は足の前後の往復全体で比較し、初回の半歩を通常周期と同一視しない。
- 足マーカーはカメラ基準の相対値。左右の比較に有用だが、地面基準の移動距離や旋回角そのものではない。検出・校正・時刻の品質を確認し、欠損や誤検出時に強い制御を入れない。
- GitHub `temesotejam/rw-walker-motion-control` はバックアップ用途。GitHub Actions のビルド・Pages デプロイ待ちを避け、この PC でビルドと書き込みを行う方針。外部への push は勝手に行わず、必要な状態と認証を確認する。
- BOOT ボタンが機構上押しにくい。Web ページから USB ポート選択後に書き込む従来の操作を優先し、長押しを前提にしない。

## 現在の状況

1. このリポジトリは `atoms3r-lateral-energy-control` のソース `9939ff1da6550ac9de21451c92c1106edd8b374f` を基準に移行し、後述の未コミット変更を加えています。Git HEAD は `2e998c1`（確認時点）。
2. 旧試験 `angle-early-eta-v1` のログ `C:\Users\arika\Downloads\energy_control_autonomous_run_1_63446214.rwlog` を解析。
3. `angle-early-weak-zero-v2` を実装。到達側の対応付けられた直近ピークが段階目標より0.5°以上不足し、通常候補の幅が早期入力の幅を上回る場合、15 ms上限の早期入力を見送り通常 ZERO で再判断する。ホスト単体・runner 抽出結合テストを通過。
4. v2 はユーザーの通常 PowerShell からビルドに成功し、ローカル Web ページで実機書き込みできたとユーザーから報告あり。PlatformIO CLI upload は COM24 で `Stub running...` の後 `Unable to verify flash chip connection (No serial data received.)` となったが、Web 方式では成功。
5. v2 の今回のログ `C:\Users\arika\Downloads\energy_control_autonomous_run_1_42043562.rwlog`（Library にも同名）を解析。CRC 正常、build ID `angle-early-weak-zero-v2`、約30秒、71ピーク。早期入力実行18回、通常 ZERO 実行50回。前回は各48回、26回。後半（15〜30秒）平均ピークの大きさは負側5.54°、正側5.77°（前回4.59°、5.86°）。各1試行で条件差もあり、因果や歩行改善の証明ではない。
6. ユーザー観察：「旋回するタイミングも多かったが、なんだかんだでまっすぐに進んでいた。長くは歩いていないし自然には歩けていない」。ログの30秒は測定時間であり、30秒連続して自然歩行できたとは意味しない。
7. v2 の約4.5〜6.7秒では入力おおむね24/15 msで比較的安定した揺れがあった。後半では49〜64 msの反対側入力後に、戻りの角速度が落ち、19.124秒・22.697秒・24.784秒に負側極小ピークが記録された。長い入力が原因か、他の接触・検出要因かは未確定。カメラには有効でない周期もある。

## 現在の次試験 `angle-early-smooth-v3`

- `platformio.ini` の `[env:atoms3cam-angle-smooth]` は `atoms3cam-angle-early` を継承し `WALK_SMOOTH_TRIAL=1` を有効化。診断 build ID は `angle-early-smooth-v3`。
- `src/tilt_priority.h` と `src/experiment_runner.cpp` に、同一到達側の直近1.5秒以内に成功した通常入力を基準に、次の通常入力の増加を最大10 msに制限する試験を追加。左右は独立。早期・救済・始動入力は履歴に入れない。入力を減らす判断、coast、既存の角度・装置保護は優先。失敗等では対応する履歴をクリア。
- 足・車輪の候補評価が同じ上限を参照するように実装。足マーカー値は今回この新しい上限の計算に直接使用していない。自然歩行や旋回の改善は実機未検証。
- `tools/test_tilt_priority.cpp` と `tools/test_tilt_priority_integration.py`、`tools/test_angle_runner_integration.py`、`tools/test_angle_build_wiring.py` でホストテスト PASS。ただし組み込み向けビルドは**まだ成功しておらず**、v3 のバイナリも未生成・未書き込み。変更は試験であり、効果は保証できない。
- 前の版と今回の版の両方のソース変更は作業ツリー内の未コミット変更。`git status` に表示される変更や未追跡ファイルは作業の一部であり、自分のものではないからという理由で削除・破棄しない。
- 機体の過度な加速や停止がないか、最初の実験は周囲を空け、ユーザーが停止できる状態で確認。目標8°を基準に、強弱を無闇に上げず試験結果から次の変更を判断する。

## ローカルビルドと Web 書き込み

この ChatGPT 作業環境では、ESP32用 `xtensa-esp32s3-elf-g++.exe --version` 自体が Windows の「システム DLL の無効な再配置」エラーとなった。一方、ユーザーが通常の PowerShell で同じコンパイラを起動すると正常にバージョン 8.4.0 を表示し、前版のビルドにも成功した。PC自体がビルド不能と断定しない。VS Code の環境で再確認する。

以前の試行で `.pio/local-core` にコピーした長いパスのツールチェーンを使ったとき、`bits/cpu_defines.h` が見つからないエラーになった。同じファイルは存在していたため原因は未確定。**ユーザー環境の元の `C:\Users\arika\.platformio` を使うビルドは成功済み**。

PowerShell の作業ディレクトリをこのリポジトリにして実行する例：

```powershell
cd "C:\Users\arika\Desktop\workspace_research\Projects\rw-walker-motion-control"
$env:PLATFORMIO_CORE_DIR = "$env:USERPROFILE\.platformio"
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run -e atoms3cam-angle-smooth
```

成功後に `python tools\prepare_smooth_flasher.py` を実行する。ビルド結果に `angle-early-smooth-v3` が含まれることを検査し、`.pio/build/atoms3cam-angle-smooth` の bootloader / partitions / firmware と、元の Arduino ESP32 2.0.16 環境の `boot_app0.bin` を `flasher/smooth/` へ準備する。ビルドと準備を連続で行う `build-smooth-web.cmd` もある。環境とパスを確認して使うこと。準備前に `flasher/smooth/` を開いても新しい試験版は存在しない。

この作業環境では最後にビルドを再試行しており、`atoms3cam-angle-smooth` 用の中間フォルダが存在する可能性があるが、**フォルダの存在はビルド成功を意味しない**。PlatformIO の `SUCCESS` と生成されたファームウェアの build ID を確認する。書き込み前には Web マニフェストの版と各ファイルの存在を確認し、以前のページや古いバイナリを新しい版と取り違えない。

従来のページは `flasher/`、旧版 v2 のバイナリは `flasher/firmware/`。新しいページは `flasher/smooth/`、新しいバイナリは `flasher/smooth/firmware/`。どちらも generated binary は `.gitignore` 対象。旧ページのまま書き込むと v2 を再書き込みするので、新版の URL と build ID を確かめること。

ユーザーが通常 PowerShell で以下を起動し、そのウィンドウを開いたまま Chrome/Edge から書き込めた：

```powershell
cd "C:\Users\arika\Desktop\workspace_research\Projects\rw-walker-motion-control\flasher"
python -m http.server 8767 --bind 127.0.0.1
```

v3 の準備後の URL は `http://127.0.0.1:8767/smooth/`。ESP Web Tools 10.1.1 のモジュール取得にはインターネット接続が必要。`flasher/soft-boot.js` の「ボタンを押さずに書き込み準備」は、実機側に対応機能があり、かつ停止中である場合に限り有効。必要な場合にのみ使い、通常は書き込みボタンでポートを選ぶ。実機接続と書き込み完了は画面やログで確認し、成功するまで完了と報告しない。

## 次のログの見方

- 機体から RWLOG を保存。新しい build ID `angle-early-smooth-v3`、CRC、測定の完了・終了理由を確認。
- 先行する v2 の `42043562` と、側別・周期別に入力幅、ゼロクロス角速度、ピーク、失速に近い場面、足マーカーの品質と左右幅を比較。単純な全期間平均だけでなく、序盤の良い歩行が継続したかを確認。
- ユーザーの実観察（自然さ、足が出る方向、旋回方向、歩行の持続）と時系列を突き合わせる。角速度積分やカメラ相対値を根拠に実際の旋回方向を断定しない。
- 作業中、未コミット変更を保護し、成果の検証結果・未確認事項を正直に伝える。
