# 足画像の受渡し待ち短縮試験版 v21（2026-11-16）

`walking-foot-frame-ready-v21` を実装・ビルドし、専用ページを準備した。
実機書込み・短縮時間・画質・足角精度・歩行改善は未確認。

- 環境: `atoms3cam-foot-frame-ready`
- ページ: http://127.0.0.1:8767/foot-frame-ready/
- 比較版: http://127.0.0.1:8767/foot-fast-downloadfix2/

## 根拠と変更

直近の52370695/202970048ログでは、driver時刻→画像受取の中央値が
50.089/50.0885 ms、認識完了まで約60.1 ms、制御受取まで約61.2/61.0 ms。
ただしdriver時刻は取得開始の時刻で、校正済みの露光時刻ではない。
約50 msの全部を不要な待ちと断定できず、画素の読出し時間も含む。

使用SDKの[Espressif cam_hal.c (f0bb429)](https://github.com/espressif/esp32-camera/blob/f0bb429/driver/cam_hal.c)
は、raw画像をDMA EOFごとにコピーするが、完成画像を次のVSYNCイベントで
queueへ送る。[ESP32-S3のDMA実装](https://github.com/espressif/esp32-camera/blob/f0bb429/target/esp32s3/ll_cam.c)
ではEOFはバイト数に基づき、DMA停止後もVSYNC割込みは別途有効である。
このため、全画素が揃った後の同期待ちを省く余地がある。

v21はGC0308・320×240・グレースケール・XCLK20MHz・内部DMAコピーに限定し、
76800 bytesが揃ったEOFでDMAを止め、完成フレームを認識側へ渡す。
次のVSYNCまでIDLEに置き、遅れて届くEOFを無視して、次画像の途中から
取得を再開しない。認識側が借用中のバッファは再利用せず、queue満杯時は
queue内の旧画像だけを置き換える。不完全画像は従来のサイズ検証で除外する。

センサー設定、画角、露光設定、画像サイズ、足角度校正、driver timestampの
定義、40ms worker周期、二重バッファ、認識アルゴリズムは継承する。
エネルギー制御はv19、ログ取得修正はv20-downloadfix2を継承する。
今回の変更は認識計算の高速化や未来角度の外挿ではない。

## 診断

RWLOG metadataの `foot_observation` に次を追加した。
いずれも起動後累計で、試行単位ではない。独立カウンタの読取であり、
稼働中に一組として同時刻の値であることは保証しない。

| 項目 | 意味 |
| --- | --- |
| `frame_publication` | `complete_qvga_y8_dma_before_vsync_v21` |
| `dma_early_frames` | 完全EOFからqueueへ渡せた枚数 |
| `dma_vsync_frames` | 従来のVSYNC経路で渡せた枚数 |
| `dma_queue_drops` | queue置換・送出失敗で破棄した枚数 |
| `dma_last_full_age_us` | 最後の完全EOFコピー終了時点のdriver開始からの経過時間 |

`dma_last_full_age_us` は最後の1枚の値で、試行の中央値ではない。
既存の各フレームの `frame_us`、`delivered_us`、`ready_us`、
`control_observed_us` を使って、前版と同じ定義で遅延分布を比較する。

## 検証結果

- 実HAL `cam_task` のホスト試験PASS。完全/不足画像、余分なEOF、queue置換、
  借用中バッファ、再送失敗/consumerとの競合、JPEG/PSRAM/異なる幅のfallback、
  overflow、32bit us wrapを確認。
- 合成イベント「40msで全画素完成、50msで次VSYNC」では早期受渡しを確認。
  **この10msはテスト入力であり、実機の短縮時間ではない。**
- 既存の実v20取得・認識ループ回帰、3版のinstaller分離試験PASS。
- 実v19/v20/v21 metadata生成・CRC・immutable export・CSV変換のホスト試験PASS。
  v21最大fixtureは986159 bytesで1MiB上限内。足1280、energy192行を保持。
- Arduino2.0.16/IDF4.4.7でビルド成功。RAM127644、Flash1924989 bytes。
- SDKカメラarchive SHA256をbuild前に照合。異なるprivate ABIはbuildを拒否する。
  mapでプロジェクトHALの使用とSDK低レベルDMAの保持を確認。
  ELFでcam_config→既存task priority wrapperも確認。
- firmware1925360 bytes、SHA256
  `cd975679b7fb1fece7d7bf1922da3a5d19a68c6fbc96af0b460a0e29bc095b02`。
  ページ・manifest・補助JS・4partsの8ファイルをHTTP200/全バイト一致で確認。
  旧配信200ファイルは変更なし。

## 次の実機確認

未保存ログを保存してからv21を書き込む。最初は歩行測定を開始せず、
プレビューに欠け/横ずれがないこと、両足の角度が前後の手振りに追従すること、
通常の更新が続くことを確認する。画像品質異常なら比較版へ戻す。

同じ機体・照明条件でログを取り、driver→受取/認識完了/制御の中央値・p95、
更新間隔、両足の欠落率と急変、early/queue discardの変化を比較する。
全画素到着自体が約50msなら、この修正の短縮幅は小さい。その場合は
センサー読出し周期の変更を別試験として検討し、露光不足・画角/校正への
影響を同時に検証する。現段階で「50msが解消」「歩行が改善」とはしない。
