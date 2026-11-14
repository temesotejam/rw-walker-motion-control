# v16 操作画面の途中切断修正

2026-11-14、PCの `http://192.168.4.1/` が初期文言「接続を確認中」のままになるとの相談を調査した。最初は機体の電源がOFFで到達できなかったが、ユーザーが電源を入れた後、状態JSONは正常に取得できた。実機の識別は `energy-pulse-lifecycle-v16`、状態は `READY_TO_MEASURE`、サンプル数0だった。

その状態でも操作画面の応答は `Content-Length: 95778` に対して16640バイトで切れ、`IncompleteRead(16640 bytes read, 79138 more expected)` となった。ページ内JavaScriptの開始位置は19737バイトで、そこまで届いていない。固定の「1.0.40」は説明文であり、実機の版を示す値ではない。

`BoundedWriteWebServer::pacedWrite()` は送信が進んでいても全体1200msで停止していた。今回、進捗停止3000msと全体15000msの上限を分けた。ソケット送信は引き続き非ブロッキングで、部分送信・再試行・タスクへのyield・異常時の切断を維持する。この時間設定は通信の実装上の上限であり、歩行のモデル値ではない。`stop()` 自体やHTTP処理全体の時間を保証する上限でもない。

修正前の実機ではHTML欠損を確認できたが、時計上限以外の通信要因もあり得る。修正版を書き込んだ後の実機で、HTMLの全長受信と画面の状態更新を確認する必要がある。

実コードをホストでコンパイルする `python tools/test_bounded_web_transport.py` では、RAM/flash両方の送信経路について、約96KBの低速受信・部分送信・再試行・時計wrapで全バイト一致を確認し、無進捗・永続的な少量進捗・切断・致命エラーでは有限時間で終了した。v16のパルス開始/終了および実入力経路の連結テストもPASS。

専用環境は `atoms3cam-energy-pulse-lifecycle-webfix`、識別は `energy-pulse-lifecycle-v16-webfix1`、準備コマンドは `python tools/prepare_energy_pulse_lifecycle_webfix_flasher.py`。配信先は `http://127.0.0.1:8767/energy-pulse-lifecycle-webfix/`。元v16の配信物は保持し、歩行のエネルギー選択・姿勢推定・パルス制御は変更しない。今回の修正は操作画面の通信のみで、歩行改善を検証したものではない。

PlatformIO build SUCCESS（RAM166780/327680、Flash1879485/3342336）。配信ページ・manifest・4つの書き込み部品をHTTP200/ローカル全バイト一致で確認。firmwareは1879856bytes、SHA256 `b01645d1e729d460044461d53b2f9440a2b9030152ad1d9895636a9a542fa351`。元v16のfirmware SHA256 `f41dc856fc00d43d15d49f90c32e790b8947a296454daf9dc047c00a051a36c2` も維持。今回は書き込み未実施。
