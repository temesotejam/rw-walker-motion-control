# RW Walker Motion Control

リアクションホイール歩行器の制御プログラムの保存用リポジトリです。GitHubへの更新で自動ビルドやデプロイは行いません。実機への書き込みはPCで行います。

初期データは atoms3r-lateral-energy-control の試験版ソース 9939ff1da6550ac9de21451c92c1106edd8b374f を基準に移行しました。試験版の ID は angle-early-eta-v1 です。今回のログに基づく改善はまだ含まれていません。

PlatformIO の設定は platformio.ini を参照してください。早期入力試験のビルドには FOOT_EARLY_TRIAL=1 と ANGLE_EARLY_TRIAL=1 が必要です。通常版のビルドと混同しないでください。

## Local trial build

Use platformio run -e atoms3cam-angle-early to select the angle-early trial flags explicitly. The toms3cam environment builds the ordinary version without those flags. The trial environment preserves the same base board and library settings.
