# 歩行動画の MASt3R 解析

MP4の指定時刻から隣り合う画像を取り出し、MASt3R が推定した三次元点と、画像間で相互に一致した特徴点を保存します。本体への書き込み、歩行測定、RWLOGの処理から独立した、PC上のオフライン解析です。

## 準備

Git、64ビットのPython 3.9〜3.12、初回のインターネット接続が必要です。リポジトリのルートから実行します。インストーラーは `analysis/local/mast3r/` 内に上流ソース、仮想環境、モデルを置きます。モデルのダウンロードだけで約2.75 GBです。

```powershell
python analysis/mast3r/install.py --device cpu
$venvPython = "analysis/local/mast3r/venv/Scripts/python.exe"
```

NVIDIA GPUを持つマシンでは、新しい保存先に `python analysis/mast3r/install.py --device cuda --runtime analysis/local/mast3r-cuda` としてCUDA 12.1用PyTorchを選べます。インストーラーがGPUを認識できることを確認します。この場合はその環境のPythonを使い、推論にも `--device cuda --source analysis/local/mast3r-cuda/source --weights analysis/local/mast3r-cuda/model` を指定します。Linuxの仮想環境のPythonは `venv/bin/python` です。

上流コードは `f5209afc300cec36239a7ac992263f36847bbba0` に固定しています。DUSt3R と CroCo はそのコミットが指定するサブモジュールを再帰取得します。NAVER公式モデルのリビジョンは `06e7259f34c3060f322df5cb0c7b9094f57e41fc` です。インストール時に safetensors のSHA-256を照合します。上流コードの使用条件は CC BY-NC-SA 4.0 で、重みは学習データ固有の利用条件も伴います。[公式実装](https://github.com/naver/mast3r/tree/f5209afc300cec36239a7ac992263f36847bbba0) · [モデル](https://huggingface.co/naver/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric/tree/06e7259f34c3060f322df5cb0c7b9094f57e41fc) · [重みの利用条件](https://github.com/naver/mast3r/blob/f5209afc300cec36239a7ac992263f36847bbba0/CHECKPOINTS_NOTICE)

## 既存の歩行動画で使う

下記は試行72996579の動画時間12.9129秒と13.0130秒を指定する例です。`--output` には毎回新規または空のフォルダーを指定します。

```powershell
& $venvPython analysis/mast3r/reconstruct_video.py --video analysis/local/72996579/shared-video.mp4 --times 12.9129 13.0130 --output analysis/local/mast3r/run-72996579 --device cpu
```

先に時刻とフレームだけを確認するには、上と別の出力先で `--prepare-only` を加えます。この確認には OpenCV のみを使います。ローカル動画を先頭からデコードし、最も近い時刻のフレームを選びます。`summary.json` に依頼時刻、選ばれたフレーム番号、時刻とその種別を記録します。利用可能なPTSが得られず公称フレームレートで時刻を補った場合は `nominal_fps` と記録します。

### 背景と機体を分ける

機体の外形マスクがあれば、各時刻1枚の元動画サイズの白黒画像を `--masks frame387-mask.png frame390-mask.png` で渡します。白の領域が機体、黒の領域が背景候補です。上流のリサイズ・中央クロップをマスクにも適用します。矩形で概算する場合は `--roi x0 y0 x1 y1` に、出力の `view0.png`、`view1.png` 上で共通に使える正規化座標（各軸0〜1）を指定します。矩形の内側には機体の隙間に見える床も含まれます。対応する両端が同じ領域にある点と、境界をまたぐ点を区別します。

### 出力の読み方

| ファイル | 内容 |
| --- | --- |
| `summary.json` | 元フレームと動画時刻、推論デバイス、点の数。領域ごとに画像上の変位の中央値をモデル画像のピクセル単位で表示 |
| `pair-00-01/view0.png`, `view1.png` | ネットワークへ入れた座標系の色画像。対応点を描くときの基準 |
| `pair-00-01/matches.csv` | 2時刻の対応画素、各点のネットワーク推定三次元座標、点群の信頼度、領域ラベル |
| `pair-00-01/sparse.ply` | 相互対応を得た1枚目の画素のモデル推定点。CloudCompare等で開けるRGB付きPLY |
| `pair-00-01/dense.ply` | 両時刻のモデル推定点。各ペアの1枚目のカメラ座標系 |
| `dense_roi.ply`, `dense_outside_roi.ply` | マスクまたは矩形を渡したときの、内側と外側の密点群 |

2枚のRGB画像は長辺512で処理します。PLYと対応点には有限・正の奥行きと点群信頼度（既定値1.5以上）の条件を適用します。相互対応点はさらに各画像の端3画素を除いています。信頼度はネットワークのスコアで、実験の測定誤差を表す値ではありません。3時刻以上を渡すと隣接ペアごとに保存します。ペア間に共通の基準が必要な全体再構成には、別途カメラ姿勢・スケールの整合が必要です。

固定カメラに向かって機体が歩く動画では、画像上の足の変位は**足の運動とカメラとの位置関係**を反映します。静止背景の対応は画像のぶれの確認に使えます。単一視点でカメラの並進が小さいと幾何学的に奥行きを決める視差も小さく、出力は学習した形状に依存します。足裏と床の隙間をmmで読むには別途スケールと奥行きの検証が必要です。この解析から得られる接地・支持側は候補で、左右の鉛直荷重や荷重移行時刻の測定には独立した検証が必要です。
