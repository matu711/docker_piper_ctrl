# カメラの接続確認とプレビュー

## 実行環境

SDK用Dockerコンテナ内で実行する。OpenCVとNumPy、およびGUI表示用のX11接続が必要。
ホストから起動中のコンテナへ入る場合は、次を実行する。

```bash
docker exec -it piper-env-image-matu bash
cd ~/double_PiPER/src/test
```

`run.sh`は起動時にホストに存在する`/dev/video*`をコンテナへ渡す。
コンテナ内にも対象デバイスが存在することを確認する。

## 確認済みのデバイス割り当て

調査時の割り当ては以下の通り。接続・再起動によって番号が変わる場合があるため、
実行前に`v4l2-ctl --list-devices`で確認する。

| デバイス | カメラ・用途 | 対応形式 | 調査結果 |
| --- | --- | --- | --- |
| `/dev/video0` | RealSense D455 深度 | `Z16` | V4L2では取得可能。使用中のOpenCV 4.7.0では開けない |
| `/dev/video2` | 同じRealSenseの赤外線 | `GREY`、`UYVY`など | OpenCVで取得可能。独立した別カメラではない |
| `/dev/video4` | RealSense D455 カラー | `YUYV` | OpenCVで取得成功 |
| `/dev/video6` | Dabai DC1（シリアル: CC1WC5201HN） | `MJPG`、`YUYV` | OpenCVで取得成功 |
| `/dev/video8` | Dabai DC1（シリアル: CC1WC5200RJ） | `MJPG`、`YUYV` | OpenCVで取得成功 |

`/dev/video1`、`3`、`5`、`7`、`9`はメタデータ用ノードであり、映像表示には使わない。
`front`、`wrist1`、`wrist2`は表示上の役割名。実際の設置位置に合わせて引数を入れ替える。

## 3台を同時表示する

```bash
cd ~/double_PiPER/src/test
python3 test_triple_camera.py \
  --front /dev/video4 \
  --wrist1 /dev/video6 \
  --wrist2 /dev/video8 \
  --width 640 --height 480 --fps 15
```

3台の映像を横並びで表示する。プレビュー画面上で`q`を押すと終了する。
ROSの起動は不要で、ロボットへの動作指令も送らない。

スクリプトの既定値は`front=/dev/video6`、`wrist1=/dev/video0`、`wrist2=/dev/video2`なので、
今回の構成では上記の引数を明示すること。

### 映像形式とFPS

既定の要求形式は`MJPG`。Dabaiは対応しているが、RealSenseカラーは`YUYV`のみ。
調査ではRealSenseは`MJPG`要求時も`YUYV`へ切り替わり、取得できた。
起動時に表示される実際のFourCC・解像度・reported FPSを確認する。

640×480のRealSenseカラーは20 FPSに対応せず、20 FPS要求時は15 FPSになった。
そのため上記コマンドでは15 FPSを指定している。
reported FPSは実測の取得速度を保証しない。

## 2台を同時表示する

```bash
cd ~/double_PiPER/src/test
python3 test_dual_camera.py \
  --front /dev/video4 \
  --wrist /dev/video6 \
  --width 640 --height 480 --fps 15
```

## 露光時間・ゲイン・明るさ・ホワイトバランスを変更する

以下はコンテナ内で実行する。対象はカラー映像の`/dev/video4`、`/dev/video6`、`/dev/video8`。
最初に対象カメラを選び、対応項目・範囲・現在値を確認する。

```bash
PIPER_CAMERA=/dev/video4
v4l2-ctl -d "$PIPER_CAMERA" --list-ctrls-menus
v4l2-ctl -d "$PIPER_CAMERA" \
  --get-ctrl=auto_exposure,exposure_time_absolute,gain,brightness,white_balance_automatic,white_balance_temperature
```

`--list-ctrls-menus`の`min`・`max`・`step`は設定可能な範囲と刻み、
`default`は既定値、`value`は現在値を表す。
`flags=inactive`の項目は、関連する自動制御を解除してから変更する。
変更前の値を控えておけば、その値を指定して元に戻せる。

### 実機で確認した範囲

| 項目 | RealSenseカラー `/dev/video4` | Dabai `/dev/video6`・`/dev/video8` |
| --- | --- | --- |
| `auto_exposure` | `1`: 手動、`3`: 自動露光 | 同左 |
| `exposure_time_absolute` | 1〜10000、step 1 | 1〜5000、step 1 |
| `gain` | 0〜128、step 1 | 0〜100、step 1 |
| `brightness` | −64〜64、step 1 | 同左 |
| `white_balance_automatic` | `0`: 手動、`1`: 自動 | 同左 |
| `white_balance_temperature` | 2800〜6500 K、step 10 | 2800〜6500 K、step 1 |

コントロール名は使用中の`v4l2-ctl`の出力に合わせている。
環境によって`exposure_auto`や`exposure_absolute`など別名になる場合は、実際の一覧にある名前を使う。

### 露光時間を手動設定する

自動露光を解除してから設定する。以下は露光時間10 msの例。

```bash
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=auto_exposure=1
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=exposure_time_absolute=100
v4l2-ctl -d "$PIPER_CAMERA" --get-ctrl=auto_exposure,exposure_time_absolute
```

V4L2仕様では値1が100 µs（0.1 ms）で、100は10 ms、200は20 msに相当する。
露光時間はフレーム間隔にも制限されるため、一覧上の最大値がそのまま使用できるとは限らない。
[Linux V4L2仕様](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-camera.html)

### ゲインと明るさを変更する

自動露光を解除した上で、次はゲイン32・明るさ0にする例。
ゲインの数値は機種依存であり、同じ値でも機種間で同じ明るさになるとは限らない。

```bash
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=auto_exposure=1
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=gain=32
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=brightness=0
v4l2-ctl -d "$PIPER_CAMERA" --get-ctrl=gain,brightness
```

### ホワイトバランスを手動設定する

自動ホワイトバランスを解除してから色温度を設定する。以下は4600 Kの例。

```bash
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=white_balance_automatic=0
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=white_balance_temperature=4600
v4l2-ctl -d "$PIPER_CAMERA" --get-ctrl=white_balance_automatic,white_balance_temperature
```

### 自動制御へ戻す

```bash
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=auto_exposure=3
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=white_balance_automatic=1
v4l2-ctl -d "$PIPER_CAMERA" --get-ctrl=auto_exposure,white_balance_automatic
```

これは自動制御を有効にする操作で、明るさなど全設定の初期化ではない。
ゲイン・露光時間・色温度は、自動制御中にはカメラ側で調整される場合がある。

### 設定の反映を確認する

3台それぞれの設定を読み戻す。

```bash
for PIPER_CAMERA in /dev/video4 /dev/video6 /dev/video8; do
  echo "$PIPER_CAMERA"
  v4l2-ctl -d "$PIPER_CAMERA" \
    --get-ctrl=auto_exposure,exposure_time_absolute,gain,brightness,white_balance_automatic,white_balance_temperature
done
```

上記の3台プレビューを起動し、実際の明るさ・色味・動体のぶれも確認する。
このREADME追記時に実機の対応項目と範囲を読み取り確認したが、設定変更は実行していない。
USB再接続・電源再投入や別アプリの起動で設定が変わる場合は、その後も読み戻して確認する。
`unknown control`なら名前を、`inactive`なら自動制御の状態を、範囲エラーなら`min/max/step`を確認する。

## 開けない場合の確認

コンテナ内でデバイス分類と対応形式を確認する。

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video4 --list-formats-ext
v4l2-ctl -d /dev/video6 --list-formats-ext
v4l2-ctl -d /dev/video8 --list-formats-ext
```

ホスト側で他プロセスの使用状況とUSB接続速度を確認する。

```bash
sudo fuser -v /dev/video4 /dev/video6 /dev/video8
lsusb -t
```

`Could not open camera: /dev/video0`の場合、今回の構成では深度用ノードを指定していることが原因。
OpenCVの詳細ログでも`Pixel format of incoming image is unsupported by OpenCV`を確認した。
カラー表示には`/dev/video4`を指定する。

## 調査で確認した範囲

- コンテナ内のユーザー`dnp218`で各デバイスへの読み書き権限を確認した。
- コンテナ内でカメラを保持する他プロセスは見つからなかった。
- `video4`、`video6`、`video8`は640×480で単独・3台同時とも各5フレーム取得に成功した。
- RealSenseはUSB 3（5000 Mbps）、Dabai 2台はUSB 2（480 Mbps）で接続されていた。
- 今回の`video0`のオープン失敗は映像形式が原因。短時間の同時取得でUSB帯域による失敗は再現しなかった。

長時間の安定性、GUIでの実際の表示、3台のフレーム同期は、この調査では検証していない。
