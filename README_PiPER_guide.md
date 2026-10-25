# 双腕PiPER：カメラ確認・データ収集・LeRobot変換・π0.5学習・推論

指定された8つのREADMEを、作業の流れに沿ってまとめた統合版です。
元の説明・コマンド・設定例をできるだけ残し、現在のファイル配置と異なる部分を修正しました。
各編の「第1章」「5.2」などの番号は、その編の中の番号を指します。
元ファイルは[元READMEとの対応](#sources)から参照できます。

確認日：2026-10-25。ローカルのファイル・コードを読み取り照合しました。
Docker起動、カメラ設定変更、CAN操作、データ転送、学習、モデルロード、実機動作は実行していません。

- [A. 実行環境とパスの変更点](#paths)
- [B. Docker・SDK・CAN・ROSの準備](#setup)
- [C. カメラの接続確認とプレビュー](#camera)
- [D. 双腕リーダーフォロワーによるデータ収集](#collect)
- [E. MCAPからLeRobot形式への変換](#conversion)
- [F. π0.5 Fine-tuning](#training)
- [G. Bundle作成・転送・推論のStart／Stop](#inference)
- [H. 推論の詳細確認・停止・トラブル対応](#details)
- [I. 旧手順・step 1000固有の補足](#legacy)
- [元READMEとの対応](#sources)

<a id="paths"></a>

## A. 実行環境とパスの変更点

### 操作する環境

| 実行場所 | 主な用途・パス |
|---|---|
| 収集・推論用PCのホスト | このリポジトリ：`/home/dnp218/docker_piper_ctrl` |
| PiPER Docker | コンテナ名：`piper-control-dev`、イメージ：`piper-control:noetic` |
| PiPER Docker | リポジトリ：`~/docker_piper_ctrl`、ROSワークスペース：`~/double_PiPER` |
| 推論用PCのホスト | Runtime：`/home/dnp218/piper_vla_runtime`、Bundle：`/home/dnp218/piper_checkpoints` |
| 学習用PC（rlsv）のホスト | `/home/rlsv/project` |
| 学習用Docker | `/workspace/piper_pi0`、`/workspace/openpi`、`/workspace/datasets` |

ホストとPiPER Dockerのパスは、`run.sh`で次のように対応します（既定ユーザー`dnp218`）。

| ホスト | PiPER Docker |
|---|---|
| `/home/dnp218/docker_piper_ctrl` | `/home/dnp218/docker_piper_ctrl` |
| `/home/dnp218/docker_piper_ctrl/double_PiPER` | `/home/dnp218/double_PiPER` |
| `/home/dnp218/docker_piper_ctrl/piper_dataset` | `/home/dnp218/piper_dataset` |
| `~/.config/piper` | `/home/dnp218/.config/piper` |

新しいターミナルからPiPER Dockerへ入る場合は、ホストで実行します。

```bash
docker exec -it piper-control-dev bash
sorc_devel
```

### 照合して修正・区別した箇所

| 元の記載 | 統合版での扱い・確認結果 |
|---|---|
| `/home/dnp218/docker_piper_env` | この作業用リポジトリは`/home/dnp218/docker_piper_ctrl`。旧ディレクトリも実在するため、混用しない |
| `piper-env-image-matu` | 現在の`run.sh`のコンテナ名`piper-control-dev`へ変更 |
| `Dockerfile`のalias | 現在は[assets/piper_shell.sh](assets/piper_shell.sh)の関数定義を参照 |
| `collect_dual_piper.launch` | 現在は存在しないため、[collect_dual_piper_mcap2.launch](double_PiPER/src/piper/launch/collect_dual_piper_mcap2.launch)を使用。キー操作・保存先・サービスも変更 |
| `piper_dual_leader.py` / `piper_dual_follower.py` / `record_dual_piper.py` | 現在の収集構成は`piper_mcap2_arm.py` / `piper_mcap2_coordinator.py` / `record_dual_piper_mcap2.py` |
| 収集先固定、`dataset_root`無効 | MCAP2では`dataset_root`が有効。既定はDocker内`~/piper_dataset/dual2` |
| 日付名のMCAPが保存先直下に並ぶ | MCAP2では`episode_<開始時刻ns>/episode_<開始時刻ns>.mcap`。変換前の整理方法を修正 |
| `t`でタスク入力、`q`でレコーダーのみ終了 | MCAP2の`t`は教示モード切替、`q`はlaunch全体終了。タスクは起動引数`task`で指定 |
| カメラが`video4/6/8`、または`video4/2/0` | 別の調査・接続構成。番号だけで統一せず、物理視点・形式を確認して引数を指定 |
| 学習Checkpointの`...bimanual_lora`と推論例の`...bimanual_delta_lora` | 異なる実験の例。実際のYAML・run・Bundleに合わせる。ディレクトリ名を機械的に置換しない |
| step 1000の「互換性対応は未実装」 | 現在のRuntimeソースには登録互換処理・観測アダプターが存在する。実行確認とは区別する（I編参照） |

学習用PCのパス・IPアドレス・コンテナIDは元READMEの例を維持しています。
学習用PCへの接続と`/workspace`のマウントは今回確認していないため、実行先で確認してください。
MCAP2出力に対する外部変換スクリプトの対応も未検証です（E編参照）。

---

<a id="setup"></a>

## B. Docker・SDK・CAN・ROSの準備

本READMEでは、Docker環境の起動からROS Masterの起動、双腕リーダーフォロワーによる教示データ収集までの手順を説明する。

### 1. Preliminary（Docker環境の起動）

#### 手順1：ユーザー名の設定

`Dockerfile` および `run.sh` 内の `USR_NAME` を任意のユーザー名に設定する。

**Dockerfile**

```dockerfile
ARG USR_NAME=<任意のユーザー名>
```

**run.sh**

```bash
export USR_NAME=<任意のユーザー名>
```

両ファイルには同じユーザー名を設定する。

#### 手順2：Dockerイメージのビルド

ホストの`build.sh`があるディレクトリで実行する。

```bash
cd /home/dnp218/docker_piper_ctrl
```

```bash
sh build.sh
```

ビルドが正常に完了したことを確認する。

#### 手順3：Dockerコンテナの起動

```bash
sh run.sh
```

コンテナが正常に起動すれば、Docker環境の初回設定は完了となる。

**2回目以降の起動**

- 通常時：手順3から実行する。
- `Dockerfile` を変更した場合：手順2から実行し、Dockerイメージを再ビルドする。
- ユーザー名を変更する場合：手順1から実行する。

---

### 2. PiPER SDKのダウンロードとROSのビルド

PiPER SDKのダウンロード、ROSワークスペースのビルド、および環境設定を行う。以下のコマンドはDockerコンテナ内で実行する。

#### 手順1：PiPER SDKのダウンロードとROSのビルド

```bash
set_sdk
```

`set_sdk`はSDKの準備と`catkin_make`を行う自作関数である。現在のイメージには`/opt/piper_sdk`のSDKが導入済みで、`~/piper_sdk`がなければ既存SDKのimportを確認してビルドする。既存の`~/piper_sdk`がある場合はそこからインストールし、両方がなければ`1_0_0_beta`をcloneする。

#### 手順2：ROS環境の読み込み

```bash
sorc_devel
```

`sorc_devel` は、ROSのcatkinワークスペースに移動し、`setup.bash` を `source` して環境設定を読み込む自作コマンドである。

現在の`assets/piper_shell.sh`では、ROS Noeticの環境読み込みに加え、以下の処理を行う。

```bash
cd ~/double_PiPER
source devel/setup.bash
```

実際の実行ディレクトリやコマンド定義は、`assets/piper_shell.sh`の関数定義を参照すること。

**注意事項**

- `set_sdk` は初回セットアップ時に実行する。
- **新しいターミナルを開いた場合も、必ず `sorc_devel` を実行する。**
- 各自作コマンドの詳細は `assets/piper_shell.sh`の関数定義を参照すること。

---

### 3. CAN通信のアクティブ化

PiPERとPC間でCAN通信を行うため、USB-CANアダプタを1つずつ接続して設定する。`can0` から順に確認する。

#### 手順1：USB-CANアダプタの接続と確認

1つ目のUSB-CANアダプタを接続し、以下のコマンドを実行する。

```bash
sudo ethtool -i can0 | grep bus
```

**出力例**

```text
bus-info: 3-11:1.0
```

表示された `bus-info` の値（この例では `3-11:1.0`）を確認する。

#### 手順2：CANインターフェースのアクティブ化

`can_activate.sh` があるディレクトリで、確認したUSBポート番号を指定して実行する。

```bash
cd ~/double_PiPER
bash can_activate.sh can0 1000000 "USBポート番号"
```

**実行例**

```bash
bash can_activate.sh can0 1000000 3-11:1.0
```

- `can0`：CANインターフェース名
- `1000000`：CAN通信速度（1 Mbps）
- `3-11:1.0`：`bus-info` で確認したUSBポート番号の例

#### 手順3：残りのCANインターフェースの設定

2つ目以降のUSB-CANアダプタも1つずつ接続し、それぞれ認識確認とアクティブ化を行う。

**USBポート番号の確認**

```bash
sudo ethtool -i can1 | grep bus
sudo ethtool -i can2 | grep bus
sudo ethtool -i can3 | grep bus
```

**アクティブ化**

各コマンドの `"USBポート番号"` を、対応する `bus-info` の値に置き換える。

```bash
bash can_activate.sh can1 1000000 "USBポート番号"
bash can_activate.sh can2 1000000 "USBポート番号"
bash can_activate.sh can3 1000000 "USBポート番号"
```

**注意：** USB接続の状況によってCANインターフェース名やUSBポート番号が変わる場合がある。必ず1つずつ接続して、各アダプタの対応関係を確認すること。

---

### 4. ROSの起動

Dockerコンテナ内でROS Masterを起動する。

#### 手順1：ROS環境の読み込み

新しいターミナルをDockerコンテナ内で開いた場合は、以下のコマンドを実行する。

```bash
sorc_devel
```

#### 手順2：ROS Masterの起動

```bash
roscore
```

正常に起動すれば、ROSノード間の通信に必要なROS Masterが使用可能になる。

**注意事項**

- `roscore` を起動したターミナルは、ROSを使用している間は閉じないこと。
- 別のターミナルでROS関連コマンドを実行する場合も、事前に `sorc_devel` を実行すること。

---

---

<a id="camera"></a>

## C. カメラの接続確認とプレビュー

以下のデバイス表・実測結果は元README作成時の調査記録です。
現在の収集・推論launchの既定値は`front=/dev/video4`、`wrist1（左）=/dev/video2`、`wrist2（右）=/dev/video0`ですが、
この調査での`video0`は深度、`video2`は赤外線です。同じ接続状態ならカラー映像用の`video4/6/8`を使い、
左右の設置位置を確認して収集・推論の両方に同じ視点を指定してください。
現物のカメラ機種・設定範囲・デバイス番号を今回再測定したものではありません。

### 実行環境

SDK用Dockerコンテナ内で実行する。OpenCVとNumPy、およびGUI表示用のX11接続が必要。
ホストから起動中のコンテナへ入る場合は、次を実行する。

```bash
docker exec -it piper-control-dev bash
cd ~/double_PiPER/src/test
```

`run.sh`は起動時にホストに存在する`/dev/video*`をコンテナへ渡す。
コンテナ内にも対象デバイスが存在することを確認する。

### カメラの接続確認

プレビューを起動する前に、接続されているカメラと対応するデバイス番号を確認する。
ホスト側とDockerコンテナ内で、次のコマンドを実行する。

```bash
v4l2-ctl --list-devices
```

表示されたカメラ名と`/dev/video*`の対応を確認し、プレビューの引数に指定する。

### 確認済みのデバイス割り当て

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

### 3台を同時表示する

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

#### 映像形式とFPS

既定の要求形式は`MJPG`。Dabaiは対応しているが、RealSenseカラーは`YUYV`のみ。
調査ではRealSenseは`MJPG`要求時も`YUYV`へ切り替わり、取得できた。
起動時に表示される実際のFourCC・解像度・reported FPSを確認する。

640×480のRealSenseカラーは20 FPSに対応せず、20 FPS要求時は15 FPSになった。
そのため上記コマンドでは15 FPSを指定している。
reported FPSは実測の取得速度を保証しない。

### 2台を同時表示する

```bash
cd ~/double_PiPER/src/test
python3 test_dual_camera.py \
  --front /dev/video4 \
  --wrist /dev/video6 \
  --width 640 --height 480 --fps 15
```

### 露光時間・ゲイン・明るさ・ホワイトバランスを変更する

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

#### 実機で確認した範囲

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

#### 露光時間を手動設定する

自動露光を解除してから設定する。以下は露光時間10 msの例。

```bash
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=auto_exposure=1
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=exposure_time_absolute=100
v4l2-ctl -d "$PIPER_CAMERA" --get-ctrl=auto_exposure,exposure_time_absolute
```

V4L2仕様では値1が100 µs（0.1 ms）で、100は10 ms、200は20 msに相当する。
露光時間はフレーム間隔にも制限されるため、一覧上の最大値がそのまま使用できるとは限らない。
[Linux V4L2仕様](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-camera.html)

#### ゲインと明るさを変更する

自動露光を解除した上で、次はゲイン32・明るさ0にする例。
ゲインの数値は機種依存であり、同じ値でも機種間で同じ明るさになるとは限らない。

```bash
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=auto_exposure=1
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=gain=32
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=brightness=0
v4l2-ctl -d "$PIPER_CAMERA" --get-ctrl=gain,brightness
```

#### ホワイトバランスを手動設定する

自動ホワイトバランスを解除してから色温度を設定する。以下は4600 Kの例。

```bash
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=white_balance_automatic=0
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=white_balance_temperature=4600
v4l2-ctl -d "$PIPER_CAMERA" --get-ctrl=white_balance_automatic,white_balance_temperature
```

#### 自動制御へ戻す

```bash
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=auto_exposure=3
v4l2-ctl -d "$PIPER_CAMERA" --set-ctrl=white_balance_automatic=1
v4l2-ctl -d "$PIPER_CAMERA" --get-ctrl=auto_exposure,white_balance_automatic
```

これは自動制御を有効にする操作で、明るさなど全設定の初期化ではない。
ゲイン・露光時間・色温度は、自動制御中にはカメラ側で調整される場合がある。

#### 設定の反映を確認する

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

### 開けない場合の確認

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

### 調査で確認した範囲

- コンテナ内のユーザー`dnp218`で各デバイスへの読み書き権限を確認した。
- コンテナ内でカメラを保持する他プロセスは見つからなかった。
- `video4`、`video6`、`video8`は640×480で単独・3台同時とも各5フレーム取得に成功した。
- RealSenseはUSB 3（5000 Mbps）、Dabai 2台はUSB 2（480 Mbps）で接続されていた。
- 今回の`video0`のオープン失敗は映像形式が原因。短時間の同時取得でUSB帯域による失敗は再現しなかった。

長時間の安定性、GUIでの実際の表示、3台のフレーム同期は、この調査では検証していない。

---

<a id="collect"></a>

## D. 双腕リーダーフォロワーによるデータ収集

双腕リーダーフォロワーを起動し、左右の実測State・送信Actionと3台のカメラ画像を同期して収集する。
旧`README_collect.md`の第5章は旧実装向けのため、この編は現行MCAP2のコードに合わせている。

### 1. 使用ファイルと機器構成

| 役割 | ファイル |
|---|---|
| 双腕起動・収集 | [collect_dual_piper_mcap2.launch](double_PiPER/src/piper/launch/collect_dual_piper_mcap2.launch) |
| 各アームの制御 | [piper_mcap2_arm.py](double_PiPER/src/piper/scripts/piper_mcap2_arm.py) |
| 左右まとめた操作 | [piper_mcap2_coordinator.py](double_PiPER/src/piper/scripts/piper_mcap2_coordinator.py) |
| 双腕データ収集 | [record_dual_piper_mcap2.py](double_PiPER/src/piper/scripts/record_dual_piper_mcap2.py) |

| アーム | CANポート |
|---|---|
| 右Leader | `can0` |
| 右Follower | `can1` |
| 左Leader | `can2` |
| 左Follower | `can3` |

カメラ引数の既定はfront=`/dev/video4`、wrist1=`/dev/video2`、wrist2=`/dev/video0`。
要求は30 FPS、記録は20 FPS、保存画像は640×480。カメラごとの取得解像度などは機種により異なる。
C編の調査構成を使用する場合は`video4/6/8`から実際の設置位置に合わせて指定する。

起動前に4つのCANとカメラの対応を確認し、同じCAN・カメラを使用する他のlaunchやテストを終了する。

### 2. 収集システムの起動

`roscore`を残し、別ターミナルのPiPER Docker内で実行する。

```bash
sorc_devel
roslaunch piper collect_dual_piper_mcap2.launch \
  can_leader_R_port:=can0 \
  can_follower_R_port:=can1 \
  can_leader_L_port:=can2 \
  can_follower_L_port:=can3 \
  front_camera:=/dev/video4 \
  wrist1_camera:=/dev/video2 \
  wrist2_camera:=/dev/video0 \
  camera_fps:=30 \
  record_fps:=20 \
  auto_enable:=true \
  task:="Assembling a box."
```

これらのCAN・カメラ・FPSは現在の既定値。機器構成が一致する場合は次でも起動できる。

```bash
roslaunch piper collect_dual_piper_mcap2.launch auto_enable:=true task:="Assembling a box."
```

既定の`auto_enable:=true auto_follow:=true`では、起動後からFollowerがLeaderへ追従する。
Leaderは起動時はCAN制御で保持し、`t`で教示モードへ切り替える。
読み取りから始める場合は`auto_enable:=false`、有効化だけして保持する場合は`auto_follow:=false`を指定する。
起動ログの実解像度・FPS・FourCCと左右の追従を確認する。

### 3. キー操作と収集の流れ

カメラプレビュー`PiPER MCAP2: front / wrist1 / wrist2`を選択して操作する。
旧版の専用タスク入力ターミナルと`Dataset Count`表示は、現行コードにはない。

| キー | 操作 |
|---|---|
| `u` | 4台を有効化。既定では追従も開始 |
| `t` | 左右Leaderを教示モードへ切り替える。タスク文字列の入力ではない |
| `c` | 追従再開。Leaderのモードは変更しない |
| `r` | 教示モード確認→追従再開→最新State・Actionを待って記録開始 |
| Space | 追従停止。記録中は先に保存・破棄する |
| `s` / `v` | 記録保存と追従停止。初期姿勢へは自動復帰しない |
| `x` | 記録破棄と追従停止 |
| `p` | 追従を止め、左右Followerの初期姿勢を保存 |
| `h` | 保存済み初期姿勢へ左右Followerを移動。Leaderは手動で合わせる |
| `e` | 状態・周囲を確認後、エラーコード0のFollower非常停止を明示的に復旧 |
| `q` | launch全体を終了。記録中データは未完了として保持 |

1. システムを起動し、左右の追従と3台の映像を確認する。
2. `t`で教示モードへ切り替える。
3. 必要ならSpace→`p`で収集用初期姿勢を保存する。
4. `r`で記録開始し、`RECORDING`を確認してLeaderで作業する。
5. `s`で保存完了を確認する。追従も停止する。
6. 必要なら`h`で戻す。次の`r`で追従と記録を再開する。
7. 終了は保存完了後に`q`、またはlaunchのターミナルでCtrl+C。

追従停止は新規指令を止める操作で、モーターは目標を保持する。
`e`だけでは再有効化・追従再開しない。

#### タスクプロンプト

現行版は起動時の`task`引数をエピソードの`meta.json.task`に保存する。
既定値は`bimanual manipulation`。日本語の指定例：

```bash
roslaunch piper collect_dual_piper_mcap2.launch \
  auto_enable:=true \
  task:="青い袋を右手で持ち上げ、左手で支える"
```

旧版の`t`→文字入力→Enterによる変更機能はない。
作業内容を変える場合は保存・終了後、`task`を変えて起動する。
変換時にはデータセット全体のタスク文だけでなく、各エピソードのタスクが反映されるか確認する。

### 4. 保存先とデータ構造

| 場所 | 既定保存先 |
|---|---|
| PiPER Docker | `~/piper_dataset/dual2` |
| ホスト | `/home/dnp218/docker_piper_ctrl/piper_dataset/dual2` |

`dataset_root:=$HOME/piper_dataset/dual2`などで変更できる。
Docker内で別の保存先を指定する場合は、ホストへ永続化されるマウント範囲も確認する。

```text
piper_dataset/dual2/
  dataset_meta.json
  episode_<開始時刻ns>/
    episode_<開始時刻ns>.mcap
    data.npz
    frames.csv
    meta.json
    front/000000.jpg ...
    wrist1/000000.jpg ...
    wrist2/000000.jpg ...
```

| データ | 内容 |
|---|---|
| State | 各Followerの実測6関節＋グリッパー、合計14次元 |
| Action | SDKへ送った分解能に合わせた6関節＋グリッパー、合計14次元。ハードウェアACKではない |
| 配列順序 | **右7次元→左7次元** |
| 単位 | 関節：rad、グリッパー：m |
| NPZ | `observation_state`と`action`はfloat32の`(N,14)` |
| NPZ・CSV | 基準時刻、元時刻・同期誤差、画像パス、再利用回数など |
| MCAP | 左右State・Actionの生サンプル、Leader値、制御状態、採用画像 |
| `meta.json` | タスク、設定、保存状態、欠落数、画像再利用回数など |

固定20 Hzグリッドに同期し、State・Actionは±30 ms、画像は±100 ms、3画像間の差は30 ms以内。
同期範囲内の画像再利用を許し、範囲外の行は保存せず欠番・欠落数を残す。
`sync_delay:=0.2`秒待ってからサンプルを選択する。ソフトウェア同期で、露光時刻の一致を保証しない。
既定1秒を超える欠落や制御・カメラ異常時は、`.tmp_episode_...`に未完了データを保持して追従停止を要求する。

収集用初期姿勢はDocker内の`~/.config/piper/mcap2_initial_right.yaml`と`mcap2_initial_left.yaml`。
`right_initial_pose_file`・`left_initial_pose_file`で変更できる。
G編の推論用`piper_dual_demo_start.yaml`とは別ファイルで、自動的には同期しない。

**収集配列の右→左と、推論配列の左→右は異なる。** 外部のLeRobot変換・学習側での並べ替えと、
`wrist1=left`・`wrist2=right`の対応を確認する。配列や画像名を推測で変更しない。

### 5. ROSサービスからの操作

GUIなしでは`no_preview:=true`とサービスを使用する。

```bash
sorc_devel
rostopic echo /dual_piper/status
rosservice call /dual_piper/enable '{}'
rosservice call /dual_piper/start_following '{}'
rosservice call /dual_recorder/start '{}'
rosservice call /dual_recorder/stop '{}'
rosservice call /dual_recorder/discard '{}'
```

制御サービスは`/dual_piper/`配下の`enable`、`enter_teaching_mode`、`start_following`、
`stop_following`、`save_initial_pose`、`go_initial_pose`、`reset_fault`、`recover_stop`、`emergency_stop`。
記録サービスは`/dual_recorder/start`、`stop`、`discard`。型はいずれも`std_srvs/Trigger`。
旧版の`/dual_state_mcap_recorder/...`はこの構成では使わない。

現行実装のカメラ処理・監視設定・制御上の制約は[README_dual_mcap2.md](double_PiPER/src/piper/README_dual_mcap2.md)を参照する。

---

<a id="conversion"></a>

## E. MCAPからLeRobot形式への変換

本READMEでは、双腕PiPERのリーダーフォロワーで収集したMCAPデータを、収集用PC（`dnp218`）からチューニング用PC（`rlsv`）へ転送し、LeRobot形式に変換するまでの手順を説明する。

### 1. 収集用PCでのデータ収集・整理

D編のMCAP2では、MCAPはホスト側の次の階層に保存される。

```text
/home/dnp218/docker_piper_ctrl/piper_dataset/dual2/
└── episode_<開始時刻ns>/
    ├── episode_<開始時刻ns>.mcap
    ├── meta.json
    ├── data.npz
    ├── frames.csv
    ├── front/
    ├── wrist1/
    └── wrist2/
```

元READMEの`mv 20261015_*.mcap ...`は、この階層には一致しない。
以下は保存完了したエピソードを明示して、転送用フォルダへMCAPをコピーする例。
`<開始時刻ns>`は実際のフォルダ名に置き換え、必要なエピソードごとに繰り返す。
元データ一式は保存先に残す。

```bash
cd /home/dnp218/docker_piper_ctrl/piper_dataset
mkdir -p dnp1_20261015
cp -n "dual2/episode_<開始時刻ns>/episode_<開始時刻ns>.mcap" dnp1_20261015/
ls -lh dnp1_20261015/
```

`.tmp_episode_*`や`.incomplete.mcap`は正常完了データと混ぜない。
データセット名`dnp1_20261015`は元の実験例。新規収集では実際の収集日・実験名に変える。
旧収集データを使う場合は、既存の配置とファイル名を確認して転送対象を選ぶ。

**変換互換性の確認：** 以下の外部スクリプトは今回読み取り確認できていない。
MCAP2のtopic・メタデータ・タスク格納方法、右→左の14次元配列、画像の左右対応を解釈できることを、
まず1エピソードで確認する。MCAP内の情報だけで不十分な変換実装なら、`meta.json`などを含む
エピソード一式を転送し、変換側の対応を確認する。パス修正だけで変換対応済みとは判断しない。

### 2. チューニング用PCへのデータ転送

収集用PCで整理したMCAPデータを、`scp` コマンドでチューニング用PC（`rlsv`）へ転送する。

#### 2.1 転送先ディレクトリ

```text
/home/rlsv/project/datasets/mcap
```

#### 2.2 SCPによるデータ転送

**収集用PC（`dnp218`）のターミナル** で、以下のコマンドを実行する。

```bash
scp -r /home/dnp218/docker_piper_ctrl/piper_dataset/dnp1_20261015 \
rlsv@163.221.133.176:/home/rlsv/project/datasets/mcap/
```

- `scp`：SSHを利用したファイル転送コマンド
- `-r`：フォルダ内のファイルをまとめて転送
- `dnp1_20261015`：転送するデータフォルダ
- `rlsv@163.221.133.176`：チューニング用PCの接続先

実行後、パスワードを要求された場合はチューニング用PCのユーザーパスワードを入力する。転送先の `mcap` ディレクトリが存在することを事前に確認する。

### 3. チューニング用PCへのSSH接続と転送データの確認

#### 3.1 SSH接続

データ転送後、**収集用PC（`dnp218`）またはローカルPCのターミナル**で、チューニング用PC（`rlsv`）にSSH接続する。

```bash
ssh rlsv@163.221.133.176
```

初回接続時は接続先のホスト鍵を確認し、問題がなければ `yes` と入力する。パスワードが求められた場合は、`rlsv` のパスワードを入力する。

接続後、以下でログイン先を確認する。

```bash
whoami
hostname
```

`whoami` の出力が `rlsv` であることを確認する。

#### 3.2 転送データの確認

SSH接続した **チューニング用PC（`rlsv`）** のターミナルで、以下のコマンドを実行する。

```bash
ls -lh /home/rlsv/project/datasets/mcap/dnp1_20261015/
```

**転送後の構成例（数値部分は開始時刻nsの省略表記）**

```text
/home/rlsv/project/datasets/mcap/
└── dnp1_20261015/
    ├── episode_153856.mcap
    ├── episode_154251.mcap
    ├── episode_155023.mcap
    └── episode_160412.mcap
```

すべてのMCAPファイルが転送されていることを確認する。

### 4. Dockerコンテナへの接続

#### 4.1 ターミナルの準備

SSH接続先のターミナルで操作する。必要に応じて、GUI環境では `Terminator` を起動してもよい。

#### 4.2 DockerコンテナIDの確認

起動中のDockerコンテナを一覧表示する。

```bash
docker ps
```

**出力例**

```text
CONTAINER ID   IMAGE                STATUS        NAMES
2e6f346ff7ff   piper-pi0-sim:111    Up 5 weeks    piper_pi0-pi0-sim-run
```

学習・データ変換に使用するコンテナの **`CONTAINER ID`** を確認する。

#### 4.3 Dockerコンテナに入る

```bash
docker exec -it <CONTAINER ID> bash
```

**実行例**

```bash
docker exec -it 2e6f346ff7ff bash
```

以降のLeRobot変換コマンドは、Dockerコンテナ内で実行する。

**注意：** `docker exec` に指定するのはDockerイメージIDではなく、`docker ps` で確認したコンテナIDである。

### 5. MCAPからLeRobot形式へのデータ変換

チューニング用PCへ転送したMCAPデータを、π0.5の学習で使用するLeRobot形式に変換する。

#### 5.1 変換スクリプト

```text
/workspace/piper_pi0/real/collect/convert_bimanual_to_lerobot.py
```

#### 5.2 変換コマンドの実行

**チューニング用PC（`rlsv`）のDockerコンテナ内** で、以下のコマンドを実行する。

```bash
uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/real/collect/convert_bimanual_to_lerobot.py \
  --input /workspace/datasets/mcap/dnp1_20261015 \
  --output /workspace/datasets/lerobot/local/dnp1_20261015 \
  --repo-id local/dnp1_20261015 \
  --gap-policy recover \
  --image-policy flag \
  --image-tolerance-ms 100
```

`dnp1_20261015` は実際のデータセット名に変更する。

| オプション | 説明 |
| --- | --- |
| `--input` | 変換元MCAPフォルダ |
| `--output` | LeRobot形式の出力先 |
| `--repo-id` | データセット識別名 |
| `--gap-policy recover` | 欠損フレームの復元処理を行う |
| `--image-policy flag` | 画像の対応付けに問題があるフレームをフラグで記録する |
| `--image-tolerance-ms 100` | 画像と基準時刻の許容時間差を100 msに設定する |

**注意：** 上記の `/workspace/datasets` はDockerコンテナ内のパスである。ホストPCの `/home/rlsv/project/datasets` が `/workspace/datasets` にマウントされていることを前提とする。実行前にコンテナ内から入力データが見えることを確認する。

```bash
ls -lh /workspace/datasets/mcap/dnp1_20261015/
```

#### 5.3 変換結果の確認

変換コマンドが**正常終了した後**、以下を実行する。

```bash
ls -lh /workspace/datasets/lerobot/local/dnp1_20261015/
```

LeRobot形式のデータファイルやメタデータが生成されていることを確認する。変換処理中は出力先がまだ作成されていない場合があるため、完了前に出力先が見つからなくても、それだけで失敗とは判断しない。

**ディレクトリ構成例（Dockerコンテナ内）**

```text
/workspace/datasets/
├── mcap/
│   └── dnp1_20261015/         # 変換元MCAPデータ
└── lerobot/
    └── local/
        └── dnp1_20261015/     # 変換後LeRobotデータ
```

以上で、双腕PiPERの収集データをLeRobot形式へ変換する手順は完了となる。

---

<a id="training"></a>

## F. π0.5 Fine-tuning

本READMEでは、双腕PiPERの教示データを使用して、π0.5モデルのFine-tuningを開始するまでの手順を説明する。

### 前提条件

- MCAPからLeRobot形式へのデータ変換が完了している。
- 学習用PC（`rlsv`）にSSH接続している。
- **学習用PCのDockerコンテナ内に入っている。**

以降のコマンドは、すべて学習用PCのDockerコンテナ内で実行する。SSH接続やDockerコンテナへの接続方法は、本書E編を参照すること。

### 1. 学習設定（YAML）の確認・作成

双腕PiPERのπ0.5 Fine-tuningでは、YAMLファイルでデータセットや学習条件を指定する。

#### 1.1 設定ファイルの保存先

設定ファイルは、次のディレクトリに保存する。

```text
/workspace/piper_pi0/configs/real/
```

既存の双腕PiPER用設定ファイル：

```text
/workspace/piper_pi0/configs/real/dnp1_20261015_bimanual_pi05.yaml
```

#### 1.2 新しい設定ファイルの作成

新しいデータセットや実験で学習する場合は、既存のYAMLファイルをコピーして作成する。

```bash
cd /workspace/piper_pi0/configs/real/
cp dnp1_20261015_bimanual_pi05.yaml new_bimanual_pi05.yaml
```

`new_bimanual_pi05.yaml` はファイル名の例である。データセット名や実験名に合わせて変更する。作成したファイルをVS Codeなどで開いて編集する。

#### 1.3 学習条件の変更

新しい設定ファイルで、少なくとも次の項目を確認・変更する。

| 設定項目 | 確認内容 |
| --- | --- |
| Dataset | 使用するLeRobotデータセット |
| Experiment Name | 実験名 |
| Training Steps | 学習ステップ数 |
| Batch Size | バッチサイズ |
| Learning Rate | 学習率 |
| Action Horizon | 予測するActionの長さ |
| Checkpoint Interval | Checkpointの保存間隔 |
| Resume | 新規学習か再開か |

既存の双腕用学習設定を基にした初期構成は、以下のとおり。

- Base Model：`pi05_base`
- LoRA：有効（Language rank 16／Action Expert rank 32）
- Action Horizon：50
- Batch Size：8

`lora_rank: null` の場合は、LanguageとAction Expertにそれぞれ既定のrankを使用する。整数を指定した場合は、両方のrankが同じ値に上書きされる。

**注意：** 新規実験では、データセットの参照先、実験名、Checkpointの保存先、正規化統計の保存・参照先を確認し、既存実験の結果と混在させないこと。

#### 1.4 学習設定の確認

編集後、学習を開始せずに設定内容を確認する。

```bash
uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/training/scripts/train_bimanual_pi05.py \
  --config /workspace/piper_pi0/configs/real/new_bimanual_pi05.yaml \
  --check-config
```

`--config` には実際に使用するYAMLファイルの絶対パスを指定する。設定を確認できたら、次の章へ進む。

### 2. Fine-tuningの実行

#### 2.1 学習の開始

Dockerコンテナ内で、以下のコマンドを実行する。

```bash
uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/training/scripts/train_bimanual_pi05.py \
  --config /workspace/piper_pi0/configs/real/new_bimanual_pi05.yaml
```

`new_bimanual_pi05.yaml` は、第1章で作成・編集した設定ファイル名に置き換える。

#### 2.2 学習開始時の処理

学習開始コマンドでは、次の処理が実行される。

1. YAMLの学習設定を読み込む。
2. 学習データセットと対応する正規化統計を確認する。
3. 正規化統計が未作成の場合は、自動的に計算する。
4. π0.5モデルを読み込み、Fine-tuningを開始する。
5. YAMLで指定した間隔でCheckpointを保存する。

**正規化統計の事前手動計算（`--norm-stats-only`）は必須ではない。** ただし、既存の統計を使用する場合は、データセットや学習設定との対応を確認すること。

#### 2.3 学習状況の確認

学習中は、ターミナルのログで学習ステップ数やLossを確認する。GPU使用状況は、必要に応じて別ターミナルで次のコマンドを実行して確認する。

```bash
nvidia-smi
```

#### 2.4 Checkpointの保存先

Checkpointは、YAMLの実験名や保存先設定に従って保存される。

既存の双腕PiPER用設定での保存先：

```text
/workspace/checkpoints/pi05_piper_bimanual_lora/dnp1_20261015_bimanual_pi05/
```

新しい設定ファイルを使用する場合は、**実際のCheckpoint保存先をYAMLで確認すること。** 保存済みCheckpointは、学習の再開や推論に使用する。

**注意事項**

- 学習中はDockerコンテナを停止しないこと。
- 学習ステップ数・Learning Rate・Checkpoint保存間隔はYAMLで変更する。
- 学習を中断する場合は、保存済みCheckpointを確認すること。

G編のBundle例は`pi05_piper_bimanual_delta_lora/dnp1_20261015_bimanual_pi05_delta`という別runである。
この編の`pi05_piper_bimanual_lora/dnp1_20261015_bimanual_pi05`と同一runとは扱わず、使用するYAMLと実際の保存先を照合する。

---

<a id="inference"></a>

## G. Bundle作成・転送・推論のStart／Stop

この編では、学習済みπ0.5モデルから推論用Bundleを作成し、双腕PiPERでの推論に使用するまでの手順を説明する。

### 前提条件

- 双腕PiPERのπ0.5 Fine-tuningを実行し、使用したいステップのCheckpointが保存されている。
- 学習用PC（`rlsv`）のDockerコンテナ内に入っている。
- 第1〜2章は学習用PCのDockerコンテナ内、第3章は学習用PCのホスト側、第4〜5章は推論用PC（dnp218）で実行する。

### 1. 学習済みCheckpointの確認

#### 1.1 Checkpointの保存先

今回の双腕PiPER学習のCheckpointは、以下のディレクトリに保存される。

```text
/workspace/checkpoints/pi05_piper_bimanual_delta_lora/dnp1_20261015_bimanual_pi05_delta/
```

※ 実際の保存先は、学習時に使用した設定によって異なる。

#### 1.2 保存済みCheckpointの確認

```bash
ls -lh /workspace/checkpoints/pi05_piper_bimanual_delta_lora/dnp1_20261015_bimanual_pi05_delta/
```

保存済みのステップ番号を確認し、推論に使用するCheckpointを決める。

#### 1.3 使用するステップの確認

例えば、`5000` ステップを使用する場合は次を実行する。

```bash
ls -lh /workspace/checkpoints/pi05_piper_bimanual_delta_lora/dnp1_20261015_bimanual_pi05_delta/5000/
```

`params` などのモデルパラメータが保存されていることを確認する。存在しないステップは指定しない。

### 2. 推論用Bundleの作成

学習済みCheckpointを推論用PCで利用できるように、Bundleとして出力する。

#### 2.1 Bundle生成スクリプト

```text
/workspace/piper_pi0/training/scripts/export_bimanual_bundle.py
```

#### 2.2 事前チェック（任意）

以下の例では、`5000` ステップを対象に、Bundleの出力を行わず事前チェックする。

```bash
PYTHONPATH=/workspace uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/training/scripts/export_bimanual_bundle.py \
  export-existing --step 5000 --check-only
```

#### 2.3 Bundleの作成

以下のコマンドを実行する。

```bash
PYTHONPATH=/workspace uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/training/scripts/export_bimanual_bundle.py \
  export-existing \
  --step 5000 \
  --output /workspace/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000
```

- `--step 5000`：Bundle化するCheckpointのステップ番号
- `--output`：Bundleの保存先ディレクトリ
- `export-existing`：保存済みCheckpointからBundleを作成するモード

※ `5000` は実行例。第1章で確認した実在するステップに変更すること。上記コマンドは、この学習runの既定保存先からCheckpointを参照する。別のrunを使う場合は、`--run-dir` または `--checkpoint` により参照先を明示する。

#### 2.4 作成したBundleの確認

```bash
ls -lh /workspace/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000/
```

Bundleには、モデルパラメータ、正規化統計、学習・推論設定、`manifest.yaml`、`validation.json`、`checksums.sha256` などが含まれる。

ファイルの整合性を確認する。

```bash
cd /workspace/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000
sha256sum -c checksums.sha256
```

すべて `OK` となることを確認する。検証に失敗したBundleは、推論用PCへ転送しない。

### 3. 推論用PCへのBundle転送

作成したBundleを、学習用PC（`rlsv`）から推論用PC（`dnp218`）へSCPで転送する。

#### 3.1 学習用PCのホスト側に戻る

Dockerコンテナ内で次を実行して、学習用PCのホスト側へ戻る。

```bash
exit
```

以降は**学習用PC（`rlsv`）のホスト側ターミナル**で実行する。

#### 3.2 ホスト側でBundleを確認する

今回の環境では、コンテナ内で作成したBundleを以下のホスト側パスから直接参照できる。

```text
/home/rlsv/project/piper_pi0/logs/checkpoint_exports/
```

以下のコマンドで転送対象のBundleが存在することを確認する。

```bash
ls -lh /home/rlsv/project/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000/
```

この環境では **`docker cp` を実行する必要はない**。

#### 3.3 SCPで推論用PCに転送する

転送先ディレクトリが未作成の場合は、まず作成する。

```bash
ssh dnp218@163.221.133.164 'mkdir -p /home/dnp218/piper_checkpoints'
```

学習用PCの**ホスト側ターミナル**で、以下のコマンドを実行する。

```bash
scp -r \
  /home/rlsv/project/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000 \
  dnp218@163.221.133.164:/home/dnp218/piper_checkpoints/
```

※ Bundle名は使用するCheckpointのステップに応じて変更すること。

#### 3.4 転送結果の確認

推論用PCに接続する。

```bash
ssh dnp218@163.221.133.164
```

転送されたBundleを確認する。

```bash
ls -lh /home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_5000/
```

チェックサムを再検証する。

```bash
cd /home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_5000
sha256sum -c checksums.sha256
```

すべて `OK` と表示されれば、転送は完了となる。

※ `dnp1_20261015_bimanual_pi05_delta_5000` は例であり、実際に作成したBundle名に置き換える。Bundle内部のファイルやフォルダ構成は変更しないこと。

### 4. 推論サーバーの起動

第4章以降は、**推論用PC（`dnp218`）を直接操作する**ことを前提とする。

#### 4.1 使用するプログラム

学習用PCと推論用PCでは使用するプログラムが異なる。

| 用途 | プログラムの場所 |
|---|---|
| 学習・Bundle生成（学習用PC） | `/workspace/piper_pi0/` |
| π0.5推論Runtime（推論用PC） | `/home/dnp218/piper_vla_runtime/` |
| ROS・PiPER制御（推論用PC） | `/home/dnp218/docker_piper_ctrl/double_PiPER/` |

`piper_vla_runtime` はモデル推論を、`double_PiPER` はROSノード・カメラ・ロボット制御を担当する。両者は別のプログラムである。

#### 4.2 推論サーバーの起動

推論用PCで以下を実行する。

```bash
cd /home/dnp218/piper_vla_runtime
export DUAL_BUNDLE=/home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_5000

./scripts/run_dual.sh \
  --checkpoint "$DUAL_BUNDLE" \
  --warmup-prompt "Assembling a box." \
  --max-execution-steps 0 \
  --controller-run
```

`--checkpoint` は使用するBundleに合わせて変更する。`--warmup-prompt` はモデル初期化時の言語指示であり、開始時のPromptは5.3のStartでも指定する。

`--max-execution-steps 0` は実行step数を無制限にする指定である。Start後は、自分でStopするまで継続できる。ただし、State・画像の鮮度異常、推論timeout、Controllerの安全判定による自動HOLDは有効なままである。

`--controller-run` はRuntimeからControllerへActionを送信する設定（Runtime側の`dry_run=false`）であり、ROS Controllerの`dry_run`設定は変更しない。5.2の`dry_run:=true`と組み合わせる場合は、Controllerの検証・目標診断まで行い、driverへ関節指令を送らない。

#### 4.3 起動状態の確認

モデルの読み込み後、ログで `READY` を確認する。サーバー起動中はこのターミナルを閉じない。

READY後は、ROS環境を読み込んだ別ターミナルから5.3のPrompt付きStartを実行する。推論サーバーの起動だけでは推論は開始しない。第5章でROS側の起動・確認を終えてからStartする。

---

### 5. 双腕PiPERの起動と推論のStart／Stop

本章では、初期姿勢への復帰、ROS launchの起動、推論開始・停止を行う。**ROS関連のコマンドは、`double_PiPER` を利用できるDockerコンテナ内で実行する**。新しいターミナルでは、ホストから`docker exec -it piper-control-dev bash`で入り、`sorc_devel`でROS環境を読み込む。

#### 5.1 初期姿勢に戻す

**推論を停止した状態で、ROSドライバなど他のプログラムが同じCANを使用していないことを確認してから**、初期姿勢復帰スクリプトを使用する。

```bash
cd ~/double_PiPER
python3 src/piper/scripts/go_dual_initial_pose.py --dry-run
```

まず `--dry-run` で設定・関節角度範囲などを検証する。問題がなければ、周囲とロボットの可動範囲を確認し、以下のコマンドで実機を初期姿勢へ移動する。

```bash
python3 src/piper/scripts/go_dual_initial_pose.py
```

初期姿勢の設定ファイル（ホスト側。Docker内では`~/double_PiPER/src/piper/config/piper_dual_demo_start.yaml`）：

```text
/home/dnp218/docker_piper_ctrl/double_PiPER/src/piper/config/piper_dual_demo_start.yaml
```

このYAMLに左右の目標関節角などが設定されている。左腕は `can3`、右腕は `can1` を使用する。現在の復帰スクリプトは両腕のROS driver経路を検出し、揃っていれば`ros_driver`、driverの証拠がなければ`direct_sdk`を選ぶ。片腕のみ・曖昧な経路では動作を拒否する。この手順では他のdriverを停止して`direct_sdk`で使用する。`--dry-run`もフィードバック読取のためCAN通信を行うが、有効化・移動指令は送らない。**双腕間の衝突判定は検証されていない**。ロボット周辺を十分に確認し、必要に応じて非常停止できる状態で実施する。

#### 5.2 推論用ROS launchの起動

別のターミナルでDockerコンテナに入り、ROS環境を読み込む。

```bash
sorc_devel
```

CANアダプタの設定およびROS Masterの起動が済んでいることを確認し、以下の設定で起動する。

```bash
roslaunch piper start_vla_dual_piper.launch \
  left_can_port:=can3 \
  right_can_port:=can1 \
  front_camera:=/dev/video4 \
  left_wrist_camera:=/dev/video2 \
  right_wrist_camera:=/dev/video0 \
  start_sdk_joint_limit:=true \
  controller_joint_limit_enabled:=false \
  dry_run:=true \
  auto_enable:=false
```

このlaunchで左右PiPERドライバ、3台のカメラ、双腕Controllerを起動する。
現在の低レベルdriverは`piper_ctrl_dual_feedback_node.py`である。
`dry_run:=true`でもCANへ接続して初期化通信を行う。実機へのVLA関節指令を抑止する設定であり、完全な無通信検証ではない。
カメラ番号はC編と現物を照合し、収集時と同じ視点を指定する。

- `start_sdk_joint_limit:=true`：SDK側のソフトウェア関節角度制限を有効にする。
- `controller_joint_limit_enabled:=false`：Controller側の関節角度制限を無効にする。
- `dry_run:=true`：実機動作を伴わない確認用設定とする。
- `auto_enable:=false`：PiPERの自動Enableを行わない。

このコマンドは**ROS側がdry-runの構成**であり、実機を動かすための起動設定ではない。起動したターミナルは閉じない。

**重要：** `go_dual_initial_pose.py` とROSドライバを同時に動かさないこと。両者が同じCANデバイスを操作すると競合する可能性がある。

#### 5.3 推論のStart

さらに別のターミナルでROS環境を読み込む。

```bash
sorc_devel
```

推論サーバーの `READY` を確認してから、言語指示（prompt）を指定して推論を開始する。

```bash
rosservice call /piper_vla_dual_runtime/start "prompt: 'Assembling a box.'"
```

このコマンドは推論開始を指示する。**第4章のサーバーは `--controller-run` だが、5.2のROS launchは `dry_run:=true`・`auto_enable:=false` のため、この編の設定は動作確認用である。** 実機を動作させる場合は別途、安全設定と起動条件を確認すること。

`--max-execution-steps 0`で起動しているため、実行step数による自動停止はない。終了したいタイミングで、次のStopを呼ぶ。

#### 5.4 推論のStop

推論を停止する場合は、以下のコマンドを実行する。

```bash
rosservice call /piper_vla_dual_runtime/stop '{}'
```

Stopは残りのActionチャンクを破棄し、両腕HOLDを要求する。電源断や装置の非常停止とは異なる。

停止を確認してから次の操作へ進む。ROS launchを終了する場合は、起動したターミナルで `Ctrl+C` を使用する。初期姿勢へ戻す場合は、推論停止後、ROSドライバを終了してCANの競合がない状態で5.1を実行する。

#### 5.5 設定ファイル・関連コードの場所

| 用途 | パス（推論用PC） |
|---|---|
| 推論用ROS launch | `/home/dnp218/docker_piper_ctrl/double_PiPER/src/piper/launch/start_vla_dual_piper.launch` |
| 初期姿勢の設定（YAML） | `/home/dnp218/docker_piper_ctrl/double_PiPER/src/piper/config/piper_dual_demo_start.yaml` |
| 初期姿勢復帰スクリプト | `/home/dnp218/docker_piper_ctrl/double_PiPER/src/piper/scripts/go_dual_initial_pose.py` |
| 双腕ROS Controller | `/home/dnp218/docker_piper_ctrl/double_PiPER/src/piper/scripts/vla_dual_piper_controller.py` |
| Runtime設定（基準YAML） | `/home/dnp218/piper_vla_runtime/configs/runtime.dual.yaml` |
| Runtime起動スクリプト | `/home/dnp218/piper_vla_runtime/scripts/run_dual.sh` |

Runtimeは起動時に一時的な設定ファイルを生成する場合がある。実際に読み込まれた設定ファイルの場所は起動ログで確認すること。

#### 5.6 実機推論時の安全上の注意

第4章の推論サーバーは `--controller-run` を指定しているが、5.2のROS launchは **`dry_run:=true`・`auto_enable:=false`** で起動する。この編の組み合わせはROS側での動作確認用設定である。

- 左右PiPERの可動範囲・相互干渉、周囲の人・物との距離を確認する。
- CANデバイスを他のドライバや初期姿勢復帰プログラムと同時に使用しない。
- HOLD・非常停止など、直ちに動作を止める手段を確認する。
- `start_sdk_joint_limit:=true` はSDK側のソフトウェア関節角度制限であり、双腕間の衝突を防ぐものではない。
- `controller_joint_limit_enabled:=false` ではController側の関節制限を使用しない。実機動作に切り替える際は、安全性を再評価すること。

これまでのROS Mock検証は実機CANでの動作保証を意味しない。実機を動かす際は立会いのもとで慎重に確認すること。

---

<a id="details"></a>

## H. 推論の詳細確認・停止・トラブル対応

G編の起動コマンドを基本とし、ここでは元`README_place.md`の観測仕様・段階的確認・診断をまとめる。
Runtime関連の数値は元文書の設定例を含むため、実際の生成YAML・起動ログも照合する。
以降のターミナルCはRuntime起動用のホスト、ターミナルDは確認・サービス操作用のPiPER Dockerを指す。

### 1. 推論で使う観測・Action・安全制御

#### 1.1 StateとActionの並び

```text
index  0..5 : 左joint1..6      [rad]
index  6    : 左gripper開幅    [m]
index  7..12: 右joint1..6      [rad]
index 13    : 右gripper開幅    [m]
```

学習内部では12関節を観測State基準のDeltaにしていますが、serverから返るActionは
**逆正規化・絶対値復元済み**です。ROS側でStateを再加算してはいけません。
`adapt_to_pi=False`で学習しているため、ALOHA固有の関節符号反転・gripper変換も追加しません。

#### 1.2 画像とROS topic

| 用途 | topic | 型 |
|---|---|---|
| 左State | `/followerL/joint_states_single` | `sensor_msgs/JointState` |
| 右State | `/followerR/joint_states_single` | `sensor_msgs/JointState` |
| Front画像 | `/camera/front/compressed` | `sensor_msgs/CompressedImage`（JPEG） |
| Left Wrist画像 | `/camera/left_wrist/compressed` | 同上 |
| Right Wrist画像 | `/camera/right_wrist/compressed` | 同上 |
| server chunk先頭 | `/vla_dual/raw_action` | `std_msgs/Float64MultiArray`、14要素 |
| 制御周期で選択したAction | `/vla_dual/selected_action` | 同上 |
| Controllerへ送るAction | `/vla_dual/approved_action` | 同上 |
| HOLD要求 | `/vla_dual/hold` | `std_msgs/Bool` |
| 左目標診断 | `/vla_dual/left/target_joint_command` | `sensor_msgs/JointState` |
| 右目標診断 | `/vla_dual/right/target_joint_command` | 同上 |
| 左driverへの関節指令 | `/followerL/joint_states` | 同上 |
| 右driverへの関節指令 | `/followerR/joint_states` | 同上 |
| Controller異常通知 | `/vla_dual/controller_fault` | `std_msgs/String` |

Runtimeは左右driverのStateを直接購読します。
補助topic `/vla/dual_joint_states` をRuntime設定へ置き換える必要はありません。

#### 1.3 周期とHOLD

- モデルHorizon：50 Action。
- 実行chunk長：20 Action。
- 実行周期：20 Hz（1 Action / 50 ms）。
- 次chunk推論要求：既定index 15。
- 推論は毎秒20回ではなく、初回とchunkごとのprefetchで要求します。
- 現在のRuntime設定は推論timeout 2秒、初回待機timeout 5秒。
- 観測timeoutは0.5秒、5観測間のstamp差は最大0.15秒。

timeout、画像停止、観測時刻差、chunk不足、Controller fault等でRuntimeはHOLDになります。
モデルが通常samplingでprefetchに間に合うかはGPUで別途確認が必要です。
HOLDを消すために設定を緩める前に、ログから原因を確認してください。

#### 1.4 Controllerの安全判定

単腕の既存処理を左右それぞれに適用します。
有限値・形式検査→関節Clamp/範囲外拒否→公式FK→Z下限→gripper範囲/Clamp→指令生成の順序です。
両腕が通るまで実送信は保留し、片腕の検証失敗なら両腕未送信・fault通知となり、明示startまで送信を停止します。

| launch設定 | 既定値 |
|---|---|
| `z_enabled` | true |
| `z_min` | 0.01 m |
| `joint_saturation_margin_rad` | 0.1 rad |
| `gripper_closed_width_m` | 0.0 m |
| `gripper_open_width_m` | 0.10 m |
| `gripper_saturation_margin_m` | 0.01 m |

G編の例では`controller_joint_limit_enabled:=false`を明示しているため、Controllerの関節Clamp・範囲外拒否は無効です。以下はこの設定がtrueの場合の説明です。

関節の物理範囲そのものはSDKの公式設定を使用します。margin内の超過は物理範囲へClampし、
margin外は拒否します。Python単体の関節margin既定値は0.05 radですが、launchは単腕同様0.1 radを渡します。
このREADMEでは新しい閾値・速度制限・双腕間衝突判定を追加しません。

### 2. 観測topicを確認する（PiPER Docker）

```bash
source /opt/ros/noetic/setup.bash
source /home/dnp218/double_PiPER/devel/setup.bash

rostopic echo -n 1 /followerL/joint_states_single
rostopic echo -n 1 /followerR/joint_states_single
rostopic type /camera/front/compressed
rostopic type /camera/left_wrist/compressed
rostopic type /camera/right_wrist/compressed
```

Stateの`name`が`joint1..joint6, gripper`、`position`が7要素であることを確認します。
周波数確認は1つずつ実行し、数秒見たらCtrl+Cで次へ進みます。

```bash
rostopic hz /followerL/joint_states_single
rostopic hz /followerR/joint_states_single
rostopic hz /camera/front/compressed
rostopic hz /camera/left_wrist/compressed
rostopic hz /camera/right_wrist/compressed
```

driverの配信ループは既定200 Hz、カメラ配信目標は20 Hzです。実測値は環境によります。
カメラは取得30 FPS・配信20 Hzですが、帯域や形式により実取得が低下する場合があります。

画像内容を確認する場合（PiPER DockerのGUI環境）：

```bash
rosrun image_view image_view image:=/camera/front _image_transport:=compressed
```

同様に`image:=/camera/left_wrist`と`image:=/camera/right_wrist`へ切り替え、左右・向き・画角を確認します。
`/compressed`を末尾へ付けるのではなく、基底topicとtransportを指定します。

### 3. Runtime・ROSの動作モード

| 確認段階 | ROS launch | Runtime | VLA由来の実機関節指令 |
|---|---|---|---|
| 推論出力のみ | `dry_run:=true` | `--dry-run` | なし。ControllerへActionを送らない |
| Controller判定まで | `dry_run:=true` | `--controller-run` | なし。左右目標診断まで |
| 実機推論 | `dry_run:=false auto_enable:=true` | `--controller-run` | あり |

手動Stopまで継続する場合、Runtimeに`--max-execution-steps 0`を指定する。
G編の設定はController判定までのモード。ROS側のdry-runはRuntime引数では変更されない。
実機へ切り替える場合はstop→既存ROS launch終了→同一CANの所有確認→実機用launchの順に行う。
`auto_enable:=true`はlaunch時点で作用し、prompt付きStartまでハードウェアEnableを待つ設定ではない。
実機用の同じカメラ・CAN指定を保ち、`dry_run:=false auto_enable:=true`へ変更する。

### 4. Runtime dry-runの出力確認

```bash
rostopic echo -n 1 /vla_dual/raw_action
rostopic echo -n 1 /vla_dual/selected_action
rostopic echo -n 1 /vla_dual/hold
rostopic echo -n 1 /vla_dual/safety_state
```

期待する動作：

- `raw_action`と`selected_action`は14要素。
- `selected_action`は実行周期で進む。
- Runtime dry-runなので`approved_action`は送信しない。
- RuntimeはHOLD=falseを送らず、HOLDはtrueのまま。
- Controllerの左右目標診断が出ないことは、この段階では正常。

`approved_action`が無い状態で`rostopic echo -n 1`すると待機し続けるため、Ctrl+Cで終了します。
この段階のtopic名の`raw`は正規化空間の意味ではなく、server返却chunk先頭の絶対値Actionです。

停止：

```bash
rosservice call /piper_vla_dual_runtime/stop '{}'
```

### 5. Controllerの判定まで確認する

**ROS launchは`dry_run:=true`のまま維持します。**
RuntimeだけをAction配信モードへ切り替えます。

1. ターミナルDでRuntime stopを呼ぶ。
2. ターミナルCで、同じ環境変数を設定したまま次を実行する。

```bash
cd /home/dnp218/piper_vla_runtime
./scripts/run_dual.sh --checkpoint "$DUAL_BUNDLE" --max-execution-steps 0 --controller-run
```

`--controller-run`は既存Runtime側の`dry_run=false`指定です。
スクリプトはclientを再作成するため、その後にstartし直します。

ターミナルD：

```bash
rosparam get /vla_dual_piper_controller/dry_run
# trueを確認する。

rosservice call /piper_vla_dual_runtime/start "prompt: 'Assembling a box.'"
```

確認するtopic：

```bash
rostopic hz /vla_dual/approved_action
rostopic echo -n 1 /vla_dual/approved_action
rostopic echo -n 1 /vla_dual/left/target_joint_command
rostopic echo -n 1 /vla_dual/right/target_joint_command
rostopic echo -n 1 /safety/left/state
rostopic echo -n 1 /safety/right/state
rostopic echo -n 1 /safety/left/reason
rostopic echo -n 1 /safety/right/reason
```

期待する動作：

- `approved_action`は正常時約20 Hz、左7→右7の14次元。
- Controllerの各腕が安全判定を通れば、左右の`target_joint_command`が出る。
- Controllerの成功状態は`DRY_RUN`。
- `/followerL/joint_states`・`/followerR/joint_states`へはVLA指令を送らない。

片腕検証失敗では両腕未送信・fault通知となり、明示startまで送信を停止します。ただし左目標の診断が出た後に右の検証が失敗する場合があり、
**目標診断が出たこととdriverへ送信されたことは別**です。
関節範囲・Z・gripperの拒否理由を読み、正常なActionへ値を強制変更して通さないでください。

確認後は必ずRuntime stopを呼びます。

```bash
rosservice call /piper_vla_dual_runtime/stop '{}'
```

### 6. 停止の意味と終了順序

| 操作 | 作用 |
|---|---|
| Runtime `/stop` | chunk破棄、遅延応答無効化、HOLD=true。通常の停止操作 |
| Controller `/stop` | 両腕HOLD、`controller_fault`通知。Runtimeにも停止を伝える補助経路 |
| 左右driver `/stop_srv` | 各driverからSDKの停止指令を送る |
| launchのCtrl+C | ROSノード終了。物理的な非常停止の代わりとは扱わない |

Runtimeのstopは「新しい指令を送らない」処理です。driverへ既に渡した目標を取り消したり、
両腕のモータを無効化したりする処理ではありません。

必要な場合の補助停止：

```bash
rosservice call /vla_dual_piper_controller/stop '{}'
```

ハードウェア停止を行う場合の既存driver service：

```bash
rosservice call /followerL/stop_srv '{}'
rosservice call /followerR/stop_srv '{}'
```

左右serviceは順番に呼ばれます。通信断や既に送信済みのActionに対して、完全な同時停止・取り消しを保証するものではありません。
driverのreset・再enableは別の復帰操作です。通常のRuntime再startと混同しないでください。

通常終了は、Runtime stop→ROS launch終了→必要ならRuntimeコンテナ停止→最後にroscore終了の順です。
Runtimeコンテナは`run_dual.sh`終了後もバックグラウンドで稼働します。

**ホスト：双腕コンテナだけを確認する**

```bash
docker ps --filter label=com.docker.compose.project=piper-vla-dual \
  --format 'table {{.ID}}\t{{.Names}}\t{{.Status}}'
```

停止する場合は表示された双腕client/policyのIDを指定します。

```bash
docker stop <双腕clientのCONTAINER_ID> <双腕policyのCONTAINER_ID>
```

単腕やPiPER Dockerを含む全コンテナへの一括停止は行いません。

### 7. 状態確認・ログの見方

#### 7.1 RuntimeとControllerは別の診断を出す

```bash
# Runtime：chunk、観測、推論timeout等
rostopic echo /vla_dual/safety_state
rostopic echo /vla_dual/safety_reason

# Controller：関節、FKのZ、gripper等
rostopic echo /safety/state
rostopic echo /safety/reason
rostopic echo /safety/left/target_z
rostopic echo /safety/right/target_z

# セッションのprompt・実行step・処理時間
rostopic echo /vla_dual/current_prompt
rostopic echo /vla_dual/execution_step
rostopic echo /vla_dual/timing
```

各`echo`は別terminalで実行するか、Ctrl+Cで終了して次へ進みます。
Runtimeの`DISPATCHED`はControllerへ送ったという意味で、実機到達や安全判定通過の確認ではありません。

#### 7.2 policy/clientのログ（ホスト）

```bash
docker ps -a --filter label=com.docker.compose.project=piper-vla-dual \
  --format 'table {{.ID}}\t{{.Names}}\t{{.Status}}'

docker logs --tail 100 <双腕policyのCONTAINER_ID>
docker logs --tail 100 <双腕clientのCONTAINER_ID>
```

`<...>`は一覧で確認した実際のIDへ置き換えます。
policyが終了していても`docker ps -a`からIDを確認できます。

#### 7.3 記録先

既定のログrootはホストの`/home/dnp218/piper_dual_logs`、client内では`/logs`です。
双腕Runtimeは設定`recording.enabled=true`ならstart時にセッション記録を開始します。
**双腕Runtimeはdry-runでも記録します。** 単腕の「実機runだけ記録」の条件とは異なります。
stop時に記録を閉じ、次のstartは別セッションになります。
単なるfault時は記録を開いたままにし、stopまたは次startで閉じる構成です。

### 8. よくあるエラーと確認先

| 症状・メッセージ | 原因候補 | 対応 |
|---|---|---|
| `sample_validity_key`のTypeError | 学習時OpenPI拡張が不足 | I編。使用環境の互換性を確認 |
| `ModuleNotFoundError: dnp1_bimanual_registration` | training root未指定、ファイル配置違い | `DUAL_TRAINING_ROOT`をbundleルートへ |
| `model_config.json`やcacheがない | `.py`だけをコピーした | bundle全体を同じ構造でmount |
| `KeyError: state`、`images`、`cam_high` | Runtimeとexportの入力形式が未接続 | I編の観測アダプターと使用環境を確認 |
| `policy contract mismatch` | manifest/返却metadataが異なる | 左右順序・50×14・単位・quantileを確認。契約だけ偽装しない |
| `training config dataset/asset ID does not match manifest` | 違うconfigやstatsを選択 | 両方`local/dnp1_20261015`で照合 |
| `norm_stats.json`がない | assetsの階層違い | `checkpoint/assets/local/dnp1_20261015/`を確認 |
| `FileNotFoundError`やOCDBTエラー | params欠落・破損 | `sha256sum -c`で再確認、export一式を再転送 |
| serviceが見つからない | client未起動、policy readiness失敗、ROS master不一致 | policy/clientログ・`ROS_MASTER_URI`を確認 |
| service型が不明 | PiPER Dockerで共有msg未ビルド/未source | `piper_vla_runtime_msgs`をビルドしてsetupをsource |
| `waiting_for_post_start_observations` | start後にStateか画像が更新されていない | 左右＋3画像の5topicを確認 |
| `observation_missing/stale/time_skew` | 観測欠落・古いstamp・カメラ遅延 | 5topicの周波数・stamp・USB帯域を確認 |
| `policy_timeout/initial_chunk_timeout/chunk_underrun` | 推論・初回compileが遅い、次chunk不足 | GPU/通常sampling時間とログを確認。無条件でtimeoutを緩めない |
| approved Actionが出ない | Runtimeがdry-run、HOLD、fault | `--controller-run`の有無とRuntime理由を確認 |
| approved Actionは出るが目標が出ない | ControllerがHOLD、検証拒否 | `/safety/.../reason`を確認 |
| 目標は出るが実機が動かない | Controller dry-run、driver未enable | 現在の検証段階を確認。dry-run中なら正常 |
| 同じCANへの通信が競合 | 単腕・教示Follower等が残っている | 重複所有を解消してから再起動 |
| カメラが開かない | 他プロセス使用、デバイス未mount、形式不一致 | Dockerのdevice一覧・FourCC/FPSを確認 |
| `already running; call ~stop first` | 実行中にstartを重ねた | stop→prompt付きstartで切替 |

### 9. 実CANを使わない確認

```bash
roslaunch piper start_vla_dual_piper.launch mock_driver:=true dry_run:=true
```

この指定では実driverと実カメラを起動しない。ただしControllerはSDKの純粋FKをimportする。
Mock launchだけでは3画像やpolicyがないため、Runtime Start後の推論までは成立しない。
元READMEにあった`test_vla_dual_piper_controller.py`・`test_vla_piper_controller.py`は現在の作業ツリーにないため、実行コマンドとして再掲しない。
過去の隔離統合テスト成果物`/tmp/vla_ros_integration/REPORT.md`は一時ファイルの記録であり、配布物・今回の検証結果とは扱わない。

---

<a id="legacy"></a>

## I. 旧手順・step 1000固有の補足

### 1. step 1000の配置・互換性記録

元`README_place.md`は、`dnp1_20261015_bimanual_pi05_delta_1000`向けの詳細な導入記録。
当時は`sample_validity_key`登録互換性とRuntime観測→学習入力の変換が未実装と記載されていた。
2026-10-25のソース確認では、現在の`/home/dnp218/piper_vla_runtime`に次の実装がある。

| 確認ファイル（Runtime配下） | 内容 |
|---|---|
| `src/piper_vla_dual_runtime/registration_compat.py` | 登録時の`sample_validity_key`互換処理 |
| `src/piper_vla_dual_runtime/observation_adapter.py` | `observation/state`→`state`、画像キー変換、HWC→CHW |
| `src/piper_vla_dual_runtime/bundle_policy.py` | 上記互換処理・アダプターの使用箇所 |
| `scripts/run_dual.sh` | `--warmup-prompt`、`--max-execution-steps`、`--controller-run`引数 |

したがって「現在も未実装」という記述は統合版の現状説明には採用しない。
ソースの存在を確認した範囲であり、稼働中イメージへの反映・モデルロード・実推論の成功は未確認。
元の互換性分析・配置一式・検証手順は[README_place.md](README_place.md)に保持している。

Bundle内の登録コード・モデル設定・統計・tokenizer cache・重みは一式で扱う。
`params`の一部や`.py`だけを抜き出さない。`DUAL_TRAINING_ROOT`は実際のBundle構造に合わせて指定する。
現在の`run_dual.sh`はBundleの`training/inference_config.json`があれば`training/`を、なければBundleルートを既定に選ぶ。既に`DUAL_TRAINING_ROOT`を設定している場合はその値を使うため、別Bundleの設定が残っていないか確認する。

学習時とRuntime入力の対応は次の通り。Stateは14次元、画像はRGBのuint8。

| Runtime側 | 学習入力側 |
|---|---|
| `observation/state` | `state` |
| `observation/front_image`（HWC） | `images.cam_high`（CHW） |
| `observation/left_wrist_image`（HWC） | `images.cam_left_wrist`（CHW） |
| `observation/right_wrist_image`（HWC） | `images.cam_right_wrist`（CHW） |

観測アダプターで関節順序・単位・gripper値を変更したり、Delta復元を重複適用したりしない。
Bundleの`sha256sum -c checksums.sha256`、登録、14次元統計、有限な`(50,14)`のAction、
実際のRuntime形式での入力、GPUでの通常sampling時間を段階的に確認する。
元READMEのCPU合成観測1回・Mockで約20 Hzという過去の記録は、実機での速度・タスク品質の保証ではない。

#### Controllerの停止・再開プロトコル

Action検証失敗では両腕へ未送信のまま`/vla_dual/controller_fault`へ通知し、faultをラッチする。
正常ActionやHOLD=falseでは解除しない。通常のprompt付きRuntime Startで再開する。
最初のchunk準備後に`/vla_dual/controller_start`（Trigger）を呼び、応答tokenを受けてからActionを送る。
Actionのlayoutには`controller_session:<token>`を付け、旧セッションの遅延Actionは無視する。
HOLD topicの値だけを解除確認に使わず、service応答・診断も確認する。

### 2. 旧推論READMEと継続推論README

[README_PiPER_pi05_inference_old.md](README_PiPER_pi05_inference_old.md)のBundle作成・転送はG編と共通。
主な相違は、旧版のRuntime起動が`--dry-run`、ROS側の関節制限引数が省略されていること。
現行launchの`start_sdk_joint_limit`・`controller_joint_limit_enabled`はともに既定true。
G編は元の新しい推論READMEの例どおりController側だけfalseを明示しており、既定値の説明とは区別する。

[README_infer.md](README_infer.md)の継続推論手順はG編に統合済み。
`--max-execution-steps 0`で実行step上限だけを無効化し、異常時のHOLDは維持する。
Stopは残りのActionチャンクを破棄して両腕HOLDを要求する。
通常のStop／StartではRuntime稼働中のモデルを毎回ロードし直す必要はない。

### 3. 旧収集手順の扱い

[README_collect.md](README_collect.md)の第5章は旧`collect_dual_piper.launch`向け。
`t`によるタスク入力、専用ターミナル、`Dataset Count`、日時名フォルダ、`q`でレコーダーだけ終了、
`/dual_state_mcap_recorder/...`などは旧版の挙動で、MCAP2の操作に混ぜない。
旧データの説明・旧プロンプト入力手順は元ファイルに残し、現行の起動・収集方法はD編へまとめた。

---

<a id="sources"></a>

## 元READMEとの対応

| 元ファイル | 統合先 |
|---|---|
| [README_camera.md](README_camera.md) | C編：接続・プレビュー・画質設定・調査記録 |
| [README_collect.md](README_collect.md) | B編：準備、D編：現行収集へ更新、I編：旧操作の相違 |
| [README_infer.md](README_infer.md) | G編：手動Stopまでの継続推論 |
| [README_LeRobot_conversion .md](<README_LeRobot_conversion .md>) | E編：整理・転送・変換 |
| [README_PiPER_pi05_training.md](README_PiPER_pi05_training.md) | F編：設定・Fine-tuning |
| [README_PiPER_pi05_inference .md](<README_PiPER_pi05_inference .md>) | G編：Bundle作成から推論まで |
| [README_PiPER_pi05_inference_old.md](README_PiPER_pi05_inference_old.md) | G編の共通手順、I編の旧版との差分 |
| [README_place.md](README_place.md) | H編：観測・確認・停止・診断、I編：step 1000固有の記録 |

関連実装：

- [Docker起動・マウント](run.sh)
- [SDK準備・ROS環境関数](assets/piper_shell.sh)
- [MCAP2収集の詳細](double_PiPER/src/piper/README_dual_mcap2.md)
- [双腕推論launch](double_PiPER/src/piper/launch/start_vla_dual_piper.launch)
- [推論用初期姿勢YAML](double_PiPER/src/piper/config/piper_dual_demo_start.yaml)
- [双腕Controller](double_PiPER/src/piper/scripts/vla_dual_piper_controller.py)
- [Runtime起動スクリプト](../piper_vla_runtime/scripts/run_dual.sh)
- [Runtime基準設定](../piper_vla_runtime/configs/runtime.dual.yaml)
