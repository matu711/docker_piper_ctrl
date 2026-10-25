# PiPER セットアップ README

本READMEでは、Docker環境の起動からROS Masterの起動、双腕リーダーフォロワーによる教示データ収集までの手順を説明する。

## 1. Preliminary（Docker環境の起動）

### 手順1：ユーザー名の設定

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

### 手順2：Dockerイメージのビルド

`build.sh` があるディレクトリで実行する。

```bash
sh build.sh
```

ビルドが正常に完了したことを確認する。

### 手順3：Dockerコンテナの起動

```bash
sh run.sh
```

コンテナが正常に起動すれば、Docker環境の初回設定は完了となる。

**2回目以降の起動**

- 通常時：手順3から実行する。
- `Dockerfile` を変更した場合：手順2から実行し、Dockerイメージを再ビルドする。
- ユーザー名を変更する場合：手順1から実行する。

---

## 2. PiPER SDKのダウンロードとROSのビルド

PiPER SDKのダウンロード、ROSワークスペースのビルド、および環境設定を行う。以下のコマンドはDockerコンテナ内で実行する。

### 手順1：PiPER SDKのダウンロードとROSのビルド

```bash
set_sdk
```

`set_sdk` は、PiPER SDKのダウンロードとROSワークスペースのCMakeビルドを行うための自作コマンドである。

### 手順2：ROS環境の読み込み

```bash
sorc_devel
```

`sorc_devel` は、ROSのcatkinワークスペースに移動し、`setup.bash` を `source` して環境設定を読み込む自作コマンドである。

現在の `Dockerfile` では、以下の処理を簡略化している。

```bash
cd ~/double_PiPER
source devel/setup.bash
```

実際の実行ディレクトリやコマンド定義は、`Dockerfile` の `alias` 部分を参照すること。

**注意事項**

- `set_sdk` は初回セットアップ時に実行する。
- **新しいターミナルを開いた場合も、必ず `sorc_devel` を実行する。**
- 各自作コマンドの詳細は `Dockerfile` の `alias` 定義を参照すること。

---

## 3. CAN通信のアクティブ化

PiPERとPC間でCAN通信を行うため、USB-CANアダプタを1つずつ接続して設定する。`can0` から順に確認する。

### 手順1：USB-CANアダプタの接続と確認

1つ目のUSB-CANアダプタを接続し、以下のコマンドを実行する。

```bash
sudo ethtool -i can0 | grep bus
```

**出力例**

```text
bus-info: 3-11:1.0
```

表示された `bus-info` の値（この例では `3-11:1.0`）を確認する。

### 手順2：CANインターフェースのアクティブ化

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

### 手順3：残りのCANインターフェースの設定

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

## 4. ROSの起動

Dockerコンテナ内でROS Masterを起動する。

### 手順1：ROS環境の読み込み

新しいターミナルをDockerコンテナ内で開いた場合は、以下のコマンドを実行する。

```bash
sorc_devel
```

### 手順2：ROS Masterの起動

```bash
roscore
```

正常に起動すれば、ROSノード間の通信に必要なROS Masterが使用可能になる。

**注意事項**

- `roscore` を起動したターミナルは、ROSを使用している間は閉じないこと。
- 別のターミナルでROS関連コマンドを実行する場合も、事前に `sorc_devel` を実行すること。

---

## 5. リーダーフォロワーによるデータ収集

双腕リーダーフォロワーを起動し、左右の実測State・送信Actionと3台のカメラ画像を同期して収集する。
記録には `collect_dual_piper.launch` を使用する。

**使用ファイル（統一後の名称）**

| 役割 | ファイル |
|---|---|
| 双腕起動・収集 | [collect_dual_piper.launch](double_PiPER/src/piper/launch/collect_dual_piper.launch) |
| 双腕Leader | [piper_dual_leader.py](double_PiPER/src/piper/scripts/piper_dual_leader.py) |
| 双腕Follower | [piper_dual_follower.py](double_PiPER/src/piper/scripts/piper_dual_follower.py) |
| 双腕データ収集 | [record_dual_piper.py](double_PiPER/src/piper/scripts/record_dual_piper.py) |

### 5.1 機器構成と起動前の確認

**CAN構成**

| アーム | CANポート |
|---|---|
| 右Leader | `can0` |
| 右Follower | `can1` |
| 左Leader | `can2` |
| 左Follower | `can3` |

**カメラ構成**

| カメラ | デバイス | 要求解像度 | 要求FPS | FourCC |
|---|---|---|---|---|
| front | `/dev/video4` | 1280×720 | 30 | MJPG |
| wrist1 | `/dev/video2` | 640×480 | 30 | MJPG |
| wrist2 | `/dev/video0` | 640×480 | 30 | MJPG |

起動前に以下を確認する。

- 第3章の手順で4つのCANをアクティブ化し、各アームとの対応を確認する。
- 同じCANを使用する別の制御launchを終了する。
- カメラテストなど、同じカメラを使用するプログラムを終了する。
- 起動時にはモーターの有効化や追従による動作があるため、アーム周辺を確認する。

### 5.2 収集システムの起動

`roscore` のターミナルを残したまま、Dockerコンテナ内で別のターミナルを開いて実行する。

```bash
sorc_devel

roslaunch piper collect_dual_piper.launch \
  can_leader_R_port:=can0 \
  can_follower_R_port:=can1 \
  can_leader_L_port:=can2 \
  can_follower_L_port:=can3 \
  front_camera:=/dev/video4 \
  wrist1_camera:=/dev/video2 \
  wrist2_camera:=/dev/video0 \
  camera_fps:=30 \
  record_fps:=20 \
  auto_enable:=true
```

これらは現在の既定値なので、通常は次のコマンドでも起動できる。

```bash
roslaunch piper collect_dual_piper.launch auto_enable:=true
```

起動すると、次の2つのウィンドウを使用できる。

1. **映像ウィンドウ：** `PiPER dual teaching recorder`。3台の映像、記録状態、`Dataset Count: XX` を表示する。
2. **キー入力ターミナル：** `PiPER-Recorder`。キー操作、タスクプロンプトの入力、記録結果の確認に使用する。

起動ログの各カメラの実解像度・FPS・FourCCを確認する。要求設定と異なる場合は警告が表示される。
通常の操作でリーダーを教示モードにし、左右のFollowerが追従することを確認してから記録する。
**`r`は記録開始、`t`はタスク入力であり、ロボットを教示モードへ切り替える操作ではない。**

### 5.3 キー操作

映像ウィンドウまたは専用ターミナルを選択してキーを押す。通常の操作ではEnterは不要。

| キー | 操作 |
|---|---|
| `t` | タスクプロンプト入力モードに切り替える |
| `r` | 新しいエピソードの記録を開始する |
| `s` | 記録を停止してエピソードを保存する |
| `x` | 記録中のエピソードを破棄する |
| `q` | レコーダーのみを終了する |

- 映像の `IDLE` は記録していない状態、`RECORDING` は記録中を示す。
- 記録開始には、左右の新しいState・Actionと3台の画像が必要。
- `s`で保存が完了すると、保存先がターミナルに表示され、映像内の件数が更新される。
- 記録開始・停止・破棄は、ロボットの追従状態や教示モードを変更しない。
- `q`でレコーダーを終了しても、リーダーフォロワー制御は継続する。
- **保存して終了する場合は、`s`の保存完了を確認してから`q`を押す。**
- 記録中のまま`q`またはCtrl+Cで終了すると、データは未完了として保持される。
- `roslaunch` を実行した元のターミナルでCtrl+Cを押すと、launch全体を終了する。
- 専用ターミナルでCtrl+Cを押すと、レコーダーのみを終了する。

### 5.4 タスクプロンプトの入力

タスクプロンプトには、そのエピソードで行う作業内容を記述する。

**入力手順**

1. 記録していない状態で`t`を押す。
2. **`PiPER-Recorder`の専用ターミナルを選択する。** 映像ウィンドウから`t`を押した場合も、文字列は専用ターミナルで入力する。
3. 表示された `Task input:` に作業内容を入力する。
4. Enterで確定する。
5. `Current task:` に確定した内容が表示されたことを確認する。
6. `r`を押して記録を開始する。

**入力例**

```text
青い袋を右手で持ち上げ、左手で支える
```

**入力時の動作**

- 日本語・英語の文字列を入力できる。
- Backspaceで文字を削除できる。
- Enterのみ、または空白のみで確定した場合は、以前のプロンプトを維持する。
- Escで入力をキャンセルできる。
- 入力中の`r/s/x/q/t`は文字として扱われる。
- 記録中や記録操作の処理中は、プロンプトを変更できない。
- 確定したプロンプトは、次に`t`で変更するまで保持される。
- `r`を押した時点のプロンプトをエピソードに固定し、各エピソードの `meta.json` の `task` に保存する。
- データ読み込み時は、データセット全体のタスク文ではなく、**各エピソードの `meta.json.task` を優先する。**

**起動時に初期プロンプトを指定する場合**

```bash
roslaunch piper collect_dual_piper.launch \
  auto_enable:=true \
  task:="青い袋を右手で持ち上げ、左手で支える"
```

`task`を指定しない場合の初期値は `bimanual demonstration`。

### 5.5 収集の流れ

1. システムを起動し、左右の追従と3台の映像を確認する。
2. `t`でタスクを入力し、Enterで確定する。
3. `r`で記録を開始し、画面の `RECORDING` を確認する。
4. リーダーを操作して作業を行う。
5. `s`で停止・保存し、保存結果と `Dataset Count` の増加を確認する。
6. 同じ作業ならそのまま`r`で次のエピソードを開始する。作業内容を変える場合は、先に`t`でタスクを変更する。
7. 収集を終了するときは、保存完了後に`q`を押し、元のターミナルでCtrl+Cを押して制御ノードも終了する。

### 5.6 保存先・フォルダ名・データ構造

ホスト側の保存先は以下に固定されている。

```text
/home/dnp218/docker_piper_env/piper_dataset
```

`dataset_root`のlaunch引数を指定しても保存先は変更されない。
現在のDocker環境では、同じホストディレクトリを指す `/home/dnp218/piper_dataset` のbind mountを経由して保存する。

保存完了時の日本時間（Asia/Tokyo）で、フォルダ名を `YYYYMMDD_HHMMSS` にする。
同じ秒に複数保存した場合は `_01`、`_02` を付け、既存フォルダを上書きしない。

```text
piper_dataset/
  dataset_meta.json
  20261015_003045/
    episode_<記録開始時刻ns>.mcap
    data.npz
    frames.csv
    meta.json
    front/
      000000.jpg ...
    wrist1/
      000000.jpg ...
    wrist2/
      000000.jpg ...
```

| データ | 内容 |
|---|---|
| State | 各Followerの実測6関節＋グリッパー、合計14次元 |
| Action | 各FollowerへのSDK指令呼び出し後の6関節＋グリッパー、合計14次元 |
| 配列順序 | 右7次元 → 左7次元 |
| 単位 | 関節：rad、グリッパー：m |
| NPZ・CSV | State・Action、基準時刻、各データの元時刻・同期誤差、画像パス、画像再利用回数 |
| JPEG | 各カメラの実解像度の画像 |
| MCAP | 元のROS State・Actionと同期行に対応する圧縮画像 |
| `meta.json` | エピソードのタスク、設定、保存状態、行数、欠落数、画像再利用回数 |

20Hzの固定基準時刻に同期し、State・Actionは±30ms、画像は±100ms、3台のカメラ間は最大30msを許容する。
画像FPSが下がっても許容範囲内の画像は再利用する。範囲外の行は保存せず、欠落数を記録する。
したがって、同期できない区間がある場合の保存行数は毎秒20行より少なくなる。

`Dataset Count: XX`は、保存先直下の保存完了エピソード数を示す。起動時と保存完了後に更新する。
`.tmp_episode_*`などの記録途中・未完了フォルダ、管理用フォルダ、サブフォルダ内のエピソードは数えない。
過去のエピソードを移動・変更・削除する処理は行わない。

### 5.7 ROSサービスからの操作

キー操作が使えない場合は、Dockerコンテナ内の別ターミナルで実行する。

```bash
sorc_devel

# 記録開始
rosservice call /dual_state_mcap_recorder/start "{}"

# 記録停止・保存
rosservice call /dual_state_mcap_recorder/stop "{}"

# 記録中エピソードの破棄
rosservice call /dual_state_mcap_recorder/discard "{}"
```

サービスから開始した場合も、その時点で確定しているタスクプロンプトを使用する。
タスク入力モード中は記録開始できないため、Enterで確定するかEscでキャンセルしてから開始する。

収集コードの詳細は [README_dual_state_mcap_edit.md](double_PiPER/src/piper/README_dual_state_mcap_edit.md) を参照する。
