# dnp1_20261015_bimanual_pi05_delta_1000：双腕PiPER推論手順

このREADMEは、学習用PCから持ってきた **step 1000の双腕π0.5 checkpoint** を、
このPCのROS1 Noetic環境で使用するための手順です。
標準の操作は、単腕と同じ **PiPER launch → Runtime起動 → prompt付きstart → stop** です。
Leaderは使用しません。

## 0. 最初に読む：現時点で起動前に解消する項目

**このcheckpointは配置済みですが、現在の推論環境へそのまま接続できる状態ではありません。**
README作成時に実コードを照合し、次の2点を確認しました。

| 項目 | checkpoint側 | 現在の推論側 | 必要な対応 |
|---|---|---|---|
| OpenPIの設定拡張 | `bimanual_data.py`が`DataConfig(sample_validity_key=...)`を使用 | vendored OpenPIの`DataConfig`にそのフィールドがない | 学習時の拡張と互換性のあるpolicy環境を用意する |
| Policyへの観測入力 | `state`、`images.cam_high/cam_left_wrist/cam_right_wrist`、CHW画像 | Runtimeは`observation/state`、`observation/{front,left_wrist,right_wrist}_image`、HWC画像を送信 | Policy入口等でキー変換とHWC→CHW変換を行う |

1つ目は登録時の`TypeError: ... unexpected keyword argument 'sample_validity_key'`、
2つ目は初回推論時の`KeyError: 'state'`などにつながる構造です。
現在の`ContractPolicy.infer()`は観測をそのまま下位policyへ渡すため、変換は自動では入りません。

**第4章の対応・第5章の確認が完了するまでは、第6章以降の実機接続手順へ進まないでください。**
このREADMEの作成では、コード修正、Docker起動、モデルロード、推論、CAN接続、実機動作は行っていません。
以下のコマンドは、利用者が段階ごとに実行する手順です。

### 確認済みと未確認の区別

| 対象 | 状況 |
|---|---|
| checkpoint配置・manifest・登録コード・統計ファイル | このPCで読み取り確認済み |
| 学習用PCでのexport検証 | 同梱`validation.json`に、実重みロード・合成観測・CPU・sampling 1回の成功記録あり |
| ROS統合 | 過去の隔離テストでMock Policy・Mock driver・SDK代替を使って確認済み。45 Action、約20.02 Hz |
| このcheckpointと現在のRuntimeの接続 | 上記2点が未対応。実行未確認 |
| 実GPUの通常sampling速度、実SDK FK、実カメラ・CAN・双腕動作 | このcheckpointでは未確認 |

CPUでsampling 1回が成功したことは、通常samplingで20 Hz実行に間に合うことや、
タスクが成功することの確認ではありません。また、step 1000という保存番号だけでは品質は判断できません。

### 2026-10-16：Controllerの停止・再開プロトコル更新

Action検証失敗は両腕未送信に加え`/vla_dual/controller_fault`へ通知し、faultをラッチする。
正常ActionやHOLD=falseでは解除しない。通常のprompt付きRuntime startで再開する。
最新Runtimeは最初のchunk準備後に`/vla_dual/controller_start`（Trigger）を呼び、
ControllerがHOLD解除を処理した応答tokenを受けてからActionを送る。
Actionのlayoutには`controller_session:<token>`を付け、旧セッションの遅延Actionは無視する。
固定sleepやHOLD topicとAction topicの到着順を、解除の保証に使用しない。
単腕の安全閾値・Clamp・判定順序に変更はない。

以下の章にあるHOLD topicの`true`は停止要求の値であり、service解除後も最新Runtimeは
その値をfalseへ更新しない。実際の解除確認は上記service応答で行う。

## 1. 使用するファイルと配置

### 1.1 現在見つかっているexport

ホストPC上の実配置は次のとおりです。全体で約6.0 GBあります。

```text
/home/dnp218/dnp1_20261015_bimanual_pi05_delta_1000/
├── manifest.yaml
├── model_config.json
├── training_config.yaml
├── dnp1_bimanual_registration.py
├── bimanual_data.py
├── checkpoint/
│   ├── params/                         # JAX / Orbax / OCDBTの重み一式
│   │   ├── _METADATA
│   │   ├── _sharding
│   │   ├── manifest.ocdbt
│   │   ├── d/...
│   │   ├── array_metadatas/...
│   │   └── ocdbt.process_0/...
│   └── assets/local/dnp1_20261015/norm_stats.json
├── cache/big_vision/paligemma_tokenizer.model
├── provenance/
│   ├── openpi_local_changes.patch
│   ├── pyproject.toml
│   ├── uv.lock
│   ├── train_bimanual_pi05.py
│   ├── source_checkpoint_metadata.json
│   └── source_verification.json
├── verify_export.py
├── validation.json
├── checksums.sha256
└── README.md
```

`params/`内部のファイルは分割checkpointの構成要素です。大きいファイルだけを抜き出さず、
ディレクトリ全体を維持します。optimizer、学習データ全体、学習元PCの絶対パスは推論に不要です。

### 1.2 実際のmanifest値

| 設定 | 今回の値 |
|---|---|
| `config_name` | `pi05_piper_bimanual_delta_lora` |
| `dataset_repo_id` | `local/dnp1_20261015` |
| `asset_id` | `local/dnp1_20261015` |
| `checkpoint_step` | `1000` |
| `checkpoint_path` | `checkpoint` |
| `training_registration` | `dnp1_bimanual_registration:register` |
| 学習時OpenPI commit | `460c59fc663d4b9fc0c80eb87e346c11ccfa86d9` |
| 学習時task / 推論prompt | **`Assembling a box.`** |

同梱登録関数は、exportルートの`model_config.json`と`bimanual_data.py`を読みます。
`training_config.yaml`は参考スナップショットであり、登録時の実際のモデル構成は`model_config.json`です。
登録関数はtokenizer用の`OPENPI_DATA_HOME`をexport内の`cache/`へ設定します。

### 1.3 推奨：checkpoint専用ディレクトリへ配置する

現行`run_dual.sh`は、指定bundleの**親ディレクトリ全体**をpolicy/clientへread-only mountします。
元の`/home/dnp218/...`を直接指定するとホーム全体がmount対象になるため、
checkpoint専用の`~/piper_checkpoints/`へ独立コピーする配置を推奨します。

**ホストPCで実行：**

```bash
mkdir -p /home/dnp218/piper_checkpoints

export DUAL_EXPORT_SOURCE=/home/dnp218/dnp1_20261015_bimanual_pi05_delta_1000
export DUAL_BUNDLE=/home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_1000

# コピー先が存在しない場合だけ実行する。既存bundleへの混在コピーを避ける。
if [ -e "$DUAL_BUNDLE" ]; then
  echo "配置済みです。上書きせず、次の整合性確認へ進んでください: $DUAL_BUNDLE"
else
  cp -a "$DUAL_EXPORT_SOURCE" "$DUAL_BUNDLE"
fi
```

既存bundleの中身が別stepの場合は、その場で上書きせず、別名ディレクトリへ配置し直します。
以降のコマンドは、上記のコピー先を使用します。

### 1.4 転送・コピー後のファイル確認

**ホストPCで実行：**

```bash
cd /home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_1000
sha256sum -c checksums.sha256
```

すべて`OK`であることを確認します。重みの読み取りには時間がかかります。
`FAILED`や欠落がある場合は推論へ進まず、元exportとコピーを確認します。
エラーを消すためにチェックサムを作り直したり、manifestや重みを上書きしたりしないでください。

## 2. 操作するPC・コンテナ・ターミナル

同じPC上で、役割の異なるコンテナを使用します。

| 場所 | 役割 | 主な操作 |
|---|---|---|
| ホスト `/home/dnp218/docker_piper_env` | PiPER用Dockerの起動 | `sh run.sh` |
| PiPER Docker `piper-env-image-matu` | ROS master・CAN driver・カメラ・Controller | `roscore`、`roslaunch`、`rosservice` |
| ホスト `/home/dnp218/piper_vla_runtime` | policy/clientコンテナの管理 | `./scripts/run_dual.sh ...` |
| 双腕policyコンテナ | GPUモデルロード、WebSocket推論 | 通常はスクリプトから起動 |
| 双腕clientコンテナ | State/画像購読、chunk管理、Action配信 | 通常はスクリプトから起動 |

基本的には次の4ターミナルを用意します。

- **A：PiPER Docker内**で`roscore`を維持。
- **B：PiPER Docker内**で双腕launchを維持。
- **C：ホスト**で双腕Runtimeを起動・ログ確認。
- **D：PiPER Docker内**でtopic確認とprompt付きstart/stop。

ホスト側の`/workspace/...`と、PiPER Docker内の`~/double_PiPER`は別のパスです。
このPCのcheckpoint指定には学習PCの`/workspace`パスを使いません。

## 3. 推論で使う観測・Action・安全制御

### 3.1 StateとActionの並び

```text
index  0..5 : 左joint1..6      [rad]
index  6    : 左gripper開幅    [m]
index  7..12: 右joint1..6      [rad]
index 13    : 右gripper開幅    [m]
```

学習内部では12関節を観測State基準のDeltaにしていますが、serverから返るActionは
**逆正規化・絶対値復元済み**です。ROS側でStateを再加算してはいけません。
`adapt_to_pi=False`で学習しているため、ALOHA固有の関節符号反転・gripper変換も追加しません。

### 3.2 画像とROS topic

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

### 3.3 周期とHOLD

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

### 3.4 Controllerの安全判定

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

関節の物理範囲そのものはSDKの公式設定を使用します。margin内の超過は物理範囲へClampし、
margin外は拒否します。Python単体の関節margin既定値は0.05 radですが、launchは単腕同様0.1 radを渡します。
このREADMEでは新しい閾値・速度制限・双腕間衝突判定を追加しません。

## 4. 必須の互換性対応：現在は未実装

この章は**対応が必要な内容の説明**です。README作成時に修正は適用していません。

### 4.1 学習時OpenPI拡張との一致

同梱`bimanual_data.py`は、次の指定を行います。

```python
config.DataConfig(prompt_from_task=True, sample_validity_key="quality.frame_valid")
```

現在の`piper_vla_runtime/vendor/openpi_pbl5/src/openpi/training/config.py`には
`sample_validity_key`がありません。同梱の
`provenance/openpi_local_changes.patch`にはその拡張と学習時の無効開始点除外処理が記録されています。

学習時commit・依存情報・差分を確認し、**双腕policy用の検証環境**で同じ登録コードが動くようにします。
パッチは`a/openpi/...`を起点とするため、適用先ディレクトリや既存変更を確認せずに適用しないでください。
単腕で使用中のOpenPIへ無条件に上書きする手順は採りません。

推論ではLeRobotの無効開始点フィルタは実行しませんが、登録コードがそのフィールドを要求するため、
登録時の互換性は必要です。manifestや`quality.frame_valid`の名称を書き換えるだけでは解決しません。

### 4.2 Runtime入力からexport入力への接続

現在のRuntimeの送信形式：

```python
{
    "observation/state": state14,
    "observation/front_image": front_hwc,
    "observation/left_wrist_image": left_hwc,
    "observation/right_wrist_image": right_hwc,
    "prompt": prompt,
}
```

今回のexportが要求する形式：

```python
{
    "state": state14,
    "images": {
        "cam_high": front_chw,
        "cam_left_wrist": left_chw,
        "cam_right_wrist": right_chw,
    },
    "prompt": prompt,
}
```

必要な入口変換の内容は以下です。

```python
# 仕様説明用の例。現在のserverへはまだ組み込まれていません。
export_observation = {
    "state": runtime_observation["observation/state"],
    "images": {
        "cam_high": runtime_observation["observation/front_image"].transpose(2, 0, 1),
        "cam_left_wrist": runtime_observation["observation/left_wrist_image"].transpose(2, 0, 1),
        "cam_right_wrist": runtime_observation["observation/right_wrist_image"].transpose(2, 0, 1),
    },
    "prompt": runtime_observation["prompt"],
}
```

ここで関節順序・単位・gripper値は変更しません。正規化・逆正規化・Delta復元も追加しません。
既存学習変換の前に、キーと画像軸だけを合わせます。
入力変換を修正した後は、実際のRuntime形式の保存観測を使い、serverまで接続して確認します。
exportの`verify_export.py`だけではこのRuntimeとの境界の一致までは検証できません。

## 5. モデル側の事前確認：ロボットへ接続しない

### 5.1 GPU・Dockerの確認（ホスト）

```bash
nvidia-smi
docker version
docker compose version
```

GPUが表示され、Docker Compose v2とGPU対応コンテナ実行環境が利用できることを確認します。
必要VRAMや通常sampling時の処理時間は、このcheckpointではこのPC上で未測定です。

### 5.2 training rootはbundle全体を指定

**Runtimeを起動するホストterminalでは、毎回次を設定します。**

```bash
export DUAL_BUNDLE=/home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_1000
export DUAL_TRAINING_ROOT="$DUAL_BUNDLE"
export DUAL_POLICY_HOST=127.0.0.1
export DUAL_POLICY_PORT=8001
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
```

policy内ではbundleが`/training`にmountされ、`PYTHONPATH`に`/training`が入ります。
`DUAL_TRAINING_ROOT`を`.py`ファイル自体や`checkpoint/`にすると登録モジュールをimportできません。
登録コードがモデル設定・統計・cacheを相対参照するため、**bundle全体**を指定します。

8001番portは双腕policy用、単腕の既定8000番とは別です。
本READMEはROS masterも同じLinux PC上にある構成を対象にしています。

### 5.3 オフライン検証の実行方法

第4章に対応したOpenPI環境内で、exportルートを`PYTHONPATH`へ追加して実行します。
ホストの素の`python3`にOpenPI/JAXが入っているとは限らないため、学習環境と互換性のあるPythonを使用します。

```bash
cd /path/to/export
PYTHONPATH="$PWD" JAX_PLATFORMS=cpu python verify_export.py
PYTHONPATH="$PWD" JAX_PLATFORMS=cpu python verify_export.py --load-model
PYTHONPATH="$PWD" JAX_PLATFORMS=cpu python verify_export.py --infer
```

`/path/to/export`は、その環境から見えるbundleルートです。
policyコンテナ内で行う場合は`/training`、Pythonは`/opt/openpi/.venv/bin/python`です。
最後の`--infer`だけが実重み・合成観測・sampling 1回のCPU推論を行います。
前2つの確認も含め、大きな重みのロードには時間とメモリが必要です。

次を確認してからROS側へ進みます。

1. `register()`が成功する。
2. `validate_training_config(register())`が成功する。
3. 統計のasset IDが`local/dnp1_20261015`、`state/actions`各14次元である。
4. 実重みがロードでき、返却Actionが有限な`(50,14)`である。
5. 第4.2節の接続を通したRuntime形式の観測でも推論できる。
6. GPUで通常samplingの処理時間を確認する。CPU sampling 1回の速度を流用しない。

## 6. PiPER DockerとROS masterの準備

**以降は実カメラ・CANを使用する手順です。第4〜5章が完了し、実機接続を行う段階で実施します。**

### 6.1 PiPER Dockerを起動する（ホスト）

まず既存コンテナを確認します。

```bash
docker ps --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}'
```

`piper-env-image-matu`が起動済みなら、新しく`run.sh`を実行せず、既存コンテナに入ります。
未起動の場合：

```bash
cd /home/dnp218/docker_piper_env
sh run.sh
```

新しいterminalから入る場合：

```bash
docker exec -it piper-env-image-matu bash
```

この`run.sh`は実機用のprivileged/host network構成です。
過去の完全隔離Mockテストの起動コマンドとは異なります。

### 6.2 ROS環境とメッセージ型を準備する（PiPER Docker）

```bash
source /opt/ros/noetic/setup.bash
cd /home/dnp218/double_PiPER
catkin_make --pkg piper_msgs piper_vla_runtime_msgs piper
source devel/setup.bash
rossrv show piper_vla_runtime_msgs/StartInference
```

期待するservice定義：

```text
string prompt
---
bool success
string message
```

既にビルド済みなら毎回ビルドする必要はなく、`source devel/setup.bash`だけで構いません。
以降のPiPER Dockerの新規terminalでも、ROSとcatkinのsetupをsourceします。
自作aliasの`sorc_devel`を使っても構いません。

SDKは既存単腕環境と同じ、公式FKと`C_PiperParamManager`を含むものが必要です。
`set_sdk`は既存READMEの初回インストール用であり、起動のたびに再実行するものではありません。

### 6.3 左右CANを確認する（PiPER Docker）

既定は左Follower=`can3`、右Follower=`can1`です。これは接続実態を保証しません。

```bash
ip -br link show type can
ethtool -i can1
ethtool -i can3
```

必要な場合だけ、実際のUSB bus-infoに置き換えて有効化します。

```bash
cd /home/dnp218/double_PiPER
bash can_activate.sh can1 1000000 "<右FollowerのUSB bus-info>"
bash can_activate.sh can3 1000000 "<左FollowerのUSB bus-info>"
```

`<...>`は説明用placeholderです。文字列のまま実行せず、`ethtool`の実結果で置き換えます。
既に正しい名前・1 Mbpsで有効なら、再設定は不要です。
同じCANを単腕launch、教示Follower、別SDKスクリプトが所有しないようにします。

### 6.4 カメラの実デバイスを確認する

ホストとPiPER Dockerの両方で、対象デバイスが見えることを確認します。

```bash
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
```

本READMEの例はfront=`/dev/video4`、left wrist=`/dev/video2`、right wrist=`/dev/video0`です。
video番号は接続順で変わるため、学習時と同じ物理視点へ割り当てます。
exportはwrist1=leftの正規化済み名称を使用していますが、実カメラの取り付けは現物確認が必要です。

カメラを後から接続した場合、既存PiPER Dockerへデバイスが渡っていない可能性があります。
その場合は実機側ノードを停止した上で、カメラが接続された状態でDockerを再作成します。

### 6.5 ROS masterを起動する（ターミナルA）

PiPER Docker内：

```bash
source /opt/ros/noetic/setup.bash
source /home/dnp218/double_PiPER/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
roscore
```

既に同じROS masterが稼働している場合は重複起動しません。
このterminalは起動したままにします。

## 7. 第1段階：実観測＋Runtime dry-run

### 7.1 ROS側をdry-run起動する（ターミナルB：PiPER Docker）

```bash
source /opt/ros/noetic/setup.bash
source /home/dnp218/double_PiPER/devel/setup.bash
roslaunch piper start_vla_dual_piper.launch \
  left_can_port:=can3 right_can_port:=can1 \
  front_camera:=/dev/video4 \
  left_wrist_camera:=/dev/video2 right_wrist_camera:=/dev/video0 \
  dry_run:=true auto_enable:=false
```

launchはdry_run中のauto_enableをfalseにするため、`auto_enable:=false`は意図を明示した指定です。
**実driverは起動します。** 初期化で`ConnectPort()`と`MotionCtrl_2(...)`が実行されます。
dry-runは「VLA関節指令を送らない」という意味で、「CAN通信なし」ではありません。

### 7.2 観測topicを確認する（ターミナルD：PiPER Docker）

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

### 7.3 Runtimeを起動する（ターミナルC：ホスト）

**第4章の2点に対応したpolicy環境を使用していることが前提です。**

```bash
cd /home/dnp218/piper_vla_runtime
export DUAL_BUNDLE=/home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_1000
export DUAL_TRAINING_ROOT="$DUAL_BUNDLE"
export DUAL_POLICY_HOST=127.0.0.1
export DUAL_POLICY_PORT=8001
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1

./scripts/run_dual.sh --checkpoint "$DUAL_BUNDLE" --dry-run
```

スクリプトは双腕専用Compose project`piper-vla-dual`でpolicyをbuild/startし、
WebSocket readinessを待ってからclientを起動します。
初回buildにはネットワーク・時間・ディスク容量が必要です。単腕の`.env`は読み書きしません。

最後に次のようなREADYが出ることを確認します。

```text
READY: /piper_vla_dual_runtime/start (prompt required); runtime dry_run=true
```

policyのreadinessに失敗した場合、スクリプトは新しいclientを起動しません。
以前のclientが残っている場合まで停止する処理ではないため、コンテナ一覧も確認します。

生成Runtime YAMLは`/tmp/piper-vla-dual.XXXXXXXX/runtime.dual.yaml`です。
スクリプトが表示したパスを確認すれば、実際に渡された`dry_run`やtopic設定を確認できます。

### 7.4 promptを指定して開始する（ターミナルD：PiPER Docker）

```bash
rosservice type /piper_vla_dual_runtime/start
rosservice type /piper_vla_dual_runtime/stop
```

期待する型：

```text
piper_vla_runtime_msgs/StartInference
std_srvs/Trigger
```

```bash
rosservice call /piper_vla_dual_runtime/start "prompt: 'Assembling a box.'"
```

`success: True`を確認します。promptは空白のみ・空文字を使わず、2048文字以内とします。
開始後に新しい左右State・3画像がそろうまで最初の推論は待機します。
手動でHOLD=falseやAction種別topicを送る必要はありません。

### 7.5 dry-runの出力確認

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

## 8. 第2段階：Controllerの判定まで確認する

**ROS launchは`dry_run:=true`のまま維持します。**
RuntimeだけをAction配信モードへ切り替えます。

1. ターミナルDでRuntime stopを呼ぶ。
2. ターミナルCで、同じ環境変数を設定したまま次を実行する。

```bash
cd /home/dnp218/piper_vla_runtime
./scripts/run_dual.sh --checkpoint "$DUAL_BUNDLE" --controller-run
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

## 9. 第3段階：実機推論を開始する

第1〜2段階でモデル・観測・左右順序・Action値・安全診断を確認してから進みます。
この章のlaunchは**実driverを自動enableし、start後は実際に両腕へ指令を送ります。**
step 1000のタスク品質は未評価です。非常停止手段と周囲を確認し、最初は短時間の動作で確認します。

### 9.1 dry-run launchを終了する

1. ターミナルD：Runtime stop。
2. ターミナルB：Ctrl+Cで現在のdry-run launchを終了。
3. 同じdriverを二重起動しないことを確認。

```bash
rosnode list
```

### 9.2 実機用launchへ切り替える（ターミナルB）

```bash
roslaunch piper start_vla_dual_piper.launch \
  left_can_port:=can3 right_can_port:=can1 \
  front_camera:=/dev/video4 \
  left_wrist_camera:=/dev/video2 right_wrist_camera:=/dev/video0 \
  dry_run:=false auto_enable:=true
```

`auto_enable=true`はlaunch時に有効になります。prompt付きstartまでハードウェアenableが待機するわけではありません。
start前はControllerがHOLDですが、driver初期化・enable・gripper初期化は別経路です。

### 9.3 RuntimeをAction配信モードで起動する（ターミナルC）

```bash
cd /home/dnp218/piper_vla_runtime
export DUAL_BUNDLE=/home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_1000
export DUAL_TRAINING_ROOT="$DUAL_BUNDLE"
export DUAL_POLICY_HOST=127.0.0.1
export DUAL_POLICY_PORT=8001
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
./scripts/run_dual.sh --checkpoint "$DUAL_BUNDLE" --controller-run
```

### 9.4 prompt付きstart（ターミナルD）

```bash
rosparam get /vla_dual_piper_controller/dry_run
# この段階ではfalse。

rosservice call /piper_vla_dual_runtime/start "prompt: 'Assembling a box.'"
```

**最初のActionが届くと実機動作が始まります。**
観測が変わらない、左右が逆、想定外の姿勢やgripper目標になる場合は停止して原因を確認します。

### 9.5 通常停止と再開

```bash
rosservice call /piper_vla_dual_runtime/stop '{}'
```

確認：

```bash
rostopic echo -n 1 /vla_dual/hold
# data: true
```

再開も同じprompt付きstartです。追加のenable操作や手動HOLD解除は通常不要です。

```bash
rosservice call /piper_vla_dual_runtime/start "prompt: 'Assembling a box.'"
```

初期姿勢へ自動で戻る処理は、このlaunch/startにはありません。
単腕用の`go_initial_pose.py`をそのまま双腕へ使う手順も含めていません。
停止後の現在姿勢から再開してよいか確認してからstartします。

## 10. 停止の意味と終了順序

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

## 11. 状態確認・ログの見方

### 11.1 RuntimeとControllerは別の診断を出す

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

### 11.2 policy/clientのログ（ホスト）

```bash
docker ps -a --filter label=com.docker.compose.project=piper-vla-dual \
  --format 'table {{.ID}}\t{{.Names}}\t{{.Status}}'

docker logs --tail 100 <双腕policyのCONTAINER_ID>
docker logs --tail 100 <双腕clientのCONTAINER_ID>
```

`<...>`は一覧で確認した実際のIDへ置き換えます。
policyが終了していても`docker ps -a`からIDを確認できます。

### 11.3 記録先

既定のログrootはホストの`/home/dnp218/piper_dual_logs`、client内では`/logs`です。
双腕Runtimeは設定`recording.enabled=true`ならstart時にセッション記録を開始します。
**双腕Runtimeはdry-runでも記録します。** 単腕の「実機runだけ記録」の条件とは異なります。
stop時に記録を閉じ、次のstartは別セッションになります。
単なるfault時は記録を開いたままにし、stopまたは次startで閉じる構成です。

## 12. よくあるエラーと確認先

| 症状・メッセージ | 原因候補 | 対応 |
|---|---|---|
| `sample_validity_key`のTypeError | 学習時OpenPI拡張が不足 | 第4.1節。環境互換性を先に解決 |
| `ModuleNotFoundError: dnp1_bimanual_registration` | training root未指定、ファイル配置違い | `DUAL_TRAINING_ROOT`をbundleルートへ |
| `model_config.json`やcacheがない | `.py`だけをコピーした | bundle全体を同じ構造でmount |
| `KeyError: state`、`images`、`cam_high` | Runtimeとexportの入力形式が未接続 | 第4.2節の入口変換を実装・検証 |
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

## 13. 実CANを使わない確認について

```bash
roslaunch piper start_vla_dual_piper.launch mock_driver:=true dry_run:=true
```

この指定では実driverと実カメラを起動しません。ただし通常のControllerは公式SDKの純粋FKをimportします。
SDK接続を含め完全に排除した検証には、過去の統合テストと同じくSDK代替・合成画像・Mock Policy・
デバイス/ホストネットワークを渡さない隔離環境が必要です。
Mock launch単体では3画像やpolicyがないため、Runtime start後の推論までは成立しません。

既存のハードウェア不要テスト：

```bash
# ホスト。ROS/SDK/CANをMock化したテスト。
cd /home/dnp218/docker_piper_env
python3 -B double_PiPER/src/test/test_vla_dual_piper_controller.py
python3 -B double_PiPER/src/test/test_vla_piper_controller.py
```

以前の隔離統合テストの成果物は`/tmp/vla_ros_integration/REPORT.md`にあります。
`/tmp`は再起動等で消えるため、永続的な配布物としては扱わないでください。
これらのMock成功は今回の実checkpoint・実FK・実機での動作確認の代わりではありません。

## 14. 毎回の操作の要点

初回の互換性対応・モデル確認・CAN/カメラ準備が完了した後の通常操作は次の4段階です。

1. PiPER DockerでROS masterを準備し、双腕launchを起動する。
2. ホストで`DUAL_TRAINING_ROOT`を指定し、対象bundleで`run_dual.sh`を起動する。
3. PiPER Dockerから`/piper_vla_dual_runtime/start`へ`Assembling a box.`を渡す。
4. `/piper_vla_dual_runtime/stop`で停止する。

| モード | ROS launch | Runtime | VLA由来の実機関節指令 |
|---|---|---|---|
| 推論出力だけ確認 | `dry_run:=true` | `--dry-run` | なし。ControllerへもActionを送らない |
| Controller判定まで確認 | `dry_run:=true` | `--controller-run` | なし。左右目標診断まで |
| 実機推論 | `dry_run:=false auto_enable:=true` | `--controller-run` | あり |

実driverを使う最初の2モードにも初期化CAN通信があります。
完全無通信のMock検証と、実観測を使うdry-runは区別してください。

## 15. このREADMEの根拠ファイル

- [双腕ROS制御の詳細](double_PiPER/src/piper/README_vla_dual.md)
- [双腕launch](double_PiPER/src/piper/launch/start_vla_dual_piper.launch)
- [双腕Controller](double_PiPER/src/piper/scripts/vla_dual_piper_controller.py)
- [単腕安全処理](double_PiPER/src/piper/scripts/vla_piper_controller.py)
- [Runtime起動スクリプト](../piper_vla_runtime/scripts/run_dual.sh)
- [双腕Compose](../piper_vla_runtime/compose.dual.yaml)
- [双腕Runtime設定](../piper_vla_runtime/configs/runtime.dual.yaml)
- [Policyローダー](../piper_vla_runtime/docker/serve_dual_piper_checkpoint.py)
- [配置済みexportの説明](../dnp1_20261015_bimanual_pi05_delta_1000/README.md)
- [配置済みexportのmanifest](../dnp1_20261015_bimanual_pi05_delta_1000/manifest.yaml)
