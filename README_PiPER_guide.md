# PiPER MCAP・VLA 操作ガイド

## Docker環境

Linux、Docker、X11、SocketCAN、USBカメラを想定します。
配置名は `docker_piper_ctrl` とし、個人のホーム名は固定しません。

```bash
# ホスト：初回buildとコンテナ起動
cd "$HOME/docker_piper_ctrl"
./build.sh --build-arg USR_NAME="$(id -un)" \
  --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)"
./run.sh
```

```bash
# PiPERコンテナ内：初回catkinビルドとメッセージ生成
set_sdk -j2
# 以後、新しい端末でsource
sorc_devel
rospack find piper
```

packageは `$HOME/docker_piper_ctrl/double_PiPER/src/piper`。
Pythonは `/usr/bin/python3`。`sudo catkin_make`は使用しません。
SDKはDockerfileの固定commitを基本とします。別の `~/piper_sdk` がある場合、set_sdkはそちらを使います。
Xauthorityまたは限定したローカルユーザーのX11許可を使用します。
run.shはhost network・privilegedを使う実機開発構成です。
カメラは起動時の `/dev/video*` を渡すため、先に接続してください。
データはホストの `piper_dataset/`、個別初期姿勢は `~/.config/piper/` に永続化します。

## CAN設定（ホスト）

```bash
cd "$HOME/docker_piper_ctrl/double_PiPER"
bash find_all_can_port.sh
# 接続実態を確認したUSB bus-infoを環境変数へ指定する
bash can_activate.sh can0 1000000 "${PIPER_CAN0_USB:?USB bus-infoを指定}"
bash can_activate.sh can1 1000000 "${PIPER_CAN1_USB:?USB bus-infoを指定}"
bash can_activate.sh can2 1000000 "${PIPER_CAN2_USB:?USB bus-infoを指定}"
bash can_activate.sh can3 1000000 "${PIPER_CAN3_USB:?USB bus-infoを指定}"
```

同じCANを収集driverとVLA driverで同時に所有させないでください。

## 単腕MCAP収集（PiPERコンテナ内）

Leader can0／Follower can1、front video4／wrist video0の例。
単腕launch既定のCAN対応はLeader can1／Follower can0なので、ここでは明示指定します。

```bash
sorc_devel
roslaunch piper collect_single_piper_mcap.launch \
  can_leader_port:=can0 \
  can_follower_port:=can1 \
  front_camera:=/dev/video4 \
  wrist_camera:=/dev/video0 \
  dataset_root:="$HOME/piper_dataset/single"
```

## 双腕MCAP収集（PiPERコンテナ内）

```bash
sorc_devel
roslaunch piper collect_dual_piper_mcap.launch \
  can_leader_R_port:=can0 \
  can_follower_R_port:=can1 \
  can_leader_L_port:=can2 \
  can_follower_L_port:=can3 \
  front_camera:=/dev/video4 \
  wrist1_camera:=/dev/video0 \
  wrist2_camera:=/dev/video2 \
  dataset_root:="$HOME/piper_dataset/dual"
```

単腕7次元＋EE pose、双腕14次元（右7→左7）。関節rad、gripper m。
画像は単腕2枚、双腕3枚。物理カメラの左右と学習変換の対応は別途確認します。
Leader–Followerはdriver側で独立して動作し、RecorderはSDKを生成しません。

### 共通操作

映像とPiPER-Recorder端末を使い、タスク文字列は端末へ入力します。

| キー | 動作 |
|---|---|
| t | タスク入力。記録中・操作処理中は編集しない |
| Enter / Esc | 確定／取消。空入力は以前のタスクを維持 |
| r | 記録開始 |
| s | 記録停止・保存 |
| x | 記録破棄 |
| q | Recorder終了。記録途中はincompleteとして保持 |

保存・破棄・終了は追従を停止しません。Recorderに追従開始・停止・復旧キーはありません。
ロボット停止はdriverの既存停止経路とハードウェア停止手段を使います。
`auto_enable:=false`もSDK初期化全体の無通信を保証しません。
sで保存結果を確認してq、最後に収集launchを終了します。物理停止・保持の確認とは別です。
MCAP・NPZ・CSV・JPEG・meta.jsonを保存し、欠落・カメラ異常・時計後退ではincomplete処理を維持します。
`preview:=false`は映像だけ省略、`enable_recording:=false`はRecorderを省略します。

## 単腕VLA（PiPERコンテナ内）

```bash
sorc_devel
roslaunch piper start_vla_piper.launch \
  can_port:=can1 \
  front_camera:=/dev/video4 wrist_camera:=/dev/video0 \
  dry_run:=true auto_enable:=false
```

## 双腕VLA（PiPERコンテナ内）

```bash
sorc_devel
roslaunch piper start_vla_dual_piper.launch \
  left_can_port:=can3 right_can_port:=can1 \
  front_camera:=/dev/video4 \
  left_wrist_camera:="${PIPER_LEFT_WRIST:?学習時と同じ物理左カメラを指定}" \
  right_wrist_camera:="${PIPER_RIGHT_WRIST:?学習時と同じ物理右カメラを指定}" \
  dry_run:=true auto_enable:=false
```

これらは実driverがCAN初期化するため、完全な非動作テストではありません。
双腕は `mock_driver:=true dry_run:=true enable_cameras:=false` で機器を使わない構成を選べます。
実機指令を許可する場合はControllerの `dry_run:=false auto_enable:=true` とRuntimeの送信モードをそれぞれ明示します。
SDK・Controller角度制限は既定true。変更する場合だけ双腕launchへ
`start_sdk_joint_limit:=false controller_joint_limit_enabled:=false` を指定します。
max_deltaは `config/vla_dual_motion_limits.yaml` の左右各0.1rad/step、20Hz基準。
FK/Z・gripper・State鮮度・HOLD/FAULT・token/statusの検証は維持します。

### 外部Policy・Runtime（ホスト）

`piper_vla_runtime_2`は別配布でπ0／π0.5の単腕・双腕を扱います。
初回buildとモデル依存はそのREADMEに従います。新Checkpoint形式を使用します。

```bash
cd "$HOME/piper_vla_runtime_2"
# 既存Policy/Runtimeが稼働中なら、この起動を重ねない
bash scripts/run_dual.sh --checkpoint "${PIPER_CHECKPOINT:?双腕Checkpointパッケージ}" --controller-run
# 単腕は scripts/run.sh と単腕Checkpointを使用
```

Runtime既定はdry-runで、selected_actionまで生成しapproved_actionは送りません。
Controller dry-runとは独立です。Controllerのtarget診断を確認する場合は、
Controller dry-run=trueを維持してRuntimeを送信モードにします。
型定義をsource済みのRuntime端末で、通常のStart／Stopを呼びます。

```bash
rosservice call /piper_vla_dual_runtime/start "{prompt: 'Describe the task.'}"
rostopic echo /vla_dual/approved_action
rosservice call /piper_vla_dual_runtime/stop '{}'
# 単腕は /piper_vla_runtime/start と /piper_vla_runtime/stop
```

双腕VLAは左7→右7で、収集保存順の右→左とは異なります。
収集用の画像取得はROS画像配信とは別で、収集launchをVLA用に併用しません。
学習・変換用project_minimalは別環境で、このリポジトリの実行時依存ではありません。

## 初期姿勢復帰（実機動作）

`config/piper_demo_start.yaml` と `config/piper_dual_demo_start.yaml` に自分の機体・作業台で確認した姿勢を設定します。
収集データから自動更新しません。

```bash
# PiPERコンテナ内
# 単腕はこの実行で実機が動く。--dry-runオプションはない。
rosrun piper go_initial_pose.py
# 双腕のみ：姿勢・経路検証（CAN接続はあり得る）
rosrun piper go_dual_initial_pose.py --dry-run
# 双腕を実際に復帰させる場合は --dry-run を外す
# 別設定：双腕は --config /absolute/path/pose.yaml
# 単腕は _config_file:=/absolute/path/pose.yaml
```

このdry-runもSDK経路ではCANへ接続します。機器に接続しない検証にはMockを使います。
復帰は既存の停止・所有権確認・姿勢検証を通し、VLAとは独立して行います。

## 機器を使わない検証

```bash
# PiPERコンテナ内。ROS master不要、SDK・カメラはMockへ置換
sorc_devel
cd "$HOME/docker_piper_ctrl"
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/double_PiPER/src/piper/scripts:$PYTHONPATH" \
  /usr/bin/python3 -m unittest discover -s double_PiPER/src/mock

# 一時workspaceでのcatkinビルド、import、launch展開
PYTHONDONTWRITEBYTECODE=1 \
  /usr/bin/python3 double_PiPER/src/test/test_piper_docker_environment.py
```

環境検証はDISPLAYがあると表示確認も行います。完全なCPU検証ではDISPLAYを渡さず、
network・実機デバイスなしの一時コンテナを使用します。
`src/test/`の校正・実機診断はMockとは別で、一括実行しません。
