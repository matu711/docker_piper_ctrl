# PiPER：環境構築・教示収集・VLA操作マニュアル

## 1. 概要

このリポジトリはROS Noetic上の単腕・双腕PiPER制御、Leader–Follower教示収集、VLA Controller、カメラ配信、初期姿勢復帰を提供します。操作PCはLinux、収集・制御環境はDockerを使用します。

| リポジトリ | 役割 |
|---|---|
| `docker_piper_ctrl`（本書） | CAN・カメラ・ROSドライバー・教示収集・VLAの指令検証と実機送信 |
| `project_minimal` | 収集データのMCAP→LeRobot変換、学習、Checkpointパッケージ出力 |
| `piper_vla_runtime_2` | 学習済みPolicyのロード、WebSocket推論、ROS観測の取得とAction選択 |

単腕・双腕の教示データ収集には共通Recorderを使います。現在のファイル名は **`collect_single_piper_mcap.launch` / `collect_dual_piper_mcap.launch`** です。VLAは`start_vla_piper.launch` / `start_vla_dual_piper.launch`を使い、収集launchと同時に同じCAN・カメラを使用しません。

以下で「ホスト」はDockerを実行するOS、「PiPERコンテナ」は`piper-control-dev`、「Runtimeコンテナ」は推論リポジトリが起動するROSクライアントを指します。制御PCと推論PCを分ける場合の通信設定は11章に示します。コードブロックの環境変数は、そのコマンドを実行する端末内で設定してください。

必要な外部依存はDocker Engine、表示用X11、USB-CAN・UVCカメラです。Policy側は対応GPU・NVIDIAドライバー・NVIDIA Container Toolkit、学習済みCheckpointパッケージが別途必要です。ROSメッセージはこのワークスペースで生成します。モデル・学習データ・認証情報はGitに含みません。

## 2. Docker環境の構築と起動

### 2.1 ホストの準備と取得

Ubuntu系ホストでDockerが利用でき、`docker version`が成功することを確認します。CAN・カメラ確認用コマンド等がなければホストへ導入します。Docker自体の導入は利用組織の手順に従ってください。

**制御PC・ホスト：**

```bash
docker version
sudo apt-get update
sudo apt-get install -y can-utils ethtool iproute2 v4l-utils xauth x11-xserver-utils
read -r -p '共有リポジトリのGit URL: ' PIPER_CTRL_REPOSITORY_URL
git clone "${PIPER_CTRL_REPOSITORY_URL:?}" "$HOME/docker_piper_ctrl"
cd "$HOME/docker_piper_ctrl"
bash build.sh --build-arg USR_NAME="$(id -un)" \
  --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)"
bash run.sh
```

取得済みなら`git clone`を繰り返さず既存の作業ディレクトリを使います。イメージ既定名は`piper-control:noetic`。変更するときはbuildとrunの両方に同じ`PIPER_IMAGE`を設定します。

イメージにはROS Noetic、`/usr/bin/python3`、NumPy、OpenCV、PyYAML、MCAP、PiPER SDK、xterm、Pillow、日本語フォント、UTF-8ロケールが入ります。SDKの上流commitはDockerfileの`PIPER_SDK_REF`で固定されています。

### 2.2 表示・デバイス・保存先

`run.sh`は`DISPLAY`、X11ソケット、Xauthorityを渡します。Cookieを使えない場合のみホストの実行ユーザーへ限定したX11許可を使い、`xhost +`による全面許可は使いません。既定の表示アプリケーションはTerminator、Recorder用には別のxtermが開きます。

| ホスト | PiPERコンテナ |
|---|---|
| `$HOME/docker_piper_ctrl` | `$HOME/docker_piper_ctrl`（ユーザー名をbuild/runで統一した場合） |
| リポジトリ内`piper_dataset/` | `$HOME/piper_dataset/` |
| `$HOME/.config/piper/` | `$HOME/.config/piper/` |
| `/tmp/.X11-unix`、起動時の`/dev/video*` | X11・カメラ接続 |

実際のrunは実行ユーザー名を用いてマウント先を構成します。`USR_NAME`を変更する場合はbuild/runで揃えてください。ソースはイメージへCOPYせずbind mountします。runは`--net=host --ipc=host --privileged`を使うため、信頼できるホストとソースで実行してください。USBカメラを後から接続した場合はデバイス公開状態を確認します。

### 2.3 SDK・catkinと端末の再接続

**初回：PiPERコンテナ内：**

```bash
set_sdk
sorc_devel
command -v python3
/usr/bin/python3 -c 'import rospy, numpy, cv2, yaml, PIL, mcap, mcap_ros1, piper_sdk; print("imports OK")'
rospack find piper
```

`set_sdk`はSDKの準備と`catkin_make --force-cmake -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2`を行います。`/opt/piper_sdk`の固定版が利用可能ならそれを使いますが、`~/piper_sdk`がある場合はそちらが優先されます。SDK版を変更する前にこの配置を確認してください。

正しいパッケージは`$HOME/docker_piper_ctrl/double_PiPER/src/piper`です。`sorc_devel`はROSとこのワークスペースを読み込み、古いoverlayやpyenvのshebangを検出すると拒否します。`set_sdk`は移転前のCMake生成物を必要に応じてバックアップし再生成します。`sudo catkin_make`、ワークスペース全体のchown、build/develの無条件削除は行いません。

**ホスト・別端末から再接続：**

```bash
docker exec -it --user "$(id -un)" piper-control-dev bash
```

**再接続後・PiPERコンテナ：**

```bash
sorc_devel
printf '%s\n' "$PIPER_WORKSPACE" "$ROS_PACKAGE_PATH"
rospack find piper
```

`set_sdk`/`sorc_devel`自体はロボットノードを起動しません。コンテナ終了で`--rm`によりコンテナは消えますが、ソース・データ・上記設定マウントはホストに残ります。

## 3. CAN接続・確認

### 3.1 物理接続とbus-infoの特定

**ホスト：**

```bash
lsusb
ip -br link show type can
sudo ethtool -i can0 | grep bus
```

表示された`bus-info:`の値がUSBポート番号です。例えば`bus-info: 3-11:1.0`なら、次の有効化コマンドへ`3-11:1.0`を指定します。他のCANも調べる場合は`can0`を`can1`、`can2`、`can3`など実際のインターフェース名へ置き換えます。

USBポート・ケーブル・アームのラベルを照合し、Leader/Follower、左/右とUSBポート番号の対応を記録します。CAN名やUSBポート番号はPC・接続口によって変わるため、実際の表示を確認してください。

### 3.2 1 Mbpsで有効化

ロボットの安全状態を確認し、対象を一台ずつ特定して実行します。

**ホスト：**

```bash
cd "$HOME/docker_piper_ctrl/double_PiPER"
bash can_activate.sh can0 1000000 "USBポート番号"
ip -details -statistics link show can0
```

`"USBポート番号"`は3.1節で確認した`bus-info:`の値へ置き換えます。例えば`3-11:1.0`だった場合は次のように指定します。

```bash
bash can_activate.sh can0 1000000 "3-11:1.0"
```

`can0`は割り当てたいCAN名です。単腕はLeader/Followerの2台、双腕は右Leader/右Follower/左Leader/左Followerの4台について、CAN名とUSBポート番号を変更して繰り返します。既存インターフェース名と衝突するrenameを避け、複数接続時はUSBポート番号を省略しません。

有効化後は`UP`、`bitrate 1000000`、エラーカウンターを確認します。インターフェースがなければUSB接続、ホストの対応ドライバー、電源を先に確認します。

CAN名はlaunch引数で指定します。収集はLeaderも必要、VLAはFollowerのみを使用します。ホストnetworkを共有するPiPERコンテナでも同じCAN名を確認できます。

## 4. カメラ接続・確認

**ホスト（またはデバイスを公開したPiPERコンテナ）：**

```bash
v4l2-ctl --list-devices
ls -l /dev/video* /dev/v4l/by-id/
read -r -p '調べる画像キャプチャデバイス: ' CAMERA_DEVICE
v4l2-ctl --device="${CAMERA_DEVICE:?}" --list-formats-ext
ls -l "${CAMERA_DEVICE:?}"
```

一台のUSBカメラから複数のvideoノードが出ることがあります。画像キャプチャ可能なノードを選びます。`/dev/v4l/by-id/`が利用できれば機体との対応を安定させやすくなります。ホストで存在してもコンテナの`ls -l`で見えなければデバイス公開を確認します。占有は`fuser /dev/videoN`（利用可能な環境）等で調べ、収集・VLA・別のプレビューを重複起動しません。

| 用途 | 要求する既定フォーマット | 要求解像度・取得FPS |
|---|---|---|
| 単腕収集front/wrist | MJPG | 各640×480、30 FPS |
| 双腕収集front / wrist1 / wrist2 | MJPG | front 1280×720、wrist各640×480、30 FPS |
| 単腕VLA front / wrist | front YUYV、wrist MJPG | 各640×480、30 FPS |
| 双腕VLA front / left_wrist / right_wrist | 各MJPG | front 1280×720、wrist各640×480、30 FPS |

収集記録とVLA配信は既定20 Hzです。要求値とカメラの実際の取得値は異なる場合があるため、実測FPS・同期エラーも確認してください。単腕VLAのfrontフォーマットは収集と異なり、非対応なら`front_fourcc`等を実機の対応値に合わせます。

**左右対応は映像で確認します。** ロボットを動かさず、手や目印を片方のカメラ前に置いて画像名と実物を照合します。収集の`wrist1/wrist2`という名前自体は左/右を保証しません。データセットに記録した対応を、学習変換と推論の`left_wrist/right_wrist`へ一貫して渡します。video番号から左右を推定して入れ替えないでください。

## 5. 単腕教示データ収集

### 5.1 起動

起動直後から既存ドライバーの初期化・enable・Leader–Follower制御が行われます。`auto_enable=false`だけで初期化中のenableをすべて抑止する実装ではありません。通電・干渉・非常停止を確認してから起動します。

**PiPERコンテナ・`sorc_devel`実行後：**

```bash
read -r -p 'Leaderの確認済みCAN名: ' CAN_LEADER
read -r -p 'Followerの確認済みCAN名: ' CAN_FOLLOWER
read -r -p 'frontの確認済み画像デバイス: ' CAM_FRONT
read -r -p 'wristの確認済み画像デバイス: ' CAM_WRIST
roslaunch piper collect_single_piper_mcap.launch \
  can_leader_port:="${CAN_LEADER:?}" \
  can_follower_port:="${CAN_FOLLOWER:?}" \
  front_camera:="${CAM_FRONT:?}" \
  wrist_camera:="${CAM_WRIST:?}" \
  dataset_root:="$HOME/piper_dataset/single"
```

CAN名・カメラ番号を直接指定する場合の例です。Leaderが`can0`、Followerが`can1`、frontが`/dev/video4`、wristが`/dev/video0`の構成で使います。実際の接続が異なる場合は各引数を変更してください。

```bash
roslaunch piper collect_single_piper_mcap.launch \
  can_leader_port:=can0 \
  can_follower_port:=can1 \
  front_camera:=/dev/video4 \
  wrist_camera:=/dev/video0 \
  dataset_root:=$HOME/piper_dataset/single
```

Leaderを動かすとFollowerが追従する構成です。Recorderを操作して追従を開始する方式ではありません。ロボットの状態を確認した後、Recorderの映像と端末を使って記録します。

### 5.2 単腕・双腕共通のRecorder操作

| キー | 動作 |
|---|---|
| `t` | Recorder端末でタスク文字列の入力を開始。記録中・記録操作中は変更不可 |
| Enter / Esc / Backspace | 入力確定 / 取消 / 1文字削除。空入力では以前のタスクを維持 |
| `r` | 新しいエピソードの記録開始。新鮮なState/Action・必要な画像を確認 |
| `s` | 記録停止・保存。追従制御は停止しない |
| `x` | 記録破棄。追従制御は停止しない |
| `q` | Recorder終了。記録途中ならincompleteとして残す |

コマンドキーは映像ウィンドウ・Recorder端末の両方から送れます。タスク文字列は**端末**へUTF-8で入力します。入力中の`r/s/x/q`等は文字として扱い、映像側のキーを入力文字に混ぜません。`r`時点のtaskをエピソード`meta.json`へ固定します。入力中もプレビューは継続します。日本語IME自体はホスト側で用意してください。

`c`、Space、`p`、`h`による追従・姿勢操作はありません。収集用Recorderは制御サービスを呼びません。GUIには記録状態・件数・フレーム数・設定FPS・カメラFPS・task・保存結果・エラーを表示し、ドライバーが提供しない追従状態は推測表示しません。

正常終了は「`s`で保存完了確認 → 必要なら`q` → アームの安全を確保 → launch端末のCtrl-C」の順です。Recorderを閉じただけではLeader–Followerノードは終了しません。異常時はハードウェアの停止手段を優先し、安全確認後に原因を解消して起動し直します。

### 5.3 引数と既定値

以下の共通引数は双腕でも同じです。秒・Hz・JPEG品質の単位に注意してください。

| 引数 | 既定値 |
|---|---|
| `enable_recording` | `true` |
| `recorder_launch_prefix` | `xterm -T PiPER-Recorder -e` |
| `record_fps` | `20` |
| `sync_tolerance` | `0.03` |
| `sync_wait` | `0.025` |
| `max_data_gap` | `1.0` |
| `enable_cameras` | `true` |
| `preview` | `true` |
| `camera_fps` | `30` |
| `camera_fourcc` | `MJPG` |
| `camera_backend` | `v4l2` |
| `camera_open_timeout` | `5.0` |
| `camera_read_timeout` | `2.0` |
| `sync_delay` | `0.2` |
| `camera_target_tolerance` | `0.10` |
| `camera_sync_tolerance` | `0.03` |
| `camera_sync_wait` | `0.10` |
| `max_image_age` | `0.5` |
| `jpeg_quality` | `95` |
| `auto_enable` | `true` |
| `gripper_val_mutiple` | `1` |


`enable_recording=false`はRecorderのみ省略します。`preview=false`は映像表示だけを省略し、端末入力は残ります。`recorder_launch_prefix:=''`でxtermを省略できますが、制御端末がないroslaunch経由では`t`文字入力を利用できません。通常は既定のxtermを使います。

単腕固有引数：

| 引数 | 既定値 |
|---|---|
| `initial_pose_mode_timeout` | `3.0` |
| `return_leader_with_follower` | `false` |
| `initial_pose_gripper_tolerance` | `0.005` |
| `initial_pose_joint_tolerance` | `0.035` |
| `initial_pose_timeout` | `15.0` |
| `initial_pose_speed` | `20` |
| `initial_pose_file` | `$(env HOME)/.config/piper/initial_pose.yaml` |
| `robot_type` | `piper_single_arm` |
| `ee_euler_topic` | `/follower/end_pose_euler` |
| `ee_topic` | `/follower/end_pose` |
| `action_topic` | `/follower/applied_joint_cmd` |
| `follower_topic` | `/follower/current_joint_obs` |
| `leader_topic` | `/leader/joint_grip_pose` |
| `dataset_root` | `$(env HOME)/piper_dataset` |
| `repo_id` | `local/piper_pickup` |
| `task` | `Align the bags, grasp them, and lift them up.` |
| `front_camera` | `/dev/video6` |
| `wrist_camera` | `/dev/video0` |
| `front_width` | `640` |
| `front_height` | `480` |
| `wrist_width` | `640` |
| `wrist_height` | `480` |
| `can_follower_port` | `can0` |
| `can_leader_port` | `can1` |


`initial_pose_*`はFollowerの保存姿勢復帰機能の設定です。収集中の初期姿勢を明示保存・復帰する場合だけ、下記サービスを使います。復帰は実際にアームを動かします。現在の姿勢・保存ファイル・経路を先に確認してください。

```bash
rosservice call /follower/save_initial_pose "{}"
# 保存済み姿勢へ動かすときのみ実行
rosservice call /follower/go_initial_pose "{}"
# 姿勢復帰で遮断されたLeader入力を再開するときのみ実行
rosservice call /follower/resume_leader_control "{}"
```

これらは`std_srvs/Trigger`です。`/follower/set_leader_control_block`は`std_srvs/SetBool`で、新しいLeader指令の遮断に使います。command faultが立っている場合は単純なresumeで解除できません。安全確認・原因解消を優先します。

## 6. 双腕教示データ収集

### 6.1 起動と対応付け

**PiPERコンテナ・`sorc_devel`実行後：**

```bash
read -r -p '右LeaderのCAN名: ' CAN_LEADER_R
read -r -p '右FollowerのCAN名: ' CAN_FOLLOWER_R
read -r -p '左LeaderのCAN名: ' CAN_LEADER_L
read -r -p '左FollowerのCAN名: ' CAN_FOLLOWER_L
read -r -p 'front画像デバイス: ' CAM_FRONT
read -r -p '収集wrist1画像デバイス: ' CAM_WRIST1
read -r -p '収集wrist2画像デバイス: ' CAM_WRIST2
roslaunch piper collect_dual_piper_mcap.launch \
  can_leader_R_port:="${CAN_LEADER_R:?}" \
  can_follower_R_port:="${CAN_FOLLOWER_R:?}" \
  can_leader_L_port:="${CAN_LEADER_L:?}" \
  can_follower_L_port:="${CAN_FOLLOWER_L:?}" \
  front_camera:="${CAM_FRONT:?}" \
  wrist1_camera:="${CAM_WRIST1:?}" \
  wrist2_camera:="${CAM_WRIST2:?}" \
  dataset_root:="$HOME/piper_dataset/dual"
```

CAN名・カメラ番号を直接指定する場合の例です。右Leader/Followerが`can0`/`can1`、左Leader/Followerが`can2`/`can3`、frontが`/dev/video4`、wrist1が`/dev/video2`、wrist2が`/dev/video0`の構成で使います。実際の接続が異なる場合は各引数を変更してください。

```bash
roslaunch piper collect_dual_piper_mcap.launch \
  can_leader_R_port:=can0 \
  can_follower_R_port:=can1 \
  can_leader_L_port:=can2 \
  can_follower_L_port:=can3 \
  front_camera:=/dev/video4 \
  wrist1_camera:=/dev/video2 \
  wrist2_camera:=/dev/video0 \
  dataset_root:=$HOME/piper_dataset/dual
```

4アームの初期化後、それぞれのLeaderとFollowerが対応して動きます。双腕FollowerはLeader EE受信を待って追従します。操作・保存・終了は5.2節と共通です。左右やカメラの対応をtaskだけで表現せず、機体ラベル・データ管理情報として残してください。

| 役割 | ROS State / Action |
|---|---|
| 右Follower | `/followerR/current_joint_obs` / `/followerR/applied_joint_cmd` |
| 左Follower | `/followerL/current_joint_obs` / `/followerL/applied_joint_cmd` |
| 単腕Follower | `/follower/current_joint_obs` / `/follower/applied_joint_cmd` |

いずれも`JointState`、関節rad・グリッパーmです。単腕は7次元、双腕保存配列は**右7→左7**です。双腕VLAの**左7→右7**とは異なります。Actionはドライバー側の適用指令であり、物理的な追従成功の証明ではありません。

双腕収集ドライバーにはVLA用の`/followerL/stop_srv`等はありません。存在しない停止サービスを呼ぶ代わりに、装置側の停止手段とlaunch終了を使います。双腕の初期姿勢復帰を行う場合は収集を終了し、CAN所有者の競合を解消してから12.5節の専用手順を使ってください。

### 6.2 双腕固有引数

共通引数は5.3節参照。

| 引数 | 既定値 |
|---|---|
| `dataset_root` | `$(env HOME)/piper_dataset/dual` |
| `repo_id` | `local/piper_dual_state` |
| `task` | `bimanual demonstration` |
| `front_camera` | `/dev/video4` |
| `wrist1_camera` | `/dev/video2` |
| `wrist2_camera` | `/dev/video0` |
| `front_width` | `1280` |
| `front_height` | `720` |
| `wrist1_width` | `640` |
| `wrist1_height` | `480` |
| `wrist2_width` | `640` |
| `wrist2_height` | `480` |
| `can_follower_R_port` | `can1` |
| `can_leader_R_port` | `can0` |
| `can_follower_L_port` | `can3` |
| `can_leader_L_port` | `can2` |
| `video_R_port` | `0` |
| `video_L_port` | `2` |


`video_R_port` / `video_L_port`は互換用に残る引数です。収集カメラの選択は`front_camera` / `wrist1_camera` / `wrist2_camera`で行います。

### 6.3 同期・制御の仕様

既定20 HzのROS時刻グリッドに対し、State/Actionは0.03秒の許容差、画像はtargetから0.10秒・カメラ相互0.03秒を使います。画像の取得時刻はV4L2由来を優先し、利用できなければread midpointを使います。許容範囲内の画像再利用は認められ、CSV/NPZに時刻・再利用情報を保存します。時計後退・長い欠落・同期不成立ではincompleteになります。

記録処理は単腕・双腕で共通です。単腕のEE観測も同期・保存します。制御は単腕の方向Clip・非有限値拒否・command faultと、双腕の既存Leader EE待機・gripper Clamp等をそれぞれ維持しています。Recorderの保存完了はロボット停止完了を意味しません。

## 7. 収集データの確認

### 7.1 ディレクトリと正常保存

既定保存先は単腕`$HOME/piper_dataset`、双腕`$HOME/piper_dataset/dual`です。本書の単腕例は区別のため`$HOME/piper_dataset/single`を指定しています。標準runでは、これらはホストの`docker_piper_ctrl/piper_dataset/`以下に保存されます。別のコンテナ内パスを指定する場合は永続mountがあることを確認します。

```text
<dataset_root>/
  dataset_meta.json
  <episode>/
    meta.json
    data.npz
    frames.csv
    *.mcap
    front/000000.jpg ...
    wrist/000000.jpg ...           # 単腕
    wrist1/ ... wrist2/ ...        # 双腕（wristの代わり）
```

MCAPのbasenameは記録開始時のエピソード識別子です。特に双腕は保存時にフォルダを改名するため、最終フォルダ名とMCAP名が一致するとは限りません。正常保存フォルダ名はAsia/Tokyoの`YYYYMMDD_HHMMSS`、衝突時は接尾辞が付きます。中断時は`.tmp_`フォルダ、partial CSV、incomplete MCAP等が残る場合があり、通常の学習入力へ混ぜません。ルートmetadataが既存でも、各エピソードの`meta.json`のtask・形式・完了状態を確認します。

| 形式 | State / Action | その他 |
|---|---|---|
| 単腕・カメラあり | float32、N×7 | `observation_ee`はN×8：XYZ m、quaternion xyzw、gripper m |
| 双腕・カメラあり | float32、N×14、右→左 | `piper_dual_state_mcap_v2` |

単腕のformatは`piper_raw_fixed_rate_synced_v6_ee_pose`。時刻列はfloat64です。単腕Writerの誤差はfloat32、双腕Writerの時刻誤差・sync spanはfloat64です。画像なし収集も可能ですが、画像ありの学習形式とは区別します。MCAPには同期行のほか、採用されなかった元のROSサンプルも入るため、MCAPメッセージ数とNPZ行数は一致しません。

### 7.2 件数・容量・内容

**PiPERコンテナ（保存だけを読む）：**

```bash
read -r -p '確認するdataset_rootの絶対パス: ' DATASET_ROOT
export DATASET_ROOT
find "${DATASET_ROOT:?}" -type f -name '*.mcap' | wc -l
du -sh "${DATASET_ROOT:?}"
/usr/bin/python3 - <<'PYCODE'
import json, os
from pathlib import Path
import numpy as np
root = Path(os.environ['DATASET_ROOT'])
for meta_path in sorted(root.glob('*/meta.json')):
    meta = json.loads(meta_path.read_text())
    episode = meta_path.parent
    print(episode.name, 'status=', meta.get('status'),
          'successful=', meta.get('successful'), 'task=', meta.get('task'))
    if (episode / 'data.npz').is_file():
        with np.load(episode / 'data.npz', allow_pickle=False) as data:
            print({key: (data[key].shape, str(data[key].dtype)) for key in data.files})
PYCODE
```

MCAP内のtopic別件数を確認するには、対象を一つ指定して読み込みます。

```bash
read -r -p '確認するMCAPの絶対パス: ' MCAP_FILE
export MCAP_FILE
/usr/bin/python3 - <<'PYCODE'
import os
from collections import Counter
from mcap.reader import make_reader
with open(os.environ['MCAP_FILE'], 'rb') as stream:
    counts = Counter(channel.topic for _, channel, _ in make_reader(stream).iter_messages())
print(dict(counts))
PYCODE
```

`successful=true`（双腕は併せて`status=saved`）、NPZ/CSV/MCAPの存在、N>0、次元、画像ファイルとCSVパス、時刻の連続性、左右対応を確認します。MCAPの存在だけで正常保存とは判定しません。

## 8. 学習用PCへのデータ転送

記録・保存が終了しているデータセットを転送します。SSH鍵等は各組織の手順で設定し、認証情報をリポジトリへ入れません。

**収集PC・ホスト：**

```bash
read -r -p '学習PCのSSH接続先（user@host）: ' TRAIN_SSH
read -r -p '学習PCの転送先・絶対パス（空白なし）: ' TRAIN_DATA_DIR
ssh "${TRAIN_SSH:?}" "mkdir -p -- '${TRAIN_DATA_DIR:?}'"
scp -r "$HOME/docker_piper_ctrl/piper_dataset/dual" \
  "${TRAIN_SSH:?}:${TRAIN_DATA_DIR:?}/"
ssh "${TRAIN_SSH:?}" "du -sh -- '${TRAIN_DATA_DIR:?}/dual'"
```

接続先とパスは実環境で入力し、パスには引用符等のシェル特殊文字を含めません。単腕なら`dual`を`single`へ置き換えます。一般形は`scp -r <ローカルディレクトリ> <user@host>:<リモート親ディレクトリ>/`です。

両PCにrsyncがある場合は差分転送・転送予定確認もできます。末尾`/`はディレクトリの中身を意味します。`--delete`は付けません。

```bash
rsync -avhn "$HOME/docker_piper_ctrl/piper_dataset/dual/" \
  "${TRAIN_SSH:?}:${TRAIN_DATA_DIR:?}/dual/"
rsync -avh --partial "$HOME/docker_piper_ctrl/piper_dataset/dual/" \
  "${TRAIN_SSH:?}:${TRAIN_DATA_DIR:?}/dual/"
```

転送後は7章と同じ方法でファイル数・metadata・NPZ・MCAPを学習PCでも確認します。MCAP→LeRobot変換と学習・Checkpoint書き出しは`project_minimal`の現行手順に従います。変換時に、双腕収集の右→左配列と学習モデルの左右契約、wrist1/2の物理対応を明示してください。

## 9. 単腕VLA Controller起動

### 9.1 検証用起動

収集launchを終了し、同じCAN・カメラを使うノードがないことを確認します。以下も実ドライバーのCAN初期化を伴います。

**PiPERコンテナ：**

```bash
sorc_devel
read -r -p '推論するFollowerのCAN名: ' CAN_FOLLOWER
read -r -p 'front画像デバイス: ' CAM_FRONT
read -r -p 'wrist画像デバイス: ' CAM_WRIST
roslaunch piper start_vla_piper.launch \
  can_port:="${CAN_FOLLOWER:?}" \
  front_camera:="${CAM_FRONT:?}" wrist_camera:="${CAM_WRIST:?}" \
  dry_run:=true auto_enable:=false
```

Controller dry-runは検証後のdriver指令送信を止めます。単腕launchは`auto_enable`をdry-runと独立にdriverへ渡すため、検証例では必ずfalseを明示します。それでもCAN無通信を意味しません。実機運転への切替は12章の条件を満たしてから行います。

### 9.2 主な引数・通信契約

| 引数 | 既定値・用途 |
|---|---|
| `can_port`, `auto_enable` | `can0`, `true` |
| `dry_run`, `action_type` | `true`, `joint_absolute` |
| `z_enabled`, `z_min`, `dh_is_offset` | `true`, `0.01` m, `1` |
| `joint_saturation_margin_rad` | `0.1` rad、SDK範囲外目標をClampできる許容幅 |
| `gripper_closed_width_m`, `gripper_open_width_m` | `0.0`, `0.10` m |
| `gripper_saturation_margin_m`, `gripper_max_width_m` | `0.01`, `0.10` m |
| `gripper_action_type`, `ee_orientation_representation` | `physical_absolute`, `euler_rpy_rad` |
| `enable_cameras`, `front_camera`, `wrist_camera` | `true`, `/dev/video4`, `/dev/video6` |
| `camera_width`, `camera_height` | `640`, `480` |
| `front_fourcc`, `wrist_fourcc` | `YUYV`, `MJPG` |
| `front_capture_fps`, `wrist_capture_fps`, `camera_publish_fps` | `30`, `30`, `20` |
| `camera_jpeg_quality`, `gripper_val_mutiple` | `90`, `1` |

現在のCheckpointパッケージ方式は7Dのjoint Actionを使います。単腕ControllerにはEE/PBL用分岐もありますが、jointモデルをその形式として起動しません。

| topic / service | 型・用途 |
|---|---|
| `/joint_states_single` | `sensor_msgs/JointState`、Follower実測7D |
| `/end_pose`, `/end_pose_euler` | EE観測 |
| `/camera/front/compressed`, `/camera/wrist/compressed` | `sensor_msgs/CompressedImage` |
| `/vla/raw_action`, `/vla/selected_action`, `/vla/approved_action` | `std_msgs/Float64MultiArray`、Runtimeの各段階 |
| `/vla/hold`, `/vla/action_type` | `std_msgs/Bool`, `std_msgs/String` |
| `/vla/target_joint_command` | `sensor_msgs/JointState`、Controller検証後の診断目標 |
| `/safety/state`, `/safety/reason` | `std_msgs/String`、HOLD・拒否理由等 |
| `/stop_srv` | `std_srvs/Trigger`、低レベルdriverの停止要求 |

起動確認は`rosnode list`、`rostopic hz /joint_states_single`、`rostopic echo -n 1 /safety/state`を使います。推論開始・停止はRuntimeのサービスで行います（12章）。単腕には双腕用の`controller_start`・session token・controller_statusはありません。

## 10. 双腕VLA Controller起動

### 10.1 検証用起動

**PiPERコンテナ：**

```bash
sorc_devel
read -r -p '左FollowerのCAN名: ' CAN_LEFT
read -r -p '右FollowerのCAN名: ' CAN_RIGHT
read -r -p 'front画像デバイス: ' CAM_FRONT
read -r -p '物理的な左wrist画像デバイス: ' CAM_LEFT_WRIST
read -r -p '物理的な右wrist画像デバイス: ' CAM_RIGHT_WRIST
roslaunch piper start_vla_dual_piper.launch \
  left_can_port:="${CAN_LEFT:?}" right_can_port:="${CAN_RIGHT:?}" \
  front_camera:="${CAM_FRONT:?}" \
  left_wrist_camera:="${CAM_LEFT_WRIST:?}" \
  right_wrist_camera:="${CAM_RIGHT_WRIST:?}" \
  dry_run:=true auto_enable:=false
```

双腕ではdriverのauto_enableを`auto_enable and not dry_run`で渡します。ただし実driverのポート接続・Mode指令は初期化時に行われます。実CANを使わないCPU確認には、別の空のROS環境で`mock_driver:=true dry_run:=true enable_cameras:=false`を追加できます。カメラなしなので、そのままPolicyのライブ観測には使えません。

### 10.2 主な引数と制限

| 引数 | 既定値・用途 |
|---|---|
| `left_can_port`, `right_can_port` | `can3`, `can1`（必ず実機の対応へ変更） |
| `dry_run`, `auto_enable`, `mock_driver` | `true`, `true`, `false` |
| `start_sdk_joint_limit` | `true`、左右SDKの関節角度Clamp |
| `controller_joint_limit_enabled` | `true`、Controllerの角度Clamp・実測範囲検証 |
| `motion_limits_file` | `$(find piper)/config/vla_dual_motion_limits.yaml` |
| `max_delta_left`, `max_delta_right` | 空文字なら上記YAML。左右各J1→J6の正の有限6値 |
| YAML内max_delta | 左右とも`[0.1, 0.1, 0.1, 0.1, 0.1, 0.1]` rad/20 Hz step |
| `state_timeout_s` | `0.5`秒、State・CAN受信進行の鮮度 |
| `action_type`, `z_enabled`, `z_min`, `dh_is_offset` | `joint_absolute`, `true`, `0.01` m, `1` |
| `joint_saturation_margin_rad` | `0.1` rad |
| `gripper_closed_width_m`, `gripper_open_width_m` | `0.0`, `0.10` m |
| `gripper_saturation_margin_m`, `gripper_max_width_m`, `gripper_val_mutiple` | `0.01` m, `0.10` m, `1` |
| `enable_cameras` | `not mock_driver` |
| `front_camera`, `left_wrist_camera`, `right_wrist_camera` | `/dev/video4`, `/dev/video2`, `/dev/video0` |
| `front_width`, `front_height`, `camera_width`, `camera_height` | `1280`, `720`, `640`, `480` |
| `front_fourcc`, `left_wrist_fourcc`, `right_wrist_fourcc` | すべて`MJPG` |
| `front_capture_fps`, `left_wrist_capture_fps`, `right_wrist_capture_fps` | すべて`30` |
| `camera_publish_fps`, `camera_jpeg_quality` | `20`, `90` |

max_deltaは`measured + clip(target - measured, -limit, limit)`です。前回目標からの差ではなく新鮮な実測Stateを基準とし、gripperは対象外です。0.1 rad/stepは現在の設定値であり、実機の安全性・速度・加速度・衝突回避を保証する認証値ではありません。別ファイルは`motion_limits_file:=/absolute/path/limits.yaml`、一時上書きは`max_delta_left:='[確認済みの6値]'`等のYAML配列を使います（この文字列は雛形であり、そのまま実行しません）。不正長・0以下・NaN/Inf等は拒否します。

SDKの静的角度範囲は左右共通で、J1 ±2.6179、J2 0〜3.14、J3 -2.967〜0、J4 ±1.745、J5 ±1.22、J6 ±2.09439 radです。実機firmwareの範囲を読み出した値ではありません。Controllerのmarginは目標をこの範囲へClampできる外側の許容幅で、範囲の拡張ではありません。SDK内のgripper静的範囲0〜0.07 mとdriver/Controllerの0〜0.10 mも別レイヤーであり、実機が0.10 mまで開けるとは限りません。

通常は両角度制限をtrueのまま使用します。`start_sdk_joint_limit:=false`は左右SDKの角度Clampを止め、`controller_joint_limit_enabled:=false`はControllerの角度Clamp・実測角度範囲検証を止めます。これは危険を伴う明示的な切替で、通常起動例には含めません。falseでもmax_delta、有限値・次元、FK/Z、gripper、State鮮度、HOLD/FAULT、token/status検査は残ります。これらは機械限界やfirmware保護の代替ではありません。

### 10.3 topic・サービスとdry-run

| topic / service | 型・用途 |
|---|---|
| `/followerL/joint_states_single`, `/followerR/joint_states_single` | `JointState`、左右各7D実測 |
| `/followerL/feedback_health`, `/followerR/feedback_health` | `String`、CAN受信進行・健全性 |
| `/vla/dual_joint_states` | `JointState`、**左7→右7** |
| `/camera/front/compressed`, `/camera/left_wrist/compressed`, `/camera/right_wrist/compressed` | `CompressedImage` |
| `/vla_dual/raw_action`, `/vla_dual/selected_action`, `/vla_dual/approved_action` | `Float64MultiArray`、14D左→右 |
| `/vla_dual/controller_status` | `String` JSON、20 Hz既定、HOLD/RUNNING/FAULT・token・時刻 |
| `/vla_dual/controller_fault` | `String` |
| `/vla_dual/left/target_joint_command`, `/vla_dual/right/target_joint_command` | `JointState`、検証後の左右目標 |
| `/vla_dual/safety_state`, `/vla_dual/safety_reason` | `String`、Runtimeの状態・理由 |
| `/vla_dual/controller_start` | `std_srvs/Trigger`、Runtimeが内部で使用 |
| `/vla_dual_piper_controller/stop` | `std_srvs/Trigger`、Controller HOLD要求 |
| `/vla_dual/initial_pose_lock` | `std_srvs/SetBool`、姿勢復帰スクリプト用 |
| `/followerL/stop_srv`, `/followerR/stop_srv` | `std_srvs/Trigger`、各driver停止要求 |

Runtime dry-runとController dry-runは別です。**Runtime dry-run=trueではraw/selectedのみ出てapprovedは送られません。Runtime=false・Controller=trueなら、token連携と安全判定に成功したActionの診断目標を確認でき、driverへの目標送信は行いません。** 初回chunk取得後にRuntimeがController Start→ACK→新鮮なRUNNING status/token照合を行います。手動Start、HOLD解除、tokenの偽装でこの手順を迂回しません。

## 11. 推論PCでのPolicyサーバー起動

### 11.1 リポジトリ・Checkpoint・イメージ

別途共有された`piper_vla_runtime_2`を推論PCへ配置します。本章は同リポジトリの`scripts/run.sh`、`run_dual.sh`、`piper_vla_common/checkpoint_package.py`のCheckpointパッケージ方式に対応します。

| モデル | 学習方式 | ロボット契約 |
|---|---|---|
| π0 / π0.5 | Full / LoRA | 単腕7D / 双腕14D（左→右） |

モデル種別・正規化・Action変換はパッケージmetadataで検証されます。上記組合せが設定として対応することと、個別Checkpointの実機検証は別です。単腕はLibero入力変換、双腕はAlohaInputs(False)を使うjoint契約であり、任意のEEモデルを自動変換する機能ではありません。

```text
<checkpoint-package>/
  metadata.json
  training.yaml
  tuning_norm_config.json
  norm_stats.json
  MANIFEST.sha256.json
  checkpoint/<step>/params/ ...
```

`--checkpoint`にはこのパッケージのルートを指定します。`params/`だけや旧Bundleを指定しません。欠けたmetadataや異なる入出力契約を推測で補いません。現行の双腕π0.5 ROSプロファイルはjoint delta学習契約を要求し、非対応形式は通常Runtime起動を拒否してofflineのみを案内します。モデル名だけで適合と判定しないでください。

**推論PC・ホスト：初回のみ：**

```bash
read -r -p '推論リポジトリの絶対パス: ' VLA_RUNTIME_DIR
cd "${VLA_RUNTIME_DIR:?}"
python3 -c 'import yaml'
nvidia-smi
bash scripts/build.sh
```

build前に、推論リポジトリの`vendor/openpi_pbl5/`と`assets/paligemma_tokenizer.model`が実体として取得済みであることを確認します。後者はDockerfileのCOPY対象でもあり、欠けたままではbuildできません。取得・配布条件は推論リポジトリの手順に従ってください。起動時も同ファイルのchecksumを検証し、外部パスを使う場合はホストで`PIPER_TOKENIZER_PATH`を設定します。自動ダウンロードは行われません。

ホストのCLIはPython3とPyYAMLを使用します。`import yaml`に失敗するUbuntuホストでは`sudo apt-get install python3-yaml`等で用意します。buildはPolicyとROS clientの共通イメージを作ります。モデルを切り替えるたびの再buildは不要です。GPU推論依存はPolicyイメージ内へ、ROSメッセージ・クライアント依存はclientイメージ内へ入ります。

### 11.2 ROS接続先

同一PCでROSを共有する場合の既定は`ROS_MASTER_URI=http://127.0.0.1:11311`、`ROS_IP=127.0.0.1`です。**別PCでは両方の端末に正しい値を設定します。** 以下のIPは入力値であり、特定PCの固定値ではありません。時計同期も必要です。

**制御PC・PiPERコンテナ：Controller launch前：**

```bash
read -r -p '制御PCの推論PCから到達可能なIP: ' CONTROL_IP
export ROS_MASTER_URI="http://${CONTROL_IP:?}:11311"
export ROS_IP="${CONTROL_IP:?}"
```

**推論PC・ホスト：run前：**

```bash
read -r -p '制御PCのROS Master IP: ' CONTROL_IP
read -r -p '推論PCの制御PCから到達可能なIP: ' INFERENCE_IP
export ROS_MASTER_URI="http://${CONTROL_IP:?}:11311"
export ROS_IP="${INFERENCE_IP:?}"
```

これらはRuntime Composeへ渡されます。ROS1は11311番以外のノード間TCP通信も使います。双方向の名前解決・到達性・Firewallを確認し、不特定ネットワークへROSを公開しません。PolicyとRuntimeは推論PC内で通信し、PiPER ControllerがPolicyへ直接WebSocket接続する構成ではありません。

### 11.3 通常サーバーとoffline検証

**推論PC・ホスト、推論リポジトリ内：いずれか一方：**

```bash
read -r -p 'Checkpointパッケージの絶対パス: ' CHECKPOINT_PACKAGE
# 単腕：Runtimeもdry-run、実機Actionを配信しない
bash scripts/run.sh --checkpoint "${CHECKPOINT_PACKAGE:?}" --dry-run
```

```bash
read -r -p '双腕Checkpointパッケージの絶対パス: ' CHECKPOINT_PACKAGE
# 双腕：Runtimeもdry-run、実機Actionを配信しない
bash scripts/run_dual.sh --checkpoint "${CHECKPOINT_PACKAGE:?}" --dry-run
```

runはPolicyとROS Runtimeの両方を起動します。既定もRuntime dry-runです。`--controller-run`を付けるとRuntimeがControllerへActionを送るモードになります。`--dry-run`と同時指定せず、Controller側の設定と区別します。

runは既存PolicyのCompose起動処理とclientの再作成を行うため、**稼働中のサーバーへ接続確認する目的でrunを再実行しません。** 実行時に表示される`/tmp/piper-package-...`のセッションディレクトリを保存してください。ここに生成済み`runtime.yaml`、`manifest.yaml`、`compose.yaml`、ログがあり、実際のruntime.dry_runは生成ファイルを確認します。元の`configs/runtime*.yaml`だけでは実行モードを判定できません。

モデルだけのoffline検証は別です。新しいGPUコンテナでモデルをロードし合成入力を試すため、既存Policyへの要求送信ではありません。ROS・実機入力を使用せず、GPUメモリにも余裕が必要です。

```bash
bash scripts/run_dual.sh --checkpoint "${CHECKPOINT_PACKAGE:?}" --offline
# 単腕は scripts/run.sh --checkpoint ... --offline
```

### 11.4 起動・WebSocket確認

**推論PC・ホスト：**

```bash
docker ps
# 双腕（単腕は下記の名前から -dual を除く）
docker logs --tail 50 piper-vla-dual-policy-1
docker logs --tail 50 piper-vla-dual-client-1
python3 scripts/wait_policy_ready.py --host 127.0.0.1 --port 8001 --attempts 1
```

実際に生成された設定をclient内から読む場合（双腕）：

```bash
docker exec piper-vla-dual-client-1 python3 -c \
  'import os,yaml; c=yaml.safe_load(open(os.environ["RUNTIME_CONFIG"])); print(c["runtime"])'
```

単腕のPolicy/client名は`piper-vla-policy-1` / `piper-vla-client-1`、WebSocketポート8000、双腕は8001です。上記チェックはWebSocket Upgrade応答までの確認で、モデルの推論成功やロボット契約一致の証明ではありません。PolicyログのCheckpoint・model/action契約と、Runtimeの接続・観測状態も確認します。

## 12. VLA推論の実行

### 12.1 操作順と検証段階

1. 装置の停止手段、機体・左右・カメラ・可動域・把持物を確認し、収集ノードを終了します。
2. 11章のROSネットワークを設定し、推論PCで対応するCheckpointのPolicy/Runtimeを起動します。最初は`--dry-run`を使います。Master未起動時はRuntimeの接続待ち状態も確認します。
3. 制御PCで9章または10章のControllerを`dry_run=true auto_enable=false`で起動します。ポートは開くので、CAN無接続の検証とは区別します。
4. 観測・画像・左右・時刻を確認し、12.3節のStartで言語指示を渡します。Runtime dry-runではraw/selectedを確認します。
5. Controller経路を確認する段階では、まず12.4節のStopを実行し、Runtimeだけを`--controller-run`で起動し直します。Controllerはdry-runのまま維持します。既存のPolicy/clientを無断で再作成しません。
6. 双腕はController status/tokenと左右target、単腕はtarget・safetyを確認します。HOLD/FAULT時は理由を解消し、停止・再開始を行います。
7. 実機運転はこの検証後、ハードウェア側の安全確認を済ませ、停止した状態でControllerを`dry_run=false auto_enable=true`へ変更して再起動した場合のみ行います。Runtimeも`--controller-run`が必要です。Start呼出しから実際の指令送信が始まります。

**実機運転用Controllerコマンド（実行するとenableされ得ます）：PiPERコンテナ。CAN・カメラ変数は9/10章で確認済みの値を同じ端末へ設定して使います。**

```bash
# 単腕：双腕とは同時実行しない
roslaunch piper start_vla_piper.launch \
  can_port:="${CAN_FOLLOWER:?}" \
  front_camera:="${CAM_FRONT:?}" wrist_camera:="${CAM_WRIST:?}" \
  dry_run:=false auto_enable:=true
```

```bash
# 双腕：左右角度制限は既定trueを維持
roslaunch piper start_vla_dual_piper.launch \
  left_can_port:="${CAN_LEFT:?}" right_can_port:="${CAN_RIGHT:?}" \
  front_camera:="${CAM_FRONT:?}" \
  left_wrist_camera:="${CAM_LEFT_WRIST:?}" \
  right_wrist_camera:="${CAM_RIGHT_WRIST:?}" \
  dry_run:=false auto_enable:=true
```

**Controller経路確認または実機運転用Runtime：推論PC・ホスト、推論リポジトリ内：**

```bash
# 対応する方のみ。先行セッションをStopした後に使う
bash scripts/run.sh --checkpoint "${CHECKPOINT_PACKAGE:?}" --controller-run
```

```bash
bash scripts/run_dual.sh --checkpoint "${CHECKPOINT_PACKAGE:?}" --controller-run
```

これは既存サーバー再利用専用コマンドではありません。現在動作中の推論を維持したい場合は既存clientの設定を確認し、そのStart/Stopのみを操作します。

### 12.2 観測・出力の確認

**PiPERコンテナ・別端末（`sorc_devel`後、同じROS接続先を設定）：**

```bash
rosnode list
# 単腕
rostopic hz /joint_states_single
rostopic hz /camera/front/compressed
rostopic hz /camera/wrist/compressed
# 双腕
rostopic hz /followerL/joint_states_single
rostopic hz /followerR/joint_states_single
rostopic hz /camera/left_wrist/compressed
rostopic hz /camera/right_wrist/compressed
rostopic hz /vla_dual/controller_status
rostopic echo -n 1 /vla_dual/controller_status
```

`rostopic hz`は確認後Ctrl-Cで終えて次へ進みます。topicの存在だけでなく、更新・stamp・内容を確認します。画像はCompressedImageのJPEGをRuntimeでdecodeしてRGBへ変換・224×224へアスペクト保持resize/padします。モデル側の正規化・逆変換はPolicyが担当し、ROS側で二重正規化しません。双腕Runtimeは左右Stateと3画像を要求し、既定の観測timeoutは0.5秒、最大skewは0.15秒です。推論timeout等は生成`runtime.yaml`を正とします。

### 12.3 言語指示とStart

StartInferenceサービスの定義は`string prompt` → `bool success, string message`です。これはRuntimeコンテナに生成される`piper_vla_runtime_msgs/StartInference`です。同じ定義を本ワークスペースにも含みます。

**ホストから対応するRuntimeコンテナへ入り、そこで実行：**

```bash
# 単腕
docker exec -it piper-vla-client-1 bash
```

```bash
# 双腕
docker exec -it piper-vla-dual-client-1 bash
```

**Runtimeコンテナ内：環境読み込み後、使用する方だけStart：**

```bash
source /opt/ros/noetic/setup.bash
# 単腕
rosservice type /piper_vla_runtime/start
rosservice call /piper_vla_runtime/start "prompt: 'Pick up the object and place it in the tray.'"
```

```bash
# 双腕
rosservice type /piper_vla_dual_runtime/start
rosservice call /piper_vla_dual_runtime/start "prompt: 'Align the bags, grasp them, and lift them up.'"
```

指示文は学習内容に合うものへ変更します。実行中にStartを重複送信せず、指示変更・再開はStop応答後に行います。双腕のController StartはRuntimeが呼ぶため手動では実行しません。

**出力確認：PiPERまたはRuntimeコンテナ・同じROS接続先の別端末：**

```bash
# 単腕
rostopic echo -n 1 /vla/raw_action
rostopic echo -n 1 /vla/selected_action
rostopic echo -n 1 /vla/target_joint_command
rostopic echo -n 1 /safety/reason
```

```bash
# 双腕
rostopic echo -n 1 /vla_dual/raw_action
rostopic echo -n 1 /vla_dual/selected_action
rostopic echo -n 1 /vla_dual/approved_action
rostopic echo -n 1 /vla_dual/left/target_joint_command
rostopic echo -n 1 /vla_dual/right/target_joint_command
rostopic echo -n 1 /vla_dual/safety_reason
```

出力がないtopicのechoは待機し続けるためCtrl-Cで終了します。rawは推論出力、selectedは時間選択結果、approvedはRuntimeの送信許可、targetはController安全判定後の診断です。targetが存在してもdry-run中はdriverへ送信しません。双腕のapprovedにはsession tokenが必要で、単に14値をpublishして動かす契約ではありません。

### 12.4 Stop・再開・終了

**Runtimeコンテナ内：対応する方のみ（`std_srvs/Trigger`）：**

```bash
rosservice call /piper_vla_runtime/stop "{}"
```

```bash
rosservice call /piper_vla_dual_runtime/stop "{}"
```

停止応答とHOLD・出力停止を確認します。再開は観測と原因解消を確認してStartを再度呼びます。新しい双腕セッションのtoken管理はRuntimeに任せます。Runtime Stop/HOLDは新規指令の遮断であり、非常停止やトルク保持・落下防止を保証しません。

必要なdriver停止要求は**PiPERコンテナ**から呼びます。装置が危険な場合はサービス応答待ちよりハードウェアの非常停止を優先します。

```bash
# 単腕VLA
rosservice call /stop_srv "{}"
```

```bash
# 双腕VLA
rosservice call /vla_dual_piper_controller/stop "{}"
rosservice call /followerL/stop_srv "{}"
rosservice call /followerR/stop_srv "{}"
```

終了時はRuntime Stop → 出力停止確認 → アーム・把持物の安全確保 → Controller launchのCtrl-Cの順です。Policy/clientを終了する場合は、実行時に表示された正しいComposeファイルを使います。

**推論PC・ホスト：Stop確認後のみ：**

```bash
read -r -p '終了するセッションの/tmp/piper-package-...パス: ' PIPER_SESSION_DIR
docker compose --env-file /dev/null -f "${PIPER_SESSION_DIR:?}/compose.yaml" down
```

### 12.5 初期姿勢復帰

初期姿勢YAMLは作業・機体に合わせて検証して用意します。同梱値を別機体・別治具へそのまま適用しません。関節はrad、gripperはmです。最終姿勢だけでなく復帰経路の干渉を確認します。

単腕は`config/piper_demo_start.yaml`の`target: {joint1: ..., ... joint6: ..., gripper: ...}`、双腕は`config/piper_dual_demo_start.yaml`の`targets.left` / `targets.right`に各7値を持ちます。双腕設定には`can_ports.left/right`もあり、復帰用CAN割当をlaunch引数とは別に照合します。

**PiPERコンテナ：RuntimeをStopし、動作許可を確認後：**

```bash
read -r -p '単腕の検証済み初期姿勢YAMLの絶対パス: ' SINGLE_POSE_YAML
read -r -p '単腕FollowerのCAN名: ' CAN_FOLLOWER
rosrun piper go_initial_pose.py \
  _config_file:="${SINGLE_POSE_YAML:?}" _can_port:="${CAN_FOLLOWER:?}"
```

単腕スクリプトに`--dry-run`はありません。既存driverがある場合はその指令経路を使い、ない場合はSDKを直接所有してenable・復帰します。既存CAN所有者を不明なまま重複起動しません。

```bash
read -r -p '双腕の検証済み初期姿勢YAMLの絶対パス: ' DUAL_POSE_YAML
rosrun piper go_dual_initial_pose.py --config "${DUAL_POSE_YAML:?}" --dry-run
# 経路・CAN・停止状態を確認し、実際に復帰するときのみ
rosrun piper go_dual_initial_pose.py --config "${DUAL_POSE_YAML:?}"
```

双腕スクリプトはRuntime/Controller停止と復帰lockを扱い、両腕の経路・姿勢・到達を検証します。`--dry-run`でも稼働RuntimeへのStop/HOLDやSDK接続・観測を行い得るため、稼働中推論への非干渉テストには使いません。復帰後の推論は安全確認後にRuntime Startで明示的に再開します。

## 13. トラブルシューティング

| 症状 | 確認・対応 |
|---|---|
| Dockerが起動しない | `docker version`、イメージ名、同名コンテナ、DISPLAY/Xauthority、X11を確認。稼働中コンテナを無断削除しない |
| xtermが見つからない・表示不能 | 現行Dockerfileでbuildしたイメージか、`command -v xterm`、`echo "$DISPLAY"`、X11 mountを確認。TTYのない状態でprefixだけ空にしても文字入力は復旧しない |
| 古いworkspaceを参照 | `type set_sdk sorc_devel`、`PIPER_WORKSPACE`、`rospack find piper`。新しいコンテナと`sorc_devel`で正しいoverlayを使う |
| pyenv bad interpreter / catkin Permission denied | `/usr/bin/python3`と生成物所有者を確認し`set_sdk`で修復。rootでbuildしない。ソースやデータ全体の権限を変更しない |
| CANがない・有効化失敗 | ホストUSB、ドライバー、bus-info、rename衝突、bitrate、UP/error状態を確認。別アームのCANを試しに指定しない |
| カメラがない・占有中 | ホストとコンテナのノード、アクセス権、接続後のデバイス公開、他の収集/配信プロセスを確認 |
| フォーマット・FPS不一致 | `--list-formats-ext`とlaunchのfourcc/解像度/FPSを照合。単腕VLA frontは既定YUYV |
| ROSノード・サービスが見えない | 全端末のROS_MASTER_URI/ROS_IP、双方向通信、時刻、`sorc_devel`・Runtime側メッセージ環境を確認 |
| Leader–Followerが始まらない | CAN役割、enable結果、Leader入力、双腕Leader EE到着を確認。Recorderのrは追従開始操作ではない |
| rが拒否・MCAP保存が失敗 | State/Action/EE/画像の鮮度・同期、task入力中か、空き容量、dataset_rootの権限・mountを確認。incompleteを正常データへ改名しない |
| Policy起動・WS接続失敗 | パッケージルート・checksum・モデル契約、GPU/メモリ、8000/8001、Policyログを確認。wait_policy_ready成功だけで推論成功とみなさない |
| rawだけ出てapprovedがない | Runtimeの生成runtime.yamlのdry_run、selected・HOLD/FAULT・観測timeoutを確認。双腕はcontroller_status受信、Start ACK、新鮮な一致tokenも必要 |
| approvedは出るがtargetがない | Controllerの拒否理由、State/feedback_health、次元・有限値・角度・FK/Z・gripperを確認。安全条件を無効化して通さない |
| targetは出るが動かない | Controller dry_runとdriver/enable状態を確認。診断targetは物理送信の証明ではない |
| 左右が逆に見える | 物理CAN/映像、収集の右→左、モデルの左→右、wrist1/2→left/right変換を順に照合 |

### CPU/Mockテスト

実機を使わない回帰テストは次をPiPERコンテナで実行します。SDK・ROS・カメラはFake境界へ置換されます。実CANを使うlaunchは実行しません。

```bash
sorc_devel
cd "$HOME/docker_piper_ctrl"
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/double_PiPER/src/piper/scripts:$PWD/double_PiPER/devel/lib/python3/dist-packages:$PYTHONPATH" \
/usr/bin/python3 -m unittest discover -s double_PiPER/src/mock
```

環境・一時catkin build・launch展開の確認は次です。X11接続のない環境では表示テストはskipされます。

```bash
PYTHONDONTWRITEBYTECODE=1 \
/usr/bin/python3 double_PiPER/src/test/test_piper_docker_environment.py
```

個人用CSVを指定したテストなどは、その入力がなければskipされる場合があります。Mock成功は実機の停止・保持・同時開始の保証ではありません。

## 14. 安全上の注意

- 起動・Enable前に関節可動域、ケーブル、治具、カメラ、把持物、周囲の人と非常停止の位置を確認します。双腕は相互干渉も確認します。
- 同じCANには一つの制御所有者だけを接続します。収集、VLA、直接SDKの姿勢復帰を競合させません。
- Recorderのs/x/q、Ctrl-C、Runtime Stop、HOLD、driver停止サービス、装置の非常停止は意味が異なります。サービス成功は物理停止完了の証明ではありません。
- 停止・電源断でトルク保持や落下防止ができるとは限りません。把持物とアームを安全に支持し、装置の手順に従います。
- SDK/Controller角度Clamp、FKの先端Z、step制限は衝突検出や全リンクの安全領域を保証しません。関節角度制限の無効化は通常運用に含めません。
- dry-runは層ごとに異なります。Runtime dry-runはapproved送信を抑止し、Controller dry-runはdriver目標送信を抑止します。実driver初期化や姿勢復帰の動作まで無効化する指定ではありません。
- 初期姿勢復帰も実動作です。機体固有のYAML、経路、CAN割当、到達判定を確認し、異常時に自動再開しません。
- 新しい機体・カメラ配置・Checkpointは、offline、Runtime dry-run、Controller dry-run、管理された実機試験の順に確認します。異常の原因を調べずHOLD/FAULT・token・鮮度検査を迂回しません。

上流PiPER ROSコードのMITライセンスは[double_PiPER/LICENSE](double_PiPER/LICENSE)に保持しています。PiPER SDK、推論Runtime、OpenPI、モデル・学習データの再配布条件は各配布元のライセンスに従ってください。
