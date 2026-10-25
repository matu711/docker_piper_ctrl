# docker-PiPer-env

カメラ確認から双腕データ収集・LeRobot変換・π0.5学習・推論までの統合手順は、[双腕PiPER統合ガイド](README_PiPER_guide.md)を参照してください。

## 1. Preriminay
手順1: Dockerfileおよびrun.sh内で定義されている`USR_NAME`変数を任意のユーザー名で設定
<Dockerfile>
```ruby:Dockerfile
ARG USR_NAME=<任意のユーザー名>
```
<run.sh>
```ruby:run.sh
export USR_NAME=<任意のユーザー名>
```

手順2. dockerのコンテナをビルドする。
```
sh build.sh
```

手順3: run.shを実行し、イメージを立ち上げる。
```
sh run.sh
```

dockerイメージが起動すれば、初回設定は完了!

## 2. 1台だけロボットをロボットを動かすとき
### 2.1 準備
手順1: PiPERの電源を入れ、PCとUSB接続を行う。


手順2: run.shを実行し、イメージを立ち上げる。
```
sh run.sh
```

手順3: PiPERのSDKをダウンロード＆ROSをCmake (以下のオリジナルコマンドを実施。詳細はDockerfileのalias部分を参照)
```
set_sdk
```

手順4: PiPERのUSB接続(CAN)を認識しているか確認する。
```
sudo ethtool -i can0 | grep bus
```
※ロボット増やすごとにcanの番号が`can0, can1,...`と増えていく。コマンドを実行すると、`1-2:1.0`のようなポート番号が出力

手順5: USB接続が認識できたら、以下のコマンドでCANをアクティブ化。
```
bash can_activate.sh can0 1000000 "USBポート番号"
```
※コマンド最後のUSBポート番号は手順4で確認したものを使う

<補足>
CANの状態を確認/操作するのに便利なコマンドを以下に記載

CANの状態を確認(すべてのCANを同時表示)
```
ip link show
```
CANを停止
```
sudo ip link set can0 down
```
CANを立ち上げ
```
sudo ip link set can0 up
```

### 2.2 テストコード
#### 2.2.1 PiPERでグリッパーの開閉と手先を円軌道に動かす
以下のコマンドを実施
```
python3 ./double_PiPER/src/test/test_ctrlPiperJoint_can0.py
```
※コード中の`piper = Piper("can0")`を、動かしたいPiPERのCAN番号に揃える

can0~can3まではスクリプトを分けて用意
- python3 test_ctrlPiperJoint_can0.py
- python3 test_ctrlPiperJoint_can1.py
- python3 test_ctrlPiperJoint_can2.py
- python3 test_ctrlPiperJoint_can3.py

#### 2.2.2 PiPERの状態読み込み
手順1: dockerイメージ内で、rosを立ち上げる
```
roscore
```

手順2: 新しいterminalのタブを開き、起動済みのdockerイメージに入る
```
docker exec -it piper-env-image bash
sorc_devel
```
※`sorc_devel`コマンドは、ROSのcatkinファイルに移動して`source catkin/devel/setup.bash`を行う自作コマンド

手順3: PiPERのROSラウンチを立ち上げる
```
roslaunch piper start_leader.launch can_port:=can1 auto_enable:=true
```
ここで、
`can_port`は、ノードを立ち上げるPiPERのCAN番号を変更するオプション(デフォルトは"can1")
`auto_enable`は、プログラムでPiPERを動かすために、PiPERのモードをチェンジするオプション

手順4: さらに、新しいterminalのタブを開き、起動済みのdockerイメージに入る
```
docker exec -it piper-env-image bash
sorc_devel
```

手順5: PiPERのJoint角度とグリッパー角度を公開するROSトピックの信号を可視化
```
rostopic echo /joint_grip_pos
```


#### 2.2.3 現在のジョイント角度を表示する

SDKのインストールとCANの有効化が済んだDockerコンテナ内で実行する。
関節1〜6をラジアンと度で1回表示する。アームやグリッパーへの動作指令は送らない。

```bash
cd ~/double_PiPER/src/test
python3 read_joint_angles.py --can can0
```

別のCANインターフェースを使う場合は指定する（既定は`can0`）。

```bash
python3 read_joint_angles.py --can can1
```

コードは[read_joint_angles.py](double_PiPER/src/test/read_joint_angles.py)。
`get_joint_states()`の角度を表示し、独自のオフセット補正や符号反転は行わない。
これは本体から返される関節角度であり、エンコーダの生カウントではない。
受信時刻が更新されない場合はエラーで終了する。

#### 2.2.4 関節6の現在位置を本体のゼロ点に設定する

**ROSノードやテストコードの動作指令を停止し、手先を支えて現在位置を保った状態で実行する。**
関節6のモーターを無効化するため、保持力が失われる。
既存のホーム姿勢や保存済み目標角度は、新しいゼロ点を基準に見直すこと。
関節6を0°として扱いたい姿勢に合わせてから、以下の順で確認・設定する。

```bash
cd ~/double_PiPER/src/test

# 全関節の現在角度を確認
python3 read_joint_angles.py --can can0

# Joint6の現在位置をゼロ点に設定
python3 set_joint6_zero.py --can can0

# 設定後の角度を確認
python3 read_joint_angles.py --can can0
```

別のCANを対象にする場合は`--can can1`などを指定する。
コードは[set_joint6_zero.py](double_PiPER/src/test/set_joint6_zero.py)。
以下の順で関節6だけを設定する。

1. 更新された関節情報を読み取り、設定前の関節6の角度を表示する。
2. `DisableArm(motor_num=6, enable_flag=0x01)`で関節6を無効化する。
3. 0.5秒待ち、`JointConfig(joint_num=6, set_zero=0xAE)`で現在位置をゼロ設定する。
4. 更新された角度を5回表示し、全て±0.1°以内なら反映を確認した旨を表示する。

**処理後も関節6は無効化したままで、自動で再有効化しない。**
SDKと、このリポジトリのSDK値をそのまま配信するROSノードで同じゼロ基準になる。
電源を入れ直した後の設定保持は、`read_joint_angles.py`で別途確認する。
ゼロ設定は毎回の起動で行う操作ではなく、基準を変更したい場合だけ実行する。

使用中の`Piper` API（SDKの`1_0_0_beta`ブランチ）とファームウェア`S-V1.8-7`で、
この無効化・ゼロ設定の手順により関節6が約−45.251°から0°になったことを実機で確認済み。

## 3. 単腕でleader-follower遠隔制御
手順1: 以下のコマンドを実施して、dockerイメージを立ち上げ、CANをアクティブ
```
sh run.sh
set_sdk
sorc_devel

bash can_activate.sh "<leader側のCAN番号>" 1000000 "<leader側のUSBポート番号>"
bash can_activate.sh "<follower側のCAN番号>" 1000000 "<follower側のUSBポート番号>"
bash can_activate.sh can0 1000000 3-11:1.0
bash can_activate.sh can1 1000000 3-12:1.0
roscore
```
※CAN番号は`can0`や`can1`など。USBポート番号は`1-2:1.0`や`1-2:1.0`など


手順2: 2つ目のterminalを開き、ROSラウンチを立ち上げる。
```
docker exec -it piper-env-image-matu
sorc_devel
roslaunch piper single_leader_follower.launch can_follower_port:=can0 can_leader_port:=can1 auto_enable:=true
```
ここで、
`can_follower_port`は、PiPER(follower側)のCAN番号を変更するオプション(デフォルトは"can0")
`can_leader_port`は、PiPER(leader側)のCAN番号を変更するオプション(デフォルトは"can1")
`auto_enable`は、プログラムでPiPERを動かすために、PiPERのモードをチェンジするオプション

## 4. 双腕でleader-follower遠隔制御
手順1: 以下のコマンドを実施して、dockerイメージを立ち上げ、CANをアクティブ
```
sh run.sh
set_sdk
sorc_devel

bash can_activate.sh can0 1000000 "<右手のleader側のUSBポート番号>"
bash can_activate.sh can1 1000000 "<右手のfollower側のUSBポート番号>"
bash can_activate.sh can2 1000000 "<左手のleader側のUSBポート番号>"
bash can_activate.sh can3 1000000 "<左手のfollower側のUSBポート番号>"

roscore
```
※USBポート番号は`1-2:1.0`や`1-2:1.0`など


手順2: 2つ目のterminalを開き、ROSラウンチを立ち上げる。
```
docker exec -it piper-env-image bash
sorc_devel
roslaunch piper dual_leader_follower.launch can_follower_R_port:=can0 can_leader_R_port:=can1 can_follower_L_port:=can2 can_leader_L_port:=can3 auto_enable:=true
```
ここで、
`can_follower_R_port`は、PiPER(右手のfollower側)のCAN番号を変更するオプション(デフォルトは"can0")
`can_leader_R_port`は、PiPER(右手のleader側)のCAN番号を変更するオプション(デフォルトは"can1")
`can_follower_L_port`は、PiPER(左手のfollower側)のCAN番号を変更するオプション(デフォルトは"can2")
`can_leader_L_port`は、PiPER(左手のleader側)のCAN番号を変更するオプション(デフォルトは"can3")
`auto_enable`は、プログラムでPiPERを動かすために、PiPERのモードをチェンジするオプション

## 5. カメラ映像のpublishとsubscribe

3台のカメラのデバイス割り当て・同時プレビュー・接続確認は、
[カメラ用README](double_PiPER/src/test/README_camera.md)を参照。

手順0: カメラのUSBをPCに接続し、デバイスを認識できているか確認
```
ls  /dev/video*
```
`/dev/video0`や`/dev/video1`が出力されれば、デバイスを認識できている。

手順1: 以下のコマンドを実施して、dockerイメージを立ち上げ、セットアップ
```
sh run.sh
set_sdk
sorc_devel
```

手順2: 新しいターミナルを開き、カメラpublisherのラウンチを起動
```
roslaunch piper camera.launch
```

手順3: カメラのsubscriberを起動
```
docker exec -it piper-env-image2 bash
sorc_devel

python3 src/piper/scripts/camera_sub_single.py
```

テストコード
`docker-PiPER-env_ver2/double_PiPer/src/test`のフォルダ内に、カメラが起動できるかを確かめるテストコード`test_camera.py`を準備

以下のコマンドを実行すると、カメラ画像をウィンドウ表示(プログラムを終了したい場合は`q`キーを押す)
```
cd ~/docker_PiPER_env_ver2/double_PiPER
python3 src/test/test_camera.py
```
また、`docker-PiPER-env_ver2/double_PiPer/src/piper/script`のフォルダ内に、publisherやsubscriberの雛形がある。
- camera_pub.py
- camera_sub_single.py (カメラを1台使用する際のsubscriber)
- camera_pub_double.py (カメラを1台使用する際のsubscriber)
※subscriberは、適宜、スクリプト内のカメラ画像を購読するROS topic名を変更する必要がある
(`single_leader_follower.launch`や`double_leader_follower.launch`のラウンチファイル内にカメラpublisher機能を追加しているが、現在、コメントアウト中)

### VLAデモデータ収集（MCAP対応）

リーダー・フォロワー制御、2台のカメラ画像配信、エピソード記録をまとめて起動する。

```bash
docker exec -it piper-env-image-matu bash
sorc_devel
roslaunch piper collect_piper_mcap.launch \
  can_follower_port:=can0 \
  can_leader_port:=can1 \
  auto_enable:=true
```

カメラウィンドウを選択して、以下のキーで操作する。

- `r`: 新しいエピソードの収集開始
- `s`: 現在のエピソードを保存し、保存成功後にfollowerを初期姿勢へ戻す
- `x`: 現在のエピソードを破棄
- `q`: 収集プログラムを終了

`h`を押した場合も従来どおりfollowerを初期姿勢へ戻す。`s`または`h`による復帰後は
leader制御が停止した状態になり、次の`r`でleaderのTEACHING_MODE確認とfollower制御の
再開を行ってから、新しいエピソードの収集を開始する。ここで使う初期姿勢は
`initial_pose_file`（既定 `~/.config/piper/initial_pose.yaml`）であり、`p`を押すと
現在のfollower姿勢をこのファイルへ保存できる。

データはホスト側の `piper_dataset/YYYYMMDD_HHMMSS/` に保存される。同じディレクトリ内に、時刻名のMCAPファイル、CSV、NPZ、JPEG画像、メタデータが作成される。

front（`/dev/video6`）とwrist（`/dev/video0`）は別スレッドで取得され、取得時刻が最も近い画像同士だけが保存される。画面の `camera`、`state`、`action` の同期誤差を確認し、すべて `SYNC OK` の範囲で収集する。保存後の `meta.json` には実測FPSとカメラ間同期誤差が記録される。

カメラは20 FPSを要求して取得し、学習フレームも固定20 FPSで保存する。実カメラが20 FPS未満の場合は、時刻誤差100 ms以内の直近画像を隣接する学習ステップで再利用し、state・actionは50 msごとに更新する。画像再利用数は`meta.json`へ保存される。1エピソードには1回の成功操作だけを収録し、往復動作や複数回の試行は別エピソードへ分ける。

画像は収集中でなくても次のROSトピックへ配信される。

- `/camera/front/compressed`
- `/camera/wrist/compressed`

## 6. VLA実機推論（PiPER単体）

VLA推論では leader PiPER や `single_leader_follower.launch` を使わない。既存の
`piper_ctrl_single_node.py` を CAN / SDK の低レベルdriverとして使い、新規の
`vla_piper_controller.py` が VLA action を ROS command に変換する。

```text
piper_vla_runtime
  /vla/approved_action, /vla/action_type, /vla/hold
        ↓
vla_piper_controller
        ↓  /joint_states または /pos_cmd
piper_ctrl_single_node.py
        ↓
can0 / PiPER
```

### 6.1 PiPER側の起動

ホストで PiPER Docker を起動し、コンテナ内で SDK / catkin 環境を準備する。
`set_sdk` は初回のみ必要である。

```bash
cd ~/docker_PiPER_env_ver2-main
sh run.sh

# PiPER Docker内
set_sdk
sorc_devel
cd ~/double_PiPER
bash can_activate.sh can0 1000000 "<can0のUSB bus-info>"
roscore
```

別ターミナルの同じ PiPER Docker で、最初は必ず dry-run を起動する。

```bash
docker exec -it piper-env-image-matu bash
sorc_devel
roslaunch piper start_vla_piper.launch \
  can_port:=can0 \
  auto_enable:=true \
  action_type:=joint_absolute \
  z_min:=0.01 \
  front_camera:=/dev/video6 \
  wrist_camera:=/dev/video0 \
  dry_run:=true
```

この launch は VLA runtime が購読する次の画像topicも起動する。

- `/camera/front/compressed`
- `/camera/wrist/compressed`

どちらも `sensor_msgs/CompressedImage`（JPEG）で、既定では front=`/dev/video4`、
wrist=`/dev/video6`、640x480 / 20 FPSである。コンテナから各デバイスが見えない場合は、
Docker起動時に対応する `--device=/dev/videoN` を追加してコンテナを作り直す。実機の
video番号はホスト側で `v4l2-ctl --list-devices` または `ls -l /dev/video*` で確認する。

`dry_run:=true` では VLA action の受信、target計算、Z safety、joint limit判定、
診断topic publishまでを行うが、`/joint_states` と `/pos_cmd` には指令を送らない。

### 6.2 デモ収集時の初期姿勢へ戻す

初期姿勢は次のYAMLから読み込む。

```text
~/double_PiPER/src/piper/config/piper_demo_start.yaml
```

ROS packageから見た同じファイルの指定は
`$(rospack find piper)/config/piper_demo_start.yaml` である。YAMLの `target` は
`joint1..joint6` がrad、`gripper`がmで、`move_duration`と`command_rate`が
補間時間と送信周期を指定する。

VLA runtimeが動作中の場合は、先に停止して `/vla/hold` を有効にする。

```bash
rosservice call /piper_vla_runtime/stop '{}'
```

初期姿勢へ戻すときは、PiPER Docker内で `go_initial_pose.py` を実行する。`roscore` は
必要だが、PiPER driverのlaunchは起動済みでも未起動でもよい。

```bash
sorc_devel
rosrun piper go_initial_pose.py \
  _config_file:=$(rospack find piper)/config/piper_demo_start.yaml \
  _can_port:=can0 \
  _z_enabled:=true \
  _z_min:=0.15
```

このPythonコードはPiPER driverの有無を自動判定する。driverが起動済みなら既存driverの
command topicを使用し、起動していなければ自身が単独でPiper SDKと `can0` を使用する。
driverの一部だけが検出された場合や複数のcommand経路がある場合は、CANの二重所有を
避けるため動作を拒否する。

実行前にYAMLの目標値、関節の公式範囲、補間軌道全体のFKによるZ下限を検証し、現在姿勢
から目標姿勢までsmoothstep補間する。実行中と終了後は `/vla/hold=true` を維持する。
このコマンドは検証だけでなく実機を動かすため、周囲と非常停止手段を確認して実行する。

### 6.3 VLA runtime側の起動

`~/piper_vla_runtime` は OpenPI policy server と ROS policy client を担当する。
PiPER単体driverの状態topicに合わせ、runtimeの `state_topic` は
`/joint_states_single` に設定する。

初回またはservice定義更新後は、PiPER Docker内で共有service packageをビルドして
同じshellでworkspaceをsourceする。

```bash
source /opt/ros/noetic/setup.bash
cd ~/double_PiPER
catkin_make --pkg piper_vla_runtime_msgs
source devel/setup.bash
```

```bash
cd ~/piper_vla_runtime
./scripts/run.sh --checkpoint ~/piper_checkpoints/current --dry-run
rosservice call /piper_vla_runtime/start \
  "prompt: 'Pick the object in the green area and place it in the red area.'"
```

#### `/start`で言語指示を切り替える

`/piper_vla_runtime/start`は独自service
`piper_vla_runtime_msgs/StartInference`を使用する。定義は次のとおり。

```srv
string prompt
---
bool success
string message
```

promptは空白を除いて1〜2048文字とし、空文字、空白のみ、2048文字超は拒否する。
推論実行中の再`/start`も拒否されるため、言語指示の標準的な切り替え手順は
`/start prompt 1` → `/stop` → `/start prompt 2`である。

```bash
# prompt 1で開始
rosservice call /piper_vla_runtime/start \
"prompt: 'Pick the object in the area labeled 1 and place it in the red area.'"

# 停止
rosservice call /piper_vla_runtime/stop '{}'

# prompt 2で再開始
rosservice call /piper_vla_runtime/start \
"prompt: 'Pick the object in the area labeled 2 and place it in the red area.'"
```

promptはmanifestやruntime YAMLを書き換えず、Runtimeの`current_prompt`へセッション状態
として保存される。各推論requestは開始時点のpromptと`session_generation`をsnapshotし、
そのrequest内で固定してPolicy Serverへ送信する。`/start`と`/stop`でgenerationを更新し、
古いgenerationの遅延responseはaction chunkへ登録せず破棄する。そのため、prompt 1の
推論中にstopしてprompt 2を開始しても、prompt 1のactionが新しいセッションへ混入しない。

新しい`/start`では、次のセッション状態をリセットする。

- active/pending action chunk
- 次に実行するaction index
- inference request event
- initial chunk待機状態
- 最後のHOLD理由
- 観測更新baseline

最初の推論は、`/start`後にjoint state、front画像、wrist画像、EE poseがそれぞれ更新
されるまで待つ。一方、Policy Server process、checkpoint、GPU上のweights、normalization
stats、WebSocket client/接続、ROS subscriber、action horizonは維持される。したがって、
promptを切り替えるだけならPolicy Serverの再起動やcheckpointの再ロードは不要である。

使用中のpromptはlatched topic `/vla/current_prompt`（`std_msgs/String`）へpublishする。
実機runのMCAP metadataにはpromptとsession generationを保存し、ROS logにはstart、stop、
generation、prompt、古いresponseの破棄を記録する。

runtimeの dry-run は PiPER側へ `/vla/approved_action` を送らない。PiPER側の
adapterだけを確認する場合は、`/vla/hold` を `false` にしてから、以下のように
actionを一度publishする。

VLA runtimeのMCAP/NPZ記録は、runtimeとPiPER controllerの`dry_run`が両方とも
`false`の実機runだけで開始される。どちらかが`true`ならログ保存は行わない。

```bash
rostopic pub -1 /vla/hold std_msgs/Bool '{data: false}'
rostopic pub -1 /vla/action_type std_msgs/String '{data: "joint_absolute"}'
rostopic pub -1 /vla/approved_action std_msgs/Float64MultiArray \
  '{data: [0, 0, 0, 0, 0, 0, 0]}'
```

#### π0.5 PBL1 joint absolute（step 59999）

```bash
cd ~/piper_vla_runtime
env -u ROS_MASTER_URI ./scripts/run.sh \
  --checkpoint ~/piper_checkpoints/pi05_piper_pbl1_joint_lora_step59999 \
  --dry-run
```

promptは`run.sh`ではなく、推論開始時の`/piper_vla_runtime/start`に指定する。
stack taskへ切り替える場合は、次を実行する。

```bash
rosservice call /piper_vla_runtime/start \
  "prompt: 'Pick the object in the green area and stack it on top of the object in the red area.'"
```

正式なserver出力は絶対関節角`[rad]`＋絶対gripper開幅`[m]`であり、
PiPER側は`action_type:=joint_absolute`を使用する。

#### π0.5 joint absolute（step 40000）

`pi05_piper_real_joint_lora_step40000`はモデル内部ではjoint residualを学習するが、
policy serverの`AbsoluteActions`後に返る値は`joint1..6 absolute [rad]`と
`gripper absolute [m]`である。client側でstateを加算せず、`joint_absolute`として使う。

```bash
# PiPER Docker内（最初は必ずdry-run）
roslaunch piper start_vla_piper.launch \
  can_port:=can0 \
  action_type:=joint_absolute \
  gripper_action_type:=physical_absolute \
  z_min:=0.15 \
  dry_run:=true

# 実機ホスト側
cd ~/piper_vla_runtime
env -u ROS_MASTER_URI ./scripts/run.sh \
  --checkpoint ~/piper_checkpoints/pi05_piper_real_joint_lora_step40000
rosservice call /piper_vla_runtime/start \
  "prompt: 'Move the white object from the green square to the red square.'"
```

この組合せではruntimeはactionを送るが、PiPER controllerが実機commandを止める。
`/vla/action_type`はmanifest内部名ではなく、robot-facing contractの
`joint_absolute`としてpublishされる。

#### PBL8（piper_pi0_pbl8_lora_step99999）

PBL8は通常の`ee_delta`ではなく、world/base frameのrotation vectorを含む累積EE
reference actionである。黒マスクはカメラへ物理的に取り付け、runtime側ではwrist画像の
左90°回転だけを行う。最初は必ずPiPER controllerをdry-runにする。

```bash
# PiPER Docker内
cd ~/double_PiPER
catkin_make
source devel/setup.bash
roslaunch piper start_vla_piper.launch \
  can_port:=can0 \
  action_type:=ee_delta_physical_world_per_control_step \
  gripper_action_type:=pbl8_normalized_absolute \
  pbl8_max_translation_delta_m:=0.003 \
  pbl8_max_rotation_delta_rad:=0.01 \
  z_min:=0.01 \
  dry_run:=true

# 実機ホスト側
cd ~/piper_vla_runtime
./scripts/run.sh \
  --checkpoint ~/piper_checkpoints/piper_pi0_pbl8_lora_step99999
rosservice call /piper_vla_runtime/start \
  "prompt: 'Pick the object in the green area and place it in the red area.'"
```

この組合せではruntimeはactionをpublishするが、PiPER controllerが`/pos_cmd`への実機
送信を止めるため、`/vla/target_ee_command`まで確認できる。PBL8 actionの上限は並進
norm 0.003 m/step、回転norm 0.01 rad/stepで、超過時はclipせずHOLDする。gripperは
`-1=0.10 m open`、`0以上=0 m closed`である。

このcheckpointのpolicy stateは7次元`[joint1..6 rad, total gripper opening m]`である。
旧PBL8 bundle向けの左右finger qposへの8次元変換は、このcheckpointには適用しない。

実機実行は、checkpointと各topicを確認した後に、PiPER側とruntime側の両方を
`dry_run:=false` / `--dry-run`なしで起動する。`/vla/hold` が `true` の間、
VLA controllerは新しい実機commandを送らない。

### 6.4 VLA関連topic

| 方向 | topic | 型 | 内容 |
| --- | --- | --- | --- |
| PiPER → VLA | `/joint_states_single` | `sensor_msgs/JointState` | joint 1–6 [rad]、gripper [m] |
| PiPER → VLA | `/end_pose` | `geometry_msgs/PoseStamped` | 現在EEF pose |
| runtime → controller | `/vla/approved_action` | `std_msgs/Float64MultiArray` | 実行候補action `[... , gripper]` |
| runtime → controller | `/vla/action_type` | `std_msgs/String` | `joint_absolute` / `joint_delta` / `ee_absolute` / `ee_delta` / `ee_delta_cumulative_reference` |
| runtime | `/vla/current_prompt` | `std_msgs/String` | 現在の推論セッションで使用する言語指示（latched） |
| runtime → controller | `/vla/hold` | `std_msgs/Bool` | `true` は HOLD |
| controller → driver | `/joint_states` | `sensor_msgs/JointState` | joint command |
| controller → driver | `/pos_cmd` | `piper_msgs/PosCmd` | EE command |
| controller diagnostics | `/safety/state`, `/safety/reason`, `/safety/target_z` | `String`, `String`, `Float64` | safety結果 |

joint actionはPiPER SDK公式FKと公式joint limitを使ってZ下限・関節範囲を判定する。
違反時は値をclipせずHOLDする。EE actionは既存driverの `/pos_cmd → EndPoseCtrl`
経路を使い、自作IKは行わない。`ee_delta` の回転差分はcheckpointが姿勢表現を
明示するまでHOLDする。

### 6.5 PBL GUIデモの流れ

PiPER・カメラ・PBL Demo ManagerはPiPER Docker内で起動する。以下は実機用の例なので、
CAN接続、カメラ番号、アーム周囲と非常停止手段を確認してから実行する。

```bash
# PiPER Docker内
sorc_devel
roslaunch piper pbl_demo.launch \
  gui:=true can_port:=can0 \
  front_camera:=/dev/video6 wrist_camera:=/dev/video0 \
  dry_run:=false
```

VLA runtimeはこのlaunchには含まれない。別ターミナルで既存のVLA runtimeを対応する
checkpointで起動し、実機で動かす場合はruntime側もdry-runにしない。起動方法は
上の「6.3 VLA runtime側の起動」を参照する。GUIが開始を指示するまで
`/piper_vla_runtime/start`を手動で呼ばない。PiPER Docker内から次の両serviceが見えることを
確認する。

```bash
rosservice info /piper_vla_runtime/start
rosservice info /piper_vla_runtime/stop
```

続いてホスト側の別ターミナルでGUIを開く。

```bash
# このリポジトリのルートで実行
./tools/run_pbl_demo_gui.sh
```

GUI操作とManagerの動きは次のとおり。

1. START画面で3・4・5段を選ぶ。GUIが1〜6から重複なしの`pick_order`を生成してManagerへ送る。Managerは再抽選せず、HOMEへ戻って最初のpromptを設定し、`WAIT_READY`で待つ。「もんだいを かえる」は開始前の再抽選。5段では下から4つ目までの合計高さが22 cm以下の問題だけを採用する。
2. 「じゅんび できた！」を押すと、`WAIT_READY`からVLAを開始する。HOME移動中に押した場合は、HOME到達後まで開始を待つ。
3. 実行中に`SPACE`または「せいこう」を押すと、VLAを停止してHOMEへ戻り、次のtarget・promptへ進む。次のtargetがある場合は、HOME到達後にGUIがVLAを自動で再開する。最終targetの成功後は完成画面を経てSTART画面へ戻る。
4. 実行中に`R`または「やりなおし」を押すと、VLAを停止してHOMEへ戻り、同じtarget・promptでRetry待機する。人が積み木を戻した後、`SPACE`または「さいかい」で再実行する。失敗時に`/stop_srv`や`/reset_srv`は呼ばない。
5. デモ中に`ESC`を押すと、VLAを停止して`/vla/hold=true`を維持し、デモをキャンセルしてSTART画面へ戻る。ESCによるキャンセルではHOME移動は行わない。START画面での`ESC`はGUIを閉じる。

HOME復帰中も`/vla/hold=true`を維持し、`/joint_states_single`の実測値で到達を確認する。
SUCCESS時の「Safe Retreat」は専用の安全な経路が未定義のため実行せず、VLA停止後に
直接HOME補間へ進む。周囲に障害物がある状態で自動的に安全な退避が行われるわけではない。

`/piper_vla_runtime/start`が見つからない場合、準備ボタンを押してもVLAは開始せず
`WAIT_READY`に留まる。runtimeの起動状態と、PiPER Dockerからのservice到達性を確認する。
また、`dry_run:=false`はPiPER側だけの設定である。runtime側が`--dry-run`なら実機への
VLA actionは送られない。GUIとManagerの順番は、GUI側の`GUI generated pick_order=...`と
Managerの`/pbl_demo_manager/gui_status`にある`pick_order`で確認できる。

### トラブルシューティング
ROSでcatkin_makeが実行できない場合は、以下のコマンドでlogを削除
```
rm -rf build/ devel/ log/
```

ROSのラウンチやトピックがうまく表示されないときは、rosのsetup.bashを再読込
```
sorc_devel
```
