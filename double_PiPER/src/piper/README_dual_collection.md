# 双腕リーダーフォロワーの教示データ収集

新規の`launch/collect_dual_piper_mcap.launch`で4台のアーム制御と3カメラの記録を起動する。
既存の単腕・双腕launch、制御ノード、レコーダーは変更していない。
同じCANやカメラを使用する既存launchと同時に起動しないこと。

## 構成

| 役割 | CANの既定値 | 状態トピック |
| --- | --- | --- |
| 右リーダー | can1 | `/leaderR/joint_grip_pose` |
| 右フォロワー | can0 | `/followerR/current_joint_obs` |
| 左リーダー | can3 | `/leaderL/joint_grip_pose` |
| 左フォロワー | can2 | `/followerL/current_joint_obs` |

実際にSDKへ送った指令は`/followerR/applied_joint_cmd`と`/followerL/applied_joint_cmd`に配信する。
これらは全て`sensor_msgs/JointState`で、各7要素（関節1〜6のrad、グリッパーのm）。
ActionはSDKのCAN変換分解能（関節0.001度、グリッパー0.000001 m）に合わせる。
モーターの有効状態・CAN制御モード・SDKの指令出力状態を確認してから送信し、
SDK呼び出し後に配信する。CANの受信確認・実際の追従達成を保証するACKではない。
Stateは本体の実測値で、Actionと分けて記録する。

カメラは`/camera/front/compressed`、`/camera/wrist1/compressed`、`/camera/wrist2/compressed`に
`sensor_msgs/CompressedImage`で配信する。

`/dual_piper/status`は`std_msgs/String`のJSONで、`ready`、`enabled`、`following`、
`right_following`、`left_following`、`returning`、`fault`を確認できる。
左右のfollowingは指令送信が有効かを示し、目標到達の判定ではない。

## 制御方針

- XYZ・姿勢による作業領域判定、領域外での拒否・座標補正は設けない。
- SDKの関節限界とグリッパーの許容値は維持する。非有限値・関節限界外の指令は異常として扱う。
- 制御は記録プロセスと分離し、約50 Hzでフィードバック確認と指令送信を行う。
- 関節・グリッパー・本体状態の更新停止、本体異常、追従中の制御モード変化や
  フォロワーモーターの無効化を検出したら、異常を保持して両フォロワーへ非常停止を送る。
- 通信回復だけでは異常を解除しない。非常停止や本体エラーの解除は本体側の手順で実施し、
  その後`reset_fault` → `enable` → `start_following`を明示的に呼ぶ。
- `auto_enable=false`では起動時のモーター有効化・モード変更を行わない。
  `enable`サービスを明示的に呼んだ場合のみ有効化する。
- 起動後は`auto_enable=true`でも追従を停止した状態。追従開始・再開時に両腕の関節差と
  グリッパー差を確認する。これは再開時の急な姿勢差を防ぐ確認で、作業領域の制限ではない。
- 停止指令の送信自体が失敗する通信障害や、制御プロセスが強制終了するケースでは
  ソフトウェアだけで物理停止を保証できない。実機の非常停止手段を確保する。

## 起動前の準備と起動コマンド

以下は今後の実機検証用手順。この実装の検証では実機ノードを起動していない。

1. 4台の実機とCAN対応を確認し、CANを1 Mbpsで有効化する。
2. リーダーとフォロワーの関節のゼロ基準が左右それぞれ一致していることを確認する。
   特に関節6のゼロ設定変更後は、保存済み初期姿勢を作り直す。
3. カメラの番号を`v4l2-ctl --list-devices`で確認する。
4. コンテナに`mcap`・`mcap-ros1-support`がなければインストールする。

```bash
python3 -m pip install mcap mcap-ros1-support
source /opt/ros/noetic/setup.bash
source ~/double_PiPER/devel/setup.bash
roslaunch piper collect_dual_piper_mcap.launch \
  auto_enable:=false \
  can_leader_R_port:=can1 can_follower_R_port:=can0 \
  can_leader_L_port:=can3 can_follower_L_port:=can2 \
  front_camera:=/dev/video4 wrist1_camera:=/dev/video6 wrist2_camera:=/dev/video8 \
  record_fps:=20 camera_fps:=30 \
  dataset_root:=$HOME/piper_dataset/dual
```

新規ノードはPythonスクリプトで実行権限を付与済み。既存ワークスペースがビルド済みなら、
新規launchを使うだけのためにROS全体を再ビルドする必要はない。
この構成はソースワークスペースで使用する。既存CMakeのinstall設定は変更していない。

初回は`auto_enable=false`で読み取りから確認する。この値でも通信異常時・終了時は
両フォロワーに非常停止指令を送るが、モーターを有効化することはない。
実機の周囲と姿勢を確認してから、別ターミナルで以下を明示的に実行する。

```bash
rosservice call /dual_piper/enable '{}'
rostopic echo /dual_piper/status
```

リーダーは教示モード、フォロワーはCAN制御モードになる。
両リーダーを対応するフォロワーの姿勢に手で合わせてから追従開始する。

```bash
rosservice call /dual_piper/start_following '{}'
```

## 初期姿勢と記録操作

制御サービスは全て`std_srvs/Trigger`で、`/dual_piper/enable`、`start_following`、
`stop_following`、`save_initial_pose`、`go_initial_pose`、`reset_fault`、`emergency_stop`。
記録サービスも`std_srvs/Trigger`で、`/dual_recorder/start`、`stop`、`discard`。
`emergency_stop`は初期姿勢操作中でも受け付け、両フォロワーへの非常停止を要求して異常を保持する。
物理非常停止の代わりにはならない。

初期姿勢はフォロワーの現在の実測値を左右別YAMLへ保存する。
リーダーの自動復帰は行わず、復帰後のリーダー位置合わせは手動で行う。

```bash
# 初期姿勢へ手動教示で移動した後、追従を止めて保存する
rosservice call /dual_piper/stop_following '{}'
rosservice call /dual_piper/save_initial_pose '{}'

# 位置合わせを確認して追従再開し、記録開始
rosservice call /dual_piper/start_following '{}'
rosservice call /dual_recorder/start '{}'

# 記録保存と両腕の追従停止（自動復帰はしない）
rosservice call /dual_recorder/stop '{}'

# 両フォロワーを保存済み初期姿勢に戻す。復帰後も追従停止のまま
rosservice call /dual_piper/go_initial_pose '{}'
```

保存先の既定値は`~/.config/piper/initial_pose_right.yaml`と`initial_pose_left.yaml`。
launch引数`right_initial_pose_file`・`left_initial_pose_file`で変更可能。
片腕の復帰失敗・タイムアウト時は両側を停止し、追従を自動再開しない。

| プレビューのキー | 操作 |
| --- | --- |
| `c` | 両フォロワーの追従開始・再開（位置合わせ確認あり） |
| Space | 両フォロワーの追従停止（記録中は先に保存・破棄する） |
| `r` | 記録開始。両腕が既に追従中であることが必要 |
| `s` | 記録保存と追従停止 |
| `x` | 現在のエピソード破棄と追従停止 |
| `p` | 左右初期姿勢を保存（追従停止中のみ） |
| `h` | 左右フォロワーを初期姿勢へ復帰 |
| `q` | 終了。記録中のデータは未完了として保持し、追従を停止 |

初期姿勢サービスは制御ノード内の別コールバックで実行し、フィードバック監視は継続する。
GUIなしで使う場合は`no_preview:=true`を指定して上記ROSサービスを使う。
記録だけを止める場合も、両腕への新規指令送信を停止する。
追従停止はモーターの無効化ではなく、最後の目標位置を保持する。

## 記録データの構造

```text
piper_dataset/dual/
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

- `observation_state`と`action`は`float32`の`(N, 14)`。
  順序は`right_joint_1 ... right_joint_6, right_gripper, left_joint_1 ... left_joint_6, left_gripper`。
- `timestamp`は記録開始からの固定20 Hzグリッド時刻、`image_ros_timestamp`は同期基準時刻。
- 3カメラと左右State/Actionそれぞれの`*_ros_timestamp`・`*_time_error`をNPZとCSVへ保存する。
  誤差の符号は「データ時刻−同期基準時刻」。`camera_time_error`は3画像の最大時刻差。
- MCAPは左右リーダーの生値、左右State/Action、制御状態、同期採用した3画像を記録する。
  State/Actionの生サンプルは、学習用行に採用されなかったものもMCAPに残る。
- メタデータには配列順序、単位、機種、タスク、デバイス、トピック、同期許容値、
  欠落数・画像再利用数、実際のカメラ設定と計測FPSを記録する。
  保存先の既存`dataset_meta.json`と配列順序・カメラ数・保存FPSが違う場合は起動を拒否する。
- State/Actionは既定でグリッド時刻から±30 ms、画像は±100 ms、3画像間は30 ms以内で採用する。
  カメラ画像はこの条件を満たす場合だけ再利用する。
- 欠落・非有限値・同期範囲外はその行を保存せず、`frame_index`の欠番を保持する。
  処理遅延で失ったグリッドも数える。ゼロ埋めや古いState/Actionの無制限な再利用はしない。
- 有効行の欠落が`max_data_gap`（既定1秒）を超えた場合、制御異常・カメラ異常・時刻の不連続時、
  未完了エピソードを`.tmp_episode_.../`に保持し追従停止を要求する。自動記録再開はしない。
- 単腕と同じ`observation_state`・`action`キー、JPEG・CSV・NPZ・MCAPの構成を使うが、
  14次元・3カメラの新フォーマットであり、7次元・2カメラ固定の学習コードは対応が必要。
  この版では手先姿勢（EE）やLeRobotへの変換は実装していない。

時刻同期はSDKの受信時刻とOpenCV読み取りの中点によるソフトウェア同期。
ハードウェア同期ではなく、カメラの実露光時刻を保証しない。`/use_sim_time`は使わない。
20 Hzは保存グリッドで、実カメラの取得FPSを保証しない。
USB帯域・実測FPS・同期欠落率を実機で確認する。
既定の保存先は`run.sh`の`piper_dataset`マウント内なのでホストに残る。
保存先や初期姿勢ファイルを変更する場合も、永続化されるマウント内に指定する。

## 主なlaunch引数

CAN・カメラ・保存先に加え、`speed`、`initial_pose_speed`、`feedback_timeout`、`mode_timeout`、
`initial_pose_timeout`、`initial_pose_joint_tolerance`、`initial_pose_gripper_tolerance`、
`resume_joint_tolerance`、`record_fps`、`camera_fps`、`camera_width`、`camera_height`、
`camera_fourcc`、`camera_backend`、`jpeg_quality`、`sync_tolerance`、`sync_wait`、
`camera_sync_tolerance`、`camera_target_tolerance`、`max_data_gap`、`task`、`repo_id`を変更できる。
`enable_recording:=false`は制御のみを起動する。

## オフライン検証

```bash
source /opt/ros/noetic/setup.bash
source ~/double_PiPER/devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 ~/double_PiPER/src/test/test_dual_collection.py
roslaunch --nodes piper collect_dual_piper_mcap.launch auto_enable:=false
```

テストではSDK・ROSの制御インターフェースをモックに置き換え、実機へ接続しない。
`--nodes`はlaunch解析とノード名の列挙のみで、ROSノードを起動しない。
