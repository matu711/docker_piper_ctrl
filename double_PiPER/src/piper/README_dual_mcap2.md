# 双腕MCAP収集：MCAP2

`collect_dual_piper_mcap2.launch`は、`collect_piper_mcap.launch`と
`dual_leader_follower.launch`で使われる単腕ドライバーを基にした新しい構成。
既存の単腕・双腕ファイルは変更していない。
`piper_mcap2_arm.py`が各CANを独立に初期化し、ROSサービスで左右の操作をまとめる。
初期姿勢のYAML形式・保存・復帰処理とMCAP・ROSバッファは単腕コードを再利用する。
カメラは`test_triple_camera.py`と同じ設定順で3台を設定し、専用プロセスで順番に読む。

## 起動

実機検証は未実施。全CANの角度受信とカメラ単独テストを確認してから使用する。
同じCAN・カメラ・ROS名前空間を使う別のlaunchやカメラテストと同時に起動しない。

| 役割 | CAN |
|---|---|
| 右リーダー | can0 |
| 右フォロワー | can1 |
| 左リーダー | can2 |
| 左フォロワー | can3 |

```bash
source /opt/ros/noetic/setup.bash
source ~/double_PiPER/devel/setup.bash
roslaunch piper collect_dual_piper_mcap2.launch auto_enable:=true
```

既定の`auto_enable:=true auto_follow:=true`では、起動後から各フォロワーがリーダーの値を受けて動く。
リーダーは既存ドライバーと同様に起動時はCAN制御で保持する。
`t`で左右リーダーを教示モードにすると手で動かせ、その動きへフォロワーが追従する。
`r`は単腕と同じく、左右リーダーの教示モード確認→追従再開→記録開始の順に処理する。
`auto_enable:=false`は読み取りから開始し、`u`で明示的に有効化・追従開始する。
有効化のみで保持したい場合は`auto_follow:=false`を指定する。
CAN構成、カメラ、保存先等は引数で変更できる。

```bash
roslaunch piper collect_dual_piper_mcap2.launch \
  auto_enable:=true \
  can_leader_R_port:=can0 can_follower_R_port:=can1 \
  can_leader_L_port:=can2 can_follower_L_port:=can3 \
  front_camera:=/dev/video4 wrist1_camera:=/dev/video2 wrist2_camera:=/dev/video0 \
  camera_fps:=30 record_fps:=20 \
  dataset_root:=$HOME/piper_dataset/dual2
```

MCAP未導入の場合は同じPython環境へ`python3 -m pip install mcap mcap-ros1-support`。
`run.sh`のマウントを使う場合、既定のデータ保存先・初期姿勢ファイルはホストに残る。
新しいPythonはソースワークスペースで実行する。launch追加だけのための再ビルドは不要。

## 操作

カメラプレビューをクリックしてキーを押す。

| キー | 操作 |
|---|---|
| `u` | 4台を有効化。既定では追従も開始する |
| `t` | 左右リーダーを教示モードへ切り替える。記録は開始しない |
| `c` | 追従再開。リーダーのモードは変更しない |
| `r` | 左右リーダーを教示モードにし、追従再開・最新の左右State/Actionを待って記録開始 |
| Space | 追従停止。記録中は先に保存・破棄する |
| `s` / `v` | 記録保存と追従停止。自動で初期姿勢には移動しない |
| `x` | 記録破棄と追従停止 |
| `p` | 追従を止め、左右フォロワーの初期姿勢を保存 |
| `h` | 保存済み初期姿勢へ両フォロワーを移動。リーダーは手動で合わせる |
| `e` | 安全を確認した後、エラーコード0のフォロワー非常停止を明示的に復旧 |
| `q` | 全launch終了。記録中のデータは未完了として保持 |

例：起動時にフォロワーがリーダーへ移動 → `t`で教示 → Space → `p` → `r` → 教示 → `s` → `h`。
`e`のみでは有効化・追従再開しない。その後の`u`では既定で追従が始まる。
`check_start_alignment:=false`が既定で、追加の開始前姿勢差チェックは無効。
このチェックを使う場合は`auto_follow:=false check_start_alignment:=true`とし、
位置合わせ後に`c`/`r`を押す。チェック有効時の許容値は関節0.2rad、グリッパー0.005m。
追従停止は新規指令の停止で、モーターは目標を保持する。

GUIなしの場合は`no_preview:=true`とサービスを使う。

```bash
rostopic echo /dual_piper/status
rosservice call /dual_piper/enable '{}'
rosservice call /dual_piper/start_following '{}'
rosservice call /dual_recorder/start '{}'
rosservice call /dual_recorder/stop '{}'
```

制御サービス：`/dual_piper/enable`、`enter_teaching_mode`、`start_following`、`stop_following`、
`save_initial_pose`、`go_initial_pose`、`reset_fault`、`recover_stop`、`emergency_stop`。
記録サービス：`/dual_recorder/start`、`stop`、`discard`。全て`std_srvs/Trigger`。
各腕にも同名のサービスを提供するが、操作には左右まとめたサービスを使用する。

初期姿勢は`~/.config/piper/mcap2_initial_right.yaml`と`mcap2_initial_left.yaml`に保存。
`right_initial_pose_file`・`left_initial_pose_file`で変更可能。関節ゼロ点変更後は保存し直す。

## 記録構造

```text
~/piper_dataset/dual2/
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

NPZの`observation_state`と`action`はfloat32の`(N,14)`。
右の関節1〜6(rad)・グリッパー(m)、続いて左の同7要素の順。
StateはSDK実測値、ActionはSDKへ送った分解能に合わせた指令値で、ハードウェアACKではない。
追従中のグリッパー指令は既存フォロワーと同じ0〜100mmに収める。
リーダーの生値・フォロワー実測値は変更せず、Actionには実際に送った制限後の値を記録する。
NaN・無限値は送信せず異常停止する。
単腕と同じMCAP/NPZ/CSV/JPEG形式を使うが、7次元・2画像固定の学習コードは対応が必要。

左右State/Action、3画像の時刻と同期誤差をNPZ・CSVへ保存する。
基準は固定20Hzグリッド。State/Actionは±30ms、画像は±100ms、3画像間の差は30ms以内。
誤差の符号はデータ時刻−基準時刻。同期範囲内なら画像の再利用を許し、その回数を記録。
未受信・古い値・同期範囲外の行は保存せず、frame_indexの欠番と欠落数を残す。
MCAPには左右State/Actionの生サンプル、左右リーダー値、制御状態、採用画像を保存する。
欠落が既定1秒を超えた場合や制御・カメラ異常時は、`.tmp_episode_...`に未完了データを残し追従停止を要求する。
ソフトウェア同期であり、画像の露光時刻を保証しない。実測FPS・欠落率の実機確認が必要。
画像時刻は有効なV4L2フレーム時刻を優先し、取得できない場合はread前後の中点を使用する。
USB転送・復号・プロセス間転送の遅れを考慮し、`sync_delay:=0.2`秒待ってから
そのグリッド時刻の画像と過去のState/Actionを選択する。時刻の書き換えや同期許容値の緩和は行わない。
取得解像度と保存解像度、報告FPSと実測FPS、時刻の取得元をメタデータに保存する。

`/dev/video4`のFemto Boltは640×480をサポートせず、MJPGの最小対応解像度は1280×720。
取得後に640×480へ縮小して保存・表示する。ほかの2台は640×480で取得する。
OpenCV内部スレッド数は1とし、初期化中の15回読み取りとカメラ別スレッドによる読み取りは使わない。
初回フレームの取得に繰り返し失敗したデバイスは、取得専用プロセス内で閉じて再初期化する。
再試行は各カメラの`camera_open_timeout`内に限定し、継続的な取得異常は隠さず記録を停止する。
V4L2内部の待ち時間は正常なプレビューテストと同じ10秒とする。初回のストリーム開始には数秒かかるため、1秒への短縮は行わない。起動待ちには3台のopenと初回readの時間を含める。
腕の初期化中でも追従停止を受け付け、停止・終了要求後の自動追従開始を抑止する。
CANモードへの切り替えは応答が確認できるまで再送する。初期化タイムアウト時はモード・モーター有効状態・未受信項目を表示し、ROS終了による中断と区別する。
初回フレームは最大3回試行し、V4L2のselect待ちは1秒に制限する。
`camera_read_timeout`（既定2秒）を超えた取得停止は記録を未完了として保持し追従停止を要求する。

Ctrl+C時は記録ファイルを先に確定し、その後カメラ取得プロセスを終了する。
ネイティブread/releaseが応答しない場合もカメラ用プロセスだけを停止し、記録ノードの終了を妨げない。
終了時のROS停止サービスは0.5秒で待ちを打ち切る。通信断時の物理停止は保証できない。
中断データは`.tmp_episode_...`内の読み取り可能な`.incomplete.mcap`、NPZ、CSV、JPEGとして保持する。

## 制御上の確認事項

XYZの作業領域判定・範囲外拒否・座標補正はない。SDKの関節限界とグリッパー限界は維持する。
Joint2の公称下限0radとJoint3の公称上限0radについてのみ、
実測ゼロ近傍のずれを許容する`zero_angle_tolerance`を既定0.05rad（約2.86°）で設ける。
関節値は補正・クリップせず記録・送信する。`zero_angle_tolerance:=0`で公称値の厳密判定に戻せる。
この許容差はソフトウェアの指令判定だけで、ハードウェアの関節限界・ゼロ点設定は変更しない。
既定は`monitor_feedback:=false stop_both_on_fault:=false`で、腕別処理を行う。
古い情報・モード変化・モーター無効化による追加の監視停止は無効で、片腕の異常を
もう片腕へ自動伝播させない。本体エラー、未受信、非有限値等の腕単位の判定は維持する。
追加監視と片腕異常時の両腕停止へ戻すには、両引数を`true`にする。
明示的な非常停止、初期姿勢復帰・有効化操作の失敗時の停止、記録中の欠落処理は維持する。
記録中に左右データが揃わなくなった場合は記録を中断し、新規指令の停止を要求する。
通信回復のみで異常になった腕を再開しない。正常終了は動作終了指令、異常時は非常停止を要求する。
通信断・プロセス強制終了ではソフトウェアの停止指令が届かない場合があるため、物理非常停止手段を確保する。
CANがUPでも受信があるとは限らない。USBエラーは本コードの再試行では解決できない。

## 実機を使わない確認

```bash
source /opt/ros/noetic/setup.bash
source ~/double_PiPER/devel/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 ~/double_PiPER/src/test/test_dual_mcap2.py
roslaunch --nodes piper collect_dual_piper_mcap2.launch auto_enable:=false
```

カメラだけの検証（CAN・ロボット制御は使用しない）：

```bash
python3 ~/double_PiPER/src/test/test_mcap2_camera_capture.py --seconds 10
```

テストはCAN・ROS実行をモックに置換。`--nodes`は名前の列挙のみでノードを起動しない。
