# 双腕PiPER VLA制御

`start_vla_dual_piper.launch` は、Leaderなしで2台の既存
`piper_ctrl_single_node.py`、双腕controller、3カメラpublisherを起動する。
policy server・推論runtimeは単腕と同様に別起動する。
単腕・教示収集コードとruntimeは変更しない。
通常操作はlaunch起動→Runtime起動→prompt付きstart→stop。手動HOLD解除や種別topic配信は不要。

## 入出力

| 種類 | topic / service | 内容 |
|---|---|---|
| Action入力 | `/vla_dual/approved_action` | `Float64MultiArray`、厳密に14要素、左7→右7 |
| Action種別 | `~action_type` parameter | `joint_absolute`固定。種別topicを必要としない |
| HOLD入力 | `/vla_dual/hold` | `Bool`、trueで停止要求。初期HOLD・fault解除には開始service応答が必要 |
| 左指令 | `/followerL/joint_states` | 既存driverの`joint_ctrl_single`へremap |
| 右指令 | `/followerR/joint_states` | 同上 |
| 左State | `/followerL/joint_states_single` | 6関節rad＋gripper開幅m |
| 右State | `/followerR/joint_states_single` | 同上 |
| 結合State | `/vla/dual_joint_states` | `JointState`、左7→右7 |
| 画像 | `/camera/{front,left_wrist,right_wrist}/compressed` | `CompressedImage`、JPEG |
| 診断 | `/safety/{left,right}/{state,reason,target_z}` | 各腕の既存安全判定結果 |
| 全体診断 | `/safety/{state,reason}` | 同一stepの結果 |
| 目標診断 | `/vla_dual/{left,right}/target_joint_command` | Clamp後の各腕目標 |
| Controller異常 | `/vla_dual/controller_fault` | `std_msgs/String`、Action検証失敗・送信例外・明示停止をRuntimeへ通知（非latched） |
| Runtime開始 | `/piper_vla_dual_runtime/start` | `piper_vla_runtime_msgs/StartInference`、`string prompt` |
| Runtime停止 | `/piper_vla_dual_runtime/stop` | `std_srvs/Trigger` |
| Controller開始確認 | `/vla_dual/controller_start` | `std_srvs/Trigger`、HOLD解除処理後に新しい32桁hex tokenを応答。Runtime内部で呼ぶ |
| Controller停止 | `/vla_dual_piper_controller/stop` | `std_srvs/Trigger`、両腕HOLD要求 |
| Driver停止 | `/followerL/stop_srv`、`/followerR/stop_srv` | 各既存driverの停止service |

Actionは復元済み絶対値
`[left_j1..j6, left_gripper, right_j1..j6, right_gripper]`。
関節rad、gripper m。PolicyのDelta復元は行わない。
絶対関節目標を現在Follower Stateの±`max_delta`に制限する。

結合Stateは両腕の最新値を結合する。厳密な時刻同期ではなく、headerには古い側の
stampを使う。各腕の元のstampやEE poseが必要なruntimeは、左右driverのtopicを直接購読する。
画像は単腕同様に各カメラの最新フレームを配信し、画像の同期・回転・モデル入力変換は行わない。

## 単腕と同じ安全処理

`StagedArm` は `VlaPiperController` を継承し、関節可動範囲とgripperの既存検証を再利用する。

1. Actionの有限値検査（各7要素）
2. `_apply_joint_saturation` による公式joint limit、margin内Clamp、margin外拒否
3. 現在の左右Follower Stateを基準に12関節の変化量Clip
4. 既存の関節範囲検証を再適用し、`_official_target_joint_z`による公式SDK FK
   （変化量Clip後の関節を使用。双腕側の薄いwrapperで非有限Zも拒否）
5. Z下限判定（`z_enabled`、`z_min`）
6. `_gripper_action_to_width_m` のphysical absolute範囲判定・Clamp
7. `JointState`生成、目標診断、dry-runまたは実指令送信

既存 `_send_joint_target` がこの順序を管理する。双腕版では最後の実指令送信だけを
保留し、左右の検証完了後に既存 `_publish_or_dry_run` で送る。
片腕の検証が失敗したstepは両腕とも送信せず、両腕にHOLD診断を出し、保留指令を破棄する。
Action検証失敗も `controller_fault` に通知し、両腕のfaultをラッチする。
Runtimeはchunkを破棄する。fault後は正常ActionやHOLD=falseでも再開しない。
既存の関節・gripperの閾値とmargin内Clamp／大幅超過拒否は単腕のまま維持する。
その前後に双腕専用のState確認と変化量Clipを追加している。
再開は通常どおりprompt付きRuntime startを呼ぶ。

送信例外では以後の腕への送信を中止するが、既にpublishした指令は取り消せない。
正常な左腕の目標診断が右腕の拒否より先に出る場合があるが、これは実指令ではない。

| 設定 | Python単体既定値 | launch既定値（単腕launchと同一） |
|---|---|---|
| `z_min` | 0.01 m | 0.01 m |
| `z_enabled` | true | true |
| `joint_saturation_margin_rad` | 0.05 rad | 0.1 rad |
| `gripper_closed_width_m` | 0.0 m | 0.0 m |
| `gripper_open_width_m` | 0.10 m | 0.10 m |
| `gripper_saturation_margin_m` | 0.01 m | 0.01 m |
| `dry_run` | true | true |

margin境界は包含し、その内側では物理範囲にClampする。
大幅な可動範囲超過は変化量Clipで隠さず、その前に両腕とも拒否する。
双腕間衝突判定は追加していない。
Action形式は双腕契約に合わせ14要素へ限定し、種別はjoint_absolute、gripperはphysical_absolute固定。

HOLDまたはController停止serviceは新規指令を止める処理であり、既存driverの停止serviceは
自動では呼ばない。通常はRuntimeのstopを使い、再開時はprompt付きstartを使う。
Runtimeは開始後の新しい観測・最初のchunkを待ってから、非同期に
`/vla_dual/controller_start`を呼ぶ。Controllerはロック内でfault解除・HOLD解除・
新token生成を処理し、その後で応答する。Runtimeはこの応答を確認するまでchunkを消費せず、
Actionを送信しない。固定sleepでの順序保証は行わない。
Actionは`Float64MultiArray.layout.dim[0].label`に
`controller_session:<token>`（size/stride=14）を付ける。
Controllerは現在のtokenと一致するActionだけを受け付けるため、再start後に届いた古いActionは無視する。
このservice応答がHOLD解除の確認であり、Bool topicの配信順には依存しない。
`/vla_dual/hold`は停止要求として維持するが、falseだけでは初期HOLD・faultを解除できない。
最新Runtimeは再開時にこのBoolをfalseへ更新しないため、Bool単体を実際の解除確認に使わない。
Controller停止serviceはRuntimeにも通知する補助経路であり、通常操作に追加で呼ぶ必要はない。

左右のROS publishは順次行う。送信前検証は両腕共通のstepとして扱うが、別CANへの
物理送信の同時性や通信障害時のロールバックは保証しない。

## SDKソフトウェア関節制限の切替

双腕VLA launchの`start_sdk_joint_limit`引数は既定`true`。
左右driverの同名private bool parameterから、公式SDKの
`C_PiperInterface(can_name=..., start_sdk_joint_limit=...)`へ渡す。
各driverはSDK生成前にCAN名と設定値をINFOログへ出す。変更は再起動時に反映される。

```bash
# 有効（既定）
roslaunch piper start_vla_dual_piper.launch start_sdk_joint_limit:=true
# SDKのソフトウェア関節制限のみ無効
roslaunch piper start_vla_dual_piper.launch start_sdk_joint_limit:=false
```

上記は使用方法の記載で、本変更の検証では起動していない。
後述の`max_delta_left/right`の事前設定など、既存の起動条件は引き続き必要。
共通driver `piper_ctrl_single_node.py`のprivate parameterの既定値もtrueなので、
これを使う他のlaunchでparameterを省略した場合も有効になる。

公式[SDKの説明](https://github.com/agilexrobotics/piper_sdk#3-simple-start)では、
この設定は制御指令とフィードバック双方のSDKソフトウェア関節制限に作用する。
SDKのgripper制限やfirmware設定は変更しない。
`start_sdk_joint_limit=false`だけではControllerの範囲検査は無効にならない。
Controller側も無効にする場合は次の引数を併用する。

```bash
roslaunch piper start_vla_dual_piper.launch \
  start_sdk_joint_limit:=false \
  controller_joint_limit_enabled:=false \
  dry_run:=true
```

`controller_joint_limit_enabled`は既定true。falseでは左右ともControllerの
SDK由来の関節範囲Clamp／margin外拒否、および実測StateのSDK関節範囲検査をスキップする。
起動時に左右の設定値をログ出力する。未設定時は従来と同じ範囲判定を行う。
12関節の変化量Clip、有限値・次元、State鮮度・CAN受信確認、FK/Z、gripper、
HOLD・faultラッチ・dry-runは両設定をfalseにしても維持する。
機械的・firmware側の制限を解除する操作ではない。
`max_delta_left/right`の事前設定は引き続き必須で、上記コマンドも実行していない。
指定した引数に未対応のSDKでは起動エラーとなり、設定を黙って無視するfallbackは行わない。
実行環境のSDKを用いた確認と実機確認は未実施。Mockで引数の伝達を検証している。

## 12関節の変化量Clip設定（必須）

`~max_delta_left`と`~max_delta_right`は、それぞれjoint1～6の**6個の正の有限値**を
rad/stepで指定する。未設定・長さ不正・0以下・NaN/Infの場合はController起動を拒否する。
gripperはこのClipの対象外で、既存の範囲検査を維持する。

20 Hzの各Actionで、SDK公式範囲にClamp済みの目標を使い、各関節について
`current + min(max_delta, max(-max_delta, target-current))`を計算する。
Pythonのmin/maxは要求された`np.clip`と同じ演算であり、追加のNumPy依存はない。
Clipは前回指令値ではなく実測Follower Stateが基準で、Policyや学習Transformは変更しない。
Runtimeの20 Hzは変更しない。この処理自体は20 Hzの送信timerや物理速度制御ではなく、
各目標と現在角の差を制限するもので、実際の速度・加速度や衝突回避を保証しない。

速度値をControllerで推測して既定値にすることはしない。
公式[MoveIt資料](https://github.com/agilexrobotics/piper_ros/blob/noetic/src/piper_moveit/README%28EN%29.md)
の表は全6関節3 rad/sだが、公式
[URDF](https://github.com/agilexrobotics/piper_ros/blob/noetic/src/piper_description/urdf/piper_description.urdf)
とこのcheckoutのURDFはjoint1～5が5 rad/s、joint6が3 rad/sで、記載に差がある。
そのため、使用機種・firmware・SDKで確認した各関節の許容速度`v_j`と、運用上認めた速度から
`max_delta_j <= v_j / 20`となる値を決め、左右それぞれ明示設定すること。
SDKの速度百分率をそのままrad/stepとして使わない。テスト内の数値はMock用で実機用設定ではない。

起動前に、確定した左右6値を含む自分のYAMLをROS parameter serverへロードする。
既存launchにはこの値を上書きするparamがないため、launchファイルの変更は不要。

```bash
# 内容: max_delta_left: [確定済み6値] / max_delta_right: [確定済み6値]
# このコマンド例は手順の記載のみ。本変更の検証ではROSを起動・操作していない。
rosparam load /absolute/path/commissioned_joint_clip.yaml /vla_dual_piper_controller
```

`~state_timeout_s`は既定0.5秒（現在のRuntimeのobservation timeoutと同じ）。
両腕とも`JointState`のjoint1～6/gripper名・7要素・有限値・有効なheader stampが必要。
名前順は正規化し、受信した値はコピーする。実測関節がSDK可動範囲外なら基準値として使わずHOLD。
source stampとローカルmonotonic受信時刻の両方で鮮度を確認し、欠落・古い状態・未来時刻・
時刻巻戻り・不正値は両腕HOLDとcontroller_fault通知にする。
20 Hz watchdogでも確認するため、新しいActionが来なくても古いStateを検出する。
復旧後も明示startまでは指令を再開しない。

`JOINT_CLIP`ログに左右ごとに`current`、`raw`、`range_clipped`、`step_clipped`、
`max_delta`を出す。これは送信候補の診断で、実際の送信成功は既存OK/DRY_RUN診断で確認する。
`dry_run=true`でも全Clipと検証を実行し、`target_joint_command`はClip後の値になるが
driverの`joint_states`へは送らない。実CANなしの検証は下記Mockテストを使用する。

## 起動手順（この変更の検証では未実行）

既存PiPER DockerとROS master、CAN設定、catkin環境を単腕と同じ手順で準備する。
左右CANとカメラの対応は実配線に合わせる。物理的な左右は自動判定しない。

### 1. PiPER Docker内：ROS側を起動

```bash
source /opt/ros/noetic/setup.bash
source ~/double_PiPER/devel/setup.bash
roslaunch piper start_vla_dual_piper.launch \
  left_can_port:=can3 right_can_port:=can1 \
  front_camera:=/dev/video4 \
  left_wrist_camera:=/dev/video2 right_wrist_camera:=/dev/video0 \
  dry_run:=true
```

frontはYUYV、wristはMJPG、取得30 FPS・配信20 Hz・640×480が既定。
カメラのFourCC・取得FPSは同名launch引数で指定できる。

### 2. ホスト側：双腕Runtimeを起動

```bash
cd /home/dnp218/piper_vla_runtime
./scripts/run_dual.sh --checkpoint ~/piper_checkpoints/current_dual --dry-run
```

Runtime dry-runでは推論とchunk選択まで行い、approved_actionとHOLD解除は送らない。
Controllerの安全判定・目標診断まで確認する場合は、ROS側の `dry_run:=true` を維持し、
Runtimeだけを次のモードで起動する。

```bash
./scripts/run_dual.sh --checkpoint ~/piper_checkpoints/current_dual --controller-run
```

`--controller-run` はRuntime側のdry_runをfalseにする既存オプション。
ROS側controllerがdry_runなら、左右driverへの関節指令は送信しない。
本番の実機起動は単腕と同じくROS側を `dry_run:=false`（auto_enable既定true）にし、
Runtimeは `--controller-run` を使う。新しいenable操作や手動HOLD解除は不要。

### 3. PiPER Docker内：promptを指定して開始・停止

独自service型をビルド済みのcatkin環境をsourceしたterminalで呼ぶ。
`start`と`stop`はtopicではなくROS service。

```bash
rosservice call /piper_vla_dual_runtime/start \
  "prompt: 'Assemble the box using both arms.'"

rosservice call /piper_vla_dual_runtime/stop '{}'

# 再開・prompt切替も同じstartを使用する。
rosservice call /piper_vla_dual_runtime/start \
  "prompt: 'Hold the box with the left arm and fold it with the right arm.'"
```

start型は `piper_vla_runtime_msgs/StartInference`：
`string prompt` → `bool success, string message`。
stop型は `std_srvs/Trigger`：空request → `bool success, string message`。
Runtimeの実コードのnode名 `piper_vla_dual_runtime`、private service `~start` / `~stop`
と、共有 `StartInference.srv` 定義で確認している。

stopはRuntimeのchunk・遅延応答を無効化し、HOLD=trueで両腕の新規指令を抑止する。
既にdriverへ渡った目標を取り消す処理ではない。
ハードウェア停止が必要な場合の既存serviceは `/followerL/stop_srv` と `/followerR/stop_srv`。
これらは通常のstart/stop操作に追加する必須手順ではない。

### dry_runとCAN通信

dry_run時はlaunchが両driverへの `auto_enable` をfalseにする。
`auto_enable:=true` を明示してもdry_run中は自動enableしない。
単腕と同じdriverを使うため、実driverの初期化では `ConnectPort()` と
`MotionCtrl_2(0x01,0x01,30,0)` が実行される。dry_runはCAN無通信を意味しない。
また、別publisher・enable service等からの外部指令を一括遮断する設定ではない。
単腕・教示launchと同じCANを同時に所有させない。

完全にデバイス通信を行わないROS構成にはMock driverを選ぶ。

```bash
roslaunch piper start_vla_dual_piper.launch mock_driver:=true dry_run:=true
```

この指定では実driverを起動せず、カメラも既定で起動しない。
Mock driverはSDK・CANをimportせず、JointStateを模擬する。
これはROS制御側の検証構成であり、Runtime推論のstartには別途3画像の合成入力と
Mock policyが必要。下記単体テストはそれらの境界をMock化し、ROS自体も起動しない。

## Mock検証

ROS・CAN・カメラ・モデルを起動しない。

```bash
python3 -B ~/double_PiPER/src/test/test_vla_dual_piper_controller.py
python3 -B ~/double_PiPER/src/test/test_vla_piper_controller.py
```

双腕テストは左右分配、両腕検証前の未送信、片腕拒否時の両腕未送信、全関節とgripperの
境界・非有限値を単腕と比較し、Clamp後FK・判定順序・HOLD・stop・dry-run・結合State・
3画像publisher・launch既定値の一致を確認する。
さらに隣接するRuntimeの実コードからtick/stop/fault処理を読み込み、Mock接続で
20 Hz実行1step相当の処理・chunk消費・prefetch・HOLD・fault連携を検証する。
service応答をEventで保留した間の未送信、fault後のHOLD=falseによる再開拒否、
明示startでの再開、旧token付きActionの拒否を確認する。
Runtimeのtopic・型・State/画像設定も読み取り照合する。実測20 Hz・実SDK FK・ROS通信・実機動作は未確認。

## CAN受信進行の監視

双腕launchの実driverは`piper_ctrl_dual_feedback_node.py`を使用する。
単腕Driverは変更せず、SDK callbackでjoint12/34/56・gripper・statusの実受信を追跡する。
全種類が更新されるまで古いJointStateを再publishせず、source stampは実受信時刻に基づく。
左右`feedback_health`（String JSON）のcount/source_stamp/ageをControllerが監視し、
片腕停止・部分停止・通信異常・Driver restartでは両腕HOLDとcontroller_faultをラッチする。
heartbeatやROS現在時刻だけでは受信期限を更新しない。復旧後は明示startが必要。
`state_timeout_s`（既定0.5秒）はDriverのfeedback_timeout_sとControllerへ共通に渡す。
State/Actionの形式・単位と既存の安全制限は維持する。旧driverとの組合せは受信証明がないためstartを拒否する。

Mockテスト：`test_dual_feedback.py`、`test_vla_dual_piper_controller.py`。
実機・CAN停止試験は未実施。HOLDは既存の送信抑止経路であり、CAN断中の物理停止保証ではない。
詳細はRuntime側`validation/can_feedback/README.md`を参照。
