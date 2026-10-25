# 双腕の初期姿勢復帰

`scripts/go_dual_initial_pose.py` は、左右Followerを同じsmoothstepの進行率で
関節空間補間する独立スクリプトです。VLA launchへの追加は不要です。
単腕用スクリプト・単腕YAML・Runtimeは変更していません。

## 実行

既存の単腕復帰と同じ、ROS Noetic・`piper_sdk`・PyYAMLを利用できるPython環境で
実行してください。ROS driver経由でも公式関節制限とFKの計算にSDKが必要です。
ROS masterがない場合も、Pythonから`rospy`と`rosgraph`をimportできる必要があります。

```bash
cd /home/dnp218/docker_piper_env/double_PiPER/src/piper/scripts

# 現在状態を取得し、左右の全補間点を検証する（動作指令なし）
python3 go_dual_initial_pose.py --dry-run

# モードを自動判定して実際に復帰する
python3 go_dual_initial_pose.py

# 別の設定ファイルを使用する
python3 go_dual_initial_pose.py --config /absolute/path/piper_dual_demo_start.yaml
```

**`--dry-run`はCAN通信を禁止するオプションではありません。** SDK方式では左右CANに
接続し現在状態を読みます。ただしenable・mode切替・関節/gripper動作・非常停止は
送りません。ROS方式では既存driverの状態を読み、関節指令をpublishしません。
どちらも稼働中のVLAは停止・HOLDにします。実CANに触れない検証には末尾のMockテストを
使用してください。本実装時にROS起動・CAN接続・実機動作は実施していません。

## 設定値と根拠

既定ファイルは`config/piper_dual_demo_start.yaml`です。
`targets.left/right`に`joint1`～`joint6`（rad）と`gripper`（全開口幅m）を設定します。
角度の左右反転、Delta復元、Clampは行いません。

既定目標は、収集済みデータの
`piper_dataset/dnp1_20261015/20261015_153856/frames.csv`、`frame_index=0`の
名前付き`state_left_*` / `state_right_*`列から採用しました。LeaderのActionではありません。
左gripperは約0.0525 m、右は約0.0001 mです。
**収録開始時の姿勢という根拠であり、任意の現在姿勢から安全に戻れる保証ではありません。**
把持物や設置位置が異なる場合は、そのまま実行しないでください。

| 設定 | 既定値 | 内容 |
|---|---|---|
| `can_ports.left/right` | `can3` / `can1` | 既存双腕VLA launchと一致 |
| `move_duration` / `command_rate` | 3秒 / 30 Hz | 90点を左右共通の時刻基準で送信 |
| `z_enabled` / `z_min` | true / 0.01 m | 各腕のFK基準座標における手先Z下限 |
| `gripper_max_width_m` | 0.10 m | 単腕復帰と同じ厳密な範囲検査 |
| `dh_is_offset` | 1 | 単腕と同じFK設定（0/1） |
| `feedback_timeout` | 1秒 | フィードバック鮮度と初回受信の制限 |
| `arrival_timeout` | 10秒 | 軌道送信後の両腕到達待ち |
| `joint_tolerance_rad` / `gripper_tolerance_m` | 0.02 rad / 0.002 m | 到達判定 |
| `sdk_enable_timeout` | 10秒 | 左右のenable・CAN mode準備の共通期限 |
| `direct_speed_percent` | 10 | 単腕と同じSDK速度指定、ROS JointStateのvelocity[6]にも設定 |
| `feedback_sync_tolerance` / `command_sync_tolerance` | 0.03秒 / 0.03秒 | 左右状態時刻差と同一stepの指令送出時間幅 |
| `driver_detection_timeout` / `service_timeout` | 2秒 / 3秒 | driver不在観測期間 / サービス応答期限 |

関節制限値はSDKの`C_PiperParamManager`から取得します。
単腕の`validate_pose_limits()`と`GoInitialPose._make_trajectory()`をそのまま呼び出し、
左右の全補間点が有限値・関節範囲・gripper範囲・FK Z検査を通るまで両腕とも指令を送りません。
SDK方式でも両腕の軌道検証が済むまではenableしません。

## ROS／SDK切替と所有権

| 検出結果 | 動作 |
|---|---|
| 左右ともfeedback publisherとcommand subscriberがそれぞれ同じ一意なdriverで、各`stop_srv`も同じnodeが提供 | ROS方式。private `can_port`がYAMLと一致することも確認 |
| 両腕のdriverの証拠がなく、所有権情報を取得できる | SDK方式。ROS masterなしでも利用可能 |
| 片腕のみ、route不完全、複数driver、CAN設定不一致・不明 | 中止。SDKへfallbackしない |
| ROS master消失・SDK実行途中でmaster出現・driver所有者変更 | 中止 |

ROS方式は以下の`JointState`を使用します。

| 腕 | 状態 | 指令 | 異常停止サービス（Trigger） |
|---|---|---|---|
| 左 | `/followerL/joint_states_single` | `/followerL/joint_states` | `/followerL/stop_srv` |
| 右 | `/followerR/joint_states_single` | `/followerR/joint_states` | `/followerR/stop_srv` |

SDK接続前に、現在のnetwork namespaceの`/proc/net/can/rcvlist_*`を調べ、対象CANまたは
`any`の既存receiverがあれば中止します。情報を取得できない場合も中止します。
CANごとの`flock`により、この復帰スクリプト同士の同時起動を拒否します。
**送信専用SDKクライアントや別network namespaceの所有者などは、この検査では検出できません。**
外部SDK・教示収集・別コンテナ等のCAN利用プロセスを停止し、所有権が不明なら実行しないでください。
本スクリプトの検査はシステム全体のCAN排他保証ではありません。

## VLA排他・停止・HOLD

ROS masterがある場合は、次の確認応答を使います。固定sleepによる順序保証はしません。

1. `/vla_dual/hold`（Bool）へtrueをpublishし、Runtimeの
   `/piper_vla_dual_runtime/stop`（Trigger）が存在すれば呼び出し、成功応答を確認。
2. Controllerが存在する場合は`/vla_dual/initial_pose_lock`（SetBool、data=true）を取得。
   Controllerはfault/HOLDをラッチし、復帰中の`/vla_dual/controller_start`を拒否。
   続いて`/vla_dual_piper_controller/stop`（Trigger）の成功応答を確認。
3. 復帰中もController所有者・driver route・未知のcommand publisherを検査。
   `/vla_dual/approved_action`を受信した場合は中止。
   停止途中の古いActionを受信して中止した場合は、停止を確認してから再試行。
4. 終了時はlockを解除するがControllerのfault/HOLDは解除しない。
   Runtimeのstartは呼ばず、推論は自動再開しない。

古いControllerに排他サービスがない場合は復帰を拒否します。更新後の双腕Controllerを
次回起動時に使用してください。通常のVLA start/stop手順への追加操作は不要です。
スクリプトを強制終了してlockが残った場合は、復帰プロセスが終了しdriverが安全に停止している
ことを確認するまでControllerを再開しないでください。自動でlockを解除するwatchdogはありません。

両腕の鮮度・時刻差を毎step確認し、期限超過時には追い付き送信をせず中止します。
片腕の通信失敗、状態異常、左右送出時間差超過、到達timeoutでは、以降の両腕送信を中止し、
ROSでは左右`stop_srv`を並行して呼び、SDKでは両腕へ`EmergencyStop(0x01)`を試みます。
片腕の停止送信が失敗しても他方の停止を試み、停止失敗はログに出します。
通信断時の物理停止はソフトウェアだけでは保証できません。

成功時は最終目標を保持し、torqueを落とすdisableはしません。
異常停止後のdriver resetや装置の非常停止解除は自動実施しません。
ROS ControllerのHOLDは「新規指令を送らない」状態であり、機械的なブレーキ保証ではありません。

## 未検証の安全項目

- 既存`piper_pinocchio/piper_pinocchio.py::check_self_collision()`は単腕モデル用です。
  左右の取付け変換を含む校正済み双腕collision modelがないため組み込んでいません。
  双腕間、机、把持物、周辺設備との干渉は未検証です。
- 30 msの送出時間幅を検査しますが、別CANへの送信は逐次です。
  左指令直後に異常が出た場合、その左指令は取り消せません。両腕の厳密な同時適用は保証しません。
- FK・関節範囲・到達判定の実機確認、CAN停止の物理的な有効性は未検証です。

## 実CAN・ROS不要のMockテスト

リポジトリのルートから実行します。ROS/SDKを偽物に差し替え、ROSノードやCANを起動しません。

```bash
python3 double_PiPER/src/test/test_go_dual_initial_pose.py
python3 double_PiPER/src/test/test_vla_dual_piper_controller.py
python3 double_PiPER/src/test/test_vla_piper_controller.py
```

復帰テスト18件、双腕Controller23件、単腕Controller32件を確認済みです。
ROS/SDK、master有無、片腕起動、CAN不一致、所有権不明、全軌道検証、dry-run、
左右分配、通信断、状態停止、到達timeout、遅延Action、排他と明示startによる再開を検証します。
既存の`test_go_initial_pose.py`は9件中8件成功、1件は既存単腕YAMLと古い期待値の不一致です。
単腕ファイルは変更していません。
