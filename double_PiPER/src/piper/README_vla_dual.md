# 単腕・双腕VLA Controller

起動手順は [共通ガイド](../../../README_PiPER_guide.md) を参照してください。
PolicyとROS Runtimeは別リポジトリ `piper_vla_runtime_2`、ロボット側はこのリポジトリで動きます。
単腕は `start_vla_piper.launch`、双腕は `start_vla_dual_piper.launch`。

## 双腕の通信契約

- `/vla_dual/approved_action`：Float64MultiArray、左7→右7、関節rad、gripper開幅m。
- `/followerL/joint_states_single`、`/followerR/joint_states_single`：実測State。
- `/camera/front/compressed`、`/camera/left_wrist/compressed`、`/camera/right_wrist/compressed`：JPEG。
- `/vla_dual/controller_start`：Runtimeが呼び、新しいsession tokenを受け取る。
- `/vla_dual/controller_status`：20Hz、HOLD/RUNNING/FAULT、token、source stamp。
- Actionのlayout labelは `controller_session:<token>`。古い・不一致のActionは採用しない。
- `/vla_dual/left/target_joint_command`、`/vla_dual/right/target_joint_command`：検証後の目標診断。
- `/vla_dual/hold`：停止要求。falseだけで初期HOLDやFAULTを解除しない。
- `/piper_vla_dual_runtime/start`：prompt付きStartInference。
- `/piper_vla_dual_runtime/stop`：Trigger。通常の推論停止入口。
- `/vla_dual_piper_controller/stop`：補助停止入口。

Runtimeは初回chunk取得後にController開始を要求し、ACKと新鮮なRUNNING status/tokenを確認します。
手動Startやtoken検証の迂回は不要です。

## 安全判定とdry-run

双腕は `vla_piper_controller.py` の単腕安全処理を継承し、左右とも検証してから送信します。
14要素・有限値、角度範囲/margin、実測Stateを基準とするmax_delta、公式SDK FK/Z、gripperを確認します。
Stateのsource stamp・ローカル受信経過時間・実CAN受信進行も検証します。
異常時は両腕FAULT/HOLDとRuntimeへの通知を行い、明示的な再開始まで再開しません。
左右のROS publishは順次であり、物理同時性や送信済み指令の撤回は保証しません。

`start_sdk_joint_limit` と `controller_joint_limit_enabled` は双腕launch既定true。
設定と数値は [制限の説明](README_vla_dual_motion_limits.md) を参照してください。
max_deltaは `config/vla_dual_motion_limits.yaml` の左右各6値、既定0.1rad/step。
FK/Z、gripper、token/statusは角度制限falseでも残ります。

Runtime dry-runではselected_actionまで生成し、approved_actionとController Startは送りません。
Controller dry-runでは受信後の検証・target診断まで行い、driverへ指令を送りません。
両者は独立した設定です。実driverのCAN初期化はController dry-runでも起き得ます。

完全な機器なし構成は双腕の `mock_driver:=true dry_run:=true enable_cameras:=false`。
Runtime Startには別途3画像入力が必要です。
単腕Controller内の既存Action形式・PBL向け処理は維持しますが、
外部Runtimeの新Checkpoint経路は関節Action契約を対象とし、EE系モデルはその契約を別途確認します。

## 回帰テスト

`src/mock/test_vla_piper_controller.py`、`test_vla_dual_piper_controller.py`、
`test_vla_dual_launch_parameters.py`、`test_dual_feedback.py` で検証します。
外部RuntimeとのMock連携は `PIPER_VLA_RUNTIME_DIR` でソースの所在を明示した場合に実行します。
