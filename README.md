# PiPER ROS1 control and Unified MCAP collection

単腕・双腕のLeader–Follower教示収集、VLA Controller、CAN・カメラ・初期姿勢復帰を提供します。
正式な収集入口は次の2本です。

- `collect_single_piper_mcap_unified.launch`
- `collect_dual_piper_mcap_unified.launch`

Docker・操作・Mockテストは [PiPERガイド](README_PiPER_guide.md) を参照してください。
Policyモデルと推論Runtimeは別リポジトリ `piper_vla_runtime_2` が担当します。
学習データ、Checkpoint、認証情報、catkin生成物は含めません。

| 機能 | 資料 |
|---|---|
| Unified収集 | [仕様](double_PiPER/src/piper/README_mcap_unified.md) |
| VLA | [Controller](double_PiPER/src/piper/README_vla_dual.md) |
| 関節・ステップ制限 | [設定](double_PiPER/src/piper/README_vla_dual_motion_limits.md) |
| 初期姿勢復帰 | [手順](double_PiPER/src/piper/README_dual_initial_pose.md) |
| CPU/Mock | `double_PiPER/src/mock/` |
| 環境・catkin検証 | `double_PiPER/src/test/test_unified_docker_environment.py` |

Recorderの保存・終了はロボット停止を意味しません。
VLAのdry-runも実driverのCAN初期化を禁止する指定ではありません。

上流PiPER ROSコードのMITライセンスは [double_PiPER/LICENSE](double_PiPER/LICENSE) に保持しています。
SDKはDockerfileで固定した上流commitを導入します。外部Runtime・モデルの配布条件は別途確認してください。
