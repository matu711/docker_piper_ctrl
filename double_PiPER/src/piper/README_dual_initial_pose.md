# 初期姿勢復帰

既存の初期姿勢復帰・停止・所有権確認・FK/Z・関節/gripper検証を維持します。
対象姿勢は `config/piper_demo_start.yaml`（単腕）、`config/piper_dual_demo_start.yaml`（双腕）。
記録した姿勢であっても、現在位置からの経路や干渉が検証済みになるわけではありません。

PiPERコンテナ内で実行します。

```bash
sorc_devel
# 単腕：この実行は実機を動かす。--dry-runオプションはない。
rosrun piper go_initial_pose.py
# 別設定は _config_file:=/absolute/path/pose.yaml

# 双腕：検証のみ。ただしSDK経路ではCAN接続を行う。
rosrun piper go_dual_initial_pose.py --dry-run
# 双腕を実際に復帰させる
rosrun piper go_dual_initial_pose.py
# 別設定は --config /absolute/path/pose.yaml
```

双腕は左右Followerを同じsmoothstep進行率で補間し、指令の準備と検証を行います。
SDK直接経路と既存ROS driver経路を判別し、不明な所有権・競合を拒否します。
既存VLAの停止・HOLD・初期姿勢所有権ロックと連携し、復帰だけでVLA推論を再開しません。
単腕も既存の停止とバックエンド判定を維持します。

機器に一切触れない確認は `src/mock/test_go_initial_pose.py` と
`src/mock/test_go_dual_initial_pose.py` を使用します。
共有コードの通常回帰は個人データセットに依存しません。
初期姿勢YAMLと保存CSVの一致を別途調べる場合だけ、
`PIPER_INITIAL_POSE_REFERENCE_CSV` に対応するCSVを明示してください。
