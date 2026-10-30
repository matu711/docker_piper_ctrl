# 単腕・双腕MCAP収集

正式な入口は `collect_single_piper_mcap.launch` と
`collect_dual_piper_mcap.launch` の2本です。
起動例は [共通ガイド](../../../README_PiPER_guide.md) を参照してください。

## 共有構成

- `control_piper.py`：Leader・単腕Follower・双腕Followerの起動Adapter。
- `record_piper_mcap.py`：共通Recorder入口。
- `piper_recorder.py`：同期、ライフサイクル、端末入力、参照カメラワーカー。
- `piper_config.py`：アーム数・画像数・topic・次元のAdapter。
- `piper_single.py`：単腕EE購読と7次元＋EE保存Adapter。
- `piper_storage.py`：双腕保存、名前予約、atomic JSON。
- `piper_gui.py`：共通プレビュー、タスク、FPS、状態表示。
- `piper_recording_common.py`：MCAP、同期バッファ、JPEG、単腕保存Writer。
- `mcap2_camera.py`：名前は維持した共通カメラプロセス管理・時刻変換。

共通Writer・バッファの関数／クラス本体は抽出時にAST一致を確認しました。
旧収集用CLI、独立カメラループ、腕の管理Coordinatorや管理専用サービスは含みません。

## 操作と制御の境界

単腕・双腕で `t/r/s/x/q` を使用します。タスク文字列はRecorder端末へ入力します。
UTF-8入力、Enter確定、Esc取消、空入力時の以前の値保持、記録中編集拒否を維持します。
r時点のtaskをエピソードmeta.jsonへ固定し、記録中エピソードを後から書き換えません。
カメラ映像・状態表示は入力中も継続します。

RecorderはSDKを生成せず、追従制御サービスも呼びません。
rで追従再開、sで追従停止、自動姿勢復帰を行う仕様ではありません。
qやshutdown途中の記録はincompleteとして残し、カメラプロセスを閉じます。
終了・保存が物理停止を意味することはありません。

単腕Followerの方向Clip、NaN/Infinity拒否、SDK量子化、command fault、保存姿勢検証を維持します。
双腕Followerは参照方式のLeader EE待機・gripper Clamp・SDK呼び出し順序を維持し、
単腕と同じ安全処理へ勝手に置き換えません。SDK・初期化条件も変更していません。

## 保存と同期

| 内容 | 単腕 | 双腕 |
|---|---|---|
| State/Action | float32 N×7 | float32 N×14、右7→左7 |
| EE | XYZ m＋quaternion xyzw＋gripper m、N×8 | 参照の保存形式を維持 |
| 画像 | front、wrist | front、wrist1、wrist2 |
| 関節・gripper | rad・m | rad・m |

MCAP・NPZ・CSV・JPEG・metadataの既存学習用形式、単位、列、型、命名を維持します。
単腕は `piper_raw_fixed_rate_synced_v6_ee_pose`、双腕カメラありは `piper_dual_state_mcap_v2`。
カメラなし形式も維持し、存在しないJPEGは生成しません。

既定20Hz固定グリッド、sync_delay 0.2秒、State/Action許容0.03秒、待機0.025秒。
画像はtarget許容0.10秒、相互許容0.03秒、待機0.10秒、最大年齢0.5秒。
画像再利用を許し、V4L2時刻を優先、取得できなければread midpointを使います。
保存はネイティブ解像度JPEG、GUIのみ縮小します。
MCAPには同期採用行以外の元ROSサンプルも記録します。
時計後退・欠落・異常時のincomplete処理を維持します。

保存先はlaunchの `dataset_root` 引数をそのまま使い、ホームユーザー名や旧Docker mountを固定しません。
既定は単腕 `$HOME/piper_dataset`、双腕 `$HOME/piper_dataset/dual`。
VLAの画像topic配信は別機能で、収集カメラは内部取得です。

## 検証

`src/mock/test_piper_collection_reference.py`、`test_piper_collection.py`、
`test_piper_recording_common.py` で合成ROS・Fake SDK・疑似カメラ・一時保存を検証します。
実CAN・実カメラ・ROS masterを使いません。実機の動作・保持・停止能力の証明ではありません。
