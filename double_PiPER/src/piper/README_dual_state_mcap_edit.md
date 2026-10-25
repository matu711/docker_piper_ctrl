# 双腕State・Actionと3台のカメラの同期収集

正常動作版 `3d3f14e` の共通Followerとlaunchを変更せず、新規コピーを使用する。
Followerの起動、追従、安全処理、Stateの50Hz publishは元の処理を維持する。
追加する処理は、SDKへの指令呼び出し後の `applied_joint_cmd` の非同期publishのみ。
レコーダーは独立したROSプロセスで、ロボットのサービス呼び出し・CAN接続を行わない。
カメラ取得は既存の `mcap2_camera.CameraCapture` を再利用した専用プロセス、同期・JPEG保存はレコーダーの別スレッドで行う。
OpenCVプレビューはメインスレッドで表示する。カメラの取得・保存処理をFollowerの制御スレッドでは実行しない。

## 起動

```bash
source /opt/ros/noetic/setup.bash
source ~/double_PiPER/devel/setup.bash

roslaunch piper collect_dual_piper.launch \
  can_leader_R_port:=can0 \
  can_follower_R_port:=can1 \
  can_leader_L_port:=can2 \
  can_follower_L_port:=can3 \
  auto_enable:=true
```

指定のCAN割り当ては新しいlaunchの既定値にも設定されている。
起動時に `PiPER-Recorder` という専用xtermを開く。キー操作はこのウィンドウで行う。
ROSを実行する環境に `xterm` とXディスプレイへの接続（`DISPLAY`）が必要。
`xterm` が未導入の場合は、その環境で `sudo apt-get install xterm` を実行する。
GUIを使わずROSサービスだけで操作する場合は `recorder_launch_prefix:=""` を追加して起動する。
制御の起動条件と `auto_enable` の扱いは正常動作版のまま。起動時にモーターを有効化する処理があるため、`auto_enable=false`を読取り専用の保証として使わない。
同じCANやROS名前空間を使う別のリーダーフォロワーlaunchと同時に起動しない。

保存先はコード内で `/home/dnp218/docker_piper_env/piper_dataset` に固定する。
`dataset_root` 引数やROSパラメーターを指定しても保存先は変わらない。
既存Docker環境では、同じホストディレクトリを指すbind mount `/home/dnp218/piper_dataset` を使用する。未マウント時にコンテナ内の別ディレクトリへ保存しない。
同期設定はlaunch引数で変更できる。

```bash
roslaunch piper collect_dual_piper.launch \
  record_fps:=20 sync_tolerance:=0.03 sync_wait:=0.025 max_data_gap:=1.0
```

カメラの既定設定：

| 名前 | デバイス | 要求解像度 | FPS | FourCC |
|---|---|---|---|---|
| front | `/dev/video4` | 1280×720 | 30 | MJPG |
| wrist1 | `/dev/video2` | 640×480 | 30 | MJPG |
| wrist2 | `/dev/video0` | 640×480 | 30 | MJPG |

デバイス・解像度・FPS・FourCCはlaunch引数から変更できる。
起動時に実フレームの解像度、V4L2の報告FPS・FourCCを確認して表示し、要求値と違う場合は警告する。
取得画像は実際の解像度でJPEG/MCAPに保存する。プレビューだけ480×270に縮小して3台を横並びに表示する。
各エピソードのメタデータには要求設定・実際の設定・取得時刻から計算した実測FPSを保存する。
30FPSは取得要求値であり、USBやCPU負荷による不足は欠落として記録される。

既存のエピソード・管理用フォルダ・`dataset_meta.json` は変更しない。各エピソードのメタデータを正として扱う。
画像なし収集を使う場合：

```bash
roslaunch piper collect_dual_piper.launch \
  enable_cameras:=false
```

画像取得を維持して表示だけ無効にする場合は `preview:=false` を指定する。

## 記録操作

正常動作版と同じ手順で追従を開始する。左右のState/Actionと3台の新しい画像が揃ったら記録可能になる。
OpenCVの `PiPER dual teaching recorder` 映像ウィンドウを選択し、次のキーを押す（Enter不要）。
自動で開いた `PiPER-Recorder` のxtermからも同じキーを使用できる。
画面には `IDLE` / `RECORDING`、欠落・古い映像には `STALE / MISSING` を表示する。
カメラ取得異常時は `CAMERA ERROR` を表示し、記録中データを未完了として保持する。

| キー | 操作 |
|---|---|
| `t` | タスクプロンプト入力モード（専用xtermで入力しEnterで確定） |
| `r` | 新しいエピソードを記録開始 |
| `s` | 記録停止・保存 |
| `x` | 記録中エピソードを破棄 |
| `q` | レコーダーのみ終了。記録中データは未完了として保存 |
| 専用xtermでCtrl+C | レコーダーのみ終了。記録中データは未完了として保存 |
| roslaunch元端末でCtrl+C | launch全体を終了。記録中データは未完了として保存 |

**記録開始・停止・破棄は追従や教示モードを変更しない。** `q`でも腕の制御ノードは継続する。
初期プロンプトはlaunchの `task` 引数から取得し、専用xtermに `Current task:` として表示する。
`t`を押したら `PiPER-Recorder` のxtermを選択して文字列を入力し、Enterで確定する。日本語も入力できる。
映像ウィンドウから`t`を押すこともできるが、文字列はxtermで入力する。入力中も映像表示は継続する。
入力中の `r/s/x/q/t` は文字として扱う。Backspaceで削除、Escで入力をキャンセルできる。
Enterのみ、または空白のみの場合は以前のプロンプトを維持する。
確定後のプロンプトは次に`t`で変更するまで保持する。記録中・記録操作の処理中には変更できない。
`r`を押した時点のプロンプトをそのエピソードに固定し、保存時に `meta.json` の `task` に記録する。
ROSサービスから記録開始した場合も、開始時点のプロンプトを使用する。入力中は記録開始を受け付けない。

```bash
roslaunch piper collect_dual_piper.launch \
  auto_enable:=true task:="青い袋を持ち上げる"
```

データ読み込み時は各エピソードの `meta.json.task` を優先する。
ルートの `dataset_meta.json.task` は過去の初期値であり、データセット全体の固定タスクを意味しない。新規エピソードでは `task_scope: episode` を保存する。
同じ保存先でエピソードごとに異なるタスクを使用でき、過去のエピソードのタスクは変更しない。

端末がない場合は別端末から次のROSサービスを使う。

```bash
rosservice call /dual_state_mcap_recorder/start "{}"
rosservice call /dual_state_mcap_recorder/stop "{}"
rosservice call /dual_state_mcap_recorder/discard "{}"
```

## トピック

| 用途 | トピック | 型 |
|---|---|---|
| 右State | `/followerR/current_joint_obs` | `sensor_msgs/JointState` |
| 右Action | `/followerR/applied_joint_cmd` | `sensor_msgs/JointState` |
| 左State | `/followerL/current_joint_obs` | `sensor_msgs/JointState` |
| 左Action | `/followerL/applied_joint_cmd` | `sensor_msgs/JointState` |

各トピックは関節1〜6（rad）とグリッパー（m）の7要素。
Actionはグリッパー制限適用後、SDKの指令呼び出しが終了した値を配信する。
関節はSDKと同じ0.001度、グリッパーは1µmの整数への丸めを反映する。
SDKがモーター無効等で関節指令を送らない条件ではActionを配信しない。
SDKはCAN送信成功・実機受付を戻り値で通知しないため、ActionはハードウェアACKを保証しない。
Stateの時刻は元ノードのpublish時刻、ActionはSDK呼び出し終了後の時刻。ロボット内部の計測・実行時刻ではない。

## 同期・欠落処理

記録開始時刻から20Hz（50ms間隔）の時刻基準を作り、4ストリームの最近傍サンプルを選ぶ。
カメラ有効時は `sync_delay`（既定200ms）遅らせて基準時刻を処理し、撮影済み画像とROSサンプルが到着する時間を確保する。
サンプル到着待ちは4ストリーム合計で最大 `sync_wait`（既定25ms）。
各サンプルの時刻が基準時刻から `sync_tolerance`（既定±30ms）を超える行は保存しない。
画像は基準時刻から `camera_target_tolerance`（既定±100ms）以内、3台の取得時刻の最大差は `camera_sync_tolerance`（既定30ms）以内とする。
単腕版の `synchronized_camera_pair` と同じく、許容条件を満たす組から基準時刻に対する最大誤差が最小の組を選び、同じ場合はカメラ間の時刻差が小さい組を優先する。
有効な組がない場合は `camera_sync_wait`（既定100ms）まで新しい画像を待つ。
30FPSに満たない場合も、許容範囲内の同じ画像を複数の20Hz基準時刻で再利用できる。元の取得時刻・フレーム番号を保持し、新しい撮影として時刻を付け直さない。
保存時に `max_image_age`（既定500ms）より古い画像や欠落画像は拒否する。
画像の取得時刻には有効なV4L2フレーム時刻をUnix時刻へ変換して使い、利用できない場合はread前後の中間時刻を使う。使用方式は実設定メタデータに記録する。
片側だけの行、ゼロ埋め、許容範囲外の古い値の再利用は行わない。
四者間の最大時間差は最大60msになり得る。実際の差は `sync_span` に保存する。
State/Action・画像とも同じメッセージが複数基準時刻の許容範囲に入る場合は再利用され得る。各行の元タイムスタンプで確認できる。
カメラごとに、直前の保存行と同じフレーム番号かどうかを `*_reused`、累積の追加使用回数を `*_reuse_count` に保存する。最初の使用は再利用回数に含めない。
処理遅延時は過去のグリッドを埋めず、欠落数を残す。
欠落が `max_data_gap`（既定1秒）を超える場合、またはROS時刻が逆行した場合は未完了として記録を終了する。
全データが同期できた20Hz基準の行だけを保存するため、欠落がある場合の保存行数は毎秒20行を下回る。
記録異常によって腕の動作を変更しない。

## 保存構造

```text
/home/dnp218/docker_piper_env/piper_dataset/
  dataset_meta.json
  YYYYMMDD_HHMMSS/
    episode_<記録開始時刻ns>.mcap
    data.npz
    frames.csv
    meta.json
    front/000000.jpg ...
    wrist1/000000.jpg ...
    wrist2/000000.jpg ...
```

未完了データは `.tmp_episode_<時刻ns>/` に保持し、MCAPは `.incomplete.mcap` とする。
保存完了時の日本時間（Asia/Tokyo）でフォルダ名を決める。例：`20261015_003045`。
同じ秒に保存した場合は `_01`、`_02` の順に接尾辞を付け、既存フォルダを上書きしない。
映像画面内の `Dataset Count: XX` は、保存先直下の保存完了エピソード数。起動時と保存完了後に更新する。
日時名・旧 `episode_<時刻ns>` 名のフォルダについて、保存完了メタデータとNPZ・CSV・MCAPの存在を確認して数える。
`.tmp_episode_*`、未完了・空エピソード、管理用フォルダ、シンボリックリンク、サブフォルダ内のエピソードは含めない。既存データの移動は行わない。
記録中は `.mcap.partial`、`frames.partial.csv`、`recording_status.json` が存在する。
MCAPは元の4つのROSメッセージを元の周期で保存する。NPZ・CSVの20Hz同期行とは件数が異なる。
同期行に対応する画像は `/camera/front/compressed`、`/camera/wrist1/compressed`、`/camera/wrist2/compressed` の `sensor_msgs/CompressedImage` としてMCAPに保存する。
画像メッセージのheaderには取得時刻を付け、JPEGにはCSV/NPZで参照できる相対パスを割り当てる。

`data.npz` の構造（Nは同期できた行数）：

| キー | 形状 | 内容 |
|---|---|---|
| `observation_state` | `(N, 14)` float32 | 右関節6＋右グリッパー、左関節6＋左グリッパー |
| `action` | `(N, 14)` float32 | 同じ順序の適用指令 |
| `timestamp` | `(N,)` float64 | 開始からの基準時刻（秒） |
| `unix_timestamp` | `(N,)` float64 | 基準時刻（Unix/ROS秒） |
| `{right,left}_{state,action}_ros_timestamp` | `(N,)` float64 | 各サンプルの元header時刻 |
| `{right,left}_{state,action}_time_error` | `(N,)` float64 | 元時刻 − 基準時刻（符号付き秒） |
| `sync_span` | `(N,)` float64 | 4つの元時刻の最大値 − 最小値（秒） |
| `{front,wrist1,wrist2}_ros_timestamp` | `(N,)` float64 | 画像取得のUnix/ROS時刻 |
| `{front,wrist1,wrist2}_time_error` | `(N,)` float64 | 画像取得時刻 − 基準時刻 |
| `{front,wrist1,wrist2}_sequence` | `(N,)` int64 | 取得プロセス内のフレーム番号 |
| `{front,wrist1,wrist2}_image` | `(N,)` Unicode | JPEG相対パス |
| `{front,wrist1,wrist2}_reused` | `(N,)` bool | 直前の保存行と同じカメラフレームを使用したか |
| `{front,wrist1,wrist2}_reuse_count` | `(N,)` int64 | その行までの累積再利用回数 |
| `camera_sync_span` | `(N,)` float64 | 3台の画像取得時刻の最大差 |
| `all_sync_span` | `(N,)` float64 | State/Action4時刻＋画像3時刻の最大差 |

CSVにも14次元のState/Action、各時刻・誤差、画像パス・フレーム番号、再利用フラグ・回数を保存する。
エピソードの `meta.json` に `reused_front_frames`、`reused_wrist1_frames`、`reused_wrist2_frames` として総再利用回数を保存する。
メタデータに軸順序・単位・FPS・トピック・同期設定・欠落数・保存状態を残す。
単腕版の `observation_state`・`action`・`timestamp`・`unix_timestamp` と保存形式を維持し、次元数と左右別の同期情報を拡張している。
同じ固定保存先で収集を継続できる。ルートに既存の `dataset_meta.json` がある場合は保持し、過去のエピソードやそのメタデータは変更しない。
タスク・カメラ設定・FPS等は各エピソードの `meta.json` を参照する。

## 検証

実機・CAN・実カメラを使用しない模擬テスト：

```bash
python3 ~/double_PiPER/src/test/test_dual_state_mcap_edit.py
```

実機起動・実カメラ取得・追従中の負荷確認は未実施。
