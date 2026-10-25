# 双腕PiPER π0.5 推論用 README

本READMEでは、学習済みπ0.5モデルから推論用Bundleを作成し、双腕PiPERでの推論に使用するまでの手順を説明する。

## 前提条件

- 双腕PiPERのπ0.5 Fine-tuningを実行し、使用したいステップのCheckpointが保存されている。
- 学習用PC（`rlsv`）のDockerコンテナ内に入っている。
- 第1〜2章は学習用PCのDockerコンテナ内、第3章は学習用PCのホスト側、第4〜5章は推論用PC（dnp218）で実行する。

## 1. 学習済みCheckpointの確認

### 1.1 Checkpointの保存先

今回の双腕PiPER学習のCheckpointは、以下のディレクトリに保存される。

```text
/workspace/checkpoints/pi05_piper_bimanual_delta_lora/dnp1_20261015_bimanual_pi05_delta/
```

※ 実際の保存先は、学習時に使用した設定によって異なる。

### 1.2 保存済みCheckpointの確認

```bash
ls -lh /workspace/checkpoints/pi05_piper_bimanual_delta_lora/dnp1_20261015_bimanual_pi05_delta/
```

保存済みのステップ番号を確認し、推論に使用するCheckpointを決める。

### 1.3 使用するステップの確認

例えば、`5000` ステップを使用する場合は次を実行する。

```bash
ls -lh /workspace/checkpoints/pi05_piper_bimanual_delta_lora/dnp1_20261015_bimanual_pi05_delta/5000/
```

`params` などのモデルパラメータが保存されていることを確認する。存在しないステップは指定しない。

## 2. 推論用Bundleの作成

学習済みCheckpointを推論用PCで利用できるように、Bundleとして出力する。

### 2.1 Bundle生成スクリプト

```text
/workspace/piper_pi0/training/scripts/export_bimanual_bundle.py
```

### 2.2 事前チェック（任意）

以下の例では、`5000` ステップを対象に、Bundleの出力を行わず事前チェックする。

```bash
PYTHONPATH=/workspace uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/training/scripts/export_bimanual_bundle.py \
  export-existing --step 5000 --check-only
```

### 2.3 Bundleの作成

以下のコマンドを実行する。

```bash
PYTHONPATH=/workspace uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/training/scripts/export_bimanual_bundle.py \
  export-existing \
  --step 5000 \
  --output /workspace/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000
```

- `--step 5000`：Bundle化するCheckpointのステップ番号
- `--output`：Bundleの保存先ディレクトリ
- `export-existing`：保存済みCheckpointからBundleを作成するモード

※ `5000` は実行例。第1章で確認した実在するステップに変更すること。上記コマンドは、この学習runの既定保存先からCheckpointを参照する。別のrunを使う場合は、`--run-dir` または `--checkpoint` により参照先を明示する。

### 2.4 作成したBundleの確認

```bash
ls -lh /workspace/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000/
```

Bundleには、モデルパラメータ、正規化統計、学習・推論設定、`manifest.yaml`、`validation.json`、`checksums.sha256` などが含まれる。

ファイルの整合性を確認する。

```bash
cd /workspace/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000
sha256sum -c checksums.sha256
```

すべて `OK` となることを確認する。検証に失敗したBundleは、推論用PCへ転送しない。


## 3. 推論用PCへのBundle転送

作成したBundleを、学習用PC（`rlsv`）から推論用PC（`dnp218`）へSCPで転送する。

### 3.1 学習用PCのホスト側に戻る

Dockerコンテナ内で次を実行して、学習用PCのホスト側へ戻る。

```bash
exit
```

以降は**学習用PC（`rlsv`）のホスト側ターミナル**で実行する。

### 3.2 ホスト側でBundleを確認する

今回の環境では、コンテナ内で作成したBundleを以下のホスト側パスから直接参照できる。

```text
/home/rlsv/project/piper_pi0/logs/checkpoint_exports/
```

以下のコマンドで転送対象のBundleが存在することを確認する。

```bash
ls -lh /home/rlsv/project/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000/
```

この環境では **`docker cp` を実行する必要はない**。

### 3.3 SCPで推論用PCに転送する

転送先ディレクトリが未作成の場合は、まず作成する。

```bash
ssh dnp218@163.221.133.164 'mkdir -p /home/dnp218/piper_checkpoints'
```

学習用PCの**ホスト側ターミナル**で、以下のコマンドを実行する。

```bash
scp -r \
  /home/rlsv/project/piper_pi0/logs/checkpoint_exports/dnp1_20261015_bimanual_pi05_delta_5000 \
  dnp218@163.221.133.164:/home/dnp218/piper_checkpoints/
```

※ Bundle名は使用するCheckpointのステップに応じて変更すること。

### 3.4 転送結果の確認

推論用PCに接続する。

```bash
ssh dnp218@163.221.133.164
```

転送されたBundleを確認する。

```bash
ls -lh /home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_5000/
```

チェックサムを再検証する。

```bash
cd /home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_5000
sha256sum -c checksums.sha256
```

すべて `OK` と表示されれば、転送は完了となる。

※ `dnp1_20261015_bimanual_pi05_delta_5000` は例であり、実際に作成したBundle名に置き換える。Bundle内部のファイルやフォルダ構成は変更しないこと。

## 4. 推論サーバーの起動

第4章以降は、**推論用PC（`dnp218`）を直接操作する**ことを前提とする。

### 4.1 使用するプログラム

学習用PCと推論用PCでは使用するプログラムが異なる。

| 用途 | プログラムの場所 |
|---|---|
| 学習・Bundle生成（学習用PC） | `/workspace/piper_pi0/` |
| π0.5推論Runtime（推論用PC） | `/home/dnp218/piper_vla_runtime/` |
| ROS・PiPER制御（推論用PC） | `/home/dnp218/docker_piper_env/double_PiPER/` |

`piper_vla_runtime` はモデル推論を、`double_PiPER` はROSノード・カメラ・ロボット制御を担当する。両者は別のプログラムである。

### 4.2 推論サーバーの起動

推論用PCで以下を実行する。

```bash
cd /home/dnp218/piper_vla_runtime
export DUAL_BUNDLE=/home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_5000

./scripts/run_dual.sh \
  --checkpoint "$DUAL_BUNDLE" \
  --warmup-prompt "Assembling a box." \
  --dry-run
```

`--checkpoint` は使用するBundleに合わせて変更する。`--warmup-prompt` はモデル初期化時の言語指示である。`--dry-run` は検証用であり、Controllerへの実機Action送信を行わない。

### 4.3 起動状態の確認

モデルの読み込み後、ログで `READY` を確認する。サーバー起動中はこのターミナルを閉じない。

**注意：** この章の `--dry-run` は実機を動かさない検証用モードである。第5章のROS側 `dry_run` とは別に指定する。

---

## 5. 双腕PiPERの起動と推論のStart／Stop

本章では、初期姿勢への復帰、ROS launchの起動、推論開始・停止を行う。**ROS関連のコマンドは、`double_PiPER` を利用できるDockerコンテナ内で実行する**。新しいターミナルでは `sorc_devel` でROS環境を読み込む。

### 5.1 初期姿勢に戻す

**推論を停止した状態で、ROSドライバなど他のプログラムが同じCANを使用していないことを確認してから**、初期姿勢復帰スクリプトを使用する。

```bash
cd ~/double_PiPER
python3 src/piper/scripts/go_dual_initial_pose.py --dry-run
```

まず `--dry-run` で設定・関節角度範囲などを検証する。問題がなければ、周囲とロボットの可動範囲を確認し、以下のコマンドで実機を初期姿勢へ移動する。

```bash
python3 src/piper/scripts/go_dual_initial_pose.py
```

初期姿勢の設定ファイル：

```text
/home/dnp218/docker_piper_env/double_PiPER/src/piper/config/piper_dual_demo_start.yaml
```

このYAMLに左右の目標関節角などが設定されている。左腕は `can3`、右腕は `can1` を使用する。初期姿勢復帰は `direct_sdk` による動作であり、**双腕間の衝突判定は検証されていない**。ロボット周辺を十分に確認し、必要に応じて非常停止できる状態で実施する。

### 5.2 推論用ROS launchの起動

別のターミナルでDockerコンテナに入り、ROS環境を読み込む。

```bash
sorc_devel
```

CANアダプタの設定およびROS Masterの起動が済んでいることを確認して、**まずは動作させない検証モード**で起動する。

```bash
roslaunch piper start_vla_dual_piper.launch \
  left_can_port:=can3 \
  right_can_port:=can1 \
  front_camera:=/dev/video4 \
  left_wrist_camera:=/dev/video2 \
  right_wrist_camera:=/dev/video0 \
  dry_run:=true \
  auto_enable:=false
```

このlaunchで左右PiPERドライバ、3台のカメラ、双腕Controllerを起動する。起動したターミナルは閉じない。

**重要：** `go_dual_initial_pose.py` とROSドライバを同時に動かさないこと。両者が同じCANデバイスを操作すると競合する可能性がある。

### 5.3 推論のStart

さらに別のターミナルでROS環境を読み込む。

```bash
sorc_devel
```

推論サーバーの `READY` を確認してから、言語指示（prompt）を指定して推論を開始する。

```bash
rosservice call /piper_vla_dual_runtime/start "prompt: 'Assembling a box.'"
```

このコマンドは推論開始を指示する。**第4章と5.2のdry-run設定では、実機は動作しない。**

### 5.4 推論のStop

推論を停止する場合は、以下のコマンドを実行する。

```bash
rosservice call /piper_vla_dual_runtime/stop '{}'
```

停止を確認してから次の操作へ進む。ROS launchを終了する場合は、起動したターミナルで `Ctrl+C` を使用する。初期姿勢へ戻す場合は、推論停止後、ROSドライバを終了してCANの競合がない状態で5.1を実行する。

### 5.5 設定ファイル・関連コードの場所

| 用途 | パス（推論用PC） |
|---|---|
| 推論用ROS launch | `/home/dnp218/docker_piper_env/double_PiPER/src/piper/launch/start_vla_dual_piper.launch` |
| 初期姿勢の設定（YAML） | `/home/dnp218/docker_piper_env/double_PiPER/src/piper/config/piper_dual_demo_start.yaml` |
| 初期姿勢復帰スクリプト | `/home/dnp218/docker_piper_env/double_PiPER/src/piper/scripts/go_dual_initial_pose.py` |
| 双腕ROS Controller | `/home/dnp218/docker_piper_env/double_PiPER/src/piper/scripts/vla_dual_piper_controller.py` |
| Runtime設定（基準YAML） | `/home/dnp218/piper_vla_runtime/configs/runtime.dual.yaml` |
| Runtime起動スクリプト | `/home/dnp218/piper_vla_runtime/scripts/run_dual.sh` |

Runtimeは起動時に一時的な設定ファイルを生成する場合がある。実際に読み込まれた設定ファイルの場所は起動ログで確認すること。

### 5.6 実機を動かす場合の注意

本READMEの第4章および5.2は、まず実機を動かさない検証用設定である。**実機推論ではRuntime側とROS側の両方で動作モードを合わせる必要がある。** Runtimeの `--dry-run` のままではControllerにActionが渡らない。Controllerまでの経路を使う場合は `--controller-run` を使用する。ROS側の `dry_run:=false`、`auto_enable:=true` への切り替えは、ドライバの排他利用、CAN・関節角制限、HOLD／非常停止、周囲の安全を確認した後に行う。

なお、これまでのROS Mock検証は実機CANでの動作保証を意味しない。初回の実機動作は立会いのもとで慎重に確認すること。
