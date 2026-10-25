# 双腕PiPER π0.5 Fine-tuning README

本READMEでは、双腕PiPERの教示データを使用して、π0.5モデルのFine-tuningを開始するまでの手順を説明する。

## 前提条件

- MCAPからLeRobot形式へのデータ変換が完了している。
- 学習用PC（`rlsv`）にSSH接続している。
- **学習用PCのDockerコンテナ内に入っている。**

以降のコマンドは、すべて学習用PCのDockerコンテナ内で実行する。SSH接続やDockerコンテナへの接続方法は、LeRobot変換用READMEを参照すること。

## 1. 学習設定（YAML）の確認・作成

双腕PiPERのπ0.5 Fine-tuningでは、YAMLファイルでデータセットや学習条件を指定する。

### 1.1 設定ファイルの保存先

設定ファイルは、次のディレクトリに保存する。

```text
/workspace/piper_pi0/configs/real/
```

既存の双腕PiPER用設定ファイル：

```text
/workspace/piper_pi0/configs/real/dnp1_20261015_bimanual_pi05.yaml
```

### 1.2 新しい設定ファイルの作成

新しいデータセットや実験で学習する場合は、既存のYAMLファイルをコピーして作成する。

```bash
cd /workspace/piper_pi0/configs/real/
cp dnp1_20261015_bimanual_pi05.yaml new_bimanual_pi05.yaml
```

`new_bimanual_pi05.yaml` はファイル名の例である。データセット名や実験名に合わせて変更する。作成したファイルをVS Codeなどで開いて編集する。

### 1.3 学習条件の変更

新しい設定ファイルで、少なくとも次の項目を確認・変更する。

| 設定項目 | 確認内容 |
| --- | --- |
| Dataset | 使用するLeRobotデータセット |
| Experiment Name | 実験名 |
| Training Steps | 学習ステップ数 |
| Batch Size | バッチサイズ |
| Learning Rate | 学習率 |
| Action Horizon | 予測するActionの長さ |
| Checkpoint Interval | Checkpointの保存間隔 |
| Resume | 新規学習か再開か |

既存の双腕用学習設定を基にした初期構成は、以下のとおり。

- Base Model：`pi05_base`
- LoRA：有効（Language rank 16／Action Expert rank 32）
- Action Horizon：50
- Batch Size：8

`lora_rank: null` の場合は、LanguageとAction Expertにそれぞれ既定のrankを使用する。整数を指定した場合は、両方のrankが同じ値に上書きされる。

**注意：** 新規実験では、データセットの参照先、実験名、Checkpointの保存先、正規化統計の保存・参照先を確認し、既存実験の結果と混在させないこと。

### 1.4 学習設定の確認

編集後、学習を開始せずに設定内容を確認する。

```bash
uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/training/scripts/train_bimanual_pi05.py \
  --config /workspace/piper_pi0/configs/real/new_bimanual_pi05.yaml \
  --check-config
```

`--config` には実際に使用するYAMLファイルの絶対パスを指定する。設定を確認できたら、次の章へ進む。

## 2. Fine-tuningの実行

### 2.1 学習の開始

Dockerコンテナ内で、以下のコマンドを実行する。

```bash
uv run --frozen --no-sync --project /workspace/openpi python \
  /workspace/piper_pi0/training/scripts/train_bimanual_pi05.py \
  --config /workspace/piper_pi0/configs/real/new_bimanual_pi05.yaml
```

`new_bimanual_pi05.yaml` は、第1章で作成・編集した設定ファイル名に置き換える。

### 2.2 学習開始時の処理

学習開始コマンドでは、次の処理が実行される。

1. YAMLの学習設定を読み込む。
2. 学習データセットと対応する正規化統計を確認する。
3. 正規化統計が未作成の場合は、自動的に計算する。
4. π0.5モデルを読み込み、Fine-tuningを開始する。
5. YAMLで指定した間隔でCheckpointを保存する。

**正規化統計の事前手動計算（`--norm-stats-only`）は必須ではない。** ただし、既存の統計を使用する場合は、データセットや学習設定との対応を確認すること。

### 2.3 学習状況の確認

学習中は、ターミナルのログで学習ステップ数やLossを確認する。GPU使用状況は、必要に応じて別ターミナルで次のコマンドを実行して確認する。

```bash
nvidia-smi
```

### 2.4 Checkpointの保存先

Checkpointは、YAMLの実験名や保存先設定に従って保存される。

既存の双腕PiPER用設定での保存先：

```text
/workspace/checkpoints/pi05_piper_bimanual_lora/dnp1_20261015_bimanual_pi05/
```

新しい設定ファイルを使用する場合は、**実際のCheckpoint保存先をYAMLで確認すること。** 保存済みCheckpointは、学習の再開や推論に使用する。

**注意事項**

- 学習中はDockerコンテナを停止しないこと。
- 学習ステップ数・Learning Rate・Checkpoint保存間隔はYAMLで変更する。
- 学習を中断する場合は、保存済みCheckpointを確認すること。
