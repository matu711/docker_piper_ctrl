# 双腕PiPER π0.5：手動Stopまで推論を続ける

対象Bundle：`dnp1_20261015_bimanual_pi05_delta_5000`

ROS master、双腕の`start_vla_dual_piper.launch`、左右State・3カメラが利用できることを
前提とします。Controllerの`max_delta_left/right`も起動前に設定しておきます。
ROS側の起動・設定は[既存の推論README](<README_PiPER_pi05_inference .md>)を参照してください。

## 1. Runtimeを起動する（推論用PCのホスト側）

```bash
cd /home/dnp218/piper_vla_runtime

./scripts/run_dual.sh \
  --checkpoint /home/dnp218/piper_checkpoints/dnp1_20261015_bimanual_pi05_delta_5000 \
  --warmup-prompt 'Assembling a box.' \
  --max-execution-steps 0 \
  --controller-run
```

`READY`が表示されるまで待ち、このターミナルは開いたままにします。
Runtimeの起動だけでは推論の実行は開始しません。

| 引数 | 内容 |
|---|---|
| `--checkpoint` | 推論に使うBundleのパス |
| `--warmup-prompt` | モデルのウォームアップに使うPrompt。開始時のPromptは次のStartで指定 |
| `--max-execution-steps 0` | Action実行数による自動停止を無効化。手動Stopまで継続可能 |
| `--controller-run` | RuntimeからControllerへActionを配信する（Runtime側の`dry_run=false`） |

`0`で無効になるのは実行step数の上限だけです。State・画像の鮮度異常、推論timeout、
Controllerの安全判定などによる自動HOLDは有効なままです。

`--controller-run`はROS Controllerの`dry_run`を変更しません。
ROS launchが`dry_run:=true`なら、Controllerは検証・目標診断まで行い、driverへ関節指令を送りません。
実機を動かすかどうかはROS側の起動設定にも依存します。

## 2. READY後、別ターミナルでStartする（ROS環境側）

既存のPiPER Dockerコンテナに入り、ROS環境を読み込みます。

```bash
sorc_devel
```

同じROS masterへ接続している状態で、Prompt付きStartを呼びます。

```bash
rosservice call /piper_vla_dual_runtime/start "prompt: 'Assembling a box.'"
```

成功応答を確認します。正常時は20 HzでActionを処理し、チャンクを更新しながら継続します。
`--max-execution-steps 0`なので、所定のstep数に達したことを理由とする停止はありません。

## 3. 手動でStopする

ROS環境を読み込んだターミナルで実行します。

```bash
rosservice call /piper_vla_dual_runtime/stop '{}'
```

Runtimeは両腕HOLDを要求し、残りのActionチャンクを破棄します。
Stopは新規VLA指令を止める操作で、電源断や装置の非常停止とは異なります。

再開する場合は、停止理由やStateなどを確認した後、同じPrompt付きStartを再度呼びます。
Runtimeが稼働中なら、通常のStop／Startのたびにモデルをロードし直す必要はありません。

このREADME追加時には、上記コマンドの実行、ROS起動、CAN接続、実機動作は行っていません。
