# 双腕VLA：関節制限とmax_delta

## 角度制限

SDKパラメータ管理クラス `C_PiperParamManager().GetJointLimitParam("j1".."j6")` の
静的定義を確認した値です。実機firmwareや機械的ストッパーの読み出し値ではありません。
SDK管理クラスのversion表記は `PIPER_SDK_VERSION_1_0_0 (1.0.0)`。
左右で同じ定義を使い、左右の符号を鏡像変換しません。

| 関節 | 下限rad | 上限rad | 下限degree | 上限degree |
|---|---:|---:|---:|---:|
| J1 | -2.6179 | 2.6179 | -149.994621 | 149.994621 |
| J2 | 0 | 3.14 | 0 | 179.908748 |
| J3 | -2.967 | 0 | -169.996578 | 0 |
| J4 | -1.745 | 1.745 | -99.981135 | 99.981135 |
| J5 | -1.22 | 1.22 | -69.900851 | 69.900851 |
| J6 | -2.09439 | 2.09439 | -119.999708 | 119.999708 |

`start_sdk_joint_limit=true` は送信と受信キャッシュにSDK Clampを適用します。
`controller_joint_limit_enabled=true` はControllerの目標Clamp・超過拒否と実測State範囲検証を行います。
どちらもlaunch既定true。明示したfalseでのみそれぞれ無効化します。

`joint_saturation_margin_rad=0.1` は角度範囲を拡張する設定ではありません。
`[lower-0.1,upper+0.1]` 内の目標は `[lower,upper]` へClampし、その外はステップClipより前に拒否します。
実測Stateの範囲にはこのmarginを追加しません。
SDK内部の角度整数丸め、driverの既存State変換係数は変更していません。

## ステップ制限

`config/vla_dual_motion_limits.yaml` を通常起動時に読み込みます。
現在のユーザー指定既定値は以下です。実機検証済み速度・加速度上限を意味しません。

```yaml
max_delta_left:  [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
max_delta_right: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
```

単位は20Hz Actionのrad/step、joint1→6。gripperは含みません。
`applied_j = measured_j + clip(target_j-measured_j, -limit_j, limit_j)`。
前回指令ではなく新鮮な実測Stateが基準で、物理速度・加速度や衝突回避の保証ではありません。
未設定・6要素以外・bool・0以下・NaN/InfはController起動を拒否します。

別ファイルは `motion_limits_file:=/absolute/path/limits.yaml`。
個別上書きは `max_delta_left:='[0.1,0.1,0.1,0.1,0.1,0.1]'` 等で指定できます。
優先順は空配列へ初期化→YAMLロード→空でない左右引数です。
欠落キーが過去のROSパラメータを再利用しないようにし、ファイル不存在・YAML不正も拒否します。

## 維持する他の検証

| 項目 | 設定・動作 |
|---|---|
| State鮮度 | 0.5秒、source/local両方とCAN受信進行 |
| controller_status | 既定20Hz、session token一致と鮮度 |
| FK/Z | z_enabled=true、z_min=0.01m、dh_is_offset=1 |
| Controller gripper | 0〜0.10m、margin 0.01m、超過拒否 |
| Driver gripper | 通常joint経路は0〜0.10m、倍率1 |
| SDK gripper定義 | 0〜0.07m、SDK側gripper制限スイッチは既定false |
| HOLD/FAULT | 両腕の新規指令を抑止、再開始に明示的な通常Startが必要 |

SDKのgripper静的範囲とController/driver範囲は既存の異なる設定です。
0.10mまで実機が開けることを保証しません。今回この設定差は変更していません。
FK/Z・gripper・max_delta・有限値・鮮度・HOLD/FAULT・token/statusは角度制限falseでも維持します。
通常driverの速度指定50%はrad/sではなく、max_deltaへ直接換算できません。
Controllerには独立した数値加速度制限はありません。

起動例は [共通ガイド](../../../README_PiPER_guide.md) を参照してください。
実driverではdry_run=true / auto_enable=falseでもCAN初期化を行い得ます。
機器なしの双腕構成は `mock_driver:=true dry_run:=true enable_cameras:=false`。
コード検証は `test_vla_dual_launch_parameters.py` とVLA Controller Mock回帰で行います。
