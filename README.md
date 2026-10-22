# analog_discovery_ros2

Digilent Analog Discovery 3（AD3）のオシロスコープ入力（2ch、14bit、最大 125 MS/s）を、
**トリガ取り込み**で ROS 2 Humble に流すドライバ。インパルスのような短い現象を、前後を含めて
高速に1波形ずつ取る用途向け。Digilent WaveForms SDK（`libdwf`）を ctypes で呼ぶ。AD2 でも動く想定（未確認）。

## 現状

| 項目 | 状態 |
| --- | --- |
| SDK 呼び出し・トリガ取り込み・ノード・メッセージ | 実装済み。偽の libdwf を使うテストと、mock（模擬インパルス）での通しの動作を確認済み |
| AD3 実機 | セットアップ（`setup.sh`）、デバイスの列挙、自動トリガでの取り込み（`ad_selftest.py`）を確認済み（2026-10-22、コンテナの `ros` ユーザー、`privileged`）。CH1 の信号でのトリガ取り込みは未確認 |
| USB 給電 | AD3 は 600 mA 以上が必要。USB 2.0 ハブを2段経由した接続では、アナログ回路が止まって取り込みが始まらなかった（今は開くときにエラーにする）。PC の USB ポートに直接つなぐ |
| トリガ位置の向き | **未確認。** SDK の `TriggerPositionSet` は「0 = バッファ中央、正 = トリガが先頭側へ」と仮定している。`ad_selftest.py --loopback` で確かめる |

## セットアップ

### 1. ホスト（初回のみ。コンテナの外で）

```bash
./udev/setup_udev.sh     # VID 1443 を 0666 にする。sudo のパスワードを聞かれる。最後に OK と出れば完了
```

Digilent の Adept Runtime も udev ルールを入れるが、それはコンテナの中に入るだけで、udev が動くホストでは効かない。

### 2. WaveForms と Adept Runtime を落とす（ブラウザで）

files.digilent.com は Cloudflare の確認があり、curl / wget では取れない。ブラウザで次の2つ
（Linux、Ubuntu、x86_64 の `.deb`）を `~/onolab_ros2/depends/digilent/` に保存する（`depends/` は git の対象外）。

- Adept 2 Runtime: <https://digilent.com/reference/software/adept/runtime>（確認済みの版: 2.30.1）
- WaveForms: <https://digilent.com/reference/software/waveforms/waveforms-3/start>（確認済みの版: 3.25.1）

### 3. コンテナ内

```bash
cd ~/onolab_ros2/ros2_ws/src/third_party/analog_discovery_ros2
./scripts/install_waveforms.sh        # sha256 を確認してから apt で入れる（既知の版以外は --no-verify が必要）
cd ~/onolab_ros2/ros2_ws
colcon build --symlink-install --packages-select analog_discovery_ros2
source install/setup.bash
```

コンテナを作り直すと WaveForms は消えるので、`install_waveforms.sh` をもう一度実行する。
コンテナは `privileged: true`（`docker/compose.yaml` の設定）で起動している必要がある。

## 使い方

```bash
ros2 run analog_discovery_ros2 ad_probe.py          # 接続中のデバイスとバッファサイズ（開かない）
ros2 run analog_discovery_ros2 ad_selftest.py       # 1回の取り込み（出力には触らない）
ros2 launch analog_discovery_ros2 analog_discovery.launch.py
ros2 launch analog_discovery_ros2 analog_discovery.launch.py backend:=mock   # 実機なし
```

別のターミナルで:

```bash
ros2 topic echo --no-arr /analog_discovery/capture
ros2 run rqt_plot rqt_plot /analog_discovery/ch1_peak_v/data
```

### トピック

| トピック | 型 | 内容 |
| --- | --- | --- |
| `~/capture` | `analog_discovery_ros2/msg/ScopeCapture` | 1回のトリガ取り込み。`ch1_v` / `ch2_v` は電圧 [V]、`trigger_index` がトリガのサンプル |
| `~/ch1_peak_v`, `~/ch2_peak_v` | `std_msgs/Float32` | 取り込みごとの、絶対値が最大のサンプル（符号付き、V）。rqt_plot 向け |

- `header.stamp` はトリガ時刻の推定。ホストの時計で「取り込み完了を見た時刻（データの読み出し前）−
  トリガ後の長さ」なので、最大で1ポーリング周期（既定 5 ms）と USB・スケジューリングの遅れの分だけ遅れる。取り込み間の正確な間隔は `device_trigger_*`
  （AD3 ではデバイスのカウンタ、10 ns）を使う。
- `auto_triggered: true` は、トリガが来ないまま `auto_timeout_s` が過ぎて取った波形で、イベントではない。

### パラメータ

[config/analog_discovery.yaml](config/analog_discovery.yaml) を参照。既定値は NF AE9501B の RF 出力（AE、100 kHz〜2 MHz）を
CH1 に入れる前提で、2026-10-22 に実機で取った波形から決めた（実験向けの調整はまだ）。

| パラメータ | 既定値 | 根拠 |
| --- | --- | --- |
| `sample_rate_hz` | 10 MHz（立ち上げ時 1 MHz） | AE9501B の帯域上限 2 MHz の 5 倍 |
| `buffer_samples` | 32768（同 8192） | AD3 の最大。残響が 2.46 ms を超えることが多かった |
| `pretrigger_samples` | 4096 = 0.41 ms（同 1024） | トリガ後を 2.87 ms 残す |
| `trigger_level_v` | 0.02 V（同 0.1 V） | 指でたたいたときのピークは 0.02〜1.16 V（中央値 0.50 V）、トリガ前のノイズは 1〜5 mV rms。90 秒で 379 回すべて取れた |
| `trigger_hysteresis_v` | 0.005 V（同 0.01 V） | レベルの 1/4（上の取り込みで使った値） |

10 MS/s・32768 サンプル・ch1 のみで、取り込み1回の周期（アーム→取り込み→読み出し）は約 12.5 ms。
これが不感時間の目安（ch2 も有効だと読み出しが増える）。残響をもっと長く取りたいときは 5 MS/s で 6.6 ms。

- `chN_range_peak_v` は ±V。AD3 は ±2.5 V と ±25 V（SDK には peak-to-peak の 5 V / 50 V で渡す）。
  デバイスが要求より狭いレンジを選んだときは起動を止める。
- `trigger_level_v` がレンジの外なら、デバイスを開く前にエラーにする（丸めない）。
- 設定はすべてデバイスを開く前に検証する（NaN も拒否する）。SDK の呼び出しが失敗したら、ノードは終了する。
- `settle_s`（既定 0.2 s）: 設定を反映したあと、最初の取り込みを始めるまで待つ時間。AD3 では、デバイスを
  開いた直後の最初の取り込みが、入力の過渡で 47〜64 ms 後に偽のトリガ（0.043〜0.072 V、CH1 は何もつながない状態）
  を起こすことがあった（4 プロセス中 3 回）。0.2 s 待つと 8 回中 0 回。観測した最長の約 3 倍にした暫定値
  （2026-10-22、OnoLab の実機）。

## 制約

- **不感時間がある。** 取り込みを読み出してから再アームするので、読み出し中のトリガは取りこぼす。
  間隔の短いイベントが続く用途には向かない（その場合は Record モードの連続記録が必要。未実装）。
- 1回の取り込みは最大 32 Ki サンプル/ch（AD3 の構成 2。`config_index: -1` で自動的に選ぶ）。
- 入力は 1 MΩ‖24 pF、±50 V まで保護（データシート）。BNC Adapter Board 経由ではシングルエンド測定になる。
  W1 / W2 の BNC は波形発生器の出力なので、信号を入れない。

## 自己診断

```bash
ros2 run analog_discovery_ros2 ad_selftest.py --loopback
```

W1 → CH1 を BNC ケーブルでつなぎ、**それ以外は何もつながない**状態で実行する。W1 から 50 Hz・±0.5 V の
矩形波を出し、取り込み（8.192 ms）の中の立ち上がりがちょうど1つで、それが `pretrigger_samples` の位置に
来るかを調べる（トリガ位置の向きの確認）。周期（20 ms）を取り込みより長くしているのは、何度も立ち上がる
波形だと、トリガ位置が整数周期ずれていても見逃すため。

## 出典

- WaveForms SDK Reference Manual（dwfsdk.pdf、2025-04-08 改訂）: 関数と定数
- Analog Discovery 3 Reference Manual / データシート: 入力仕様、バッファサイズ
- パッケージの sha256: Arch AUR の PKGBUILD（digilent.adept.runtime 2.30.1、digilent.waveforms 3.25.1、2026-10-22 に参照）
- USB ID: OnoLab の実機で `lsusb`（2026-10-22）
