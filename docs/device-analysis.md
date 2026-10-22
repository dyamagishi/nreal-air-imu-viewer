# 初代Nreal AirのHID通信と取得データ

初代Nreal Air (`VID:PID = 3318:0424`) のUSB HID通信、センサーデータ復号、および工場校正データの取得仕様です。

> **対象環境・留意事項**  
> - 対象機種: 初代Nreal Air（他機種やファームウェア差異での互換性は未検証）
> - 読み出し専用: センサーレポートの受信と校正JSONの読み出しのみを行い、ファームウェア書き換えやディスプレイ・輝度設定の変更は行いません。
> - 個体情報（シリアル番号や測定履歴等）はGit管理外のローカル日誌に分離して保管します。

---

## HIDインターフェース構成

初代Nreal Airは複数のHIDインターフェースを列挙します。

| インターフェース | 用途 / 観測内容 | 主な通信 |
|:---:|---|---|
| **3** | **IMU制御およびセンサーデータ配信** | コマンド送受信、約1,000Hz・64バイトのIMU連続レポート |
| **4** | **MCU制御・本体イベント** | デバイス情報の照会、ボタン押下などの本体通知 |
| **5** | **未解釈データ** | 64バイトの未解釈レポート |
| **6** | **Consumer Control** | 標準HIDコントロール（連続取得時のイベントなし） |

---

## 通信プロトコル仕様

### 1. コマンドプロトコル

IMUおよびMCUへのコマンド送受信は以下のパケット構造を取ります。ヘッダーのコマンド長・コマンドIDを照合し、末尾4バイトのCRC32を検証します。

- **IMU制御（Interface 3）**: プレフィックス `0xaa` で開始
  - `0x19`: IMUストリーミング配信の開始 / 停止
  - `0x14`: 工場校正データのデータ長取得
  - `0x15`: 工場校正データ（JSON文字列）のセグメント読み取り
- **MCU照会（Interface 4）**: プレフィックス `0xfd` で開始

### 2. IMUセンサーレポート（Interface 3）

IMU配信開始後、`0x01 0x02` で始まる64バイトのレポートが約1,000Hzで送信されます。

| オフセット | バイト長 | 型 / エンディアン | 項目 | 換算式 / 備考 |
|:---:|:---:|---|---|---|
| 0–1 | 2 | raw | レポート識別子 | 固定値 `0x01 0x02` |
| 2–3 | 2 | signed 16-bit (LE) | 温度 | Monado換算: `raw / 132.48 + 25.0` [℃] |
| 4–11 | 8 | unsigned 64-bit (LE) | デバイス時刻 | ハードウェアタイムスタンプ (μs単位) |
| 12–17 | 6 | uint16 × 3 (LE) | ジャイロ倍率・除数 | 3軸共通または軸別のスケーリング係数 |
| 18–26 | 9 | signed 24-bit × 3 (LE) | ジャイロ (3軸) | 角速度値 [rad/s] |
| 27–32 | 6 | uint16 × 3 (LE) | 加速度倍率・除数 | 3軸共通または軸別のスケーリング係数 |
| 33–41 | 9 | signed 24-bit × 3 (LE) | 加速度 (3軸) | 加速度値 [m/s²] |
| 42–47 | 6 | uint16 × 3 (**BE**) | 地磁気倍率・除数 | ※ここだけ**ビッグエンディアン** |
| 48–53 | 6 | unsigned 16-bit × 3 (LE) | 地磁気 (3軸) | `(raw - 32768) * scale`（任意単位） |
| 54–63 | 10 | raw hex | 末尾フィールド | byte 62は地磁気更新フラグとして利用（その他は未解釈） |

> **注意点**:
> - レポートに含まれるジャイロ・加速度・地磁気は、それぞれセンサー固有の直交座標系を持ち、軸方向が揃っていません。
> - 姿勢推定に入力する前に、後述の工場校正値に基づきセンサー間の座標軸回転（軸合わせ）を適用します。

### 3. 工場校正データ（Factory Calibration JSON）

本体内蔵フラッシュから読み出されるJSON文字列には、工場出荷時の各センサー校正値が記録されています。

- **校正パラメーター**: `accel_bias`, `gyro_bias`, `mag_bias`, スケール補正行列, センサー間クォータニオン (`accel_q_gyro`, `gyro_q_mag`) など
- **適用方式**: Monadoドライバーの `read_sample_and_apply_calibration` の計算順序に準拠
- **その他のキー**: `RGB_camera` や `SLAM_camera` などのキーが含まれますが、これらは共通スキーマによる定義であり、ハードウェアとしてのカメラ搭載を意味するものではありません。

---

## 生データ収集ツール (`air-capture`)

実機から全IMUサンプル（約1,000Hz）とHIDトランザクションをキャプチャするCLIツールです。

### 実行方法

```sh
uv run air-capture --seconds 15
```

実行すると、タイムスタンプ付きのディレクトリ（`data/captures/YYYYMMDD-HHMMSS/`）に以下のファイル群が出力されます。

| ファイル | 内容 |
|---|---|
| `imu.csv` | 換算後の時刻、温度、3軸ジャイロ、3軸加速度、3軸地磁気 |
| `imu.jsonl` | 各サンプルの生整数値、換算値、デバイス時刻、末尾フィールド |
| `transactions.jsonl` | コマンド送受信パケットおよび全受信レポートのHEX列と受信ホスト時刻 |
| `factory_calibration.bin` | 本体から読み出した校正データのバイナリ生データ |
| `factory_calibration.json` | 整形（インデント）された工場校正JSON |
| `devices.json` | 列挙されたHIDインターフェース一覧 |
| `metadata.json` | デバイス情報（ファームウェアバージョン等）の照会応答 |
| `summary.json` | 総パケット数、実効サンプリング周期、各軸の基本統計値 |

---

## 仕様の参照元・謝辞

本実装のリバースエンジニアリングおよび仕様把握にあたり、以下のオープンソース実装を照合元として参照しています。

- [ar-drivers-rs / nreal_air.rs](https://github.com/badicsalex/ar-drivers-rs/blob/master/src/nreal_air.rs): HIDインターフェース番号、制御コマンド、校正JSONの取得手順、ジャイロ・加速度の復号ロジック
- [Monado / xreal_air_packet.c](https://gitlab.freedesktop.org/monado/monado/-/blob/main/src/xrt/drivers/xreal_air/xreal_air_packet.c): 地磁気フィールドのバイトオーダー（ビッグエンディアン）およびオフセット符号表現
- [Monado / xreal_air_hmd.c](https://gitlab.freedesktop.org/monado/monado/-/blob/main/src/xrt/drivers/xreal_air/xreal_air_hmd.c): `read_sample_and_apply_calibration` による校正パラメータ適用手順、温度・物理単位換算式
- [android-nreal / NrealDeviceThread.java](https://github.com/enricoros/android-nreal/blob/main/app/src/main/java/com/enricoros/nreal/driver/NrealDeviceThread.java): レポートパケット構造、本体ボタン押下イベントの解釈

