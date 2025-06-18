# orca-m5stick-cplus-device

**TGS2025「シャチで狩り」学生チャレンジプロジェクト**  
愛知工業大学の学生が開発した、M5StickC Plusをゲーム「シャチで狩り」で使用する「シャチデバイス」として制御するArduinoスケッチです。  
MacBook Air M1上のArduino IDE 2.3.6でのビルド・アップロードを想定しています。
---

## ディレクトリ構成

    orca-m5stick-cplus-device/
    ├ README.md
    ├ .gitignore
    └ orca-m5stick-cplus-device.ino

---

## 動作確認環境

- **ハードウェア**
    - M5StickC Plus  
    - USB Type-C ケーブル(データ通信対応のもの)  

- **ソフトウェア**
    - macOS 11 Big Sur 以降  
    - Arduino IDE 2.3.6（Apple Silicon 用）  
    - ボードマネージャで「M5StickC Plus」ライブラリをインストール  

---

## セットアップ手順

1. **Arduino IDE の設定**
    - 省略

2. **ソースコードを開く**
    - Arduino IDE で `orca-m5stick-cplus-device.ino` を開く。  
    - ツールバーの「ツール > ボード」で「M5Stick-C-Plus」を選択。  
    - 「ツール > シリアルポート」で接続先を選ぶ。

3. **ビルド＆アップロード**
    - ツールバーの「✔︎」ボタンでビルドを実行。  
    - 「→（アップロード）」ボタンで M5StickC Plus に書き込む。  
    - シリアルモニタ（例：ボーレート115200）で動作ログを確認。

---



---

## トラブルシューティング

- **ボードが認識されない**
    - USB ケーブルがデータ通信対応か確認  
    - macOS の「システム設定 > セキュリティ」でドライバインストールの許可を確認

- **ビルドエラーが出る**
    - M5StickC Plus 用ライブラリが最新か確認  
    - Arduino IDE の環境設定で「Apple Silicon 用」を使用しているか再確認

---

## 開発者情報

- **メンテナ**：MMD-Lucky  
- **GitHub**：https://github.com/MMD-Lucky/orca-m5stick-cplus-device  
