# UnityとROSで学ぶ移動ロボット入門 UI作成編

本教材は Unity と ROS を組み合わせたロボット開発のための入門用教材です。

## 導入方法

本教材は、[Git](https://git-scm.com/) を使用して取得してお使い下さい。

Gitを使うのが初めての方は、「Git Windows 使い方」 などのキーワードでネット検索すると多くの情報が得られます。

Gitがインストールできたら、Git Bash、コマンドプロンプト または Power Shell などから以下のコマンド順番に実行して行きます。

### GitHubからファイルを一式を取得する

GitHubからファイルを一式を取得(clone)します。
```
$ git clone https://github.com/unity3d-jp/Unity-ROS-MobileRobot-UI-Tutorial.git
```
(ここで左端の `$` はシステムがユーザーの入力を促していることを表す記号で皆さんが入力する必要はありません。お使いの環境によっては `$` の他に `>` などの場合もあります。)

### 作業フォルダに移動
```
$ cd Unity-ROS-MobileRobot-UI-Tutorial
```

### ROS-TCP-Endpointの取り込み

以下のコマンドを使って、後述する `ROS-TCP-Endpoint` を取り込みます。
```
$ git submodule update --init --recursive
```

以上で、必要なファイル一式が `Unity-ROS-MobileRobot-UI-Tutorial` フォルダの中に格納されました。

## 次のステップへ

次は、[docs/intro0.md](docs/intro0.md) にお進み下さい。

(ドキュメント類は、Webブラウザを使ってGitHub上のファイルを参照した方が見やすい場合もあります。)

## ライセンスについて

* 本教材は [Apache License 2.0](./LICENSE-2.0.txt) で提供されています。(以下の3<sup>rd</sup>パーティ製ソフトウェアを除く)

* 3<sup>rd</sup>パーティ製ソフトウェアについては [OSSライセンスに基づく表記](./THIRD-PARTY-LICENSE.md) を参照して下さい。
