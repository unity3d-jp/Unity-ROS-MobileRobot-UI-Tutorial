# Raspberry Pi Mouse位置姿勢の可視化
## 概要

Gazebo上のロボットの位置姿勢を取得し、Unity上に反映させる方法を紹介します。

本ステップ実行後の状態のSceneファイルは[`MobileRobotUITutorialProject/Assets/Scenes/Step4.unity`](../MobileRobotUITutorialProject/Assets/Scenes/Step4.unity)から入手できます。

## 動作確認済環境

* Windows 10 Home バージョン 20H2
* Unity 2020.3.17f
* [Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) v0.5.0
* Docker Desktop 3.6.0

## 手順

### 1. ROSトピックのサブスクライバスクリプトの登録

ROSから配信されるトピックを受信するために、サブスクライバをUnityに登録します。

Hierarchyウィンドウを右クリックして`Create Empty`を選択し、`Subscriber`としてGameObjectを作成します。

![](./images/step4-5.png)


作成した`Subscriber`オブジェクトに`Assets/Scripts/TfSubscriber.cs`スクリプトをアタッチします。  
（`Hierarchy`ウィンドウからGameObjectを選択してInspectorウィンドウ上で任意のスクリプト コンポーネントを追加することも可能です。）

![](./images/step4-6.gif)

Inspectorウィンドウで`Tf Subscriber`コンポーネントの`Mobile Robot`に`raspimouse`オブジェクトを指定します。

![](./images/step4-7.gif)

以上でROSから配信されるロボットの位置姿勢を受信してUnity上のロボットに反映させる準備ができました。

### 2. Unityプロジェクトの実行

[STEP3](./step3.md)と同様に、

1. 再生モードでUnityプロジェクトを実行
2. Gazeboシミュレータを起動
3. 操作ボタンから移動指令送信

の手順でロボットを動かしてみます。

![](./images/step4-8.gif)

[STEP3](./step3.md)までは操作ボタンを押した際にGazebo上のロボットが移動するだけでしたが、Gazebo上の移動ロボットに合わせてUnityのロボットも動かすことができました。
操作ボタンを使わなくてもGazebo上でロボットを動かすことでも反映されます。

![](./images/step4-9.gif)

Consoleを見ると位置姿勢の受信結果と座標変換のデバッグログを確認できます。  
ROSとUnityでは座標系が異なるため、この差分を吸収するために`TfSubscriber.cs`スクリプト内で座標変換をしています。
座標変換については次の「位置姿勢の表現方法とROSとUnityにおける座標系の違い」を参照してください。

## 位置姿勢の表現方法とROSとUnityにおける座標系の違い
### 位置の表現方法
ロボットの位置を表現するには原点を基準にした前後、左右、上下方向の距離を用います。例えば下図のように原点から見たロボット位置を前後、左右、上下の順に書くと(4,2,0)と表現できます。このように数値で表現された位置を座標、表現方法の決まりごとを座標系といいます。ちなみに今回用いる座標系は3つの方向がそれぞれ直角に交わっているため直交座標系とよばれています。

![](./images/step4-99.png)

### 姿勢の表現方法
ロボットの姿勢はクォータニオンとよばれる形式が用いられます。位置と同じように回転する軸の方向を3つの要素で表現し、回転量をもう1つの要素で表現する方法です。これは人の直感的な認識に沿わないので数値を見ても姿勢を想像することが難しいのですが、コンピュータ上で処理する際にとても便利な形式になっています。クォータニオンの値を人が直接扱うことはあまりないためわからなくても問題ありません。クォータニオンはこのような祭りが開催されるほどステップアップのための重要なテーマになっています。

* [納涼クォータニオン夏祭り | Unity Learning Materials](https://learning.unity3d.jp/5830/)

### UnityとROSの座標系の違い
UnityもROSもロボット位置にx,y,zの3つ、姿勢の場合はx,y,z,wの4つの変数が割り当てていますが、それぞれの座標系で変数と方向の対応が異なる点に注意する必要があります。Unity座標系におけるx,y,zはそれぞれ右、上、前方向に対応していますが、ROS座標系では前、左、上の順で対応しています。

![](./images/step4-100.png)

UnityとROSの座標系の変換を変数で表現してみます。Unity座標系における位置(x,y,z)がROS座標系では(z,−x,y)、ROS座標系における位置(x,y,z)がUnity座標系では(−y,z,x)となります。姿勢も位置と同様にUnity座標系における姿勢(x,y,z,w)がROS座標系では(z,-x,y,-w)、ROS座標系における姿勢(x,y,z,w)がUnity座標系では(-y,z,x,-w)になります。非常にややこしいのですが変換のコードさえ書いてしまえば毎回考える必要はありません。今回の説明を読んでピンとこない方もUnityとROSの間では何かしら変換が必要であることだけを意識していただければ問題ないです。

![](./images/step4-101.png)

### 本STEPのまとめ

Gazebo上のロボットの位置姿勢をUnity上で表示じて可視化する方法とその確認方法を紹介しました。
またUnityとROSの座標系の違いについても紹介しました。

次は[STEP5](./step5.md)でロボットのオドメトリを可視化する方法を紹介します。

---

* [目次](./intro2.md)
* [STEP3](./step3.md)
* [STEP5](./step5.md)