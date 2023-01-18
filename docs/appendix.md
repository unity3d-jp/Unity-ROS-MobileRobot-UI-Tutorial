# 付録

## 概要

STEP1～7では扱わなかった内容についても紹介します。

## 動作確認済環境

* Windows 10 Home バージョン 21H2
* Unity 2021.3.4f1
* [Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector) v0.7.0
* Docker Desktop 4.9.1

## 独自のROSメッセージへの対応

[Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)では扱うすべてのROSのメッセージファイルをC#にて定義する必要があります。
[`com.unity.robotics.ros-tcp-connector/Runtime/Messages`](https://github.com/Unity-Technologies/ROS-TCP-Connector/tree/v0.7.0/com.unity.robotics.ros-tcp-connector/Runtime/Messages)から予めUnity用に用意されているROSのメッセージ形式を確認できます。  
使うROSのメッセージがROS-TCP-Connectorに予め含まれていない場合にのみ、C#のスクリプトを用意する必要があります。

### 1. ROSのメッセージファイルを用意

変換したいROSのメッセージファイルを用意します。  
例えばRaspberry Pi Mouse実機に搭載されているLEDの点灯・消灯状態は`LedValues.msg`というメッセージファイルにて定義されている形式で管理されます。

https://github.com/rt-net/raspimouse/blob/1.0.0/raspimouse_msgs/msg/LedValues.msg

### 2. ROSのメッセージファイルからC#スクリプト作成

変換にはROS-TCP-Connectorのメッセージ作成機能を用います。

メニューバーから`Robotics` -> `Generate ROS Messages...`を選択します。

![](./images/appendix-1.png)

`ROS message path`を指定します。
ROSのメッセージファイルをダウンロードしてきたフォルダを指定します。

![](./images/appendix-2.png)

対象の`.msg`ファイルを選択して`Build msg`をクリックします。

![](./images/appendix-3.png)

以上でROSのメッセージファイル作成は完了です。Unityプロジェクト内の`Assets/RosMessages`フォルダにC#のスクリプトが作成されています。

![](./images/appendix-4.png)

## JoystickによるRaspberry Pi Mouseの操作

Input SystemのOn-Screen Stickを用いてスクリーン上にJoyStickを作成し、Raspberry Pi Mouseを操作します。

### 1. Input Systemの確認

Package Managerを開いてInput Systemがインストールされているか確認します。

メニューバーから `Window` -> `Package Manager` を選択します。

![](./images/appendix-13.png)

`Package Manager`を開いたら、`Unity Registry`を選択して、`Input System`にチェックマークが付いているか確認します。

![](./images/appendix-28.png)

もしもインストールされていない場合は、インストールを実行してください。

![](./images/appendix-14.png)

### 2. On-Screen Stickの配置と設定

Input SystemのOn-Screen Stickを配置します。

前のステップで作成したロボット操作用ボタンの代わりにOn-Screen Stickを使用するため、
Hierarchyの`Canvas` -> `Panel`を選択し、
Inspectorでチェックを外して`Panel`を非表示にします。

![](./images/appendix-16.png)

Hierarchyの`Canvas`の子オブジェクトとして`Image`を作成します。

![](./images/appendix-17.png)

`Image`のInspectorでSource Imageに`Knob`を選択します。

![](./images/appendix-18.png)

`Image`のInspectorで`Anchor Presets`を右下に設定し、
さらにPositionとWidthとHeightを以下のように設定します。

- Pos X : -200
- Pox Y : 200
- Pos Z : 0
- Width : 50
- Height : 50

![](./images/appendix-19.png)

`Image`のInspectorで`Add Component`を押し、
`On-Screen Stick`を選択します。
さらに`On-Screen Stick`の`Control Path`に`Gamepad` -> `Left Stick`を選択します。

![](./images/appendix-20.png)

`EventSystem`のInspectorで`Standalone Input Module`の
`Replace with InputSystemUIInputModule`を押します。

![](./images/appendix-21.png)

### 3. On-Screen Stick用のスクリプトを設定

まず準備として、`UnityScripts/JoyStick`フォルダをUnityプロジェクトの`Assets`フォルダにコピー（`Assets/JoyStick`として配置）します。

次に、Hierarchyの`Publisher`の`CmdVelPublisher`のチェックを外します。

さらに`Publisher`のAdd Componentを押して`OnScreenStickCmdVelPublisher`を追加します。

![](./images/appendix-22.png)

### 4. Input Actionの設定

Hierarchyの`Publisher`にアタッチしてある`OnScreenStickCmdVelPublisher`の`Move Action`の＋マークを押します。
そして`add Up\Down\Left\Right Composite`を選択します。

`2D Vector`をダブルクリックして、
`Composite Type`を`2D Vector`、
`Mode`を`Analog`に設定します。

![](./images/appendix-23.png)

さらに、`Up`には`Gamepad` -> `Left Stick` -> `Up`を選択します。
`Down`と`Left`と`Right`も同じように設定します。

![](./images/appendix-24.png)

### 5. インターフェースの調整

On-Screen Stickのインターフェースを調整します。

Hierarchyの`Canvas`の子オブジェクトとして、`UI` -> `Image`を作成します。
名前は`WhiteCircle`とします。
また、画像で示すように`Image`の上に`WhiteCircle`を配置するようにしてください。

![](./images/appendix-25.png)

`WhiteCircle`のInspectorで、`Anchor Presets`を右下に設定します。
さらにPositionとWidthとHeightを以下のように設定します。

- Pos X : -200
- Pos Y : 200
- Pos Z : 0
- Width : 100
- Height : 100

また、画像で示すように`Source Image`に`Assets/JoyStick/Textures/white_circle.png`を設定してください。

![](./images/appendix-26.png)

`Source Image`に`white_circle`が設定できない場合、
`white_circle`のInspectorで`Texture Type`が`Sprite (2D and UI)`になっているか確認してください。

![](./images/appendix-27.png)

### 6. On-Screen StickでRaspberry Pi Mouseを操作

以上の作業を終えると、On-Screen StickでRaspberry Pi Mouseを操作できます。

![](./images/appendix-2.gif)
