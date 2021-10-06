using UnityEngine;
using UnityEngine.UI;
using Unity.Robotics.ROSTCPConnector;
using CompressedImageMsg = RosMessageTypes.Sensor.CompressedImageMsg;

/// <summary>
/// LiDARスキャンデータ（LaserScanMsg）を受信するためのクラス
/// 主にスキャンデータの取得および座標変換に使い、描画は別のスクリプトを用意することを想定
/// </summary>
public class ImageSubscriber : MonoBehaviour
{
    // 受信するROSのトピック名
    [SerializeField] string rosTopicName = "webcam/image_raw/compressed";
    // デバッグモードとするかどうか（デバッグモードではコンソールにログを出力）
    [SerializeField] bool isDebugMode = true;
    // カメラデータを貼り付けるRawImageオブジェクト
    [SerializeField] RawImage rawImage;

    private Texture2D texture2D;
    private byte[] imageData;
    private bool isMessageReceived;

    /// <summary>
    /// 初期化用のイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// </summary>
    void Start()
    {
        ROSConnection.instance.Subscribe<CompressedImageMsg>(rosTopicName, ImageMsgUpdate);
        texture2D = new Texture2D(1, 1);
    }

    /// <summary>
    /// 1フレーム毎に呼び出されるイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// </summary>
    void Update()
    {
        if (isMessageReceived)
        {
            ProcessMessage();
        }
    }

    /// <summary>
    /// ROSトピックを受け取った際に呼ばれるコールバック関数
    /// </summary>
    void ImageMsgUpdate(CompressedImageMsg rawImage)
    {
        imageData = rawImage.data;
        isMessageReceived = true;
        if (isDebugMode)
        {
            Debug.Log("rawImage recieved. length :" + System.Buffer.ByteLength(imageData));
        }
    }

    /// <summary>
    /// ImageMsgのデータをTextureに反映する関数
    /// </summary>
    void ProcessMessage()
    {
        texture2D.LoadImage(imageData);
        texture2D.Apply();
        rawImage.texture = texture2D;
        isMessageReceived = false;
    }
}