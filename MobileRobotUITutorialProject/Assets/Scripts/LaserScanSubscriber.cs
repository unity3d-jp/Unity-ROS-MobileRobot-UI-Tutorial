using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// 座標変換用
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
// 各種ROSトピック形式
using LaserScanMsg = RosMessageTypes.Sensor.LaserScanMsg;

/// <summary>
/// LiDARスキャンデータ（LaserScanMsg）を受信するためのクラス
/// 主にスキャンデータの取得および座標変換に使い、描画は別のスクリプトを用意することを想定
/// </summary>
public class LaserScanSubscriber : MonoBehaviour
{

    // 受信するROSのトピック名
    [SerializeField] string rosTopicName = "scan";
    // デバッグモードとするかどうか（デバッグモードではコンソールにログを出力）
    [SerializeField] bool isDebugMode = true;

    private float[] scanRangesArray;
    private float scanAngleMin = 0;
    private float scanAngleMax = 0;

    /// <summary>
    /// 初期化用のイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// </summary>
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<LaserScanMsg>(rosTopicName, ScanUpdate);
    }


    /// <summary>
    /// ROSトピックを受け取った際に呼ばれるコールバック関数
    /// </summary>
    void ScanUpdate(LaserScanMsg scanMessage)
    {
        if (isDebugMode)
        {
            Debug.Log(scanMessage);
        }

        scanRangesArray = scanMessage.ranges;
        scanAngleMax = scanMessage.angle_max;
        scanAngleMin = scanMessage.angle_min;
    }

    /// <summary>
    /// 外部からスキャン結果の配列にアクセスするための関数
    /// </summary>
    public float[] GetScanRanges()
    {
        return scanRangesArray;

    }

    /// <summary>
    /// 外部からスキャン結果にアクセスするための関数
    /// </summary>
    public float GetScanAngleMin()
    {
        return scanAngleMin;
    }

    /// <summary>
    /// 外部からスキャン結果にアクセスするための関数
    /// </summary>
    public float GetScanAngleMax()
    {
        return scanAngleMax;
    }
}