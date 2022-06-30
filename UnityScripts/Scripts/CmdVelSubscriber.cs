using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// 各種ROSトピック形式
using TwistMsg = RosMessageTypes.Geometry.TwistMsg;

/// <summary>
/// CmdVel（TwistMsg）を受信するためのクラス
/// 主にデバッグ用に使うことを想定
/// </summary>
public class CmdVelSubscriber : MonoBehaviour
{
    // 受信するROSのトピック名
    [SerializeField] string rosTopicName = "cmd_vel";
    // デバッグモードとするかどうか（デバッグモードではコンソールにログを出力）
    [SerializeField] bool isDebugMode = true;

    /// <summary>
    /// 初期化用のイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// </summary>
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(rosTopicName, CmdVelUpdate);
    }

    /// <summary>
    /// ROSトピックを受け取った際に呼ばれるコールバック関数
    /// </summary>
    void CmdVelUpdate(TwistMsg twistMessage)
    {
        if (isDebugMode)
        {
            Debug.Log(twistMessage);
        }
    }
}