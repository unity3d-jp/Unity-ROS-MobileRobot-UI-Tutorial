using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// 各種ROSトピック形式
using TwistMsg = RosMessageTypes.Geometry.TwistMsg;

/// <summary>
/// CmdVel（TwistMsg）を送信するためのクラス
/// UIから各関数を呼び出して使うことを想定
/// </summary>
public class CmdVelPublisher : MonoBehaviour
{
    // Variables required for ROS communication
    // 送信するROSのトピック名
    [SerializeField] string topicName = "cmd_vel";

    // 送信する速度指令の基準となる値（各関数でこの速度に所定の倍率をかける）
    [SerializeField] float linearVel;

    // 送信する角速度指令の基準となる値（各関数でこの角速度に所定の倍率をかける）
    [SerializeField] float angularVel;

    // ROS Connector
    private ROSConnection ros;

    private TwistMsg cmdVelMessage = new TwistMsg();

    /// <summary>
    /// 初期化用のイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
        SetStopVel();
        Publish();
    }

    /// <summary>
    /// 前進方向の速度指令を設定
    /// </summary>
    /// <param name="ratio">指令する速度の倍率</param>
    public void SetForwardVel(float ratio=1.0f)
    {
        cmdVelMessage.linear.x = linearVel * ratio;
    }

    /// <summary>
    /// 後進方向の速度指令を設定
    /// </summary>
    /// <param name="ratio">指令する速度の倍率</param>
    public void SetBackwardVel(float ratio=1.0f)
    {
        cmdVelMessage.linear.x = -linearVel * ratio;
    }

    /// <summary>
    /// 右旋回の角速度指令を設定
    /// </summary>
    /// <param name="ratio">指令する角速度の倍率</param>
    public void SetRightTurnVel(float ratio=1.0f)
    {
        cmdVelMessage.angular.z = -angularVel * ratio;
    }

    /// <summary>
    /// 左旋回の角速度指令を設定
    /// </summary>
    /// <param name="ratio">指令する角速度の倍率</param>
    public void SetLeftTurnVel(float ratio=1.0f)
    {
        cmdVelMessage.angular.z = angularVel * ratio;
    }

    /// <summary>
    /// 停止の速度指令、角速度指令を設定
    /// </summary>
    public void SetStopVel()
    {
        cmdVelMessage.linear.x = 0;
        cmdVelMessage.angular.z = 0;
    }

    /// <summary>
    /// 速度指令、角速度指令を送信
    /// </summary>
    public void Publish()
    {
        // ROSのros_tcp_endpointのdefault_server_endpoint.pyにメッセージを送信
        ros.Publish(topicName, cmdVelMessage);
    }
}
