using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// 座標変換用
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
// 各種ROSトピック形式
using OdometryMsg = RosMessageTypes.Nav.OdometryMsg;
using PointMsg = RosMessageTypes.Geometry.PointMsg;
using QuaternionMsg = RosMessageTypes.Geometry.QuaternionMsg;

/// <summary>
/// オドメトリデータ（OdometryMsg）を受信するためのクラス
/// 主にオドメトリデータの取得および座標変換に使い、描画は別のスクリプトを用意することを想定
/// </summary>
public class OdomSubscriber : MonoBehaviour
{

    [SerializeField] string rosTopicName = "odom";
    [SerializeField] bool isDebugMode = true;

    private Vector3 robotPosition = new Vector3();
    private Quaternion robotPosture = new Quaternion();

    /// <summary>
    /// 初期化用のイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// </summary>
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<OdometryMsg>(rosTopicName, OdomUpdate);
    }

    /// <summary>
    /// ROSトピックを受け取った際に呼ばれるコールバック関数
    /// </summary>
    void OdomUpdate(OdometryMsg odomMessage)
    {
        if (isDebugMode)
        {
            Debug.Log(odomMessage);
        }

        PointMsg rosOdomPositionMsg = odomMessage.pose.pose.position;
        QuaternionMsg rosOdomPostureMsg = odomMessage.pose.pose.orientation;

        // OdomのTwist情報も使う場合は以下を使用
        // Vector3Msg rosOdomTwistLinearMsg = odomMessage.twist.linear;
        // Vector3Msg rosOdomTwistAngularMsg = odomMessage.twist.angular;

        // 座標変換
        robotPosition = rosOdomPositionMsg.From<FLU>();
        robotPosture = rosOdomPostureMsg.From<FLU>();
    }

    /// <summary>
    /// 外部からオドメトリ（ROS上のオドメトリ情報をUnity座標に変換したもの）の位置情報にアクセスするための関数
    /// </summary>
    public Vector3 GetRobotPosition()
    {
        return robotPosition;

    }

    /// <summary>
    /// 外部からオドメトリ（ROS上のオドメトリ情報をUnity座標に変換したもの）の姿勢情報にアクセスするための関数
    /// </summary>
    public Quaternion GetRobotPosture()
    {
        return robotPosture;
    }
}