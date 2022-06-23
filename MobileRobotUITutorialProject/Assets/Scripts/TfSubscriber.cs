using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
// 座標変換用
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

// 各種ROSトピック形式
using RosMessageTypes.Geometry;
using Tf = RosMessageTypes.Tf2.TFMessageMsg;
using RosPosVector3 = Unity.Robotics.ROSTCPConnector.ROSGeometry.Vector3<Unity.Robotics.ROSTCPConnector.ROSGeometry.FLU>;
using RosQuaternion = Unity.Robotics.ROSTCPConnector.ROSGeometry.Quaternion<Unity.Robotics.ROSTCPConnector.ROSGeometry.FLU>;

/// <summary>
/// ロボットの位置姿勢データ（TFMessageMsg）を受信するためのクラス
/// 位置姿勢データの取得、座標変換を行い、指定したGameObjectの位置姿勢を指定する用途を想定
/// </summary>
public class TfSubscriber : MonoBehaviour
{
    // Variables required for ROS communication
    [SerializeField] string rosTopicName = "tf";

    [SerializeField] GameObject mobileRobot;

    [SerializeField] bool isDebugMode = true;

    private int numRobotLinks = 4;

    private Transform[] robotLinkPositions;

    /// <summary>
    /// 初期化用のイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// </summary>
    void Start()
    {
        // // Get ROS connection static instance
        ROSConnection.GetOrCreateInstance().Subscribe<Tf>(rosTopicName, TfUpdate);

        robotLinkPositions = new Transform[numRobotLinks];

        string footprint_link = "base_footprint";
        robotLinkPositions[0] = mobileRobot.transform.Find(footprint_link).transform;

        string body_link = footprint_link + "/base_link";
        robotLinkPositions[1] = mobileRobot.transform.Find(body_link).transform;

        string right_wheel_link = body_link + "/right_wheel_link";
        robotLinkPositions[2] = mobileRobot.transform.Find(right_wheel_link).transform;

        string left_wheel_link = body_link + "/left_wheel_link";
        robotLinkPositions[3] = mobileRobot.transform.Find(left_wheel_link).transform;


        if (isDebugMode)
        {
            Debug.Log(robotLinkPositions[1].position + " " + robotLinkPositions[1].rotation);
        }
    }

    /// <summary>
    /// デバッグ時の文字列生成用関数
    /// </summary>
    string Vector3ToString(Vector3 vec, string titile = "")
    {
        return titile + " (" + vec[0] + ", " + vec[1] + ", " + vec[2] + ")";
    }

    /// <summary>
    /// デバッグ時の文字列生成用関数
    /// </summary>
    string RosPosVector3ToString(RosPosVector3 vec, string titile = "")
    {
        return titile + " (" + vec.x + ", " + vec.y + ", " + vec.z + ")";
    }

    /// <summary>
    /// ROSトピックを受け取った際に呼ばれるコールバック関数
    /// </summary>
    void TfUpdate(Tf tfMessage)
    {
        for (int i = 0; i < tfMessage.transforms.Length; i++)
        {
            if (tfMessage.transforms[i].child_frame_id == "base_footprint")
            {

                Vector3Msg rosPosMsg = tfMessage.transforms[i].transform.translation;
                QuaternionMsg rosQuaternionMsg = tfMessage.transforms[i].transform.rotation;

                // 座標変換
                // 座標変換に関するのドキュメント
                // https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/v0.5.0/ROSGeometry.md
                // 上記ドキュメントの実装箇所
                // https://github.com/Unity-Technologies/ROS-TCP-Connector/blob/v0.5.0/com.unity.robotics.ros-tcp-connector/Runtime/ROSGeometry/CoordinateSpaces.cs
                RosPosVector3 rosPos = rosPosMsg.As<FLU>();
                Vector3 unityPos = rosPos.toUnity;
                if (isDebugMode)
                {
                    Debug.Log(RosPosVector3ToString(rosPos, "tf(ros)") + " --> " + Vector3ToString(unityPos, "tf(unity)"));
                }

                RosQuaternion rosQuaternion = rosQuaternionMsg.As<FLU>();
                Quaternion unityQuaternion = rosQuaternion.toUnity;

                // 座標変換はこちらでもOK (1)
                // Vector3 unityPos = tfMessage.transforms[i].transform.translation.From<FLU>();
                // Quaternion unityQuaternion = tfMessage.transforms[i].transform.rotation.From<FLU>();

                // 座標変換はこちらでもOK (2)
                // Vector3 unityPos = new Vector3((float)-tfMessage.transforms[i].transform.translation.y,
                //                             (float)tfMessage.transforms[i].transform.translation.z,
                //                             (float)tfMessage.transforms[i].transform.translation.x);
                // Quaternion unityQuaternion = new Quaternion((float)-tfMessage.transforms[i].transform.rotation.y,
                //                                 (float)tfMessage.transforms[i].transform.rotation.z,
                //                                 (float)tfMessage.transforms[i].transform.rotation.x,
                //                                 (float)-tfMessage.transforms[i].transform.rotation.w);

                robotLinkPositions[0].position = unityPos;
                robotLinkPositions[0].rotation = unityQuaternion;
                if (isDebugMode)
                {
                    Debug.Log(Vector3ToString(unityPos, "pos"));
                }
            }
        }
    }
}
