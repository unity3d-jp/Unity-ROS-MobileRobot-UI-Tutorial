using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using OdometryMsg = RosMessageTypes.Nav.OdometryMsg;
using PointMsg = RosMessageTypes.Geometry.PointMsg;
using QuaternionMsg = RosMessageTypes.Geometry.QuaternionMsg;

// This script is adapted from https://github.com/YusukeKato/raspimouse_odometry_tuning_unity/blob/master/Assets/Scripts/OdometryTuner.cs
// which is released under the MIT License.
// Copyright (c) 2018 YusukeKato

/// <summary>
/// オドメトリ位置を可視化するためのクラス
/// （可視化のみを行い、座標変換を含むオドメトリの位置計算は予め行っている前提）
/// </summary>
public class OdometryViewer : MonoBehaviour
{

    [SerializeField] bool isDebugMode = true;
    [SerializeField] int lengthOfHistory = 10;
    [SerializeField] float odometryDistanceThreshold = 0.03f;
    [SerializeField] GameObject subscriberGameObject;

    [SerializeField] GameObject arrowPrefab;

    private List<GameObject> arrowList;

    private Vector3 previousRobotPosition = new Vector3();

    /// <summary>
    /// 初期化用のイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// ここで点群（オドメトリの矢印）用の配列を初期化
    /// </summary>
    void Start()
    {
        arrowList = new List<GameObject>(lengthOfHistory);

    }

    /// <summary>
    /// 1フレーム毎に呼び出されるイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// ここで点群の描画位置を決定
    /// </summary>
    void Update()
    {
        Vector3 currentRobotPosition = subscriberGameObject.GetComponent<OdomSubscriber>().GetRobotPosition();
        Quaternion currentRobotPosture = subscriberGameObject.GetComponent<OdomSubscriber>().GetRobotPosture();

        if (isDebugMode)
        {
            Debug.Log("diff: " + Vector3.Distance(currentRobotPosition, previousRobotPosition));
        }
        if (Vector3.Distance(currentRobotPosition, previousRobotPosition) > odometryDistanceThreshold)
        {
            GameObject arrow = Instantiate(arrowPrefab);
            arrow.transform.parent = gameObject.transform;
            arrow.transform.position = currentRobotPosition;
            arrow.transform.rotation = currentRobotPosture;
            arrow.transform.Rotate(0, 180, 0); // ロボットの向きに合わせて矢印を回転
            if (arrowList.Count >= lengthOfHistory) {
                Destroy(arrowList[0]);
                arrowList.RemoveRange(0, 1);
            }
            arrowList.Add(arrow);
            previousRobotPosition = currentRobotPosition;

            if (isDebugMode)
            {
                Debug.Log("arrow list length: " + arrowList.Count);
            }
        }


    }


}