using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// LiDARのスキャン情報を可視化するためのクラス
/// （可視化のみを行い、座標変換を含むスキャン結果の位置計算は予め行っている前提）
/// </summary>
public class PointCloudLocator : MonoBehaviour
{
    [SerializeField] GameObject pointPrefab;
    [SerializeField] float pointSize = 0.02f;
    [SerializeField] GameObject subscriberGameObject;
    [SerializeField] bool isDebugMode = true;

    private static readonly int MaxPointCount = 100;
    private GameObject[] points = new GameObject[MaxPointCount];

    /// <summary>
    /// 初期化用のイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// ここで点群用の配列を初期化
    /// </summary>
    void Start()
    {
        for (int i = 0; i < MaxPointCount; i++)
        {
            points[i] = Instantiate(pointPrefab);
            points[i].transform.parent = gameObject.transform;
            points[i].transform.localScale = Vector3.zero;
        }
    }

    /// <summary>
    /// 1フレーム毎に呼び出されるイベント関数
    /// https://docs.unity3d.com/ja/2020.3/Manual/ExecutionOrder.html
    /// ここで点群の描画位置を決定
    /// </summary>
    void Update()
    {
        float[] scanRanges = subscriberGameObject.GetComponent<LaserScanSubscriber>().GetScanRanges();
        float scanAngleMax = subscriberGameObject.GetComponent<LaserScanSubscriber>().GetScanAngleMax();
        float scanAngleMin = subscriberGameObject.GetComponent<LaserScanSubscriber>().GetScanAngleMin();

        if (scanRanges != null && scanRanges.Length > 0)
        {
            int divider = scanRanges.Length / MaxPointCount;

            for (int i = 0; i < MaxPointCount; i++)
            {
                float scanRange = scanRanges[divider * i];
                float scanAngle = (scanAngleMax - scanAngleMin) / MaxPointCount * i;
                points[i].transform.parent = gameObject.transform;
                if (scanRange != Mathf.Infinity)
                {
                    points[i].transform.localScale = new Vector3(pointSize, pointSize, pointSize);
                    points[i].transform.localPosition = new Vector3(scanRange * -Mathf.Sin(scanAngleMin + scanAngle), 0, scanRange * Mathf.Cos(scanAngleMin + scanAngle));
                }
                else
                {
                    points[i].transform.localScale = Vector3.zero;
                    points[i].transform.localPosition = Vector3.zero;
                }
                if (isDebugMode)
                {
                    Debug.Log("scanRanges.Length: " + scanRanges.Length);
                    Debug.Log("angle: " + scanAngle + ", range: " + scanRange);
                }
            }
        }
    }
}
