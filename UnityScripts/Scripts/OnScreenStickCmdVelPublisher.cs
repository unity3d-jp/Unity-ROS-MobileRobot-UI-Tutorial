using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using TwistMsg = RosMessageTypes.Geometry.TwistMsg;

public class OnScreenStickCmdVelPublisher : MonoBehaviour
{
    [SerializeField] private InputAction _moveAction;

    [SerializeField] string topicName = "cmd_vel";
    private ROSConnection ros;
    private TwistMsg cmdVelMessage = new TwistMsg();

    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
        StopPublish();

        _moveAction.performed += _ =>
        {
            var value = _moveAction.ReadValue<Vector2>();
            Debug.Log($"Move: ({value.x}, {value.y})");
            cmdVelMessage.linear.x = value.y * 0.2f;
            cmdVelMessage.angular.z = value.x * -1.0f;
            Publish();
        };

        _moveAction.canceled += _ =>
        {
            StopPublish();
        };
    }

    private void OnEnable()
    {
        _moveAction.Enable();
    }

    private void OnDisable()
    {
        _moveAction.Disable();
    }

    private void OnDestroy()
    {
        _moveAction.Dispose();
    }

    private void StopPublish()
    {
        cmdVelMessage.linear.x = 0.0f;
        cmdVelMessage.angular.z = 0.0f;
        Publish();
    }

    private void Publish()
    {
        ros.Publish(topicName, cmdVelMessage);
    }
}
