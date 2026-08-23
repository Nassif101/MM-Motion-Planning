using RosMessageTypes.Rosgraph;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    [DisallowMultipleComponent]
    public sealed class SimulationClockPublisher : MonoBehaviour
    {
        [SerializeField]
        private string topicName = "/clock";

        [SerializeField, Min(1.0f)]
        private float frequencyHz = 50.0f;

        [SerializeField, Min(0.0f)]
        private float startupDelaySeconds = 0.5f;

        private ROSConnection ros;
        private ClockMsg message;
        private double nextPublishTime;
        private double previousTime;

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<ClockMsg>(topicName);
            message = new ClockMsg();

            var now = Time.timeAsDouble;
            nextPublishTime = now + startupDelaySeconds;
            previousTime = now;
        }

        public void Configure(float frequency, float startupDelay)
        {
            frequencyHz = Mathf.Max(1.0f, frequency);
            startupDelaySeconds = Mathf.Max(0.0f, startupDelay);
        }

        private void Update()
        {
            var now = Time.timeAsDouble;
            if (!PublicationSchedule.IsDue(
                    now,
                    frequencyHz,
                    ref nextPublishTime,
                    ref previousTime))
            {
                return;
            }

            message.clock = RosTimeUtility.FromSeconds(now);
            ros.Publish(topicName, message);
        }
    }
}
