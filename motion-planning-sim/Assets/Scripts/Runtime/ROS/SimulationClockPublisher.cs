using RosMessageTypes.Rosgraph;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    [DefaultExecutionOrder(-200), DisallowMultipleComponent]
    public sealed class SimulationClockPublisher : MonoBehaviour
    {
        [SerializeField]
        private string topicName = "/clock";

        [SerializeField, Min(1.0f)]
        private float frequencyHz = 50.0f;

        [SerializeField, Min(0.0f)]
        private float startupDelaySeconds = 0.5f;

        private ROSConnection ros;
        public long PublishedTicks { get; private set; }
        public double LastPublishedTime { get; private set; }
        private double nextPublishTime;
        private double previousTime;
        private readonly PhysicsStepClock physicsClock = new PhysicsStepClock();

        private void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<ClockMsg>(topicName);
            var now = Time.fixedTimeAsDouble;
            RosTimeUtility.PhysicsTimeSeconds=now;
            nextPublishTime = now + startupDelaySeconds;
            previousTime = now;
        }

        public void Configure(float frequency, float startupDelay)
        {
            frequencyHz = Mathf.Max(1.0f, frequency);
            startupDelaySeconds = Mathf.Max(0.0f, startupDelay);
        }

        private void FixedUpdate()
        {
            var now = physicsClock.Advance(Time.fixedTimeAsDouble,Time.fixedDeltaTime);
            RosTimeUtility.PhysicsTimeSeconds=now;
            if (!PublicationSchedule.IsDue(
                    now,
                    frequencyHz,
                    ref nextPublishTime,
                    ref previousTime))
            {
                return;
            }

            // Each queued message owns its stamp, including catch-up physics steps.
            ros.Publish(topicName, new ClockMsg(RosTimeUtility.FromSeconds(now)));
            LastPublishedTime=now;
            ++PublishedTicks;
        }
    }
}
