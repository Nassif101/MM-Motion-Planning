using System;
using UnitySensors.Interface.Std;
using UnitySensors.Sensor;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    /// <summary>
    /// Latches the canonical physics time whenever a sensor starts a new sample.
    /// </summary>
    public sealed class SensorSampleTimeLatch
    {
        private bool hasSample;
        private float lastSensorTime;

        public double LatchedSeconds { get; private set; }

        /// <summary>Returns true when <paramref name="sensorTime"/> identifies a new sample.</summary>
        public bool Observe(float sensorTime, double physicsTimeSeconds)
        {
            if (hasSample && sensorTime == lastSensorTime)
            {
                return false;
            }

            hasSample = true;
            lastSensorTime = sensorTime;
            LatchedSeconds = physicsTimeSeconds;
            return true;
        }
    }

    /// <summary>
    /// UnitySensors header time source that reports the physics-clock tick of the
    /// sensor's latest sample instead of render-frame <c>Time.time</c>.
    /// </summary>
    /// <remarks>
    /// UnitySensors samples in a coroutine after <c>Update</c>; raycasts query the
    /// physics state of the most recent <c>FixedUpdate</c>, whose canonical time is
    /// <see cref="RosTimeUtility.PhysicsTimeSeconds"/>. Latching in <c>LateUpdate</c>
    /// pairs each buffered sample with that tick until the next sample replaces it.
    /// <see cref="ITimeInterface"/> is single precision (about 0.24 ms resolution
    /// after one simulated hour).
    /// </remarks>
    [DisallowMultipleComponent]
    public sealed class PhysicsClockSensorTime : MonoBehaviour, ITimeInterface
    {
        [SerializeField]
        private UnitySensor sensor;

        private readonly SensorSampleTimeLatch latch = new SensorSampleTimeLatch();

        public float time => (float)latch.LatchedSeconds;

        public void Configure(UnitySensor configuredSensor)
        {
            sensor = configuredSensor != null
                ? configuredSensor
                : throw new ArgumentNullException(nameof(configuredSensor));
        }

        private void Awake()
        {
            if (sensor == null)
            {
                throw new InvalidOperationException(
                    "PhysicsClockSensorTime requires the sampled UnitySensor.");
            }
        }

        private void LateUpdate()
        {
            latch.Observe(sensor.time, RosTimeUtility.PhysicsTimeSeconds);
        }
    }
}
