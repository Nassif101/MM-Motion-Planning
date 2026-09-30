using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

namespace MotionPlanningSim.ROS
{
    // Connects the scene's ROSConnection once every component has registered its topics.
    // ROS-TCP-Connector v0.7.0 builds queued registrations in one serializer shared by the
    // main thread and its connection thread, which re-sends all registrations on connect, so
    // topics registered while the connection comes up can interleave on the wire. The scene's
    // ROSConnection therefore does not connect on start (see MobileManipulatorRosSetup).
    [DisallowMultipleComponent]
    public sealed class RosConnectAfterStart : MonoBehaviour
    {
        private IEnumerator Start()
        {
            // Every scene component's OnEnable and Start have run by the next frame.
            yield return null;
            var ros = ROSConnection.GetOrCreateInstance();
            if (!ros.HasConnectionThread) ros.Connect();
        }
    }
}
