using System.Collections;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnitySensors.Sensor.LiDAR;

namespace MotionPlanningSim.Sensors
{
    /// <summary>
    /// UnitySensors' raycast lidar (same scan pattern, raycasts and point conversion, through the
    /// package's public jobs) with range noise drawn independently per point and scan
    /// (<see cref="LidarRangeNoise"/>) instead of the package's repeating sequence.
    /// </summary>
    public sealed class IidNoiseRaycastLiDARSensor : LiDARSensor
    {
        [SerializeField] private LayerMask _raycastLayerMask = 1;

        private Transform sensorTransform;
        private JobHandle jobHandle;
        private IUpdateRaycastCommandsJob updateRaycastCommandsJob;
        private LidarRangeNoiseJob noiseJob;
        private IRaycastHitsToPointsJob raycastHitsToPointsJob;
        private NativeArray<float3> directions;
        private NativeArray<RaycastCommand> raycastCommands;
        private NativeArray<RaycastHit> raycastHits;
        private NativeArray<float> noises;
        private bool initialized;

        protected override void Init()
        {
            if (scanPattern == null)
            {
                Debug.LogWarning("IidNoiseRaycastLiDARSensor: no scan pattern; the sensor stays idle.");
                return;
            }

            Initialize();
        }

        public override void Initialize()
        {
            base.Initialize();
            sensorTransform = transform;

            // The pattern is stored twice so a scan window can wrap without a modulo per point.
            directions = new NativeArray<float3>(scanPattern.size * 2, Allocator.Persistent);
            for (var i = 0; i < scanPattern.size; i++)
            {
                directions[i] = directions[i + scanPattern.size] = scanPattern.scans[i];
            }

            raycastCommands = new NativeArray<RaycastCommand>(pointsNum, Allocator.Persistent);
            raycastHits = new NativeArray<RaycastHit>(pointsNum, Allocator.Persistent);
            noises = new NativeArray<float>(pointsNum, Allocator.Persistent);

            updateRaycastCommandsJob = new IUpdateRaycastCommandsJob
            {
                origin = sensorTransform.position,
                rotation = sensorTransform.rotation,
                maxRange = maxRange,
                queryParameters = new QueryParameters { layerMask = _raycastLayerMask },
                directions = directions,
                indexOffset = 0,
                raycastCommands = raycastCommands,
            };
            noiseJob = new LidarRangeNoiseJob
            {
                seed = (uint)System.Environment.TickCount,
                scan = 0,
                sigma = gaussianNoiseSigma,
                noises = noises,
            };
            raycastHitsToPointsJob = new IRaycastHitsToPointsJob
            {
                minRange = minRange,
                sqrMinRange = minRange * minRange,
                maxRange = maxRange,
                maxIntensity = maxIntensity,
                directions = directions,
                indexOffset = 0,
                raycastHits = raycastHits,
                noises = noises,
                points = pointCloud.points,
            };
            initialized = true;
        }

        protected override IEnumerator UpdateSensor()
        {
            if (!initialized)
            {
                yield break;
            }

            updateRaycastCommandsJob.origin = sensorTransform.position;
            updateRaycastCommandsJob.rotation = sensorTransform.rotation;

            var commands = updateRaycastCommandsJob.Schedule(pointsNum, 1024);
            var noise = noiseJob.Schedule(pointsNum, 1024, commands);
            var raycasts = RaycastCommand.ScheduleBatch(raycastCommands, raycastHits, 1024, noise);
            jobHandle = raycastHitsToPointsJob.Schedule(pointsNum, 1024, raycasts);
            jobHandle.Complete();

            noiseJob.scan += 1;
            updateRaycastCommandsJob.indexOffset = (updateRaycastCommandsJob.indexOffset + pointsNum) % scanPattern.size;
            raycastHitsToPointsJob.indexOffset = (raycastHitsToPointsJob.indexOffset + pointsNum) % scanPattern.size;
            yield return null;
        }

        protected override void OnSensorDestroy()
        {
            jobHandle.Complete();
            if (!initialized)
            {
                return;  // an idle sensor allocated nothing, not even the point cloud
            }

            noises.Dispose();
            directions.Dispose();
            raycastCommands.Dispose();
            raycastHits.Dispose();
            base.OnSensorDestroy();
        }
    }
}
