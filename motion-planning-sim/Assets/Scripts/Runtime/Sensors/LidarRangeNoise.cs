using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Random = Unity.Mathematics.Random;

namespace MotionPlanningSim.Sensors
{
    /// <summary>
    /// Gaussian lidar range noise, independent for every point of every scan.
    /// UnitySensors' IUpdateGaussianNoisesJob copies one Random state into every scheduled job,
    /// so each scan restarts the same sequence: a 20000-point Mid-360 scan held about 3200
    /// distinct values, identical from scan to scan, and a beam's noise recurred whenever the
    /// scan pattern did (a large draw on the arm mount leaked through the self-filter in the
    /// same spot every cycle). Here the noise is a pure function of (seed, scan, index).
    /// </summary>
    public static class LidarRangeNoise
    {
        public static float Sample(uint seed, uint scan, int index, float sigma)
        {
            var random = Random.CreateFromIndex(math.hash(new uint3(seed, scan, (uint)index)));
            // Box-Muller; 1 - u keeps the logarithm finite.
            var u1 = 1.0f - random.NextFloat();
            var u2 = random.NextFloat();
            return sigma * math.sqrt(-2.0f * math.log(u1)) * math.cos(2.0f * math.PI * u2);
        }
    }

    [BurstCompile]
    public struct LidarRangeNoiseJob : IJobParallelFor
    {
        public uint seed;
        public uint scan;
        public float sigma;
        [WriteOnly] public NativeArray<float> noises;

        public void Execute(int index)
        {
            noises[index] = LidarRangeNoise.Sample(seed, scan, index, sigma);
        }
    }
}
