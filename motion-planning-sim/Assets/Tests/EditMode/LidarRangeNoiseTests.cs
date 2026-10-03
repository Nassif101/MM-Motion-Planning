using System;
using System.Collections.Generic;
using System.Linq;
using MotionPlanningSim.Sensors;
using NUnit.Framework;
using Unity.Collections;
using Unity.Jobs;

namespace MotionPlanningSim.Tests
{
    public sealed class LidarRangeNoiseTests
    {
        private const float Sigma = 0.02f;
        private const int Points = 200000;

        private static float[] Scan(uint seed, uint scan) =>
            Enumerable.Range(0, Points).Select(i => LidarRangeNoise.Sample(seed, scan, i, Sigma)).ToArray();

        [Test]
        public void IsZeroMeanGaussianWithItsTails()
        {
            var noise = Scan(7, 0);
            var mean = noise.Average(x => (double)x);
            var sd = Math.Sqrt(noise.Average(x => (double)x * x) - mean * mean);
            Assert.That(Math.Abs(mean), Is.LessThan(0.0005));
            Assert.That(sd, Is.EqualTo(Sigma).Within(0.02 * Sigma));
            // Two-sided 3 sigma: 0.27 %; 4 sigma: 6.3e-5 (about 13 of 200000). The package
            // generator never went past about 3.5 sigma.
            Assert.That(noise.Count(x => Math.Abs(x) > 3 * Sigma) / (double)Points, Is.EqualTo(0.0027).Within(0.0005));
            Assert.That(noise.Count(x => Math.Abs(x) > 4 * Sigma), Is.InRange(3, 40));
        }

        [Test]
        public void EveryPointDrawsItsOwnValue()
        {
            // The package job produced about 3200 distinct values for a 20000-point scan.
            var distinct = new HashSet<float>(Scan(7, 0).Take(20000)).Count;
            Assert.That(distinct, Is.GreaterThan(19900));
        }

        [Test]
        public void ConsecutiveScansAreIndependent()
        {
            var first = Scan(7, 0);
            var second = Scan(7, 1);
            Assert.That(first.Zip(second, (a, b) => a == b).Count(same => same), Is.LessThan(5));
            var correlation = first.Zip(second, (a, b) => (double)a * b).Average() / (Sigma * Sigma);
            Assert.That(Math.Abs(correlation), Is.LessThan(0.02));
        }

        [Test]
        public void TheJobWritesTheSampleOfEachIndex()
        {
            var noises = new NativeArray<float>(1000, Allocator.TempJob);
            try
            {
                new LidarRangeNoiseJob { seed = 3, scan = 11, sigma = Sigma, noises = noises }
                    .Schedule(noises.Length, 64).Complete();
                for (var i = 0; i < noises.Length; i++)
                {
                    Assert.That(noises[i], Is.EqualTo(LidarRangeNoise.Sample(3, 11, i, Sigma)).Within(1e-6f));
                }
            }
            finally
            {
                noises.Dispose();
            }
        }
    }
}
