using MotionPlanningSim.Environment;
using NUnit.Framework;
using UnityEngine;

namespace MotionPlanningSim.Tests
{
    public class Nav2MapGeometryTests
    {
        private static readonly Nav2GridSpec Grid = new Nav2GridSpec(-20, -20, 20, 20, 0.05f);

        [Test]
        public void ConstructionSiteGridIsEightHundredByEightHundred()
        {
            Assert.That(Grid.Width, Is.EqualTo(800));
            Assert.That(Grid.Height, Is.EqualTo(800));
        }

        [Test]
        public void UnityWorldAxesConvertToRosFluMapAxes()
        {
            Assert.That(
                Nav2MapGeometry.UnityWorldToRosMap(new Vector3(3, 7, 5)),
                Is.EqualTo(new Vector2(5, -3)));
            Assert.That(
                Nav2MapGeometry.RosMapToUnityWorld(new Vector2(5, -3), 7),
                Is.EqualTo(new Vector3(3, 7, 5)));
        }

        [Test]
        public void UnityBoundsProjectIntoExpectedRosCells()
        {
            var bounds = new Bounds(new Vector3(0, 1.5f, 19.8f), new Vector3(40, 3, 0.4f));

            Assert.That(
                Nav2MapGeometry.TryGetOverlappingCells(
                    bounds,
                    Grid,
                    0.02f,
                    3.2f,
                    out var minimumX,
                    out var minimumY,
                    out var maximumX,
                    out var maximumY),
                Is.True);
            Assert.That(minimumX, Is.EqualTo(792));
            Assert.That(maximumX, Is.EqualTo(799));
            Assert.That(minimumY, Is.EqualTo(0));
            Assert.That(maximumY, Is.EqualTo(799));
        }

        [Test]
        public void PgmRowsAreTopDownWhileOccupancyCellsAreBottomUp()
        {
            var cells = new[]
            {
                true, false,
                false, true
            };

            var pixels = Nav2MapEncoding.ToPgmImageRows(cells, 2, 2);

            Assert.That(pixels, Is.EqualTo(new byte[] { 254, 0, 0, 254 }));
        }
    }
}
