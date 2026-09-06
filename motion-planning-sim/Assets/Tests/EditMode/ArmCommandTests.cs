using System;
using MotionPlanningSim.Control;
using NUnit.Framework;
namespace MotionPlanningSim.Tests
{
    public class ArmCommandTests
    {
        [Test] public void ExplicitNamesOverridePacketOrdering()
        {
            Assert.That(ArmCommandValidation.Map(new[]{"a","b"},new[]{"b","a"},new[]{2.0,1.0},new[]{0.0,0.0}),Is.EqualTo(new[]{1,0}));
        }
        [Test] public void RejectsUnknownDuplicateAndNonfinite()
        {
            Assert.Throws<ArgumentException>(()=>ArmCommandValidation.Map(new[]{"a","b"},new[]{"a","a"},new[]{0.0,0.0},new[]{0.0,0.0}));
            Assert.Throws<ArgumentException>(()=>ArmCommandValidation.Map(new[]{"a"},new[]{"b"},new[]{0.0},new[]{0.0}));
            Assert.Throws<ArgumentException>(()=>ArmCommandValidation.Map(new[]{"a"},new[]{"a"},new[]{double.NaN},new[]{0.0}));
        }
        [Test] public void RadiansConvertOnlyAtDriveBoundary()
        {
            Assert.That(ArmCommandValidation.DriveDegrees(1,1),Is.EqualTo(57.2957795f).Within(1e-5));
            Assert.That(ArmCommandValidation.DriveDegrees(1,-1),Is.EqualTo(-57.2957795f).Within(1e-5));
            Assert.That(ArmCommandValidation.RosRadians(1,-1),Is.EqualTo(-1));
        }
    }
}
