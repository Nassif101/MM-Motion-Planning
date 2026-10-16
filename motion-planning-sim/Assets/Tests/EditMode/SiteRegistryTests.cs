using System;
using MotionPlanningSim.Environment;
using NUnit.Framework;

namespace MotionPlanningSim.Tests
{
    public class SiteRegistryTests
    {
        [Test]
        public void V1GridIsUnchanged()
        {
            var site = SiteRegistry.Get("construction_site");
            Assert.That(site.ScenePath, Is.EqualTo("Assets/Scenes/ConstructionSiteV1.unity"));
            Assert.That(site.Grid.MinRosX, Is.EqualTo(-20.0f));
            Assert.That(site.Grid.MinRosY, Is.EqualTo(-20.0f));
            Assert.That(site.Grid.MaxRosX, Is.EqualTo(20.0f));
            Assert.That(site.Grid.MaxRosY, Is.EqualTo(20.0f));
            Assert.That(site.Grid.Resolution, Is.EqualTo(0.05f));
        }

        [Test]
        public void V2GridIs50By40()
        {
            var site = SiteRegistry.Get("construction_site_v2");
            Assert.That(site.ScenePath, Is.EqualTo("Assets/Scenes/ConstructionSiteV2.unity"));
            Assert.That(site.Grid.MinRosX, Is.EqualTo(-25.0f));
            Assert.That(site.Grid.MaxRosX, Is.EqualTo(25.0f));
            Assert.That(site.Grid.Width, Is.EqualTo(1000));
            Assert.That(site.Grid.Height, Is.EqualTo(800));
        }

        [Test]
        public void UnknownSiteListsKnownSites()
        {
            var error = Assert.Throws<ArgumentException>(() => SiteRegistry.Get("nope"));
            Assert.That(error.Message, Does.Contain("construction_site"));
            Assert.That(error.Message, Does.Contain("construction_site_v2"));
        }
    }
}
