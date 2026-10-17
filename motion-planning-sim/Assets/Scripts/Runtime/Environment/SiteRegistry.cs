using System;
using System.Collections.Generic;
using System.Linq;

namespace MotionPlanningSim.Environment
{
    public sealed class SiteDefinition
    {
        public string Name;
        public string ScenePath;
        public Nav2GridSpec Grid;
        // Unity-height band (metres) whose colliders the 2D Nav2 map rasterizes.
        public float MapMinHeight;
        public float MapMaxHeight;
    }

    // The construction sites the map and box exporters know: scene and Nav2 grid per site name.
    public static class SiteRegistry
    {
        public const string DefaultSite = "construction_site";

        private static readonly SiteDefinition[] Sites =
        {
            new SiteDefinition
            {
                Name = "construction_site",
                ScenePath = "Assets/Scenes/ConstructionSiteV1.unity",
                Grid = new Nav2GridSpec(-20.0f, -20.0f, 20.0f, 20.0f, 0.05f),
                MapMinHeight = 0.02f,
                MapMaxHeight = 3.2f
            },
            new SiteDefinition
            {
                Name = "construction_site_v2",
                ScenePath = "Assets/Scenes/ConstructionSiteV2.unity",
                Grid = new Nav2GridSpec(-25.0f, -20.0f, 25.0f, 20.0f, 0.05f),
                // Base-band top (B5 base_band 0.60 m): overhead members stay out of the 2D route map.
                MapMinHeight = 0.02f,
                MapMaxHeight = 0.60f
            }
        };

        public static IReadOnlyList<SiteDefinition> All => Sites;

        public static SiteDefinition Get(string name)
        {
            var site = Sites.FirstOrDefault(s => s.Name == name);
            if (site == null)
            {
                throw new ArgumentException(
                    $"Unknown site '{name}'; known sites: {string.Join(", ", Sites.Select(s => s.Name))}.");
            }
            return site;
        }
    }
}
