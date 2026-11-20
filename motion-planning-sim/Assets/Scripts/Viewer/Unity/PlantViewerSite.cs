using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace MotionPlanningSim.Viewer
{
    /// <summary>
    /// What the viewer scene was built from (design 6.1): the site model's checksum, compared with the session
    /// Hello's <c>site_checksum</c>, and the site model's geom names, so scenario boxes the site already has are
    /// not drawn twice.
    /// </summary>
    public sealed class PlantViewerSite : MonoBehaviour
    {
        [SerializeField] private string checksum = "";
        [SerializeField] private string[] siteGeoms = Array.Empty<string>();
        private HashSet<string> lookup;

        public string Checksum => checksum;
        public int SiteGeomCount => siteGeoms.Length;

        public void Configure(string siteChecksum, IEnumerable<string> geomNames)
        {
            checksum = siteChecksum ?? "";
            siteGeoms = geomNames?.ToArray() ?? Array.Empty<string>();
            lookup = null;
        }

        public bool HasSiteGeom(string name)
        {
            lookup ??= new HashSet<string>(siteGeoms, StringComparer.Ordinal);
            return lookup.Contains(name);
        }
    }
}
