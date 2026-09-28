using System;
using System.Collections.Generic;
using System.Linq;
using MotionPlanningSim.ROS;
using UnityEngine;

namespace MotionPlanningSim.Environment
{
    /// <summary>Aggregates robot-environment contacts reported per physics tick.</summary>
    public sealed class ContactLedger
    {
        public sealed class Entry
        {
            public string RobotCollider;
            public string Other;
            public float MaxPenetration;
            public double FirstTime;
            public int Ticks;
        }

        private readonly Dictionary<(string, string), Entry> entries = new Dictionary<(string, string), Entry>();

        public IReadOnlyCollection<Entry> Entries => entries.Values;

        public void Record(string robotCollider, string other, float penetration, double time)
        {
            if (!entries.TryGetValue((robotCollider, other), out var entry))
            {
                entry = new Entry { RobotCollider = robotCollider, Other = other, FirstTime = time };
                entries[(robotCollider, other)] = entry;
            }

            entry.MaxPenetration = Math.Max(entry.MaxPenetration, penetration);
            entry.Ticks++;
        }

        public void Clear() => entries.Clear();
    }

    /// <summary>
    /// Detects penetration between the robot's colliders (including the payload) and any
    /// other non-trigger collider except the named ground, every physics tick. Used by the
    /// navigation scenario runner to fail runs with contact; it never alters physics.
    /// </summary>
    [DisallowMultipleComponent]
    public sealed class RobotContactMonitor : MonoBehaviour
    {
        private const float MinimumPenetration = 0.001f;

        [SerializeField]
        private string groundName = "CompactedSiteGround";

        private readonly ContactLedger ledger = new ContactLedger();
        private readonly Collider[] candidates = new Collider[64];
        private Collider[] robotColliders = Array.Empty<Collider>();
        private HashSet<Collider> robotSet = new HashSet<Collider>();

        public ContactLedger Ledger => ledger;
        public long Ticks { get; private set; }
        public bool BufferOverflowed { get; private set; }

        public void ResetMonitor()
        {
            robotColliders = GetComponentsInChildren<Collider>(false)
                .Where(c => c.enabled && !c.isTrigger).ToArray();
            robotSet = new HashSet<Collider>(robotColliders);
            ledger.Clear();
            Ticks = 0;
            BufferOverflowed = false;
        }

        private void Awake() => ResetMonitor();

        private void FixedUpdate()
        {
            Ticks++;
            foreach (var mine in robotColliders)
            {
                if (mine == null || !mine.enabled)
                {
                    continue;
                }

                var bounds = mine.bounds;
                var count = Physics.OverlapBoxNonAlloc(bounds.center, bounds.extents, candidates,
                    Quaternion.identity, Physics.AllLayers, QueryTriggerInteraction.Ignore);
                BufferOverflowed |= count == candidates.Length;
                for (var index = 0; index < count; index++)
                {
                    var other = candidates[index];
                    if (robotSet.Contains(other) || other.name == groundName)
                    {
                        continue;
                    }

                    if (Physics.ComputePenetration(
                            mine, mine.transform.position, mine.transform.rotation,
                            other, other.transform.position, other.transform.rotation,
                            out _, out var distance) && distance > MinimumPenetration)
                    {
                        ledger.Record(mine.name, other.name, distance, RosTimeUtility.PhysicsTimeSeconds);
                    }
                }
            }
        }
    }
}
