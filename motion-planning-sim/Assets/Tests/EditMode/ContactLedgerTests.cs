using System.Linq;
using MotionPlanningSim.Environment;
using NUnit.Framework;

namespace MotionPlanningSim.Tests
{
    public class ContactLedgerTests
    {
        [Test]
        public void KeepsMaximumPenetrationFirstTimeAndTickCountPerPair()
        {
            var ledger = new ContactLedger();
            ledger.Record("PayloadPanel", "GatePostLeft", 0.002f, 10.0);
            ledger.Record("PayloadPanel", "GatePostLeft", 0.010f, 10.02);
            ledger.Record("PayloadPanel", "GatePostLeft", 0.004f, 10.04);
            ledger.Record("base_link", "GatePostRight", 0.003f, 11.0);

            var panel = ledger.Entries.Single(e => e.Other == "GatePostLeft");
            Assert.That(panel.MaxPenetration, Is.EqualTo(0.010f));
            Assert.That(panel.FirstTime, Is.EqualTo(10.0));
            Assert.That(panel.Ticks, Is.EqualTo(3));
            Assert.That(ledger.Entries.Count, Is.EqualTo(2));

            ledger.Clear();
            Assert.That(ledger.Entries, Is.Empty);
        }
    }
}
