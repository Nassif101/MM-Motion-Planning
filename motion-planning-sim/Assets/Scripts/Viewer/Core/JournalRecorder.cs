using System;
using System.IO;
using Newtonsoft.Json.Linq;

namespace MotionPlanningSim.Viewer.Core
{
    /// <summary>
    /// The view journal across sessions (design 6.6): one file per session, continued only by the server's replay of
    /// the journaled session after a reconnect; a reset starts a new file even when its Hello is identical (Task 5
    /// review I3). Recording is optional: the first failure to open or write stops it for good and is kept in
    /// <see cref="Error"/>, and nothing is thrown to the caller, whose controls must keep working (review I1).
    /// </summary>
    public sealed class JournalRecorder : IDisposable
    {
        readonly Func<int, TextWriter> open;
        readonly Func<double> wallClock;
        ViewJournal journal;
        JObject journaled;
        JObject journaledExtra;

        /// <param name="open">Opens journal file n (from 1); it must never overwrite an existing file.</param>
        public JournalRecorder(Func<int, TextWriter> open, Func<double> wallClock)
        {
            this.open = open;
            this.wallClock = wallClock;
        }

        /// <summary>The journal files opened.</summary>
        public int Files { get; private set; }

        /// <summary>Why recording stopped, or null while it works.</summary>
        public string Error { get; private set; }

        /// <param name="extra">Header fields beyond the session's identity: a directing pass's log (spec 4C).</param>
        public void OnSession(JObject hello, bool replay, JObject extra = null)
        {
            if (Error != null) return;
            if (replay && journal != null && JToken.DeepEquals(journaled, hello)) return;
            Open(hello, extra);
        }

        /// <summary>A new take of the same session (spec 4C: a restart of a directing pass): a new file, the same header.
        /// Nothing without a session.</summary>
        public void NewTake()
        {
            if (Error != null || journaled == null) return;
            Open(journaled, journaledExtra);
        }

        void Open(JObject hello, JObject extra)
        {
            Close();
            try
            {
                var writer = open(Files + 1);
                ++Files;
                journal = new ViewJournal(writer, wallClock);
                journal.WriteHeader(hello, extra);
                journaled = hello;
                journaledExtra = extra;
            }
            catch (Exception e)
            {
                Fail(e);
            }
        }

        public void Write(double simTime, string kind, JObject fields = null, bool thinnable = false)
        {
            if (journal == null) return;
            try
            {
                journal.Write(simTime, kind, fields, thinnable);
            }
            catch (Exception e)
            {
                Fail(e);
            }
        }

        public void Dispose() => Close();

        void Fail(Exception e)
        {
            Error = e.Message;
            Close();
        }

        void Close()
        {
            var j = journal;
            journal = null;
            journaled = null;
            journaledExtra = null;
            try
            {
                j?.Dispose();
            }
            catch (Exception e)
            {
                Error ??= e.Message;  // a flush on close can fail too
            }
        }
    }
}
