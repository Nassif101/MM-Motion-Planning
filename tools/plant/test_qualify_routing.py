"""Analytical fixtures, independent geometry and bounded process handling; no planner trials."""

import dataclasses
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import qualify_routing as qr


class QualifyRoutingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.primitives = self.root / "source_primitives.json"
        self.primitives.write_bytes(b'{"synthetic_test_input":true}\n')
        self.directory = self.root / "qualification"
        self.directory.mkdir()
        self.fixtures = qr.generate_fixtures(self.directory, self.primitives)

    def fixture(self, name):
        return next(item for item in self.fixtures if item.name == name)

    @staticmethod
    def found(fixture, poses=None):
        poses = [list(p) for p in (poses or [fixture.start, fixture.goal])]
        return {"status": "found", "kind": "stationary" if fixture.name == "heading_only" else "translating",
                "requested_goal": list(fixture.goal), "resolved_goal": list(fixture.goal), "poses": poses,
                "cost": 1.0, "length_m": sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                             for a, b in zip(poses, poses[1:])),
                "connector_policy": "test"}

    @staticmethod
    def capture(code=0, timed_out=False, output_limited=False):
        return {"returncode": code, "timed_out": timed_out, "output_limited": output_limited,
                "error": None, "elapsed_s": 0.01, "captured_bytes": 0}

    def fake_process(self, raw, code=0, timed_out=False, output_limited=False):
        def run(command, directory, timeout_s):
            self.assertEqual(Path(command[1]).parent, directory.resolve())
            (directory / "stdout.bin").write_bytes(raw)
            (directory / "stderr.bin").write_bytes(b"retained diagnostic\n")
            return self.capture(code, timed_out, output_limited)
        return run

    def test_fixture_bundle_is_fixed_portable_and_has_correct_pgm_orientation(self):
        self.assertEqual(len(self.fixtures), 8)
        manifest = json.loads((self.directory / "fixtures.json").read_text())
        self.assertEqual(manifest["primitive_sha256"], hashlib.sha256(self.primitives.read_bytes()).hexdigest())
        self.assertEqual(manifest["footprint"], [[0.1, 0.08], [0.1, -0.08], [-0.1, -0.08], [-0.1, 0.08]])
        header = b"P5\n60 60\n255\n"
        raw = (self.fixture("detour").case_path.parent / "map.pgm").read_bytes()
        self.assertEqual(raw[:len(header)], header)
        pixels = raw[len(header):]
        self.assertEqual(len(pixels), 60 * 60)
        self.assertEqual(pixels[(59 - 0) * 60 + 29], 0)  # south is the last PGM row
        self.assertEqual(pixels[29], 255)  # wall ends below the north edge
        for fixture in self.fixtures:
            case = json.loads(fixture.case_path.read_text())
            self.assertFalse(Path(case["map"]).is_absolute())
            self.assertFalse(Path(case["primitives"]).is_absolute())
            self.assertEqual((fixture.case_path.parent / case["primitives"]).read_bytes(), self.primitives.read_bytes())

    def test_independent_geometry_checks_interior_crossings_and_closed_boundaries(self):
        horizontal = [(-2, -0.1), (2, -0.1), (2, 0.1), (-2, 0.1)]
        vertical = [(-0.1, -2), (0.1, -2), (0.1, 2), (-0.1, 2)]
        self.assertFalse(any(qr.point_in_polygon(p, vertical) for p in horizontal))
        self.assertFalse(any(qr.point_in_polygon(p, horizontal) for p in vertical))
        self.assertTrue(qr.polygons_intersect(horizontal, vertical))
        self.assertTrue(qr.polygons_intersect([(0, 0), (1, 0), (1, 1), (0, 1)],
                                             [(1, 1), (2, 1), (2, 2), (1, 2)]))
        self.assertFalse(qr.segments_intersect((0, 0), (1, 0), (2, 0), (3, 0)))

    def test_validator_finds_a_full_cell_inside_footprint_without_vertex_contact(self):
        fixture = dataclasses.replace(self.fixture("open"), start=(0, 0, 0), goal=(0, 0, 0),
                                      occupied=frozenset({(30, 30)}),
                                      footprint=((-0.4, -0.4), (0.4, -0.4), (0.4, 0.4), (-0.4, 0.4)))
        checked = qr.validate_path(fixture, self.found(fixture))
        self.assertFalse(checked["pass"])
        self.assertIn("occupied cell", checked["reason"])

    def test_validator_checks_between_clear_path_endpoints(self):
        fixture = self.fixture("impassable_wall")
        self.assertIsNone(qr._pose_collision(fixture, fixture.start))
        self.assertIsNone(qr._pose_collision(fixture, fixture.goal))
        checked = qr.validate_path(fixture, self.found(fixture))
        self.assertFalse(checked["pass"])
        self.assertIn("occupied cell", checked["reason"])
        self.assertGreater(checked["samples_checked"], 1)

    def test_yaw_interpolation_uses_short_turn_across_wrap(self):
        fixture = dataclasses.replace(self.fixture("heading_only"), start=(0, 0, 3.12), goal=(0, 0, -3.12))
        checked = qr.validate_path(fixture, self.found(fixture))
        self.assertTrue(checked["pass"], checked)
        self.assertLess(checked["samples_checked"], 10)

    def test_validator_rejects_bad_endpoints_nonfinite_and_false_length(self):
        fixture = self.fixture("open")
        result = self.found(fixture)
        self.assertTrue(qr.validate_path(fixture, result)["pass"])
        result["poses"][-1][0] += 0.01
        self.assertFalse(qr.validate_path(fixture, result)["pass"])
        result = self.found(fixture)
        result["poses"][0][2] = float("nan")
        self.assertFalse(qr.validate_path(fixture, result)["pass"])
        result = self.found(fixture)
        result["length_m"] = 0.0
        self.assertFalse(qr.validate_path(fixture, result)["pass"])

    def test_expected_no_path_and_invalid_goal_accept_typed_nonzero_exit(self):
        for name, status in (("impassable_wall", "no_path"), ("outside_goal", "invalid_goal")):
            raw = json.dumps({"status": status, "reason": "fixture", "poses": []}).encode()
            with patch.object(qr, "capture_process", self.fake_process(raw, code=2)):
                result = qr.run_case(Path(sys.executable), self.fixture(name), 30)
            self.assertTrue(result["pass"], result)
            self.assertEqual(result["result"]["poses"], [])
            self.assertIsNone(result["validation"])

    def test_found_status_with_failure_exit_is_not_accepted(self):
        fixture = self.fixture("open")
        raw = json.dumps(self.found(fixture)).encode()
        with patch.object(qr, "capture_process", self.fake_process(raw, code=2)):
            result = qr.run_case(Path(sys.executable), fixture, 30)
        self.assertFalse(result["pass"])
        self.assertIn("requires exit 0", result["errors"][0])

    def test_malformed_ambiguous_and_nonfinite_process_json_is_retained_but_fails(self):
        for raw in (b"not JSON", b'{"status":[]}', b'{"status":"found","cost":NaN}',
                    b'{"status":"no_path","status":"found"}'):
            with patch.object(qr, "capture_process", self.fake_process(raw)):
                result = qr.run_case(Path(sys.executable), self.fixture("open"), 30)
            self.assertFalse(result["pass"])
            self.assertEqual((self.fixture("open").case_path.parent / "stdout.bin").read_bytes(), raw)

    def test_timeout_and_output_limit_cannot_pass_even_with_valid_prefix(self):
        raw = json.dumps(self.found(self.fixture("open"))).encode()
        for timed_out, limited in ((True, False), (False, True)):
            with patch.object(qr, "capture_process", self.fake_process(raw, timed_out=timed_out, output_limited=limited)):
                result = qr.run_case(Path(sys.executable), self.fixture("open"), 30)
            self.assertFalse(result["pass"])
            self.assertIsNone(result["result"])

    def test_real_subprocess_stub_is_killed_on_timeout(self):
        result = qr.capture_process([sys.executable, "-c", "import time; time.sleep(30)"], self.directory, 0.02)
        self.assertTrue(result["timed_out"])
        self.assertNotEqual(result["returncode"], 0)
        self.assertLess(result["elapsed_s"], 5.5)

    def test_real_subprocess_stub_output_is_bounded(self):
        result = qr.capture_process([sys.executable, "-c", "import sys; sys.stdout.buffer.write(b'x' * 10000)"],
                                    self.directory, 5.0, output_limit=64)
        self.assertTrue(result["output_limited"])
        captured = sum((self.directory / name).stat().st_size for name in ("stdout.bin", "stderr.bin"))
        self.assertLessEqual(captured, 64)

    def test_existing_evidence_is_not_overwritten(self):
        sentinel = self.directory / "retain.txt"
        sentinel.write_text("existing evidence")
        with self.assertRaises(ValueError):
            qr.run_suite(Path(sys.executable), self.primitives, self.directory)
        self.assertEqual(sentinel.read_text(), "existing evidence")

    def test_suite_report_records_inputs_limits_and_scope(self):
        target = self.root / "new_suite"
        with patch.object(qr, "run_case", lambda binary, fixture, timeout: {"name": fixture.name, "pass": True}):
            result = qr.run_suite(Path(sys.executable), self.primitives, target, 12.0)
        written = json.loads((target / "report.json").read_text())
        self.assertTrue(result["pass"])
        self.assertEqual(len(written["cases"]), 8)
        self.assertEqual(written["limits"]["timeout_s_per_case"], 12.0)
        self.assertIn("sampled evidence", written["validator"]["limit"])
        self.assertIn("not robot or Nav2", written["scope"])


if __name__ == "__main__":
    unittest.main()
