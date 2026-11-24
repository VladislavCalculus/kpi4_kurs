import csv
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from common import ROOT, bridge, rows

DEMO = os.environ.get("KPI4_DEMO", str(ROOT / "build" / "kpi4_demo"))
PLAN = "rinit 4 0 0 8 60 0 6 60 50 10 110 50 8"


class RouteFollowerTests(unittest.TestCase):
    def test_turns_use_next_speed_without_empty_command_cycle(self):
        r = rows(bridge(PLAN, "rstep 0 0 0 0", "rstep 60 0 0 6",
                        "rstep 60 50 10 0", "rstep 110 50 0 8",
                        "rstep 500 500 - -"), "ROUTE")
        self.assertEqual(r[0][:3], ["1", "0", "1"])
        self.assertAlmostEqual(float(r[0][4]), 6)
        self.assertEqual(r[1][:3], ["1", "1", "2"])
        self.assertAlmostEqual(float(r[1][3]), 10)
        self.assertAlmostEqual(float(r[1][4]), 0, places=5)
        self.assertEqual(r[2][:3], ["1", "2", "3"])
        self.assertAlmostEqual(float(r[2][4]), 8)
        self.assertEqual(r[3], ["9", "3", "-", "-", "-"])
        self.assertEqual(r[4], r[3])  # Completion is terminal, with no target.

    def test_invalid_state_does_not_advance_or_make_target(self):
        r = rows(bridge(PLAN, "rstep 60 0 - -", "rstep 60 0 nan 0",
                        "rstep 3000 0 0 0", "rstep 60 0 0 0"), "ROUTE")
        for record in r[:3]:
            self.assertEqual(record, ["5", "0", "1", "-", "-"])
        self.assertEqual(r[3][1:3], ["1", "2"])

    def test_crossing_endpoint_plane_does_not_skip_missed_waypoint(self):
        record = rows(bridge(PLAN, "rstep 70 0 0 8"), "ROUTE")[0]
        self.assertEqual(record[:3], ["2", "0", "1"])
        self.assertLess(float(record[4]), 0)

    def test_reject_entire_bad_route_before_motion(self):
        for plan in ("rinit 0", "rinit 1 0 0 8", "rinit 3 0 0 8 60 0 8 60 0 8",
                     "rinit 3 0 0 8 60 0 8 60 50 nan",
                     "rinit 3 0 0 8 60 0 8 3000 50 8"):
            with self.subTest(plan=plan):
                record = rows(bridge(plan, "rstep 0 0 0 0"), "ROUTE")[0]
                self.assertEqual(record, ["3", "0", "-", "-", "-"])

    def test_nearby_goals_and_speed_saturation(self):
        records = rows(bridge("rinit 3 0 0 8 4 0 8 8 0 8", "rstep 6 0 0 0"), "ROUTE")
        self.assertEqual(records[0], ["9", "2", "-", "-", "-"])
        record = rows(bridge("rinit 2 0 0 8 60 0 20", "rstep 0 0 0 0"), "ROUTE")[0]
        self.assertAlmostEqual(float(record[4]), 12)


class RouteDemoTests(unittest.TestCase):
    def run_demo(self, directory, *options):
        result = subprocess.run((DEMO, "--scenario", "route", "--output", str(directory), *options),
                                text=True, capture_output=True, check=True)
        self.assertIn("result=PASS", result.stdout)
        self.assertEqual(result.stderr, "")
        with (directory / "route.csv").open() as stream:
            records = list(csv.DictReader(stream))
        with (directory / "summary.csv").open() as stream:
            summary = next(csv.DictReader(stream))
        self.assertEqual(summary["completed"], "1")
        self.assertEqual(summary["completed_segments"], "3")
        self.assertLessEqual(float(summary["max_waypoint_distance_m"]), 4)
        self.assertLessEqual(float(summary["time_s"]), 60)
        self.assertEqual(records[-1]["status"], "mission_completed")
        self.assertEqual(records[-1]["command_available"], "0")
        self.assertEqual(records[-1]["active_segment"], "")
        for i, row in enumerate(records):
            self.assertAlmostEqual(float(row["time_s"]), i * 0.05)
            for key, value in row.items():
                if key not in ("status", "state_reason") and value:
                    self.assertTrue(math.isfinite(float(value)), (key, value))
            if row["command_available"] == "1":
                self.assertEqual(row["state_reason"], "accepted")
                self.assertLessEqual(math.hypot(float(row["command_east_mps"]),
                                               float(row["command_north_mps"])), 12.000001)
            else:
                self.assertEqual(row["command_east_mps"], "")
                self.assertEqual(row["command_north_mps"], "")
                self.assertEqual(row["correction_applied"], "0")
                self.assertEqual(float(row["model_target_east_mps"]), 0)
                self.assertEqual(float(row["model_target_north_mps"]), 0)
        # No skipped or repeated progression, including noisy/faulted runs.
        changes = [int(r["completed_segments"]) for i, r in enumerate(records)
                   if i == 0 or r["completed_segments"] != records[i - 1]["completed_segments"]]
        self.assertEqual(changes, [0, 1, 2, 3])
        return records, summary

    def test_nominal_noise_offsets_and_speeds(self):
        with tempfile.TemporaryDirectory() as tmp:
            for index, options in enumerate(((), ("--noise", "0.35"),
                    ("--speed", "6", "--offset", "-6", "--noise", "0.35"),
                    ("--speed", "10", "--offset", "6", "--noise", "0.35"))):
                with self.subTest(options=options):
                    records, summary = self.run_demo(Path(tmp) / str(index), *options)
                    self.assertLessEqual(float(summary["max_cross_track_m"]), 8)
                    self.assertEqual(summary["rejected_samples"], "0")
                    self.assertEqual(summary["missing_samples"], "0")
                    self.assertIn("corrected", {r["status"] for r in records})
            straight, _ = self.run_demo(Path(tmp) / "calm", "--wind", "0")
            self.assertIn("on_course", {r["status"] for r in straight})

    def test_fault_inhibits_commands_and_progress_then_recovers(self):
        with tempfile.TemporaryDirectory() as tmp:
            for fault, reason, status in (("stale", "measurement_stale", "stale_state"),
                    ("missing", "missing", "state_missing"), ("no_fix", "no_fix", "invalid_state")):
                with self.subTest(fault=fault):
                    records, summary = self.run_demo(Path(tmp) / fault, "--noise", "0.35", "--fault", fault)
                    bad = [r for r in records if 4 <= float(r["time_s"]) < 6]
                    self.assertEqual(len(bad), 40)
                    self.assertEqual({r["state_reason"] for r in bad}, {reason})
                    self.assertEqual({r["status"] for r in bad}, {status})
                    self.assertEqual({r["command_available"] for r in bad}, {"0"})
                    self.assertEqual({r["completed_segments"] for r in bad}, {records[79]["completed_segments"]})
                    self.assertEqual({r["active_segment"] for r in bad}, {records[79]["active_segment"]})
                    self.assertLess(float(bad[-1]["velocity_east_mps"]),
                                    0.05 * float(bad[0]["velocity_east_mps"]))
                    self.assertAlmostEqual(float(bad[-1]["velocity_north_mps"]), 2, delta=0.2)
                    if fault == "stale":
                        self.assertGreaterEqual(min(float(r["measurement_age_s"]) for r in bad), 0.5)
                        self.assertEqual(len({r["measured_east_m"] for r in bad}), 1)
                    elif fault == "missing":
                        self.assertEqual({r["measured_east_m"] for r in bad}, {""})
                    self.assertEqual(records[120]["state_reason"], "accepted")
                    self.assertEqual(records[120]["command_available"], "1")
                    self.assertEqual(summary["recovered"], "1")
                    self.assertEqual(int(summary["missing_samples"]), 40 if fault == "missing" else 0)
                    self.assertEqual(int(summary["rejected_samples"]), 0 if fault == "missing" else 40)

    def test_noise_seed_repeatability_and_argument_errors(self):
        with tempfile.TemporaryDirectory() as tmp:
            a, b, c = (Path(tmp) / name for name in ("a", "b", "c"))
            self.run_demo(a, "--noise", "0.35", "--seed", "42")
            self.run_demo(b, "--noise", "0.35", "--seed", "42")
            self.run_demo(c, "--noise", "0.35", "--seed", "43")
            self.assertEqual((a / "route.csv").read_bytes(), (b / "route.csv").read_bytes())
            self.assertNotEqual((a / "route.csv").read_bytes(), (c / "route.csv").read_bytes())
            for option, value in (("--seed", "-1"), ("--seed", "4294967296"),
                    ("--speed", "0"), ("--noise", "nan"), ("--noise", "-0.1"),
                    ("--noise", "1"), ("--offset", "21"), ("--fault", "bad"), ("--scenario", "bad")):
                result = subprocess.run((DEMO, "--scenario", "route", "--output", str(Path(tmp) / "bad"),
                                         option, value), capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
            self.assertFalse((Path(tmp) / "bad").exists())


if __name__ == "__main__":
    unittest.main()
