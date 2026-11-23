import csv
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from common import ROOT, bridge, rows


# Independent WGS84 differential scales at the equator (metres/degree).
EAST_SCALE = 111319.49079327357
NORTH_SCALE = 110574.27582159435
DEMO = os.environ.get("KPI4_DEMO", str(ROOT / "build" / "kpi4_demo"))


def guide(end=(100, 0), position=(20, 10), velocity=(0, 8), **options):
    values = [end[1] / NORTH_SCALE, end[0] / EAST_SCALE,
              position[1] / NORTH_SCALE, position[0] / EAST_SCALE,
              velocity[0], velocity[1], options.get("speed", 8),
              options.get("max_speed", 12), options.get("deadband", 0.1),
              options.get("min_segment", 1), options.get("reach", 3),
              options.get("lookahead", 5), options.get("horizon", 1),
              options.get("cruise", 8), options.get("origin_lat", 0),
              options.get("origin_lon", 0)]
    for index, value in options.get("raw", {}).items():
        values[index] = value
    record = rows(bridge("guide " + " ".join(map(str, values))), "GUIDE")[0]
    fields = ("length", "progress", "cross", "distance", "closest_east", "closest_north",
              "north", "east")
    return dict(status=int(record[0]), correcting=record[1] == "1", limited=record[2] == "1",
                **{name: None if value == "-" else float(value)
                   for name, value in zip(fields, record[3:])})


class GuidanceTests(unittest.TestCase):
    def test_signed_cross_track_and_endpoint_distance(self):
        g = guide()
        self.assertEqual(g["status"], 2)
        self.assertTrue(g["correcting"])
        self.assertAlmostEqual(g["cross"], 10)
        self.assertAlmostEqual(g["progress"], 0.2)
        self.assertAlmostEqual(g["distance"], math.hypot(80, 10))
        self.assertAlmostEqual(g["closest_east"], 20)
        self.assertAlmostEqual(g["closest_north"], 0)
        self.assertLess(g["north"], 0)
        self.assertGreater(g["east"], 0)
        self.assertAlmostEqual(math.hypot(g["north"], g["east"]), 8)

    def test_rotated_and_reversed_segments(self):
        for end, position, cross, direction, sign in (
            ((100, 0), (20, -10), -10, "north", 1),
            ((-100, 0), (-20, 10), -10, "north", -1),
            ((0, 100), (10, 20), -10, "east", -1),
            ((0, -100), (-10, -20), -10, "east", 1),
        ):
            with self.subTest(end=end, position=position):
                g = guide(end, position)
                self.assertAlmostEqual(g["cross"], cross)
                self.assertGreater(sign * g[direction], 0)

    def test_along_track_motion_and_clamped_projection(self):
        g = guide(position=(-10, 0))
        self.assertEqual(g["status"], 1)
        self.assertFalse(g["correcting"])
        self.assertAlmostEqual(g["closest_east"], 0)
        self.assertAlmostEqual(g["east"], 8)
        self.assertAlmostEqual(g["north"], 0)
        beyond = guide(position=(110, 0))
        self.assertEqual(beyond["status"], 2)
        self.assertAlmostEqual(beyond["closest_east"], 100)
        self.assertLess(beyond["east"], 0)

    def test_reached_means_distance_to_endpoint(self):
        for position in ((100, 0), (99, 1)):
            g = guide(position=position)
            self.assertEqual(g["status"], 8)
            self.assertIsNone(g["east"])
            self.assertFalse(g["correcting"])
        self.assertNotEqual(guide(position=(20, 0))["status"], 8)
        self.assertNotEqual(guide(position=(100, 4))["status"], 8)

    def test_deadband_and_speed_aware_lookahead(self):
        self.assertFalse(guide(position=(20, 0.05))["correcting"])
        self.assertTrue(guide(position=(20, 0.2))["correcting"])
        slow = guide(velocity=(0, 2))
        fast = guide(velocity=(0, 20))
        self.assertGreater(abs(slow["north"]), abs(fast["north"]))
        self.assertGreater(fast["east"], slow["east"])

    def test_speed_limit_and_fallback(self):
        g = guide(speed=40)
        self.assertTrue(g["limited"])
        self.assertAlmostEqual(math.hypot(g["north"], g["east"]), 12)
        fallback = guide(speed="-", cruise=6)
        self.assertAlmostEqual(math.hypot(fallback["north"], fallback["east"]), 6)

    def test_reject_bad_mission_without_target(self):
        for kwargs in ({"end": (0, 0)}, {"end": (0.5, 0)}, {"end": (2100, 0)},
                       {"speed": 0}, {"speed": -1}, {"speed": "nan"},
                       {"origin_lat": 81}, {"raw": {0: "nan"}}):
            with self.subTest(kwargs=kwargs):
                g = guide(**kwargs)
                self.assertEqual(g["status"], 3)
                self.assertIsNone(g["east"])
                self.assertIsNone(g["cross"])

    def test_reject_bad_state_without_target(self):
        for kwargs in ({"position": (2100, 0)}, {"velocity": ("-", "-")},
                       {"velocity": ("nan", 0)}, {"velocity": (0, "inf")},
                       {"raw": {2: 91}}, {"raw": {3: "nan"}}):
            with self.subTest(kwargs=kwargs):
                g = guide(**kwargs)
                self.assertEqual(g["status"], 5)
                self.assertIsNone(g["north"])

    def test_reject_bad_configuration_without_target(self):
        for key in ("min_segment", "reach", "lookahead", "horizon", "cruise", "max_speed"):
            for value in (0, -1, "nan", "inf"):
                with self.subTest(key=key, value=value):
                    g = guide(**{key: value})
                    self.assertEqual(g["status"], 4)
                    self.assertIsNone(g["east"])
        self.assertEqual(guide(deadband=-1)["status"], 4)
        self.assertEqual(guide(deadband="nan")["status"], 4)


class ProjectionTests(unittest.TestCase):
    def test_metres_degrees_and_dateline(self):
        result = rows(bridge("project 0 0 0.001 0.001",
                             "project 0 179.999 0 -179.999"), "PROJECT")
        self.assertAlmostEqual(float(result[0][0]), EAST_SCALE * 0.001)
        self.assertAlmostEqual(float(result[0][1]), NORTH_SCALE * 0.001)
        self.assertAlmostEqual(float(result[1][0]), EAST_SCALE * 0.002, places=6)
        self.assertAlmostEqual(float(result[1][1]), 0)

    def test_roundtrip_and_range_rejections(self):
        p = rows(bridge("unproject 50 30 100 -40"), "PROJECT")[0]
        q = rows(bridge(f"project 50 30 {p[0]} {p[1]}"), "PROJECT")[0]
        self.assertAlmostEqual(float(q[0]), 100, places=6)
        self.assertAlmostEqual(float(q[1]), -40, places=6)
        invalid = ("project 81 0 81 0", "project 0 0 nan 0", "project 0 0 0 1",
                   "project 0 181 0 0", "unproject 0 0 2001 0", "unproject 0 0 0 inf")
        for row in rows(bridge(*invalid), "PROJECT"):
            self.assertEqual(row, ["-"])


class DemoTests(unittest.TestCase):
    def run_demo(self, directory, wind):
        result = subprocess.run((DEMO, "--output", str(directory), "--wind", str(wind)),
                                text=True, capture_output=True, check=True)
        self.assertIn("result=PASS", result.stdout)
        self.assertEqual(result.stderr, "")
        with (directory / "summary.csv").open() as stream:
            summary = {row["mode"]: row for row in csv.DictReader(stream)}
        paths = {}
        for mode in ("baseline", "corrected"):
            with (directory / f"{mode}.csv").open() as stream:
                paths[mode] = list(csv.DictReader(stream))
        return summary, paths

    def test_closed_loop_with_opposite_disturbances_and_no_wind(self):
        with tempfile.TemporaryDirectory() as tmp:
            for wind in (2, -2, 0):
                with self.subTest(wind=wind):
                    summary, paths = self.run_demo(Path(tmp) / str(wind), wind)
                    fixed = summary["corrected"]
                    self.assertEqual(fixed["reached"], "1")
                    self.assertLessEqual(float(fixed["final_distance_m"]), 3)
                    self.assertLessEqual(float(fixed["max_cross_track_m"]), 5)
                    if wind:
                        self.assertEqual(summary["baseline"]["reached"], "0")
                        self.assertEqual(summary["baseline"]["crossed_end"], "1")
                        self.assertGreater(float(summary["baseline"]["max_cross_track_m"]), 20)
                        self.assertLess(float(fixed["max_cross_track_m"]), 3)
                        last_moving = paths["corrected"][-2]
                        self.assertLess(wind * float(last_moving["command_north_mps"]), 0)
                    else:
                        self.assertEqual(summary["baseline"]["reached"], "1")
                        self.assertEqual(paths["baseline"], [
                            {**r, "status": "open_loop" if r["status"] == "on_course" else r["status"]}
                            for r in paths["corrected"]])
                    self.assertTrue(all(r["correction_applied"] == "0" for r in paths["baseline"]))
                    for records in paths.values():
                        for i, r in enumerate(records):
                            self.assertAlmostEqual(float(r["time_s"]), i * 0.05)
                            self.assertTrue(all(math.isfinite(float(v)) for k, v in r.items() if k != "status"))
                            self.assertLessEqual(math.hypot(float(r["command_east_mps"]),
                                                          float(r["command_north_mps"])), 12.000001)

    def test_repeatable_and_bad_arguments(self):
        with tempfile.TemporaryDirectory() as tmp:
            first, second = Path(tmp) / "a", Path(tmp) / "b"
            self.run_demo(first, 2)
            self.run_demo(second, 2)
            for name in ("baseline.csv", "corrected.csv", "summary.csv"):
                self.assertEqual((first / name).read_bytes(), (second / name).read_bytes())
            for value in ("nan", "inf", "2junk", "6"):
                result = subprocess.run((DEMO, "--output", str(Path(tmp) / "invalid"),
                                         "--wind", value), capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
            self.assertFalse((Path(tmp) / "invalid").exists())


if __name__ == "__main__":
    unittest.main()
