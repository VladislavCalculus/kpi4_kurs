import unittest

from common import bridge, estimated, feed, gps, rows


def sample(command="direct", **changes):
    values = dict(source=1, measured=1000000, received=1020000, lat=50,
                  lon=30, alt=120, reference=0, north=2, east=-3,
                  heading=270, accuracy=4, fix=2)
    values.update(changes)
    names = ("source", "measured", "received", "lat", "lon", "alt",
             "reference", "north", "east", "heading", "accuracy", "fix")
    return command + " " + " ".join(str(values[name]) for name in names)


class StateTests(unittest.TestCase):
    def test_estimated_position(self):
        output = bridge(feed(estimated(), 2))
        self.assertEqual(rows(output, "FRAME")[0][2:5], ["7", "1", "33"])
        value = rows(output, "EST")[0]
        self.assertEqual(value[0], "4000000000")
        for actual, expected in zip(value[1:],
                                    [-43.2345678, 179.1234567, -12.345, 2.5,
                                     -3.21, 4.56, -0.7, 270.01]):
            self.assertAlmostEqual(float(actual), expected, places=10)
        zero = bridge(feed(estimated(time=0, lat=0, lon=0, alt=0, relative=0,
                                     north=0, east=0, down=0, heading=0)))
        self.assertEqual(rows(zero, "FRAME")[0][6], "1")
        self.assertEqual(rows(zero, "EST")[0][-1], "0")
        self.assertEqual(rows(bridge(feed(estimated(heading=65535))), "EST")[0][-1], "-")
        bound = rows(bridge(feed(estimated(lat=900000000, lon=-1800000000,
                                          north=-32768, east=32767, heading=35999))), "EST")[0]
        for index, expected in ((1, 90), (2, -180), (5, -327.68),
                                (6, 327.67), (8, 359.99)):
            self.assertAlmostEqual(float(bound[index]), expected)
        for args, reason in ((dict(lat=900000001), 1), (dict(lat=-900000001), 1),
                             (dict(lon=1800000001), 2), (dict(lon=-1800000001), 2),
                             (dict(heading=36000), 3), (dict(heading=65534), 3)):
            self.assertEqual(rows(bridge(feed(estimated(**args))), "ESTERR"),
                             [[str(reason)]])
        probes = rows(bridge(feed(estimated()), "probe estimated 254 27 33",
                            "probe estimated 254 29 33",
                            "probe estimated 253 28 0"), "PROBE")
        self.assertEqual(probes, [["rejected", "0"], ["rejected", "0"], ["ignored"]])

    def test_raw_gps(self):
        output = bridge(feed(gps(time=1700000000123456, fix=6), 2))
        value = rows(output, "GPS")[0]
        self.assertEqual(value[:2], ["1700000000123456", "6"])
        for index, expected in ((2, -43.2345678), (3, 179.1234567),
                                (4, -12.345), (5, 1.23), (6, 4.56),
                                (7, 7.89), (8, 270.01), (9, 14),
                                (10, -11.234), (11, 0.25), (12, 1.2),
                                (13, 0.34), (14, 1.23456), (15, 360)):
            self.assertAlmostEqual(float(value[index]), expected, places=9)
        missing = dict(time=0, fix=1, lat=0, lon=0, alt=0, eph=65535,
                       epv=65535, speed=65535, course=65535,
                       satellites=255, ellipsoid=0, hacc=0, vacc=0,
                       velacc=0, hdgacc=0, yaw=0)
        for version in (1, 2):
            output = bridge(feed(gps(**missing, version=version)))
            self.assertEqual(rows(output, "FRAME")[0][6], "30")
            self.assertEqual(rows(output, "GPS")[0][5:], ["-"] * 11)
        known_zero = rows(bridge(feed(gps(**{**missing, "fix": 3, "eph": 0,
                                         "epv": 0, "speed": 0, "course": 0,
                                         "satellites": 0}))), "GPS")[0]
        self.assertEqual(known_zero[5:10], ["0", "0", "0", "0", "0"])
        partial = bridge(feed(gps(**{**missing, "vacc": 1000})))
        self.assertGreater(int(rows(partial, "FRAME")[0][6]), 38)
        self.assertEqual(rows(partial, "GPS")[0][10:13], ["0", "0", "1"])
        for args, reason in ((dict(lat=900000001), 1), (dict(lon=-1800000001), 2),
                             (dict(course=36000), 3), (dict(yaw=36001), 4)):
            self.assertEqual(rows(bridge(feed(gps(**args))), "GPSERR"), [[str(reason)]])
        unknown = rows(bridge(feed(gps(fix=254, yaw=65535))), "GPS")[0]
        self.assertEqual(unknown[1], "254")
        self.assertEqual(unknown[-1], "-")
        probes = rows(bridge(feed(gps()), "probe gps 254 29 24",
                            "probe gps 253 53 24", "probe gps 253 52 0"), "PROBE")
        self.assertEqual(probes, [["rejected", "0"], ["rejected", "0"], ["ignored"]])

    def test_provider_matches_simulation(self):
        actual = rows(bridge("state 0", "clock 1000 1250 2000", feed(estimated())), "STATE")[0]
        simulated = rows(bridge(sample("sim", source=2, measured=1000, received=1250,
                                       lat=-43.2345678, lon=179.1234567, alt=-12.345,
                                       north=-3.21, east=4.56, heading=270.01,
                                       accuracy="-", fix="-")), "STATE")[0]
        self.assertEqual([actual[0], simulated[0]], ["0", "3"])
        for left, right in zip(actual[1:], simulated[1:]):
            if left == "-" or right == "-": self.assertEqual(left, right)
            else: self.assertAlmostEqual(float(left), float(right), places=10)
        raw = rows(bridge("state 0", "clock 2000 2100 2200",
                          feed(gps(time=1700000000123456, fix=3, lat=100000000,
                                   lon=-200000000, alt=1000, eph=10000, epv=200,
                                   speed=500, course=9000, hacc=250, yaw=27000))), "STATE")[0]
        self.assertEqual(raw[1:5], ["2000", "2100", "10", "-20"])
        self.assertAlmostEqual(float(raw[7]), 0, places=10)
        self.assertAlmostEqual(float(raw[8]), 5, places=10)
        self.assertEqual(raw[9:], ["270", "0.25", "2"])
        unknown = rows(bridge("state 0", feed(gps(fix=254, speed=65535,
                                                  course=65535, yaw=65535))), "STATE")[0]
        self.assertEqual(unknown[7:], ["-", "-", "-", "0.25", "-"])
        self.assertEqual(rows(bridge("state 0", feed(gps(yaw=36000))), "STATE")[0][9], "0")

    def test_validator(self):
        prefix = ("validator 200000 50000 5 1", "clock 0 0 1100000")
        self.assertEqual(rows(bridge(*prefix, sample()), "VALID"), [["accepted"]])
        self.assertEqual(rows(bridge(*prefix, sample(measured=900000, received=950000,
                                                    accuracy=5, fix=1)), "VALID"),
                         [["accepted"]])
        cases = [({"lat": "nan"}, 2), ({"lon": 180.000001}, 2),
                 ({"alt": "inf"}, 3), ({"reference": 99}, 3),
                 ({"east": "-inf"}, 4), ({"heading": 360}, 5),
                 ({"accuracy": -1}, 6), ({"source": 99}, 1),
                 ({"fix": 99}, 7), ({"fix": 0}, 8), ({"fix": "-"}, 9),
                 ({"accuracy": "-"}, 10), ({"accuracy": 5.01}, 11),
                 ({"measured": 1020001}, 12), ({"received": 1100001}, 13),
                 ({"measured": 899999}, 14), ({"received": 1050001}, 15)]
        for changes, reason in cases:
            with self.subTest(changes=changes):
                self.assertEqual(rows(bridge(*prefix, sample(**changes)), "VALID"),
                                 [["rejected", str(reason)]])
        self.assertEqual(rows(bridge("validator 200000 50000 - -",
                                      "clock 0 0 1100000",
                                      sample(source=0, accuracy="-", fix="-")), "VALID"),
                         [["accepted"]])
        for configuration in ("validator 200000 50000 nan 1",
                              "validator 200000 50000 5 0"):
            self.assertEqual(rows(bridge(configuration, "clock 0 0 1100000", sample()), "VALID"),
                             [["rejected", "0"]])
        output = bridge(*prefix, sample(), sample(),
                        sample(measured=999999, received=1020001),
                        sample(measured=1000001, received=1020001),
                        sample(source=2), sample(measured=1000002, received=1020000))
        self.assertEqual(rows(output, "VALID"),
                         [["accepted"], ["rejected", "16"], ["rejected", "16"],
                          ["accepted"], ["accepted"], ["rejected", "16"]])


if __name__ == "__main__":
    unittest.main()
