import unittest

from common import ack, bridge, count, feed, item, request, rows


class MissionTests(unittest.TestCase):
    def test_wide_system_target(self):
        wire = bytes.fromhex(rows(bridge("wide_count 70000"), "WIRE")[0][0])
        self.assertEqual(rows(bridge(feed(wire)), "MCOUNT"),
                         [["42", "190", "70000", "1", "2", "513", str(0xFEDCBA98)]])

    def test_transfer_fields(self):
        output = bridge(feed(count(513, kind=2, opaque=0xFEDCBA98), 1),
                        feed(request(65535), 2),
                        feed(item(257, lat=-2147483647, lon=2147483647,
                                  alt=123.875, hold=1.125, coordinate_frame=6,
                                  current=1, autocontinue=0, kind=1), 1),
                        feed(ack(13, kind=1, opaque=0x12345678), 2))
        self.assertEqual(rows(output, "MCOUNT"),
                         [["42", "190", "7", "1", "2", "513", str(0xFEDCBA98)]])
        self.assertEqual(rows(output, "MREQUEST"),
                         [["7", "1", "42", "190", "0", "65535"]])
        mission_item = rows(output, "MITEM")[0]
        self.assertEqual(mission_item[:11],
                         ["42", "190", "7", "1", "1", "257", "6", "16", "1", "0", "1.125"])
        self.assertEqual(mission_item[-3:], ["-2147483647", "2147483647", "123.875"])
        self.assertEqual(rows(output, "MACK"),
                         [["7", "1", "42", "190", "1", "13", str(0x12345678)]])

    def test_normalizer(self):
        cases = [(5, 900000000, -1800000000, "125.5", 0, 90, -180, 125.5),
                 (6, -900000000, 1800000000, "-20.25", 1, -90, 180, -20.25),
                 (5, -431234567, 1799999999, "0", 0, -43.1234567, 179.9999999, 0),
                 (11, 0, 0, "nan", 2, 0, 0, None)]
        for code, lat, lon, alt, reference, expected_lat, expected_lon, expected_alt in cases:
            value = rows(bridge(f"normalize {code} 16 17 {lat} {lon} {alt}"), "NORMAL")[0]
            self.assertEqual(value[:2], ["accepted", "17"])
            self.assertAlmostEqual(float(value[2]), expected_lat)
            self.assertAlmostEqual(float(value[3]), expected_lon)
            self.assertEqual(int(value[4]), reference)
            self.assertEqual(None if value[5] == "-" else float(value[5]), expected_alt)
        for code in (0, 1, 2, 3, 4, 255):
            self.assertEqual(rows(bridge(f"normalize {code} 16 17 0 0 1"), "NORMAL"),
                             [["rejected", "0"]])
        for lat, lon, alt, error in ((900000001, 0, 1, 2), (-900000001, 0, 1, 2),
                                     (0, 1800000001, 1, 3), (0, -1800000001, 1, 3),
                                     (2147483647, 0, 1, 1), (0, 2147483647, 1, 1),
                                     (0, 0, "inf", 4)):
            self.assertEqual(rows(bridge(f"normalize 5 16 17 {lat} {lon} {alt}"), "NORMAL"),
                             [["rejected", str(error)]])

    def test_waypoint_only(self):
        ids = [16, 22, 21, 20, 17, 18, 19, 300, 65535]
        self.assertEqual(rows(bridge(*(f"command {code}" for code in ids)), "COMMAND"),
                         [[str(code), "accepted" if code == 16 else "rejected"] for code in ids])

    def test_upload_and_ack(self):
        output = bridge("upload 7 1 2", "ack 0", feed(count(2)), "ack 0",
                        feed(item(1)), feed(item(0, sender=43)),
                        feed(item(0, kind=1)), feed(count(1)),
                        feed(item(0)), "ack 0",
                        feed(item(1, lat=-400000000, hold=3.5)),
                        feed(count(1)), "ack 0", "ack 0",
                        feed(count(1)), feed(item(0)), "ack 1")
        self.assertEqual(rows(output, "UPREQ"),
                         [["42", "190", "0", "0"], ["42", "190", "0", "1"],
                          ["42", "190", "0", "0"]])
        self.assertEqual(rows(output, "UPREJECT"),
                         [["9", "-"], ["7", "-"], ["8", "-"], ["5", "-"], ["5", "-"]])
        self.assertEqual(rows(output, "UPCOMPLETE"),
                         [["42", "190", "0", "2"], ["42", "190", "0", "1"]])
        self.assertEqual(rows(output, "UPITEM")[:2],
                         [["0", "50", "30", "0", "120", "16", "0"],
                          ["1", "-40", "30", "0", "120", "16", "3.5"]])
        self.assertEqual(rows(output, "UPACK"),
                         [["42", "190", "0", "0"], ["42", "190", "0", "1"]])
        self.assertEqual(len(rows(output, "UPACKNONE")), 4)

    def test_upload_rejections(self):
        cases = [(["upload 7 1 2", feed(count(1, target=8))], ["1", "-"]),
                 (["upload 7 1 2", feed(count(1, kind=1))], ["2", "3"]),
                 (["upload 7 1 2", feed(count(0))], ["3", "1"]),
                 (["upload 7 1 2", feed(count(3))], ["4", "4"]),
                 (["upload 7 1 2", feed(count(2)), feed(item(0, command=21))], ["10", "3"]),
                 (["upload 7 1 2", feed(count(1)), feed(item(0, lat=900000001))], ["11", "5"]),
                 (["upload 7 1 0", feed(count(1))], ["0", "-"])]
        for commands, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(rows(bridge(*commands), "UPREJECT")[-1], expected)
        output = bridge("upload 7 1 2", feed(count(1, component=0)), feed(item(0)),
                        "ack 0", feed(ack(0)))
        self.assertEqual(len(rows(output, "UPCOMPLETE")), 1)
        self.assertEqual(len(rows(output, "UPACK")), 1)
        self.assertFalse(rows(output, "UPREJECT"))


if __name__ == "__main__":
    unittest.main()
