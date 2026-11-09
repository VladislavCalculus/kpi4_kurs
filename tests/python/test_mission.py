import unittest

from common import ack, bridge, count, feed, item, request, rows


class MissionTests(unittest.TestCase):
    def test_wide_system_target(self):
        wire = bytes.fromhex(rows(bridge("wide_count 70000"), "WIRE")[0][0])
        self.assertEqual(rows(bridge(feed(wire)), "MCOUNT"),
                         [["42", "190", "70000", "1", "2", "513", str(0xFEDCBA98)]])

    def test_short_or_unrelated_ack_is_ignored(self):
        output = bridge(feed(ack(0)), "probe mission 253 2 47",
                        "probe mission 253 3 0")
        self.assertEqual(rows(output, "PROBE"), [["ignored"], ["ignored"]])

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
                         [["42", "190", "0", "0"], ["42", "190", "0", "0"],
                          ["42", "190", "0", "1"],
                          ["42", "190", "0", "0"]])
        self.assertEqual(rows(output, "UPREJECT"),
                         [["7", "-"], ["8", "-"], ["5", "-"], ["5", "-"]])
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

    def test_duplicate_and_out_of_order_items_request_expected_sequence(self):
        output = bridge("upload 7 1 3", feed(count(3)), feed(count(3)),
                        feed(item(1)), feed(item(0)), feed(item(0)),
                        feed(item(2)), feed(item(1)), feed(item(2)),
                        "ack 0", feed(item(2)), feed(item(2, lat=123000000)))
        self.assertEqual([row[-1] for row in rows(output, "UPREQ")],
                         ["0", "0", "0", "1", "1", "1", "2"])
        self.assertEqual(len(rows(output, "UPCOMPLETE")), 1)
        self.assertEqual(len(rows(output, "UPITEM")), 3)
        self.assertEqual(rows(output, "UPACK"),
                         [["42", "190", "0", "0"], ["42", "190", "0", "0"]])
        self.assertEqual(rows(output, "UPREJECT"), [["6", "-"]])

    def test_timeout_boundaries_retry_limit_and_restart(self):
        output = bridge("upload 7 1 2", feed(count(2)),
                        "tick 249999", "tick 250000", "tick 499999",
                        "tick 500000", "tick 750000", "tick 1000000",
                        "tick 1250000", "tick 1500000", feed(count(1)))
        self.assertEqual([row[-1] for row in rows(output, "UPREQ")],
                         ["0"] * 7)  # initial, five retries, fresh upload
        self.assertEqual(rows(output, "UPREJECT"), [["12", "15"]])
        self.assertEqual(len(rows(output, "UPCOMPLETE")), 0)

        reset = bridge("upload 7 1 2", feed(count(2)), "tick 250000",
                       feed(item(0)), "tick 499999", "tick 500000")
        self.assertEqual([row[-1] for row in rows(reset, "UPREQ")],
                         ["0", "0", "1", "1"])
        self.assertFalse(rows(reset, "UPREJECT"))
        no_retries = bridge("upload 7 1 2 100 0", feed(count(1)),
                            "tick 99", "tick 100")
        self.assertEqual(rows(no_retries, "UPREQ"), [["42", "190", "0", "0"]])
        self.assertEqual(rows(no_retries, "UPREJECT"), [["12", "15"]])
        self.assertEqual(rows(bridge("upload 7 1 2 0 5", feed(count(1))), "UPREJECT"),
                         [["0", "-"]])

    def test_changed_count_is_not_a_duplicate(self):
        output = bridge("upload 7 1 3", feed(count(3)),
                        feed(count(3, opaque=1)), feed(item(0)))
        self.assertEqual(rows(output, "UPREJECT"), [["5", "-"]])
        self.assertEqual([row[-1] for row in rows(output, "UPREQ")], ["0", "1"])

    def test_identical_nan_final_item_replays_ack(self):
        missing_altitude = item(0, alt=float("nan"))
        output = bridge("upload 7 1 2", feed(count(1)),
                        feed(missing_altitude), "ack 0", feed(missing_altitude))
        self.assertEqual(len(rows(output, "UPCOMPLETE")), 1)
        self.assertEqual(rows(output, "UPACK"),
                         [["42", "190", "0", "0"], ["42", "190", "0", "0"]])

    def test_failed_replacement_keeps_previous_accepted_plan(self):
        output = bridge("upload 7 1 2", feed(count(1)), feed(item(0)),
                        "ack 0", "active", feed(count(2)),
                        feed(item(0, lat=-400000000)), "active",
                        feed(item(1, command=21)), "active",
                        feed(count(2)), "tick 250000", "tick 500000",
                        "tick 750000", "tick 1000000", "tick 1250000",
                        "tick 1500000", "active")
        self.assertEqual(rows(output, "ACTIVE"),
                         [["42", "1", "50"]] * 4)
        self.assertEqual(rows(output, "UPREJECT"),
                         [["10", "3"], ["12", "15"]])
        self.assertEqual(len(rows(output, "UPCOMPLETE")), 1)


class PlanStoreTests(unittest.TestCase):
    def test_versions_and_immutable_snapshot(self):
        output = bridge("upload 7 1 2", "start", "plan",
                        feed(count(1)), feed(item(0, hold=3.5)),
                        "ack 0", "plan", "plan_raw", "save",
                        feed(count(1)), feed(item(0, lat=-400000000)),
                        "ack 0", "plan", "saved", "start", "start",
                        "discard", "plan")
        self.assertEqual(rows(output, "START"), [["1"], ["0"], ["3"]])
        self.assertEqual(rows(output, "STORE"),
                         [["accepted", "1"], ["accepted", "2"]])
        self.assertEqual(rows(output, "PLAN"),
                         [["-"], ["1", "0", "42", "190", "1", "50"],
                          ["2", "0", "42", "190", "1", "-40"],
                          ["1", "0", "42", "190", "1", "50"],
                          ["2", "0", "42", "190", "1", "-40"]])
        self.assertEqual(rows(output, "DISCARD"), [["0"]])
        self.assertEqual(rows(output, "PLANRAW"),
                         [["1", "1", "0", "16", "500000000", "3.5"]])

    def test_preflight_conflict_requires_explicit_discard(self):
        output = bridge("upload 7 1 2", feed(count(1)), feed(item(0)),
                        "ack 0", feed(count(1, sender=43)),
                        feed(item(0, sender=43, lat=-400000000)),
                        "ack 0", "plan", "start",
                        feed(count(1)), feed(item(0)), "ack 0", "start",
                        "discard", "plan", "start",
                        feed(count(1, sender=43)),
                        feed(item(0, sender=43, lat=-400000000)),
                        "ack 0", "plan", "start")
        self.assertEqual(rows(output, "STORE"),
                         [["accepted", "1"], ["rejected", "1"],
                          ["rejected", "2"], ["accepted", "2"]])
        self.assertEqual(rows(output, "START"),
                         [["2"], ["2"], ["1"], ["0"]])
        self.assertEqual(rows(output, "PLAN"),
                         [["1", "0", "42", "190", "1", "50"], ["-"],
                          ["2", "0", "43", "190", "1", "-40"]])
        self.assertEqual(rows(output, "DISCARD"), [["1"]])
        self.assertEqual([row[-1] for row in rows(output, "UPACK")],
                         ["0", "14", "14", "0"])

    def test_started_plan_keeps_its_source(self):
        output = bridge("upload 7 1 2", feed(count(1)), feed(item(0)),
                        "ack 0", "start",
                        feed(count(1)), feed(item(0, lat=-400000000)),
                        "ack 0", feed(count(1, sender=43)),
                        feed(item(0, sender=43, lat=100000000)),
                        "ack 0", "discard", "plan")
        self.assertEqual(rows(output, "START"), [["0"]])
        self.assertEqual(rows(output, "STORE"),
                         [["accepted", "1"], ["rejected", "3"],
                          ["rejected", "4"]])
        self.assertEqual(rows(output, "DISCARD"), [["0"]])
        self.assertEqual(rows(output, "PLAN"),
                         [["1", "0", "42", "190", "1", "50"]])
        self.assertEqual([row[-1] for row in rows(output, "UPACK")],
                         ["0", "14", "14"])

    def test_channel_is_part_of_source_identity(self):
        output = bridge("upload 7 1 2", feed(count(1), 1), feed(item(0), 1),
                        "ack 0", feed(count(1), 2), feed(item(0), 2),
                        "ack 0", "start", "plan")
        self.assertEqual(rows(output, "STORE"),
                         [["accepted", "1"], ["rejected", "1"]])
        self.assertEqual(rows(output, "START"), [["2"]])
        self.assertEqual(rows(output, "PLAN"),
                         [["1", "1", "42", "190", "1", "50"]])

    def test_incomplete_candidate_does_not_replace_plan(self):
        output = bridge("upload 7 1 2", feed(count(1)), feed(item(0)),
                        "ack 0", feed(count(1)), feed(item(0, lat=-400000000)),
                        "corrupt_staged", "ack 0", "plan")
        self.assertEqual(rows(output, "STORE"),
                         [["accepted", "1"], ["rejected", "0"]])
        self.assertEqual([row[-1] for row in rows(output, "UPACK")],
                         ["0", "5"])
        self.assertEqual(rows(output, "PLAN"),
                         [["1", "0", "42", "190", "1", "50"]])


if __name__ == "__main__":
    unittest.main()
