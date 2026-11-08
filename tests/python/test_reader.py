import unittest

from common import ROOT, bridge, cli, feed, fixture, rows


class ReaderTests(unittest.TestCase):
    def test_frames_and_damage(self):
        for name, magic in (("heartbeat_v1.bin", "254"), ("heartbeat_v2.bin", "253")):
            data = fixture(name)
            output = bridge(feed(data))
            self.assertEqual(rows(output, "COUNT"), [["0", "1", "0", "0"]])
            self.assertEqual(rows(output, "FRAME")[0][1:5], [magic, "1", "1", "0"])
            self.assertEqual(bytes.fromhex(rows(output, "FRAME")[0][-1]), data)
        for name, counts in (("heartbeat_v2_bad_crc.bin", ["0", "0", "1", "0"]),
                             ("heartbeat_v2_truncated.bin", ["0", "0", "0", "1"])):
            output = bridge(feed(fixture(name)))
            self.assertEqual(rows(output, "COUNT"), [counts])
            self.assertFalse(rows(output, "FRAME"))

    def test_split_points_and_multiple_frames(self):
        v1, v2 = fixture("heartbeat_v1.bin"), fixture("heartbeat_v2.bin")
        baseline = rows(bridge(feed(v2)), "FRAME")
        for split in range(1, len(v2)):
            output = bridge(feed(v2[:split]), feed(v2[split:]))
            self.assertEqual(rows(output, "COUNT"),
                             [["0", "0", "0", "1"], ["0", "1", "0", "0"]])
            self.assertEqual(rows(output, "FRAME"), baseline)
        parts = [fixture(f"heartbeat_v2_chunks/{n:02}.bin") for n in (1, 2, 3)]
        self.assertEqual(rows(bridge(*(feed(p) for p in parts)), "FRAME"), baseline)
        combined = v1 + v2 + v1
        output = bridge(feed(combined))
        self.assertEqual(rows(output, "COUNT"), [["0", "3", "0", "0"]])
        self.assertEqual([bytes.fromhex(r[-1]) for r in rows(output, "FRAME")],
                         [v1, v2, v1])

    def test_channels_and_recovery(self):
        v1, v2 = fixture("heartbeat_v1.bin"), fixture("heartbeat_v2.bin")
        commands = []
        for i in range(max(len(v1), len(v2))):
            if i < len(v1): commands.append(feed(v1[i:i + 1], 1))
            if i < len(v2): commands.append(feed(v2[i:i + 1], 2))
        output = bridge(*commands)
        self.assertEqual([r[0] for r in rows(output, "FRAME")], ["1", "2"])
        self.assertEqual(rows(output, "COUNT")[-1], ["2", "1", "0", "0"])
        bad = fixture("heartbeat_v2_bad_crc.bin")
        output = bridge(feed(v1, 1), feed(v2, 2), feed(bad[:10], 1),
                        feed(v2[:7], 2), feed(bad[10:], 1), feed(v2[7:], 2))
        self.assertEqual(rows(output, "COUNT")[-2:],
                         [["1", "1", "1", "0"], ["2", "2", "0", "0"]])
        self.assertEqual(rows(output, "REJECT"), [["1", "0"]])
        output = bridge(feed(fixture("recovery_stream.bin")))
        self.assertEqual(rows(output, "COUNT"), [["0", "2", "2", "0"]])
        self.assertEqual([r[1] for r in rows(output, "REJECT")], ["0", "0"])
        self.assertEqual([r[4] for r in rows(output, "FRAME")], ["0", "0"])
        self.assertEqual(rows(bridge(feed(b"\0\x7f\x12" + v2)), "COUNT"),
                         [["0", "1", "0", "0"]])
        output = bridge(feed(b"\xfd\0" + v2))
        self.assertEqual(rows(output, "COUNT"), [["0", "1", "1", "0"]])
        self.assertEqual(rows(output, "REJECT"), [["0", "2"]])

    def test_heartbeat_and_log(self):
        for name, expected in (
            ("heartbeat_gcs_v2.bin", ["6", "8", "GCS", "INVALID", "STANDBY"]),
            ("heartbeat_vehicle_v2.bin", ["2", "12", "QUADROTOR", "PX4", "ACTIVE"]),
            ("heartbeat_unknown_v2.bin", ["255", "255", "UNKNOWN", "UNKNOWN", "UNKNOWN"]),
        ):
            beat = rows(bridge(feed(fixture(name))), "HEARTBEAT")[0]
            self.assertEqual(beat[:2] + beat[-3:], expected)
        beat = rows(bridge(feed(fixture("heartbeat_gcs_v2.bin"))), "HEARTBEAT")[0]
        self.assertEqual(beat[2:6], ["4", "305419896", "3", "3"])
        for name, version in (("heartbeat_v1.bin", 1), ("heartbeat_v2.bin", 2)):
            output = bridge(feed(fixture(name)), "format 123")
            self.assertEqual(rows(output, "FORMAT"), [[
                f"frame time=1970-01-01T00:00:00.123Z mavlink={version} sysid=1 compid=1 msgid=0 seq=0 len=9 signed=no"]])
        self.assertEqual(bridge("format 123"), [["FORMATNONE"]])

    def test_cli_file_stdin_and_errors(self):
        path = lambda name: str(ROOT / "fixtures" / name)
        output = cli("--input", path("heartbeat_v2.bin"), "--chunk-size", "5")
        self.assertIn("bytes=21 chunks=5", output)
        self.assertIn("accepted=1 rejected=0 incomplete=0", output)
        self.assertIn("bytes=21", cli("--input", "-", "--chunk-size", "5",
                                      data=fixture("heartbeat_v2.bin")))
        self.assertIn("bytes=5", cli("--input", "-", "--chunk-size", "2",
                                      data=b"\0\xfd\0\xff\n"))
        for name, fragment in {
            "heartbeat_v2_bad_crc.bin": "accepted=0 rejected=1 incomplete=0",
            "heartbeat_v2_truncated.bin": "accepted=0 rejected=0 incomplete=1",
            "recovery_stream.bin": "accepted=2 rejected=2 incomplete=0 bad_crc=2 bad_signature=0 invalid_header=0",
            "heartbeat_gcs_v2.bin": "type=GCS(6) autopilot=INVALID(8) status=STANDBY(3)",
            "heartbeat_unknown_v2.bin": "type=UNKNOWN(255) autopilot=UNKNOWN(255) status=UNKNOWN(255)",
        }.items():
            self.assertIn(fragment, cli("--input", path(name)))
        for name, version in (("heartbeat_v1.bin", 1), ("heartbeat_v2.bin", 2)):
            self.assertRegex(cli("--input", path(name)),
                             rf"frame time=.*Z mavlink={version} sysid=1 compid=1 msgid=0 seq=0 len=9 signed=no")


if __name__ == "__main__":
    unittest.main()
