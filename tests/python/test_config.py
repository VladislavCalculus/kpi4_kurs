import tempfile
import unittest
from pathlib import Path

from common import bridge, rows


VALID = """# host demonstration values
source = direct_gnss
reading_type = gnss_fix
max_measurement_age_us = 200000
max_delivery_delay_us = 50000
max_horizontal_accuracy_m = 5
minimum_gps_fix = 2d
"""


def changed(**values):
    lines = VALID.splitlines()
    for key, value in values.items():
        lines = [f"{key} = {value}" if line.strip().startswith(key + " ") else line
                 for line in lines]
    return "\n".join(lines) + "\n"


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def write(self, name, content):
        path = self.root / name
        path.write_text(content)
        return str(path)

    def test_load_and_apply_to_validator(self):
        path = self.write("valid.cfg", VALID)
        output = bridge("config_current", f"config_load {path}", "config_current",
                        "config_apply", "clock 0 0 1100000",
                        "direct 1 1000000 1020000 50 30 120 0 2 -3 270 4 1",
                        "direct 1 1000001 1020001 50 30 120 0 2 -3 270 6 1")
        self.assertEqual(rows(output, "CONFIG"), [["accepted", "1"]])
        self.assertEqual(rows(output, "CONFIGCURRENT"),
                         [["-"], ["1", "1", "1", "200000", "50000", "5", "1", path]])
        self.assertEqual(rows(output, "CONFIGAPPLY"), [["accepted"]])
        self.assertEqual(rows(output, "VALID"), [["accepted"], ["rejected", "11"]])

    def test_incomplete_or_invalid_file_keeps_previous_snapshot(self):
        valid = self.write("valid.cfg", VALID)
        cases = [
            (VALID.replace("minimum_gps_fix = 2d\n", ""), 4, "minimum_gps_fix"),
            (VALID + "source=simulation\n", 3, "source"),
            (VALID + "sensor_wire=uart\n", 2, "sensor_wire"),
            (VALID + "bad line\n", 1, "bad line"),
            (changed(max_measurement_age_us="0"), 5, "max_measurement_age_us"),
            (changed(max_measurement_age_us="18446744073709551616"), 5,
             "max_measurement_age_us"),
            (changed(max_delivery_delay_us="250000"), 6, "max_delivery_delay_us"),
            (changed(max_horizontal_accuracy_m="nan"), 5, "max_horizontal_accuracy_m"),
            (changed(max_horizontal_accuracy_m="-1"), 5, "max_horizontal_accuracy_m"),
            (changed(minimum_gps_fix="4d"), 5, "minimum_gps_fix"),
            (changed(reading_type="position_estimate"), 6, "reading_type"),
            (changed(source="simulation"), 6, "reading_type"),
        ]
        commands = [f"config_load {valid}"]
        for index, (content, _, _) in enumerate(cases):
            path = self.write(f"bad{index}.cfg", content)
            commands.extend((f"config_load {path}", "config_current"))
        commands.extend((f"config_load {self.root / 'missing.cfg'}", "config_current",
                         f"config_load {valid}", "config_current"))
        output = bridge(*commands)
        outcomes = rows(output, "CONFIG")
        self.assertEqual(outcomes[0], ["accepted", "1"])
        for actual, (_, code, field) in zip(outcomes[1:-2], cases):
            self.assertEqual(actual[1:], [str(code), field])
        self.assertEqual(outcomes[-2][0:3], ["rejected", "0", str(self.root / "missing.cfg")])
        self.assertEqual(outcomes[-1], ["accepted", "2"])
        currents = rows(output, "CONFIGCURRENT")
        self.assertTrue(all(current == currents[0] for current in currents[:-1]))
        self.assertEqual(currents[-1][0], "2")

    def test_reading_compatibility_and_optional_quality_limits(self):
        position = self.write("position.cfg", changed(
            source="controller_telemetry", reading_type="position_estimate",
            max_horizontal_accuracy_m="none", minimum_gps_fix="none"))
        simulation = self.write("simulation.cfg", changed(
            source="simulation", reading_type="simulated_sample",
            max_horizontal_accuracy_m="none", minimum_gps_fix="none"))
        invalid = self.write("invalid.cfg", changed(
            source="direct_gnss", reading_type="position_estimate",
            max_horizontal_accuracy_m="none", minimum_gps_fix="none"))
        output = bridge(f"config_load {position}", "config_current",
                        f"config_load {simulation}", "config_current",
                        f"config_load {invalid}", "config_current")
        self.assertEqual(rows(output, "CONFIG"),
                         [["accepted", "1"], ["accepted", "2"],
                          ["rejected", "6", "reading_type"]])
        currents = rows(output, "CONFIGCURRENT")
        self.assertEqual(currents[0][1:7], ["0", "0", "200000", "50000", "-", "-"])
        self.assertEqual(currents[1][1:3], ["3", "2"])
        self.assertEqual(currents[1], currents[2])


if __name__ == "__main__":
    unittest.main()
