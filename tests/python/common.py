import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / "fixtures"
BRIDGE = os.environ.get("KPI4_BRIDGE", str(ROOT / "build" / "kpi4_test_bridge"))
CLI = os.environ.get("KPI4_CLI", str(ROOT / "build" / "kpi4_cli"))


def fixture(name):
    return (FIXTURES / name).read_bytes()


def feed(data, channel=0):
    return f"feed {channel} {data.hex()}"


def bridge(*commands):
    result = subprocess.run((BRIDGE,), input="\n".join(commands) + "\n",
                            text=True, capture_output=True, check=True)
    return [line.split("\t") for line in result.stdout.splitlines()]


def rows(records, kind):
    return [record[1:] for record in records if record[0] == kind]


def cli(*args, data=None):
    return subprocess.run((CLI, *args), input=data, capture_output=True,
                          check=True).stdout.decode()
