import os
from pathlib import Path
import subprocess
import struct

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


CRC_EXTRA = {44: 221, 47: 153, 51: 196, 73: 38}
MIN_LEN = {44: 4, 47: 3, 51: 4, 73: 37}


def crc_x25(data):
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 255)
        tmp ^= (tmp << 4) & 255
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 65535
    return crc


def frame(msgid, payload, *, sysid=42, compid=190, seq=0, version=2):
    if version == 2:
        payload = payload[:max(MIN_LEN[msgid], len(payload.rstrip(b"\0")))]
        header = bytes((len(payload), 0, 0, seq, sysid, compid,
                        msgid & 255, (msgid >> 8) & 255, (msgid >> 16) & 255))
        magic = b"\xfd"
    else:
        payload = payload[:MIN_LEN[msgid]]
        header = bytes((len(payload), seq, sysid, compid, msgid))
        magic = b"\xfe"
    return magic + header + payload + struct.pack(
        "<H", crc_x25(header + payload + bytes((CRC_EXTRA[msgid],))))


def count(size, *, target=7, component=1, kind=0, sender=42, opaque=0):
    return frame(44, struct.pack("<HBBBI", size, target, component, kind, opaque),
                 sysid=sender)


def request(seq, *, target=42, component=190, kind=0):
    return frame(51, struct.pack("<HBBB", seq, target, component, kind),
                 sysid=7, compid=1)


def item(seq, *, command=16, lat=500000000, lon=300000000, alt=120.0,
         hold=0.0, target=7, component=1, kind=0, sender=42,
         coordinate_frame=5, current=0, autocontinue=1):
    payload = struct.pack("<ffffiifHHBBBBBB", hold, 0.0, 0.0, 0.0,
                          lat, lon, alt, seq, command, target, component,
                          coordinate_frame, current, autocontinue, kind)
    return frame(73, payload, sysid=sender)


def ack(result, *, target=42, component=190, kind=0, opaque=0):
    return frame(47, struct.pack("<BBBBI", target, component, result, kind, opaque),
                 sysid=7, compid=1)
