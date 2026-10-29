"""Regenerate the small heartbeat frame fixtures with PyMavlink."""

from pathlib import Path

from pymavlink.dialects.v10 import common as mavlink1
from pymavlink.dialects.v20 import common as mavlink2


def heartbeat_frame(dialect):
    sender = dialect.MAVLink(None)
    sender.srcSystem = 1
    sender.srcComponent = 1
    message = dialect.MAVLink_heartbeat_message(
        type=0,
        autopilot=0,
        base_mode=0,
        custom_mode=0,
        system_status=0,
        mavlink_version=3,
    )
    return message.pack(sender)


def main():
    directory = Path(__file__).resolve().parent
    valid_v1 = heartbeat_frame(mavlink1)
    valid_v2 = heartbeat_frame(mavlink2)

    bad_crc = bytearray(valid_v2)
    bad_crc[10] ^= 1  # Change payload without updating the checksum.

    (directory / "heartbeat_v1.bin").write_bytes(valid_v1)
    (directory / "heartbeat_v2.bin").write_bytes(valid_v2)
    (directory / "heartbeat_v2_bad_crc.bin").write_bytes(bad_crc)
    (directory / "heartbeat_v2_truncated.bin").write_bytes(valid_v2[:-1])

    parts = directory / "heartbeat_v2_chunks"
    parts.mkdir(exist_ok=True)
    for name, chunk in zip(("01.bin", "02.bin", "03.bin"),
                           (valid_v2[:4], valid_v2[4:13], valid_v2[13:])):
        (parts / name).write_bytes(chunk)


if __name__ == "__main__":
    main()
