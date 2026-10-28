"""SP USB link probe. Default is read-only. Requires pyserial for hardware access."""
import argparse
import math
import struct
import time


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


def frame(payload):
    return payload + struct.pack("<H", crc16(payload))


def command(mode=0, yaw=0.0, pitch=0.0):
    # fire=0, velocity/acceleration=0, shoot_count=0. Never requests firing.
    return frame(struct.pack("<2sBB6fB", b"SP", mode, 0, yaw, 0, 0, pitch, 0, 0, 0))


def extract(buffer):
    packets = []
    while True:
        start = buffer.find(b"SP")
        if start < 0:
            buffer[:] = buffer[-1:] if buffer.endswith(b"S") else b""
            break
        del buffer[:start]
        if len(buffer) < 45:
            break
        raw = bytes(buffer[:45])
        if crc16(raw[:-2]) != struct.unpack_from("<H", raw, 43)[0]:
            del buffer[0]
            continue
        packets.append(struct.unpack("<2sBB9fHBH", raw))
        del buffer[:45]
    return packets


def self_test():
    assert crc16(b"123456789") == 0x6F91
    assert len(command()) == 31 and command()[-2:] == bytes.fromhex("4b ac")
    up = frame(struct.pack("<2sBB9fHB", b"SP", 1, 2, 1, 0, 0, 0, 0, 0, 0, 0, 23, 0, 0))
    assert len(up) == 45 and up[-2:] == bytes.fromhex("c4 9a")
    for split in range(46):
        buf = bytearray(up[:split]); packets = extract(buf)
        buf.extend(up[split:]); packets += extract(buf)
        assert len(packets) == 1 and packets[0][2] == 2
    print("PASS: CRC, 45/31-byte samples and all uplink split boundaries")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="C-board native USB CDC, e.g. /dev/ttyACM0 or COM20")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--once", action="store_true", help="read and display one valid uplink frame")
    parser.add_argument("--yaw-delta-deg", type=float, default=None, help="explicitly send a relative test target; positive is left")
    parser.add_argument("--pitch-delta-deg", type=float, default=None, help="explicitly send a relative test target; negative is up")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.port:
        parser.error("--port is required")
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    for delta in (args.yaw_delta_deg, args.pitch_delta_deg):
        if delta is not None and not math.isfinite(delta):
            parser.error("angle deltas must be finite")
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc
    control = args.yaw_delta_deg is not None or args.pitch_delta_deg is not None
    buffer = bytearray()
    target = None
    deadline = time.monotonic() + args.seconds
    last_print = last_send = 0.0
    received = 0
    with serial.Serial(args.port, 115200, timeout=0.02, write_timeout=0.2) as port:
        try:
            while time.monotonic() < deadline:
                buffer.extend(port.read(port.in_waiting or 1))
                for packet in extract(buffer):
                    received += 1
                    mode, enemy = packet[1:3]
                    yaw, yaw_vel, pitch, pitch_vel, bullet_speed = packet[7:12]
                    now = time.monotonic()
                    if now - last_print >= 0.2 or args.once:
                        print(f"rx={received} mode={mode} enemy={enemy} yaw_deg={math.degrees(yaw):.2f} "
                              f"pitch_deg={math.degrees(pitch):.2f} yaw_vel={yaw_vel:.3f} "
                              f"pitch_vel={pitch_vel:.3f} bullet_speed={bullet_speed:.2f}", flush=True)
                        last_print = now
                    if control and target is None:
                        if mode not in (1, 2, 3):
                            raise SystemExit("Uplink is IDLE. Send vision_enable on USART1 first.")
                        target = (yaw + math.radians(args.yaw_delta_deg or 0),
                                  pitch + math.radians(args.pitch_delta_deg or 0))
                        print(f"Sending absolute target yaw={math.degrees(target[0]):.2f}deg "
                              f"pitch={math.degrees(target[1]):.2f}deg, fire=0", flush=True)
                    if args.once:
                        return
                now = time.monotonic()
                if target is not None and now - last_send >= 0.02:
                    port.write(command(1, *target))
                    last_send = now
            if not received:
                raise SystemExit("No valid SP uplink. Check flashed firmware, native USB port and port ownership.")
        finally:
            if target is not None:
                port.write(command())  # revoke control on normal exit/Ctrl+C; MCU also has a local timeout


if __name__ == "__main__":
    main()
