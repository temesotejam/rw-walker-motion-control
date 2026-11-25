"""Download the walker's immutable RWLOG over its Wi-Fi using a bound PC interface.

Example: python tools/download_rwlog.py
Repeat the command to resume from this tool's own .rwlog.part journal. Browser
storage is separate. An optional --diagnostic FILE pins the exact failed run.
"""
import argparse
from contextlib import contextmanager
import http.client
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import socket
import struct
import sys
import tempfile
import time
from urllib.parse import urlencode
import zlib


DEVICE = "192.168.4.1"
CHUNK = 4096
MAX_FILE = 9 * 1024 * 1024
FRAME = struct.Struct("<4sIII")
KEYS = ("boot_id", "run_id", "token", "bytes", "crc32", "filename")


class DownloadError(Exception):
    """Stop with the previous verified checkpoint intact."""


class ReplyError(DownloadError):
    def __init__(self, status, message):
        super().__init__(f"HTTP {status}: {message}")
        self.status = status


def interface(value):
    if value == "auto":
        addresses = {entry[4][0] for entry in socket.getaddrinfo(
            socket.gethostname(), None, socket.AF_INET, socket.SOCK_STREAM)}
        walker_addresses = sorted(address for address in addresses if address.startswith("192.168.4.")
                                  and address.rsplit(".", 1)[1] not in ("0", "1", "255"))
        if len(walker_addresses) != 1:
            raise ValueError("Connect to the walker Wi-Fi, or specify its PC interface with --source "
                             f"(candidate addresses: {walker_addresses}).")
        value = walker_addresses[0]
    ip = ipaddress.IPv4Address(value)
    if ip not in ipaddress.IPv4Network("192.168.4.0/24") or (int(ip) & 255) in (0, 1, 255):
        raise ValueError("Specify this PC's 192.168.4.x address on the walker Wi-Fi.")
    return str(ip)


def transport(source):
    source = interface(source)
    def request(method, path):
        # A numeric destination and a bound local IP bypass proxies, redirects
        # and the PC's default internet adapter. One request owns one socket.
        connection = http.client.HTTPConnection(DEVICE, timeout=5, source_address=(source, 0))
        deadline = time.monotonic() + 7
        try:
            connection.connect()
            sock = connection.sock
            connection.request(method, path, headers={"Accept-Encoding": "gzip", "Connection": "close"})
            response = connection.getresponse()
            wire = bytearray()
            while not response.isclosed():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("HTTP body deadline")
                sock.settimeout(min(5, remaining))
                block = response.read1(16384)
                if not block:
                    break
                wire.extend(block)
                if len(wire) > 65536:
                    raise DownloadError("Oversize response")
            advertised = response.getheader("Content-Length")
            if advertised is not None and int(advertised) != len(wire):
                raise OSError("Truncated HTTP response")
            if not 200 <= response.status < 300:
                raise ReplyError(response.status, wire[:120].decode("utf-8", "replace"))
            encoding = response.getheader("Content-Encoding", "identity").lower()
            if encoding == "gzip":
                decoder = zlib.decompressobj(31)
                decoded = decoder.decompress(wire, 65537)
                if len(decoded) > 65536 or not decoder.eof or decoder.unused_data:
                    raise OSError("Invalid gzip response")
                return decoded
            if encoding != "identity":
                raise DownloadError("Unsupported HTTP content encoding")
            return bytes(wire)
        finally:
            connection.close()
    return request


def identity(m):
    if (m.get("phase") != "ready" or m.get("hashed_bytes") != m.get("bytes")
            or m.get("chunk_bytes") != CHUNK
            or not all(type(m.get(k)) is int for k in ("boot_id", "run_id", "crc32", "bytes"))
            or not (0 < m["boot_id"] <= 0xffffffff and 0 < m["run_id"] <= 65535
                    and 0 <= m["crc32"] <= 0xffffffff and 114 <= m["bytes"] <= MAX_FILE)
            or not re.fullmatch(r"[0-9a-f]{16}", str(m.get("token")))
            or not re.fullmatch(r"[a-zA-Z0-9_-]+_[0-9]+_[0-9]+\.rwlog", str(m.get("filename")))):
        raise DownloadError("Invalid or unsealed walker export manifest")
    return {k: m[k] for k in KEYS}


def unpack(packet, offset, length):
    if len(packet) != FRAME.size + length:
        raise OSError("Incomplete packet")
    magic, start, size, check = FRAME.unpack_from(packet)
    payload = packet[FRAME.size:]
    if (magic, start, size, check) != (b"FCH1", offset, length, zlib.crc32(payload)):
        raise OSError(f"Invalid packet identity or CRC at {offset}")
    return payload


@contextmanager
def checkpoint(path, key):
    with path.open("a+b") as f:
        f.seek(0)
        if os.name == "nt":
            import msvcrt
            msvcrt.locking(f.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Closing f releases the lock, including during exception unwinding.
        signature = json.dumps(key, sort_keys=True, separators=(",", ":")).encode() + b"\n"
        header = f.readline(1024)
        if not header:
            f.write(signature)
            f.flush()
            os.fsync(f.fileno())
        elif header != signature:
            raise DownloadError(f"Different run or invalid checkpoint; kept {path}")
        data = bytearray()
        while len(data) < key["bytes"]:
            position = f.tell()
            header = f.read(FRAME.size)
            if not header:
                break
            if len(header) == FRAME.size:
                magic, offset, count, _ = FRAME.unpack(header)
                if magic != b"FCH1" or offset != len(data) or not 0 < count <= min(CHUNK, key["bytes"] - offset):
                    raise DownloadError(f"Invalid checkpoint frame at {position}; kept {path}")
                payload = f.read(count)
                if len(payload) == count:
                    data.extend(unpack(header + payload, offset, count))
                    continue
            f.truncate(position)  # interrupted append, previous CRCs already verified
            f.seek(position)
            break
        if f.read(1):
            raise DownloadError(f"Unexpected trailing checkpoint data in {path}")
        yield f, data


def valid_file(data, key):
    return (len(data) == key["bytes"] and zlib.crc32(memoryview(data)[:-4]) == key["crc32"]
            and int.from_bytes(data[-4:], "little") == key["crc32"])


def publish(dest, data):
    with tempfile.NamedTemporaryFile(dir=dest.parent, delete=False) as f:
        temp = Path(f.name)
        try:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        except BaseException:
            f.close()
            temp.unlink()
            raise
    try:
        if os.name == "nt":
            os.rename(temp, dest)  # fails if dest already exists
        else:
            os.link(temp, dest)  # atomic no-clobber publication
    finally:
        temp.unlink(missing_ok=True)


def download(request, directory, expected=None, *, attempts=8, sleep=time.sleep, report=print):
    def call(method, path, retries=attempts):
        for i in range(retries):
            try:
                return request(method, path)
            except (OSError, http.client.HTTPException, zlib.error, ReplyError) as exc:
                if isinstance(exc, ReplyError) and exc.status not in (408, 429, 500, 502, 503, 504):
                    raise
                if i + 1 == retries:
                    raise
                sleep(min(0.25 * 2**i, 2))

    def manifest():
        m = json.loads(call("GET", "/export/manifest"))
        if (not isinstance(m, dict) or m.get("phase") not in ("empty", "building", "ready", "error")
                or type(m.get("boot_id")) is not int or not 0 < m["boot_id"] <= 0xffffffff
                or m.get("chunk_bytes") != CHUNK or m.get("chunks_per_response") != 4
                or m.get("stream_chunks") != 16 or m.get("completion_ack") is not True
                or not all(type(m.get(k)) is int for k in ("run_id", "crc32", "bytes", "hashed_bytes", "metadata_bytes"))
                or not all(isinstance(m.get(k), str) for k in ("token", "filename", "error"))):
            raise DownloadError("Device did not identify a walker export")
        if m["phase"] == "ready":
            identity(m)
        if expected and (m["boot_id"] != expected["boot_id"] or
                         (m.get("run_id") and m["run_id"] != expected["run_id"])):
            raise DownloadError("Requested run is no longer on this device")
        return m

    m = manifest()
    boot, client = m["boot_id"], secrets.randbelow(0xffffffff) + 1
    if expected and m["phase"] == "ready" and identity(m) != expected:
        raise DownloadError("Requested log differs from the current sealed export")
    started = False
    run = m.get("run_id")
    try:
        try:
            call("POST", f"/export/prepare?client={client}")
            started = True
        except ReplyError as exc:
            if exc.status != 409 or m["phase"] != "ready":
                raise
            # Some stopped builds retain an immutable ready export after their
            # normal measurement controls have stopped accepting Prepare.
        build_deadline = time.monotonic() + 120
        while True:
            m = manifest()
            if m["boot_id"] != boot or (run and m.get("run_id") and run != m["run_id"]):
                raise DownloadError("Device or measurement changed during preparation")
            run = m.get("run_id") or run
            if m["phase"] == "ready":
                break
            if m["phase"] == "error" or time.monotonic() >= build_deadline:
                raise DownloadError(f"Export has not sealed: {m.get('error') or m['phase']}")
            sleep(0.5)
        key = identity(m)
        if expected and key != expected:
            raise DownloadError("New manifest does not match the requested recording")
        directory.mkdir(parents=True, exist_ok=True)
        dest = directory / key["filename"]
        part = dest.with_suffix(".rwlog.part")
        if dest.exists():
            if dest.stat().st_size != key["bytes"] or not valid_file(dest.read_bytes(), key):
                raise DownloadError(f"Existing file differs; kept {dest}")
        else:
            with checkpoint(part, key) as (f, data):
                report(f"{len(data)}/{key['bytes']} bytes; checkpoint {part}")
                window = CHUNK * min(4, max(1, int(m.get("stream_window_chunks", 2))))
                while len(data) < key["bytes"]:
                    offset = len(data)
                    for attempt in range(attempts):
                        n = min(key["bytes"] - offset, window)
                        query = {"token":key["token"], "offset":offset, "length":n}
                        if n > CHUNK:
                            query["stream"] = 1
                        try:
                            raw = call("GET", "/export/chunk?" + urlencode(query), 1)
                            frames = bytearray()
                            at = 0
                            while len(frames) < n:
                                count = min(CHUNK, n - len(frames))
                                end = at + FRAME.size + count
                                frames.extend(unpack(raw[at:end], offset + len(frames), count))
                                at = end
                            if at != len(raw):
                                raise OSError("Unexpected trailing packet bytes")
                            break
                        except (OSError, http.client.HTTPException, zlib.error, ReplyError) as exc:
                            if isinstance(exc, ReplyError) and exc.status not in (408,429,500,502,503,504):
                                raise
                            if attempt + 1 == attempts:
                                raise DownloadError(f"Interrupted at {offset} bytes; rerun to resume") from exc
                            # The device may get only two TCP segments out of a 4 KiB
                            # packet. A shorter range is independently framed and has
                            # its own CRC; journal it at its real byte offset.
                            window = max(1024, window // 2)
                            report(f"{offset} bytes; retry {attempt + 1} with up to {window} bytes ({exc})")
                            sleep(min(0.25 * 2**attempt, 2))
                    f.write(raw)
                    f.flush()
                    os.fsync(f.fileno())
                    data.extend(frames)
                    if offset // 262144 != len(data) // 262144:
                        report(f"{len(data)}/{key['bytes']} bytes")
                if not valid_file(data, key):
                    raise DownloadError(f"Whole-file CRC mismatch; checkpoint kept in {part}")
                publish(dest, data)
            part.unlink()
        if m.get("completion_ack") is True:
            args = urlencode({k:key[k] for k in ("token","crc32","run_id")})
            try:
                call("POST", "/export/complete?" + args)
            except (OSError, http.client.HTTPException, ReplyError) as exc:
                raise DownloadError(f"Saved and CRC verified: {dest}; completion acknowledgement failed. "
                                    "Rerun to send the acknowledgement.") from exc
        started = False
        return dest
    finally:
        if started and run:
            try:
                call("POST", "/export/pause?" + urlencode(dict(client=client, boot_id=boot, run_id=run)), 1)
            except (OSError, DownloadError, http.client.HTTPException):
                pass  # the device also releases transfer priority after its lease expires


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", default="auto", help="PC IPv4 on walker Wi-Fi (default: unique 192.168.4.x)")
    p.add_argument("--out-dir", type=Path, default=Path.home() / "Downloads")
    p.add_argument("--diagnostic", type=Path, help="Pin the saved diagnostic's export manifest")
    args = p.parse_args()
    try:
        pinned = None
        if args.diagnostic:
            d = json.loads(args.diagnostic.read_text(encoding="utf-8-sig"))
            pinned = identity(d["browser_transfer"]["manifest"])
        print(download(transport(args.source), args.out_dir.expanduser(), pinned))
    except (OSError, DownloadError, ValueError, KeyError, http.client.HTTPException, zlib.error) as err:
        sys.exit(str(err))
