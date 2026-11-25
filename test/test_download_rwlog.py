"""Exercise the standalone RWLOG client without any robot connection."""
import gzip
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from pathlib import Path
import random
import struct
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
from urllib.parse import parse_qs, urlsplit
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import download_rwlog as client


class Device:
    def __init__(self):
        data = random.Random(57).randbytes(58043)
        self.content = data + struct.pack("<I", zlib.crc32(data))
        self.m = dict(phase="ready", boot_id=317, run_id=1, chunk_bytes=4096, stream_chunks=16,
                      chunks_per_response=4, metadata_bytes=1024, error="",
                      stream_window_chunks=4, token="0123456789abcdef", crc32=zlib.crc32(data),
                      hashed_bytes=len(self.content), bytes=len(self.content),
                      filename="energy_control_autonomous_run_1_35758615.rwlog", completion_ack=True)
        self.stop_at = None
        self.stop_above = None
        self.offsets = []
        self.posts = []
        self.status_ready_conflict = False
        self.bad_packet = False

    def __call__(self, method, path):
        uri = urlsplit(path)
        args = {k:v[0] for k,v in parse_qs(uri.query).items()}
        if method == "POST":
            assert uri.path in ("/export/prepare", "/export/complete", "/export/pause")
            self.posts.append((uri.path, args))
            if uri.path == "/export/prepare" and self.status_ready_conflict:
                raise client.ReplyError(409, "completed_run_required")
            return b"ok"
        assert method == "GET"
        if uri.path == "/export/manifest":
            return json.dumps(self.m).encode()
        assert uri.path == "/export/chunk" and args["token"] == self.m["token"]
        off, n = int(args["offset"]), int(args["length"])
        self.offsets.append((off,n))
        if (self.stop_at is not None and off >= self.stop_at) or (self.stop_above and n > self.stop_above):
            raise TimeoutError("injected")
        out = bytearray()
        for i in range(off, off+n, client.CHUNK):
            block = self.content[i:min(i+client.CHUNK,off+n)]
            crc = zlib.crc32(block)
            out += client.FRAME.pack(b"FCH1", i, len(block), crc ^ self.bad_packet) + block
        return bytes(out)


class DownloadTests(unittest.TestCase):
    def setUp(self):
        self.dev = Device()
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.dest = self.root / self.dev.m["filename"]
        self.part = self.dest.with_suffix(".rwlog.part")

    def run_client(self, **kwargs):
        return client.download(self.dev,self.root,attempts=3,sleep=lambda _:None,report=lambda _:None,**kwargs)

    def test_resume_first_missing_exact_crc_and_real_complete_ack(self):
        self.dev.stop_at = client.CHUNK * 4
        with self.assertRaisesRegex(client.DownloadError, "16384"):
            self.run_client()
        self.assertEqual(self.dev.posts[-1][0],"/export/pause")
        self.assertEqual(self.dev.posts[-1][1]["client"],self.dev.posts[0][1]["client"])
        self.assertEqual([n for off,n in self.dev.offsets if off == 16384], [16384,8192,4096])
        self.assertFalse(self.dest.exists())
        with self.part.open("ab") as f:
            f.write(b"FC")  # process killed mid-append
        self.dev.stop_at=None
        self.dev.offsets.clear()
        self.assertEqual(self.run_client(),self.dest)
        self.assertEqual(self.dev.offsets[0][0],16384)
        self.assertEqual(self.dest.read_bytes(),self.dev.content)
        self.assertFalse(self.part.exists())
        self.assertEqual(self.dev.posts[-1],("/export/complete",dict(token=self.dev.m["token"],
                         run_id="1",crc32=str(self.dev.m["crc32"]))))
        self.dev.offsets.clear()
        self.run_client()
        self.assertEqual(self.dev.offsets,[])

    def test_mismatched_run_preserves_part_and_refuses_replacement(self):
        self.dev.stop_at=16384
        with self.assertRaises(client.DownloadError):
            self.run_client()
        saved = self.part.read_bytes()
        pinned = client.identity(self.dev.m)
        self.dev.m["boot_id"] += 1
        posts = len(self.dev.posts)
        with self.assertRaisesRegex(client.DownloadError, "no longer"):
            self.run_client(expected=pinned)
        self.assertEqual(len(self.dev.posts),posts)
        with self.assertRaisesRegex(client.DownloadError,"Different run"):
            self.run_client()
        self.assertEqual(self.part.read_bytes(), saved)

    def test_subframe_progress_survives_outage_inside_a_four_kib_interval(self):
        self.dev.stop_above = 2048
        self.dev.stop_at = 6144
        options = dict(attempts=8,sleep=lambda _:None,report=lambda _:None)
        with self.assertRaisesRegex(client.DownloadError,"at 6144"):
            client.download(self.dev,self.root,**options)
        self.assertIn((0,2048),self.dev.offsets)
        self.dev.stop_at = None
        self.dev.stop_above = 1024
        self.dev.offsets.clear()
        dest=client.download(self.dev,self.root,**options)
        self.assertEqual(self.dev.offsets[0][0],6144)
        self.assertIn((6144,1024),self.dev.offsets)
        self.assertEqual(dest.read_bytes(),self.dev.content)

    def test_bad_packets_and_whole_crc_never_publish(self):
        self.dev.bad_packet=True
        with self.assertRaisesRegex(client.DownloadError,"Interrupted at 0"):
            self.run_client()
        self.assertFalse(self.dest.exists())
        self.dev.bad_packet=False
        self.dev.content=self.dev.content[:-4] + b"bad!"
        with self.assertRaisesRegex(client.DownloadError,"Whole-file CRC"):
            self.run_client()
        self.assertFalse(self.dest.exists())
        self.assertTrue(self.part.is_file())

    def test_ready_export_and_preexisting_file_preservation(self):
        self.dev.status_ready_conflict=True
        self.run_client()
        self.assertEqual(self.dest.read_bytes(),self.dev.content)
        self.dest.write_bytes(b"prior user file")
        with self.assertRaisesRegex(client.DownloadError,"Existing file differs"):
            self.run_client()
        self.assertEqual(self.dest.read_bytes(),b"prior user file")

    def test_auto_bound_interface_and_signature_before_any_post(self):
        def lookup(*ips):
            return patch.object(client.socket, "getaddrinfo", return_value=[
                (client.socket.AF_INET, client.socket.SOCK_STREAM, 0, "", (ip, 0)) for ip in ips])
        with lookup("10.11.12.1", "192.168.4.2", "127.0.0.1", "192.168.4.2"):
            self.assertEqual(client.interface("auto"), "192.168.4.2")
        for ips in [("10.11.12.1",), ("192.168.4.2", "192.168.4.5"), ("192.168.4.1",)]:
            with lookup(*ips), self.assertRaisesRegex(ValueError,"walker Wi-Fi"):
                client.interface("auto")
        del self.dev.m["stream_chunks"]
        with self.assertRaisesRegex(client.DownloadError,"did not identify"):
            self.run_client()
        self.assertEqual(self.dev.posts,[])

    def test_invalid_sealed_size_before_any_post(self):
        self.dev.m["hashed_bytes"] -= 1
        with self.assertRaisesRegex(client.DownloadError,"Invalid or unsealed"):
            self.run_client()
        self.assertEqual(self.dev.posts,[])

    def test_compressed_http_with_close_uses_bound_source_and_validates_length(self):
        payload=b"packet" * 900
        compressed=gzip.compress(payload)
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Encoding","gzip")
                self.send_header("Content-Length",str(len(compressed)+(self.path=="/torn")))
                self.send_header("Connection","close")
                self.end_headers()
                self.wfile.write(compressed)
            def log_message(self,*args):
                pass
        server=HTTPServer(("127.0.0.1",0),Handler)
        t=threading.Thread(target=server.serve_forever,daemon=True)
        t.start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        original=client.http.client.HTTPConnection
        def local(address,timeout,source_address):
            self.assertEqual(address,"192.168.4.1")
            self.assertEqual(source_address,("192.168.4.2",0))
            return original("127.0.0.1", server.server_port,timeout=timeout,source_address=("127.0.0.1",0))
        for address in ("127.0.0.1","0.0.0.0","192.168.4.1","192.168.4.0","192.168.4.255"):
            with self.assertRaises(ValueError):
                client.interface(address)
        with patch.object(client.http.client,"HTTPConnection",local):
            fetch=client.transport("192.168.4.2")
            self.assertEqual(fetch("GET","/whole"),payload)
            with self.assertRaises(OSError):
                fetch("GET","/torn")


if __name__ == "__main__":
    unittest.main()
