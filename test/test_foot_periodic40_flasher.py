"""Offline checks for the current download paths and wrong-build rejection."""
import json
import hashlib
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import prepare_foot_periodic40_flasher as flasher


class InstallerTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        directory = self.root / "flasher"
        self.page = directory / flasher.PAGE
        self.page.mkdir(parents=True)
        for file in ("index.html", "manifest.json", *flasher.SCRIPTS):
            shutil.copyfile(ROOT / "flasher" / file, directory / file)
        for file in ("index.html", "manifest.json"):
            shutil.copyfile(ROOT / "flasher" / flasher.PAGE / file, self.page / file)
        self.build = self.root / ".pio/build" / flasher.ENVIRONMENT
        self.build.mkdir(parents=True)
        self.boot_app = self.root / "boot_app0.bin"
        self.boot_app.write_bytes(b"OTA")
        self.image = bytes([0xe9]) + bytes(11) + b"\x09\x00" + bytes(10)
        self.application = self.image + b'"diagnostic_build_id":"' + flasher.BUILD_ID.encode() + b'",\x00'
        (self.build / "firmware.bin").write_bytes(self.application)
        (self.build / "bootloader.bin").write_bytes(self.image)
        entry = struct.Struct("<HBBII16sI")
        (self.build / "partitions.bin").write_bytes(
            entry.pack(0x50aa, 0, 0x10, 0x10000, 0x330000, b"app0", 0) +
            entry.pack(0x50aa, 1, 0, 0xe000, 0x2000, b"otadata", 0))

    def test_main_and_direct_page_share_one_firmware(self):
        hashes = flasher.prepare(self.root, self.boot_app)
        self.assertEqual(hashes, flasher.check(self.root))
        self.assertIn(f'url={flasher.PAGE}/', (self.root / "flasher/index.html").read_text(encoding="utf-8"))
        for folder in (self.root / "flasher", self.page):
            manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
            binaries = manifest["builds"][0]["parts"]
            self.assertEqual([folder / part["path"] for part in binaries],
                             [self.page / "firmware" / name for name in flasher.PARTS])

    def test_web_text_cannot_authorize_the_wrong_firmware(self):
        known = flasher.prepare(self.root, self.boot_app)
        (self.build / "firmware.bin").write_bytes(
            self.image + b'"diagnostic_build_id":"walking-foot-periodic40-v38",\x00' + flasher.BUILD_ID.encode())
        with self.assertRaisesRegex(ValueError, "identity"):
            flasher.prepare(self.root, self.boot_app)
        self.assertEqual(flasher.check(self.root), known)
        (self.build / "firmware.bin").write_bytes(self.application)
        (self.build / "partitions.bin").write_bytes(b"wrong partitions")
        with self.assertRaisesRegex(ValueError, "Partition"):
            flasher.prepare(self.root, self.boot_app)
        self.assertEqual(flasher.check(self.root), known)

    def test_packaging_failure_rolls_back_changed_helpers(self):
        known = flasher.prepare(self.root, self.boot_app)
        helper = self.root / "flasher/soft-boot.js"
        original = helper.read_bytes()
        helper.write_bytes(original + b"\n// new helper")
        (self.build / "firmware.bin").write_bytes(self.application + b"modified build")
        replace = flasher.os.replace

        def fail_firmware(source, destination):
            if destination.name == "firmware.bin":
                raise PermissionError("firmware busy")
            replace(source, destination)

        with patch.object(flasher.os, "replace", side_effect=fail_firmware):
            with self.assertRaises(PermissionError):
                flasher.prepare(self.root, self.boot_app)
        helper.write_bytes(original)
        self.assertEqual(flasher.check(self.root), known)

    def test_comparison_requires_the_original_app_and_keeps_the_main_manifest(self):
        hashes = flasher.prepare(self.root, self.boot_app)
        archived = self.root / "archived.bin"
        original = self.application.replace(flasher.BUILD_ID.encode(), flasher.CARRY1_ID.encode())
        archived.write_bytes(original)
        with patch.object(flasher, "CARRY1_SHA256", hashlib.sha256(original).hexdigest()):
            flasher.check_carry1(self.page, archived)
            self.assertEqual(flasher.check(self.root), hashes)
            archived.write_bytes(original + b"unverified")
            with self.assertRaisesRegex(ValueError, "recorded walking trial"):
                flasher.check_carry1(self.page, archived)
            flasher.check_carry1(self.page)
            manifest = self.page / flasher.CARRY1_MANIFEST
            altered = json.loads(manifest.read_bytes())
            altered["builds"][0]["parts"][-1]["path"] = "firmware.bin"
            manifest.write_text(json.dumps(altered), encoding="utf8")
            with self.assertRaisesRegex(ValueError, "comparison manifest"):
                flasher.check_carry1(self.page)


if __name__ == "__main__":
    unittest.main()
