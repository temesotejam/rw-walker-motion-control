"""Offline checks for the current download paths and wrong-build rejection."""
import json
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch
from urllib.parse import urljoin

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
        for file in ("index.html", "manifest.json"):
            shutil.copyfile(ROOT / "flasher" / file, directory / file)
        (directory / "foot-periodic40").mkdir()
        shutil.copyfile(ROOT / "flasher/foot-periodic40/index.html", directory / "foot-periodic40/index.html")
        for file in ("index.html", "manifest.json", *flasher.SCRIPTS):
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
        query = f"?v={flasher.BUILD_ID}"
        self.assertTrue(flasher.URL.endswith("/" + query))
        expected = [urljoin(flasher.URL, "firmware/" + name + query) for name in flasher.PARTS]
        for folder, relative in ((self.root / "flasher", "/manifest.json"), (self.page, "manifest.json")):
            manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
            binaries = manifest["builds"][0]["parts"]
            manifest_url = urljoin(flasher.URL, relative + query)
            self.assertEqual([urljoin(manifest_url, part["path"]) for part in binaries], expected)

    def test_stale_cache_urls_are_rejected(self):
        for file in (self.page / "index.html", self.page / "manifest.json", self.page.parent / "manifest.json",
                     self.page.parent / "index.html", self.page.parent / "foot-periodic40/index.html"):
            with self.subTest(file=file):
                original = file.read_text(encoding="utf-8")
                file.write_text(original.replace(flasher.VERSION_QUERY, ""), encoding="utf-8")
                with self.assertRaises(ValueError):
                    flasher.check_page(self.root)
                file.write_text(original, encoding="utf-8")

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

    def test_packaging_failure_rolls_back_changed_images(self):
        known = flasher.prepare(self.root, self.boot_app)
        (self.build / "bootloader.bin").write_bytes(self.image + b"new bootloader")
        (self.build / "firmware.bin").write_bytes(self.application + b"modified build")
        replace = flasher.os.replace

        def fail_firmware(source, destination):
            if destination.name == "firmware.bin":
                raise PermissionError("firmware busy")
            replace(source, destination)

        with patch.object(flasher.os, "replace", side_effect=fail_firmware):
            with self.assertRaises(PermissionError):
                flasher.prepare(self.root, self.boot_app)
        self.assertEqual(flasher.check(self.root), known)

    def test_update_retires_previous_images_away_from_cached_installer_urls(self):
        old = [Path("support-transfer/firmware/carry1-manifest.json"),
               Path("support-transfer/firmware/carry1-a03957520afb5c9be6364efa366b592799a9011963c036eadf2d973d059c6d5e.bin"),
               Path("foot-periodic40/firmware/firmware.bin"),
               *(Path("foot-periodic40", name) for name in flasher.SCRIPTS)]
        for relative in old:
            source = self.root / "flasher" / relative
            source.parent.mkdir(parents=True, exist_ok=True)
            source.write_bytes(relative.as_posix().encode())
        hashes = flasher.prepare(self.root, self.boot_app)
        self.assertEqual(hashes, flasher.prepare(self.root, self.boot_app))
        archive, = (self.root / ".pio/retired-installers").iterdir()
        for relative in old:
            self.assertFalse((self.root / "flasher" / relative).exists())
            self.assertEqual((archive / relative).read_bytes(), relative.as_posix().encode())
        self.assertEqual((self.page / "firmware" / "firmware.bin").read_bytes(), self.application)

    def test_only_one_installer_is_published(self):
        html = self.page / "index.html"
        self.assertNotIn("carry1-manifest", html.read_text(encoding="utf-8"))
        old = ROOT / "flasher" / "foot-periodic40"
        self.assertIn('url=../support-transfer/', (old / "index.html").read_text(encoding="utf-8"))
        self.assertFalse((old / "manifest.json").exists())
        with html.open("a", encoding="utf-8") as stream:
            stream.write('<esp-web-install-button manifest="other.json"></esp-web-install-button>')
        with self.assertRaisesRegex(ValueError, "Installer page"):
            flasher.check_page(self.root)


if __name__ == "__main__":
    unittest.main()
