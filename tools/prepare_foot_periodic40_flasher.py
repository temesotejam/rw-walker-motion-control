"""Prepare or check the current Web Serial installer without contacting the walker."""
import argparse
import copy
import errno
from functools import partial
import hashlib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import struct
import tempfile
from urllib.parse import urljoin
from urllib.request import ProxyHandler, build_opener
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
ENVIRONMENT = "atoms3cam-foot-periodic40"
BUILD_ID = "walking-support-transfer-v39-walk7-logfix1"
PAGE = "support-transfer"
VERSION_QUERY = f"?v={BUILD_ID}"
URL = f"http://127.0.0.1:8767/{PAGE}/{VERSION_QUERY}"
PARTS = {"bootloader.bin": 0, "partitions.bin": 0x8000,
         "boot_app0.bin": 0xe000, "firmware.bin": 0x10000}
SCRIPTS = ("flasher-completion.js", "soft-boot.js")


def check_page(root):
    flasher = root / "flasher"
    directory = flasher / PAGE
    html = (directory / "index.html").read_text(encoding="utf-8")
    if (f"<code>{BUILD_ID}</code>" not in html or f'manifest="manifest.json{VERSION_QUERY}"' not in html
            or html.count("<esp-web-install-button ") != 1):
        raise ValueError(f"Installer page does not identify {BUILD_ID}")
    if any(f'src="{name}"' not in html for name in SCRIPTS):
        raise ValueError("Installer page is missing a helper script")
    for filename, prefix in (("index.html", ""), ("foot-periodic40/index.html", "../")):
        redirect = (flasher / filename).read_text(encoding="utf-8")
        destination = prefix + PAGE + "/" + VERSION_QUERY
        if f'url={destination}"' not in redirect or f'href="{destination}"' not in redirect:
            raise ValueError(f"Installer redirect in {filename} differs from the current version")
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    # ESP Web Tools resolves each part against the manifest URL. A relative
    # path does not inherit the manifest's query, so version each part as well.
    parts = [{"path": "firmware/" + name + VERSION_QUERY, "offset": offset} for name, offset in PARTS.items()]
    if (manifest.get("version") != BUILD_ID or manifest.get("new_install_improv_wait_time") != 0
            or manifest.get("builds") != [{"chipFamily": "ESP32-S3", "parts": parts}]):
        raise ValueError("Unexpected installer identity or flash layout")
    # Older consumers can still open /manifest.json from the root. Both
    # manifests point at the same four firmware files.
    alias = copy.deepcopy(manifest)
    for part in alias["builds"][0]["parts"]:
        part["path"] = PAGE + "/" + part["path"]
    if json.loads((flasher / "manifest.json").read_text(encoding="utf-8")) != alias:
        raise ValueError("Root manifest differs from the dedicated manifest")
    return directory


def validate_parts(parts):
    if any(not data for data in parts.values()):
        raise ValueError("An installer binary is empty")
    # Bound the boot images to their flash regions and the application to app0
    # in default_8MB.csv (ends at 0x340000).
    limits = {"bootloader.bin": 0x8000, "partitions.bin": 0x1000,
              "boot_app0.bin": 0x2000, "firmware.bin": 0x330000}
    for name, data in parts.items():
        if len(data) > limits[name]:
            raise ValueError(f"{name} exceeds its flash region")
    for name in ("firmware.bin", "bootloader.bin"):
        data = parts[name]
        if len(data) < 24 or data[0] != 0xe9 or data[12:14] != b"\x09\x00":
            raise ValueError(f"{name} does not contain an ESP32-S3 image")
    entry = struct.Struct("<HBBII16sI")
    table = parts["partitions.bin"]
    layouts = {}
    for start in range(0, len(table) - entry.size + 1, entry.size):
        magic, kind, subtype, offset, size, _, _ = entry.unpack_from(table, start)
        if magic == 0x50aa:
            layouts[kind, subtype] = offset, size
    if layouts.get((0, 0x10)) != (0x10000, 0x330000) or layouts.get((1, 0)) != (0xe000, 0x2000):
        raise ValueError("Partition table does not match the app0 and OTA regions")
    # Builds can contain multiple version strings in the embedded browser code.
    # The firmware's RWLOG metadata serializer emits this independent literal.
    identity = b'"diagnostic_build_id":"' + BUILD_ID.encode() + b'",\x00'
    if identity not in parts["firmware.bin"]:
        raise ValueError(f"Application does not contain the expected identity {BUILD_ID}")


def boot_app_path():
    packages = Path(os.environ.get("PLATFORMIO_PACKAGES_DIR") or
                    Path(os.environ.get("PLATFORMIO_CORE_DIR", Path.home() / ".platformio")) / "packages")
    for suffix in ("@3.20016.0", ""):
        package = packages / ("framework-arduinoespressif32" + suffix)
        metadata = package / "package.json"
        if metadata.is_file() and json.loads(metadata.read_text(encoding="utf-8")).get("version") == "3.20016.0":
            return package / "tools/partitions/boot_app0.bin"
    raise FileNotFoundError("PlatformIO Arduino ESP32 framework 3.20016.0 is not installed")


def check(root=ROOT):
    directory = check_page(root)
    parts = {name: (directory / "firmware" / name).read_bytes() for name in PARTS}
    validate_parts(parts)
    for name in SCRIPTS:
        if (directory / name).read_bytes() != (directory.parent / name).read_bytes():
            raise ValueError(f"Packaged helper differs from source: {name}")
    return {name: hashlib.sha256(data).hexdigest() for name, data in parts.items()}


def retire_previous_installers(root):
    # Git updates the old landing pages, but generated files survive checkout.
    # Keep those images outside the served tree, including for cached old pages.
    public = root / "flasher"
    old_files = [Path(PAGE, "firmware", "carry1-manifest.json"),
                 Path(PAGE, "firmware", "carry1-a03957520afb5c9be6364efa366b592799a9011963c036eadf2d973d059c6d5e.bin")]
    old_files.extend(Path("foot-periodic40", "firmware", name) for name in PARTS)
    present = [relative for relative in old_files if (public / relative).is_file()]
    if present:
        retired = root / ".pio" / "retired-installers"
        retired.mkdir(parents=True, exist_ok=True)
        archive = Path(tempfile.mkdtemp(prefix="pre-walk1-", dir=retired))
        for relative in present:
            destination = archive / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            os.replace(public / relative, destination)


def prepare(root=ROOT, boot_app=None):
    directory = check_page(root)
    build = root / ".pio/build" / ENVIRONMENT
    parts = {name: (boot_app or boot_app_path()).read_bytes() if name == "boot_app0.bin"
             else (build / name).read_bytes() for name in PARTS}
    validate_parts(parts)
    files = {name: (directory.parent / name).read_bytes() for name in SCRIPTS}
    files.update({f"firmware/{name}": data for name, data in parts.items()})
    # Validate all inputs before changing the staged installer. Replace each
    # completed file atomically and restore earlier changes on a failed copy.
    (directory / "firmware").mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=directory) as tmpdir:
        temp = Path(tmpdir)
        replaced = []
        try:
            for name, data in files.items():
                dest = directory / name
                previous = dest.read_bytes() if dest.is_file() else None
                if previous == data:
                    continue
                next_file = temp / (dest.name + ".next")
                next_file.write_bytes(data)
                backup = temp / (dest.name + ".old") if previous is not None else None
                if backup is not None:
                    backup.write_bytes(previous)
                os.replace(next_file, dest)
                replaced.append((dest, backup))
        except OSError:
            for dest, backup in reversed(replaced):
                if backup is None:
                    dest.unlink()
                else:
                    os.replace(backup, dest)
            raise
    retire_previous_installers(root)
    return check(root)


def serve(root=ROOT):
    check(root)
    directory = root / "flasher"
    handler = partial(SimpleHTTPRequestHandler, directory=str(directory))
    try:
        server = ThreadingHTTPServer(("127.0.0.1", 8767), handler)
    except OSError as exc:
        if exc.errno not in (errno.EADDRINUSE, 10048):
            raise
        # An earlier launcher may already be serving this workspace.
        # Confirm its files before handing the browser to a reused port.
        client = build_opener(ProxyHandler({}))
        names = ["index.html", "manifest.json", *SCRIPTS, *("firmware/" + n for n in PARTS)]
        for name in names:
            expected = (directory / PAGE / name).read_bytes()
            versioned = name in ("index.html", "manifest.json") or name.startswith("firmware/")
            with client.open(urljoin(URL, name) + (VERSION_QUERY if versioned else ""), timeout=5) as response:
                if response.read(len(expected) + 1) != expected:
                    raise ValueError("Port 8767 is serving a different installer")
        webbrowser.open_new_tab(URL)
        return
    with server:
        print(URL, flush=True)
        webbrowser.open_new_tab(URL)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="Verify the currently staged files without replacing them")
    mode.add_argument("--serve", action="store_true", help="Verify and open the current installer in a browser")
    args = parser.parse_args()
    try:
        if args.serve:
            serve()
        else:
            hashes = check() if args.check else prepare()
            print(json.dumps({"version": BUILD_ID, "url": URL, "sha256": hashes}, indent=2))
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Installer: {exc}\n")
