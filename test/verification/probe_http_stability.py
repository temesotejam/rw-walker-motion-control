"""Exercise the walker's HTTP page, idle status and immutable grayscale previews."""
import argparse
from collections import Counter
import gzip
import hashlib
import http.client
import ipaddress
import json
from pathlib import Path
import struct
import time
import zlib


def request(args, method, path, info=None):
    connection = http.client.HTTPConnection(str(args.address), timeout=args.timeout,
                                            source_address=(str(args.source), 0))
    try:
        connection.request(method, path, headers={'Accept-Encoding': 'gzip', 'Connection': 'close'})
        response = connection.getresponse()
        if response.status != 200:
            raise ValueError(f'{response.status} {response.read(100)!r}')
        length = int(response.getheader('Content-Length'))
        if not 0 < length <= 512 * 1024:
            raise ValueError(f'invalid Content-Length {length}')
        body = response.read()
        if len(body) != length:
            raise ValueError(f'{len(body)} of {length} bytes')
        encoding = response.getheader('Content-Encoding', 'identity').lower()
        if encoding == 'gzip':
            body = gzip.decompress(body)
        if info is not None:
            info.update(encoding=encoding, wire_bytes=length, decoded_bytes=len(body))
        return body
    finally:
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=ipaddress.IPv4Address, default='192.168.4.2')
    parser.add_argument('--address', type=ipaddress.IPv4Address, default='192.168.4.1')
    parser.add_argument('--seconds', type=float, default=180)
    parser.add_argument('--timeout', type=float, default=3)
    parser.add_argument('--min-bytes', type=int, default=0, help='Minimum whole-image CRC-verified bytes')
    parser.add_argument('--heap', action='store_true', help='Also sample the manifest internal heap stats every five seconds')
    parser.add_argument('--frozen', action='store_true', help='Re-read one sealed preview, resuming at the first missing chunk')
    parser.add_argument('--out', type=Path, help='Save summary and timestamped failures as JSON')
    args = parser.parse_args()
    if args.seconds <= 0 or args.timeout <= 0 or args.min_bytes < 0:
        parser.error('seconds and timeout must be positive, min-bytes nonnegative')

    start = time.monotonic()
    next_status = start
    next_heap = start
    heap = []
    counts, errors = Counter(), []
    pending = {}
    boot = build = page_hash = first_status = last_status = None
    first_foot = last_foot = None
    max_status_gap = 0
    page_info = {}
    last_failure = None
    image = None
    offset = checksum = verified = 0
    stopped = False
    final_state = None
    aborted = False
    while time.monotonic() - start < args.seconds:
        now = time.monotonic()
        stage = 'status'
        try:
            if now >= next_status:
                next_status = now + 2
                state = json.loads(request(args, 'GET', '/status.json?view=live'))
                if state.get('running') is not False or state['motor_mA'] != 0:
                    raise RuntimeError('Stopped machine required')
                if boot is not None and state['boot_id'] != boot:
                    raise RuntimeError('Walker rebooted during probe')
                boot, build = state['boot_id'], state['diagnostic_build_id']
                final_state = {'state': state.get('state'), 'ready': state.get('ready'),
                               'running': state['running'], 'motor_mA': state['motor_mA']}
                elapsed = round(time.monotonic() - start, 3)
                if last_status is not None:
                    max_status_gap = max(max_status_gap, elapsed - last_status)
                last_status = elapsed
                if first_status is None:
                    first_status = last_status
                foot = state.get('foot_periodic', {})
                last_foot = {k: foot.get(k) for k in ('count', 'missed', 'both_valid', 'published_hz')}
                if first_foot is None:
                    first_foot = last_foot
                stopped = True
            elif not stopped:
                time.sleep(min(.1, next_status - now))
                continue
            elif args.heap and now >= next_heap:
                stage = 'manifest'
                next_heap = now + 5
                snapshot = json.loads(request(args, 'GET', '/export/manifest'))
                if snapshot['boot_id'] != boot:
                    raise RuntimeError('Walker rebooted during probe')
                heap.append({'seconds': round(time.monotonic() - start, 3), **snapshot['export_memory']})
            elif page_hash is None:
                stage = 'page'
                page = request(args, 'GET', '/', page_info)
                if not page.rstrip().endswith(b'</html>') or b'</script>' not in page:
                    raise ValueError('Incomplete page')
                page_hash = hashlib.sha256(page).hexdigest()
            elif image is None:
                stage = 'capture'
                candidate = json.loads(request(args, 'POST', '/vision/capture'))
                if candidate['format'] != 'gray8' or candidate['bytes'] != candidate['width'] * candidate['height']:
                    raise ValueError('Invalid preview metadata')
                if not 0 < candidate['bytes'] <= 320 * 240 or not 0 < candidate['token'] <= 0xffffffff:
                    raise ValueError('Invalid image size')
                if not 0 <= candidate['crc32'] <= 0xffffffff:
                    raise ValueError('Invalid preview CRC')
                image = candidate
                offset = checksum = 0
            else:
                stage = 'chunk'
                length = min(4096, image['bytes'] - offset)
                path = f"/vision/chunk?token={image['token']}&offset={offset}&length={length}"
                packet = request(args, 'GET', path)
                data = packet[16:]
                if (len(data) != length or
                        struct.unpack('<4I', packet[:16]) != (0x31484346, offset, length, zlib.crc32(data))):
                    raise ValueError('Chunk header, length or CRC mismatch')
                next_checksum = zlib.crc32(data, checksum)
                if offset + length == image['bytes']:
                    if next_checksum != image['crc32']:
                        raise ValueError('Whole-image CRC mismatch')
                    verified += image['bytes']
                    counts['images'] += 1
                    if not args.frozen:
                        image = None
                    offset = checksum = 0
                else:
                    offset += length
                    checksum = next_checksum
                time.sleep(.02)
            counts[stage] += 1
            for entry in pending.pop(stage, []):
                entry['recovered_s'] = round(time.monotonic() - start, 3)
        except (OSError, EOFError, ValueError, KeyError, TypeError, struct.error, http.client.HTTPException, RuntimeError) as exc:
            elapsed = round(time.monotonic() - start, 3)
            entry = {'seconds': elapsed, 'stage': stage, 'error': str(exc)}
            if stage == 'chunk' and image is not None:
                entry.update(token=image['token'], offset=offset, image_index=counts['images'])
            if stage == 'capture' and 'preview_frame_unavailable' in str(exc):
                counts['preview_frame_unavailable'] += 1
            errors.append(entry)
            pending.setdefault(stage, []).append(entry)
            last_failure = elapsed
            if not args.frozen:
                image = None
            stopped = False  # Verify idle status again before capturing.
            if isinstance(exc, (PermissionError, RuntimeError)) or (args.frozen and len(pending[stage]) >= 9):
                aborted = True
                break
    complete = (not aborted and not pending and bool(counts['status'] and counts['page'] and counts['images'])
                and verified >= args.min_bytes)
    stable = not errors and complete
    result = dict(build=build, boot_id=boot, source=str(args.source), seconds=round(time.monotonic() - start, 3),
                  first_status_s=first_status, last_status_s=last_status, last_failure_s=last_failure,
                  counts=dict(counts), verified_image_bytes=verified, page_sha256=page_hash, page=page_info,
                  max_status_gap_s=round(max_status_gap, 3), first_foot=first_foot, last_foot=last_foot,
                  frozen=args.frozen, frozen_token=image['token'] if args.frozen and image else None,
                  final_state=final_state, heap=heap, errors=errors, stable=stable,
                  completed=complete, completed_with_recovery=complete and bool(errors))
    report = json.dumps(result, indent=2)
    print(report, flush=True)
    if args.out:
        args.out.write_text(report + '\n', encoding='utf-8')
    return 0 if stable else 1


if __name__ == '__main__':
    raise SystemExit(main())
