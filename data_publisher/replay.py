#!/usr/bin/env python3
"""Publish a recorded rgbd.pose dataset as a continuous 30 Hz ZMQ stream."""

import argparse
import json
import struct
import time
from pathlib import Path

import msgpack
import zmq


DEFAULT_DATA_ROOT = Path(__file__).resolve().parent.parent / 'data_save'
DEFAULT_ENDPOINT = 'ipc:///tmp/rgbd_pose_replay.ipc'
PART_LENGTH = struct.Struct('<Q')
FPS = 30


def find_dataset(root):
    datasets = sorted(
        (path for path in root.iterdir()
         if path.is_dir() and (path / 'manifest.json').is_file()),
        key=lambda path: path.stat().st_mtime,
    )
    if not datasets:
        raise ValueError(f'no recording found in {root}')
    return datasets[-1]


def load_index(dataset):
    with (dataset / 'manifest.json').open(encoding='utf-8') as file:
        manifest = json.load(file)
    if manifest.get('format') != 'rgbd_pose_zmq_v1':
        raise ValueError(f'unsupported recording format: {manifest.get("format")}')
    with (dataset / 'index.jsonl').open(encoding='utf-8') as file:
        entries = [json.loads(line) for line in file if line.strip()]
    if not entries:
        raise ValueError(f'empty recording: {dataset}')
    data_size = (dataset / 'messages.bin').stat().st_size
    for entry in entries:
        if entry['offset'] < 0 or entry['size'] < 4 * PART_LENGTH.size or \
                entry['offset'] + entry['size'] > data_size:
            raise ValueError(f'invalid index entry: {entry}')
    return entries


def read_parts(data, entry):
    data.seek(entry['offset'])
    parts = []
    for _ in range(4):
        header = data.read(PART_LENGTH.size)
        if len(header) != PART_LENGTH.size:
            raise IOError(f'truncated header at offset {entry["offset"]}')
        length = PART_LENGTH.unpack(header)[0]
        part = data.read(length)
        if len(part) != length:
            raise IOError(f'truncated message at offset {entry["offset"]}')
        parts.append(part)
    if data.tell() != entry['offset'] + entry['size']:
        raise IOError(f'record size mismatch at offset {entry["offset"]}')
    return parts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path,
                        help='Recording directory; defaults to the newest one in data_save')
    parser.add_argument('--endpoint', default=DEFAULT_ENDPOINT)
    parser.add_argument('--max-frames', type=int, default=0,
                        help='Stop after this many frames; 0 means loop forever')
    args = parser.parse_args()
    if args.max_frames < 0:
        parser.error('--max-frames must be nonnegative')

    dataset = (args.dataset or find_dataset(DEFAULT_DATA_ROOT)).expanduser().resolve()
    entries = load_index(dataset)
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 2)
    socket.setsockopt(zmq.LINGER, 0)
    try:
        socket.bind(args.endpoint)
        print(f'[replay] {dataset} ({len(entries)} frames) -> {args.endpoint}, {FPS} Hz',
              flush=True)

        # Allow subscribers that started first to complete their subscription.
        time.sleep(0.25)
        start_mono_ns = time.monotonic_ns()
        start_unix_ns = time.time_ns()
        position = 0
        direction = 1
        published = 0
        with (dataset / 'messages.bin').open('rb') as data:
            while not args.max_frames or published < args.max_frames:
                deadline = start_mono_ns + published * 1_000_000_000 // FPS
                remaining = deadline - time.monotonic_ns()
                if remaining > 0:
                    time.sleep(remaining / 1_000_000_000)

                parts = read_parts(data, entries[position])
                meta = msgpack.unpackb(parts[1], raw=False)
                if not isinstance(meta, dict) or meta.get('type') != 'rgbd_pose':
                    raise ValueError(f'invalid rgbd.pose metadata at record {position}')
                # Only the sequence and master capture time are synthesized.
                # Camera and publisher diagnostics describe the original recording.
                meta['frame_id'] = published
                meta['timestamp_ns'] = start_unix_ns + published * 1_000_000_000 // FPS
                parts[1] = msgpack.packb(meta, use_bin_type=True)
                socket.send_multipart(parts)
                published += 1

                if len(entries) > 1:
                    if position == len(entries) - 1:
                        direction = -1
                    elif position == 0:
                        direction = 1
                    position += direction

                if published % (FPS * 10) == 0:
                    print(f'[replay] published={published} source_record={position}',
                          flush=True)
    except KeyboardInterrupt:
        print('\n[replay] stopped', flush=True)
    finally:
        socket.close(0)
        context.term()


if __name__ == '__main__':
    main()
