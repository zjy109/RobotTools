#!/usr/bin/env python3
"""Record every received rgbd.pose message as a lossless offline dataset."""
import argparse
import json
import os
import signal
import struct
import time
from datetime import datetime
from pathlib import Path

import msgpack
import zmq

U64 = struct.Struct('<Q')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--endpoint', default='ipc:///tmp/rgbd_pose.ipc')
    parser.add_argument('--topic', default='rgbd.pose')
    parser.add_argument('--output', type=Path, help='New dataset directory')
    parser.add_argument('--duration', type=float, default=0, help='Seconds; 0 means until Ctrl+C')
    parser.add_argument('--max-frames', type=int, default=0, help='0 means unlimited')
    parser.add_argument('--receive-hwm', type=int, default=1000)
    args = parser.parse_args()
    if args.duration < 0 or args.max_frames < 0 or args.receive_hwm < 1:
        parser.error('duration and max-frames must be nonnegative; receive-hwm must be positive')
    output = (args.output or Path(__file__).resolve().parent /
              datetime.now().strftime('recording_%Y%m%d_%H%M%S')).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=False)
    manifest_path = output / 'manifest.json'
    manifest = dict(format='rgbd_pose_zmq_v1', status='recording',
                    endpoint=args.endpoint, subscription_topic=args.topic,
                    started_unix_ns=time.time_ns(), frames=0, invalid_messages=0,
                    frame_id_gaps=0, files={'messages': 'messages.bin', 'index': 'index.jsonl'})

    def write_manifest():
        temp = manifest_path.with_suffix('.json.tmp')
        with temp.open('w', encoding='utf-8') as f:
            json.dump(manifest, f, ensure_ascii=False, indent=2)
            f.write('\n')
        os.replace(temp, manifest_path)

    write_manifest()
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, args.receive_hwm)
    sock.setsockopt(zmq.SUBSCRIBE, args.topic.encode())
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(args.endpoint)
    stop = False

    def stop_handler(_signum, _frame):
        nonlocal stop
        stop = True

    old_int = signal.signal(signal.SIGINT, stop_handler)
    old_term = signal.signal(signal.SIGTERM, stop_handler)
    deadline = time.monotonic() + args.duration if args.duration else None
    last_id = None
    last_print = time.monotonic()
    error = None
    print(f'[record] {args.endpoint} topic={args.topic} -> {output}', flush=True)
    try:
        with (output / 'messages.bin').open('wb') as data, \
             (output / 'index.jsonl').open('w', encoding='utf-8') as index:
            while not stop:
                if deadline is not None and time.monotonic() >= deadline:
                    break
                if args.max_frames and manifest['frames'] >= args.max_frames:
                    break
                timeout = 200 if deadline is None else max(0, min(200, int((deadline - time.monotonic()) * 1000)))
                if not sock.poll(timeout, zmq.POLLIN):
                    continue
                parts = sock.recv_multipart()
                if len(parts) != 4:
                    manifest['invalid_messages'] += 1
                    continue
                try:
                    meta = msgpack.unpackb(parts[1], raw=False)
                    frame_id = int(meta['frame_id'])
                    timestamp_ns = int(meta['timestamp_ns'])
                    if (meta.get('type') != 'rgbd_pose' or
                        len(parts[2]) != int(meta['width']) * int(meta['height']) * 3 or
                        len(parts[3]) != int(meta['depth_width']) * int(meta['depth_height']) * 2):
                        raise ValueError('unexpected message type or image size')
                except (ValueError, TypeError, KeyError, msgpack.ExtraData, msgpack.FormatError) as exc:
                    manifest['invalid_messages'] += 1
                    print(f'[record] skipped invalid message: {exc}', flush=True)
                    continue
                offset = data.tell()
                for part in parts:
                    data.write(U64.pack(len(part)))
                    data.write(part)
                index.write(json.dumps(dict(record=manifest['frames'], offset=offset,
                                            size=data.tell() - offset, frame_id=frame_id,
                                            timestamp_ns=timestamp_ns,
                                            received_unix_ns=time.time_ns()),
                                       separators=(',', ':')) + '\n')
                manifest['frames'] += 1
                if last_id is not None and frame_id > last_id + 1:
                    manifest['frame_id_gaps'] += frame_id - last_id - 1
                last_id = frame_id
                if manifest['frames'] % 30 == 0:
                    data.flush()
                    index.flush()
                    os.fsync(data.fileno())
                    os.fsync(index.fileno())
                if time.monotonic() - last_print >= 5:
                    print(f"[record] frames={manifest['frames']} gaps={manifest['frame_id_gaps']} "
                          f"size={data.tell() / 1e9:.2f} GB", flush=True)
                    last_print = time.monotonic()
            data.flush()
            index.flush()
            os.fsync(data.fileno())
            os.fsync(index.fileno())
    except Exception as exc:
        error = exc
        manifest['error'] = repr(exc)
    finally:
        manifest['ended_unix_ns'] = time.time_ns()
        manifest['status'] = 'error' if error else 'complete'
        write_manifest()
        sock.close(0)
        ctx.term()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        print(f"[record] {manifest['status']}: {manifest['frames']} frames; {output}", flush=True)
    if error:
        raise error


if __name__ == '__main__':
    main()
