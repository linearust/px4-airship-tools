#!/usr/bin/env python3
"""Logic 2 (Saleae) control over its MCP server, and capture decoding.

Logic 2 must be running with "Enable MCP Server" ticked in its settings
(default 127.0.0.1:10530). Launch it with:

    env -u ELECTRON_RUN_AS_NODE ~/Applications/Logic.AppImage

Probe order on this bench, channel D0..D7:

    D0..D3  MAIN1-4  control surfaces (fins)   50 Hz
    D4      MAIN5    starboard pod tilt servo  50 Hz
    D5      MAIN6    port pod tilt servo       50 Hz
    D6      AUX1     starboard pod motor       400 Hz
    D7      AUX2     port pod motor            400 Hz

The original Logic has a fixed input threshold: never send
digitalThresholdVolts, it kills the app.
"""
import json
import os
import statistics
import struct
import sys
import time
import urllib.error
import urllib.request

MCP = 'http://127.0.0.1:10530/mcp'

CHANNELS = ['MAIN1 fin', 'MAIN2 fin', 'MAIN3 fin', 'MAIN4 fin',
            'MAIN5 tilt S', 'MAIN6 tilt P', 'AUX1 motor S', 'AUX2 motor P']

MOTOR_S, MOTOR_P = 6, 7
TILT_S, TILT_P = 4, 5


def call(tool, args=None, timeout=900):
    body = json.dumps({'jsonrpc': '2.0', 'id': 1, 'method': 'tools/call',
                       'params': {'name': tool, 'arguments': args or {}}}).encode()
    req = urllib.request.Request(MCP, data=body,
                                 headers={'Content-Type': 'application/json',
                                          'Accept': 'application/json, text/event-stream'})
    with urllib.request.urlopen(req, timeout=timeout) as response:
        text = response.read().decode()

    if text.lstrip().startswith('{'):
        reply = json.loads(text)
    else:  # server-sent events
        reply = None
        for line in text.splitlines():
            if line.startswith('data:'):
                reply = json.loads(line[5:].strip())
        if reply is None:
            raise RuntimeError('MCP reply not understood: %r' % text[:200])

    if 'error' in reply:
        raise RuntimeError('MCP error: %r' % reply['error'])

    result = reply['result']

    if result.get('isError'):
        raise RuntimeError('tool error: %r' % result)

    for item in result.get('content', []):
        if item.get('type') == 'text':
            try:
                return json.loads(item['text'])
            except json.JSONDecodeError:
                return item['text']

    return result


def available():
    try:
        call('get_devices', timeout=5)
        return True
    except (urllib.error.URLError, OSError, RuntimeError):
        return False


def start(duration_s, sample_rate=2000000):
    result = call('start_capture', {
        'logicDeviceConfiguration': {
            'logicChannels': {'digitalChannels': list(range(8))},
            'digitalSampleRate': sample_rate,
        },
        'captureConfiguration': {'timedCaptureMode': {'durationSeconds': duration_s}},
    })
    capture_id = result['captureId'] if isinstance(result, dict) else result
    return capture_id, time.time()


def finish(capture_id, sal_path, export_dir):
    """Wait for the capture, save it, export the documented binary format.

    export_raw_data_binary answers {} even when it worked, so the files are
    the only proof; never close the capture before they exist.
    """
    call('wait_capture', {'captureId': capture_id})

    for _ in range(10):
        try:
            call('save_capture', {'captureId': capture_id, 'filepath': sal_path})
            if os.path.exists(sal_path) and os.path.getsize(sal_path):
                break
        except RuntimeError as error:
            print('save_capture 재시도: %s' % error, file=sys.stderr)
        time.sleep(2)

    os.makedirs(export_dir, exist_ok=True)

    for _ in range(15):
        try:
            call('export_raw_data_binary', {'captureId': capture_id, 'directory': export_dir,
                                            'analogDownsampleRatio': 1})
        except RuntimeError as error:
            print('export 재시도: %s' % error, file=sys.stderr)
        if os.path.exists(os.path.join(export_dir, 'digital_0.bin')):
            break
        time.sleep(2)

    if not os.path.exists(os.path.join(export_dir, 'digital_0.bin')):
        # Closing now would throw the measurement away. Leave it open in
        # Logic 2 so it can be exported by hand.
        print('내보내기가 안 됐다. 캡처 %s 를 Logic 2 에 열어 둔다.' % capture_id, file=sys.stderr)
        raise RuntimeError('export_raw_data_binary 가 파일을 남기지 않았다')

    call('close_capture', {'captureId': capture_id})
    return export_dir


def read_export(export_dir):
    """Saleae binary export v0: '<SALEAE>', int32 version, int32 type,
    uint32 initial state, double begin, double end, uint64 n, double[n] edge
    times. Returns {channel: (initial_state, begin, end, [edge times])}."""
    out = {}

    for name in sorted(os.listdir(export_dir)):
        if not (name.startswith('digital_') and name.endswith('.bin')):
            continue

        data = open(os.path.join(export_dir, name), 'rb').read()

        if data[:8] != b'<SALEAE>':
            raise RuntimeError('%s: not a Saleae export' % name)

        initial_state, = struct.unpack_from('<I', data, 16)
        begin, end = struct.unpack_from('<dd', data, 20)
        count, = struct.unpack_from('<Q', data, 36)
        out[int(name[8:-4])] = (initial_state, begin, end,
                                list(struct.unpack_from('<%dd' % count, data, 44)))

    return out


def pulses(initial_state, edges):
    """[(rising edge time, high width in us)] from an edge list."""
    state, rise, out = initial_state, None, []

    for edge in edges:
        state ^= 1
        if state:
            rise = edge
        elif rise is not None:
            out.append((rise, (edge - rise) * 1e6))

    return out


def widths(export_dir, window=0.25):
    """{channel: [(time, median pulse width us)]} sampled every `window`."""
    out = {}

    for channel, (initial_state, _begin, _end, edges) in read_export(export_dir).items():
        found = pulses(initial_state, edges)
        rows = []

        if found:
            time_s, last = found[0][0], found[-1][0]

            while time_s <= last:
                in_window = [w for (t, w) in found if time_s <= t < time_s + window]
                if in_window:
                    rows.append((time_s, statistics.median(in_window)))
                time_s += window

        out[channel] = rows

    return out


def at(rows, when, tolerance=0.15):
    """Pulse width at a time, or nan."""
    near = [w for (t, w) in rows if abs(t - when) < tolerance]
    return near[0] if near else float('nan')


def median(rows, start, end):
    """Median pulse width over the half-open window [start, end), or nan."""
    window = [w for (t, w) in rows if start <= t < end]
    return statistics.median(window) if window else float('nan')


def slope(rows, start, end):
    """Least-squares slope in us/s over a window."""
    points = [(t, w) for (t, w) in rows if start <= t <= end]

    if len(points) < 3:
        return float('nan')

    n = len(points)
    sum_t = sum(t for t, _ in points)
    sum_w = sum(w for _, w in points)
    sum_tt = sum(t * t for t, _ in points)
    sum_tw = sum(t * w for t, w in points)
    return (n * sum_tw - sum_t * sum_w) / (n * sum_tt - sum_t * sum_t)


if __name__ == '__main__':
    print(json.dumps(call('get_devices'), indent=1))
