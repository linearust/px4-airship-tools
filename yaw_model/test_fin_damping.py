#!/usr/bin/env python3
"""Tests of fin_damping.py, the G3 comparison. Plain python:

    python3 test_fin_damping.py

The engine's printed numbers are read from its chain logs and a block that
lacks one fails loudly; each kinematics record on disk is matched to the run
whose printed fit it reproduces; the model's stream starts where the record's
did; a stream that starts before the push sees the hull at rest; on the
vectored _fin plant, with the record's own stream, the model reproduces the
record's fit and r(t), and a 2 % change of the fins' lift slope shows.
"""
import math
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import fin_damping as fd  # noqa: E402

FAILS = []


def check(name, ok, detail=''):
    print('%s  %s%s' % ('PASS' if ok else 'FAIL', name, ('  -- ' + detail) if detail else ''))
    if not ok:
        FAILS.append(name)


def test_engine_runs():
    v, c = fd.engine_runs('airship_vectored_independent'), fd.engine_runs('cloudship')
    keys = ('I', 'dI', 'K_s_fin', 'K_d_fin', 'K_s_am_fin', 'K_d_held', 'I_am')
    complete = all(all(isinstance(r[k], float) and math.isfinite(r[k]) for k in keys) for r in v + c)
    check('the printed runs are read: two vectored blocks (chain5, chain9), one Cloudship block (chain5), every '
          'number finite, I_am = I + dI',
          [r['file'] for r in v] == list(fd.CHAIN_LOGS) and [r['file'] for r in c] == [fd.CHAIN_LOGS[0]] and complete
          and all(r['I_am'] == r['I'] + r['dI'] for r in v + c),
          ', '.join('%s K_d %.4f' % (r['where']['K_d_fin'], r['K_d_fin']) for r in v + c))
    check('the held damping is the run\'s own rigid fin damping', all(r['K_d_held'] == r['K_d_fin'] for r in v + c))


def test_block_missing_a_line():
    """A block whose K_d line failed (the check prints '없음' then) is not read
    as a run without it."""
    tmp = tempfile.mkdtemp()
    real_out, real_logs = fd.SITL_OUT, fd.CHAIN_LOGS

    try:
        with open(os.path.join(real_out, real_logs[1]), encoding='utf-8') as handle:
            text = handle.read()

        broken = text.replace('K_d(fin) ', 'K_d(fin) 없음 ', 1)
        assert broken != text
        with open(os.path.join(tmp, 'broken.txt'), 'w', encoding='utf-8') as handle:
            handle.write(broken)

        fd.SITL_OUT, fd.CHAIN_LOGS = tmp, ('broken.txt',)
        fd.engine_runs.cache_clear()

        try:
            fd.engine_runs('airship_vectored_independent')
            raised = None
        except LookupError as exc:
            raised = str(exc)
    finally:
        fd.SITL_OUT, fd.CHAIN_LOGS = real_out, real_logs
        fd.engine_runs.cache_clear()
        shutil.rmtree(tmp)

    check('a block that lacks a printed number raises', raised is not None and 'K_d_fin' in raised, raised or 'no raise')


def test_records_and_starts():
    want = {'airship_vectored_independent_fin': 'chain9_20260927.txt', 'cloudship_fin': 'chain5_20260925.txt',
            'airship_vectored_independent_am_fin': 'chain9_20260927.txt', 'cloudship_am_fin': 'chain5_20260925.txt'}
    got = {}

    for model in want:
        rows, run, fit = fd.engine_record(model)
        got[model] = run['file'] if run else None

    check('each record on disk refits to exactly the run that wrote it', got == want, str(got))
    rows, run, fit = fd.engine_record('airship_vectored_independent_fin')
    offsets, interval, gaps = fd.record_stream(rows)
    k, err, u = fd.matched_start('airship_vectored_independent_fin', offsets, rows[0][1])
    a = fd.AM['PUSH_ACCEL'][0]
    check('the model\'s stream starts where the record\'s did: its first u within half a step of the push\'s '
          'acceleration (%.5f vs %.5f)' % (u, rows[0][1]), err < 0.5 * a * fd.DT, '%+d steps' % k)
    check('the record\'s poses every 0.020 s, 150 of them: the check\'s 60 Hz assumption is not what it sampled',
          abs(interval - 0.02) < 1e-9 and len(offsets) == int(fd.AM['PUSH_S'] * fd.AM['RATE_HZ']),
          'interval %.4f, %d poses' % (interval, len(offsets)))


def test_stream_before_push():
    F, couple = fd._push('airship_vectored_independent_fin')
    rows = fd.Probe('airship_vectored_independent_fin').run(F, couple, fd.even(10, 0.02), late=-0.1)
    kin = fd.AM['kinematics'](rows)
    early = [x for x in kin if x[0] < -0.02]
    check('a stream that starts before the push sees the hull at rest until the wrench comes on',
          early and all(x[1] == 0.0 and x[3] == 0.0 for x in early) and kin[-1][1] > 0.0,
          '%d rows before, last u %.4f' % (len(early), kin[-1][1]))


def test_vectored_reproduces_the_record():
    m = fd.measure('airship_vectored_independent_fin')
    rel = m['D'] / m['record_fit']['D'] - 1
    r = m['rows_diff']['r']
    rms = math.degrees(math.sqrt(sum(x * x for x in r) / len(r)))
    check('vectored _fin on the record\'s stream: K_d / I within 0.5 %% of the record\'s fit (%+.3f %%) and r within '
          '0.01 deg/s rms (%.4f)' % (100 * rel, rms), abs(rel) < 0.005 and rms < 0.01 and m['t_skew'] < 1e-9)
    lever = fd.fin_leverage('airship_vectored_independent_fin', (0.98, 1.02))
    check('a 2 % change of the fins\' lift slope moves K_d / I by more than 1.5 % each way and the r residual '
          'above three times the files\'',
          lever[0][1] / m['D'] < 0.985 and lever[1][1] / m['D'] > 1.015 and min(x[2] for x in lever) > 3 * math.radians(rms),
          ', '.join('x%g: %+.2f %%, r rms %.4f deg/s' % (x[0], 100 * (x[1] / m['D'] - 1), math.degrees(x[2]))
                    for x in lever))


if __name__ == '__main__':
    test_engine_runs()
    test_block_missing_a_line()
    test_records_and_starts()
    test_stream_before_push()
    test_vectored_reproduces_the_record()
    print('\n%d failed' % len(FAILS) + (': ' + ', '.join(FAILS) if FAILS else ''))
    sys.exit(1 if FAILS else 0)
