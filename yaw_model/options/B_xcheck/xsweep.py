#!/usr/bin/env python3
"""Gate runs for the cross-check, in a pool.

    python3 xsweep.py OUT.jsonl JOBSET [--pool 14]

JOBSET:
  rec       option B's recommended set, every plant: 8 timing members x noise x seeds 1-5
  ff0       P 0.65 I 0.07 FF 0 on vec complete and vec fin (B ran no FF 0 ensemble there):
            nominal timing x noise x seeds 1-5
  ffsens    FF around B's choice at P 0.65 I 0.07, nominal x noise x seeds 1-5:
            vec complete 0.7 0.8 0.9 1.1 1.2 1.4; cs complete 0 0.1; vec fin 0.1 0.2 0.45 0.6;
            and the wrong-plant case: vec rigid at 0.3 and 1.0, cs rigid at 0.05 and 0.1
  act       actuator variants on the recommended set, nominal x noise x seeds 1-5:
            tilt slew 60/240 (allocator + pod), a joint physically limited to 60 while the
            allocator believes 120; Cloudship tail lag 0.025/0.05 s (cs complete at B's
            recommended 0.05, which B did not run)
"""
import json
import math
import multiprocessing as mp
import sys
import time

sys.dont_write_bytecode = True
import xcommon as xc  # noqa: E402

SEEDS = [1, 2, 3, 4, 5]
NOM = ('nominal', {})


def jobs(name):
    out = []
    if name == 'rec':
        for t in xc.PLANTS:
            P, I, FF = xc.REC[t]
            out += [(t, P, I, FF, m, s, {}) for m in xc.TIMING for s in SEEDS]
    elif name == 'ff0':
        for t in (xc.VC, xc.VF):
            out += [(t, 0.65, 0.07, 0.0, NOM, s, {}) for s in SEEDS]
    elif name == 'ffsens':
        for t, ffs in ((xc.VC, (0.7, 0.8, 0.9, 1.1, 1.2, 1.4)), (xc.CC, (0.0, 0.1)), (xc.VF, (0.1, 0.2, 0.45, 0.6)),
                       (xc.VR, (0.3, 1.0)), (xc.CR, (0.05, 0.1))):
            for ff in ffs:
                out += [(t, 0.65, 0.07, ff, NOM, s, {}) for s in SEEDS]
    elif name == 'ideal':
        # the value B's own parameter description prescribes (steady-turn torque / rate
        # at 6.67 deg/s, derive_ff_out.txt closed loop), nominal x noise x seeds 1-5
        for t, ff in ((xc.VC, 0.887), (xc.CC, 0.141), (xc.VF, 0.375)):
            out += [(t, 0.65, 0.07, ff, NOM, s, {}) for s in SEEDS]
    elif name == 'act':
        for a in ({'slew': 60.0}, {'slew': 240.0}, {'phys_slew': 60.0}):
            for t in (xc.VR, xc.VC, xc.VF):
                P, I, FF = xc.REC[t]
                out += [(t, P, I, FF, NOM, s, a) for s in SEEDS]
            for t in (xc.VC, xc.VF):          # the same actuator without FF, for the comparison
                out += [(t, 0.65, 0.07, 0.0, NOM, s, a) for s in SEEDS]
        for a in ({'tail_tau': 0.025}, {'tail_tau': 0.05}):
            for t in (xc.CR, xc.CC):
                P, I, FF = xc.REC[t]
                out += [(t, P, I, FF, NOM, s, a) for s in SEEDS]
    else:
        raise SystemExit('unknown job set ' + name)
    return out


def clean(v):
    if isinstance(v, float) and not math.isfinite(v):
        return None
    if isinstance(v, dict):
        return {k: clean(x) for k, x in v.items()}
    return v


def one(job):
    t, P, I, FF, (label, tkw), seed, act = job
    t0 = time.time()
    try:
        res, _ = xc.run(t, P, I, FF, tkw=tkw, seed=seed, **act)
        checks = {k: clean(v) for k, v in xc.judge(res, t).items()}
        err, status = None, res.status
    except Exception as e:
        import traceback
        checks, err, status = {}, traceback.format_exc(), None
    return {'target': t, 'P': P, 'I': I, 'FF': FF, 'member': label, 'seed': seed, 'act': act,
            'status': status, 'checks': checks, 'err': err, 'secs': round(time.time() - t0, 1)}


def main():
    out, name = sys.argv[1], sys.argv[2]
    pool_n = int(sys.argv[sys.argv.index('--pool') + 1]) if '--pool' in sys.argv else 14
    js = jobs(name)
    # the long vectored runs first so the pool's tail is short
    js.sort(key=lambda j: 0 if 'vectored' in j[0] else 1)
    print('%s: %d jobs' % (name, len(js)), flush=True)
    with mp.Pool(pool_n) as pool, open(out, 'a') as fh:
        for i, r in enumerate(pool.imap_unordered(one, js), 1):
            fh.write(json.dumps(r, ensure_ascii=False) + '\n')
            fh.flush()
            if i % 20 == 0 or i == len(js):
                print('%d/%d' % (i, len(js)), flush=True)
    print('DONE ' + name, flush=True)


if __name__ == '__main__':
    main()
