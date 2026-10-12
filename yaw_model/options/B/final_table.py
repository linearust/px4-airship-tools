#!/usr/bin/env python3
"""The judged table for a set of per-plant settings from the ensemble files.

    python3 final_table.py            # the recommended set and the single-FF comparison
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True
import ffcommon as fc  # noqa: E402
from summarize import ORDER, describe  # noqa: E402

SETS = {
    'RECOMMENDED per-plant FF, P 0.65 I 0.07': ('ens2.jsonl', {
        'gz_airship_vectored_independent': (0.65, 0.07, 0.0),
        'gz_airship_vectored_independent_am_fin_drag': (0.65, 0.07, 1.0),
        'gz_cloudship': (0.65, 0.07, 0.0),
        'gz_cloudship_am_fin_drag': (0.65, 0.07, 0.05),
        'gz_airship_vectored_independent_fin': (0.65, 0.07, 0.3)}),
    'per-plant FF, P 0.70 I 0.09 (cs complete FF 0.1)': ('ens.jsonl', {
        'gz_airship_vectored_independent': (0.7, 0.09, 0.0),
        'gz_airship_vectored_independent_am_fin_drag': (0.7, 0.09, 1.0),
        'gz_cloudship': (0.7, 0.09, 0.0),
        'gz_cloudship_am_fin_drag': (0.7, 0.09, 0.1),
        'gz_airship_vectored_independent_fin': (0.7, 0.09, 0.3)}),
    'ONE FF for all, P 0.70 I 0.09 FF 0.1': ('ens.jsonl', {t: (0.7, 0.09, 0.1) for t in fc.TARGETS5}),
    'reference FF 0, P 0.70 I 0.09': ('ens.jsonl', {t: (0.7, 0.09, 0.0) for t in fc.TARGETS5}),
    'reference FF 0, P 0.65 I 0.07 (vec complete, vec fin not run at FF 0)': ('ens2.jsonl', {
        t: (0.65, 0.07, 0.0) for t in ('gz_airship_vectored_independent', 'gz_cloudship', 'gz_cloudship_am_fin_drag')}),
    'cs complete alternatives at P 0.65 I 0.07': ('ens2.jsonl', {'gz_cloudship_am_fin_drag': (0.65, 0.07, 0.1)}),
}


def main():
    for title, (path, cfg) in SETS.items():
        rows = [json.loads(l) for l in open(os.path.join(HERE, path))]
        print('\n##### %s  (%s)' % (title, path))
        for t, (P, I, FF) in cfg.items():
            rs = [r for r in rows if r['target'] == t and (r['P'], r['I'], r['FF']) == (P, I, FF) and not r['act']]
            if not rs:
                continue
            print('  %-12s P %.2f I %.2f FF %.2f  (%d runs: 8 timing members x gyro noise x seeds 1-5)'
                  % (fc.SHORT[t], P, I, FF, len(rs)))
            for c in ORDER:
                recs = [r['checks'][c] for r in rs if c in r['checks']]
                if recs:
                    tag, det = describe(c, recs)
                    print('    %-13s %-16s %s' % (c, tag, det))


if __name__ == '__main__':
    main()
