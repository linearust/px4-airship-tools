#!/usr/bin/env python3
"""Tables from beyond.jsonl / beyond_act.jsonl / wcock.jsonl: per plant, per
scenario metric, one column per candidate:gains, mean over seeds (None -> 'never')."""
import collections, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.dont_write_bytecode = True
import option_d as od

METRICS = {
    'step0.2': ['overshoot_pct', 'stop_1', 'coast_deg', 'reverse_peak', 'drift_end_deg'],
    'step0.5': ['overshoot_pct', 't90', 'settle10', 'integ_at_release', 'stop_1', 'stop_03', 'coast_deg',
                'reverse_peak', 'drift_end_deg', 'rate_10s', 'rel_u_std', 'rel_tilt_deg_per_min', 'rel_N_sign_per_min'],
    'step1.0': ['overshoot_pct', 'stop_1', 'coast_deg', 'reverse_peak', 'drift_end_deg'],
    'partial': ['settle_new_sp', 'undershoot', 'mean_last5'],
    'rev_step': ['t_reach90', 'settle10', 'overshoot_pct'],
    'rev_ramp': ['t_reach90', 'settle10', 'overshoot_pct'],
    'wind0.0185': ['peak', 'max_dpsi', 'ss_rate', 'ss_integ'],
    'wind0.05': ['peak', 'max_dpsi', 'ss_rate', 'ss_integ', 'recover_03'],
    'wind0.1': ['peak', 'max_dpsi', 'ss_rate', 'ss_integ'],
    'wind0.2': ['peak', 'max_dpsi', 'ss_rate', 'ss_integ'],
    'relwind+': ['trim_before', 'integ_at_release', 'stop_1', 'coast_deg', 'reverse_peak', 'drift_end_deg'],
    'relwind-': ['trim_before', 'integ_at_release', 'stop_1', 'coast_deg', 'reverse_peak', 'drift_end_deg'],
    'nudge': ['mean_rate', 'dpsi_vs_sp', 'events'],
    'edge': ['mean_rate', 'dpsi_vs_sp', 'events'],
    'rest': ['u_std', 'du_rms', 'N_sign_per_min', 'tilt_deg_per_min', 'rate_std'],
    'restwind': ['u_std', 'du_rms', 'N_sign_per_min', 'tilt_deg_per_min', 'rate_std'],
}


def fmt(v):
    if v is None:
        return 'never'
    if isinstance(v, str):
        return v
    a = abs(v)
    return ('%.4f' if a < 0.1 else '%.3f' if a < 1 else '%.2f' if a < 100 else '%.0f') % v


def mean(vals):
    if any(v is None for v in vals):
        known = [v for v in vals if v is not None]
        return None if not known else ('%s (%d/%d never)' % (fmt(sum(known) / len(known)), len(vals) - len(known), len(vals)))
    return sum(vals) / len(vals)


def main(path, only=None):
    rows = [json.loads(l) for l in open(os.path.join(HERE, path))]
    by = collections.defaultdict(list)
    cols = []
    for r in rows:
        var = ('slew%g' % r['slew']) if r.get('slew') else ('tau%g' % r['tail_tau']) if r.get('tail_tau') else ''
        col = '%s:%s%s' % (r['name'], r['gains'], (':' + var) if var else '')
        if col not in cols:
            cols.append(col)
        by[(r['target'], r['scen'], col)].append(r['m'])
    cols.sort()
    scens = sorted({k[1] for k in by}, key=lambda s: (list(METRICS).index(s) if s in METRICS else 99, s))
    out = []
    for t in od.TARGETS5:
        out.append('\n######## %s' % od.SHORT[t])
        tcols = [c for c in cols if any((t, s, c) in by for s in scens)]
        if only:
            tcols = [c for c in tcols if any(o in c for o in only.split(','))]
        out.append('%-34s %s' % ('', ' '.join('%-17s' % c[:17] for c in tcols)))
        for s in scens:
            mets = METRICS.get(s) or sorted({k for c in tcols for m in by.get((t, s, c), []) for k in m})
            for m in mets:
                cells = []
                for c in tcols:
                    ms = by.get((t, s, c))
                    cells.append('%-17s' % (fmt(mean([x.get(m) for x in ms]))[:17] if ms else '-'))
                out.append('%-34s %s' % ((s + ' ' + m)[:34], ' '.join(cells)))
    print('\n'.join(out))


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'beyond.jsonl', sys.argv[2] if len(sys.argv) > 2 else None)
