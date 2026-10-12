import json, sys
sys.dont_write_bytecode = True
import xcommon as xc
for fn in sys.argv[1:]:
    rows=[json.loads(l) for l in open(fn)]
    rows.sort(key=lambda r:(json.dumps(r['act']), xc.PLANTS.index(r['target']), r['P'], r['I'], r['FF'], str(r['kind'])))
    for r in rows:
        x=r['res']; h='%-12s P%.2f I%.2f FF%.2f %-5s %s' % (xc.NAME[r['target']], r['P'], r['I'], r['FF'], r['kind'], json.dumps(r['act']) if r['act'] else '')
        if r['err']: print(h, 'ERR', r['err'][-300:]); continue
        if r['kind']=='gust':
            print(h, 'rest: peak %.2f rec %.1f integ/need %.2f tq_hp %.2e | turn: base %.2f (drift5s %.3f) dev %.2f rec %.1f dev_end %.2f integ %.3f->%.3f | jitter %.2e' % (x['peak'],x['recover'],x['integ_ratio'],x['tq_hp_rest'],x['turn_base'],x['turn_base_drift_5s'],x['turn_dev_peak'],x['turn_recover'],x['turn_dev_end'],x['turn_integ_before'],x['turn_integ_end'],x['tq_hp_turn_jitter']))
        else:
            print(h, 'sp %.2f t90 %s peak %.2f over %.0f%% ss %.2f tq %.3f integ %.3f | r0 %.2f rel %s under %.2f end %.2f' % (x['sp'], ('%.1f'%x['t90']) if x['t90'] is not None else '-', x['peak'], x['over_pct'], x['ss'], x['ss_torque'], x['integ_end'], x['r0'], ('%.1f'%x['rel']) if x['rel'] is not None else 'never', x['under'], x['rel_end']))
