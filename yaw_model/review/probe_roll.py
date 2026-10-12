import math, sys, os
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import planar, gz_model
DT = 0.004
def roll_period(model, kick_nm, kick_s=0.9, seconds=8.0):
    p = planar.RollPlanarPlant('probe', model=model)
    s, phi = [0.0]*4, 0.0
    ts, phis = [], []
    t = 0.0
    while t < kick_s + seconds:
        ext = [0.0]*6
        if t < kick_s:
            ext[3] = kick_nm
        k1, d1 = p._deriv(s, phi, ext)
        s2 = [s[i] + 0.5*DT*k1[i] for i in range(4)]; k2, d2 = p._deriv(s2, phi+0.5*DT*d1, ext)
        s3 = [s[i] + 0.5*DT*k2[i] for i in range(4)]; k3, d3 = p._deriv(s3, phi+0.5*DT*d2, ext)
        s4 = [s[i] + DT*k3[i] for i in range(4)]; k4, d4 = p._deriv(s4, phi+DT*d3, ext)
        s = [s[i] + DT*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6 for i in range(4)]
        phi += DT*(d1+2*d2+2*d3+d4)/6
        t += DT
        if t >= kick_s:
            ts.append(t); phis.append(phi)
    mean = sum(phis)/len(phis)
    cr = []
    for i in range(1, len(phis)):
        a, b = phis[i-1]-mean, phis[i]-mean
        if a != 0 and (a < 0) != (b < 0):
            cr.append(ts[i-1] + DT*a/(a-b))
    return 2*(cr[-1]-cr[0])/(len(cr)-1), math.degrees(max(phis)-min(phis))/2
for m, gz in (('cloudship', 1.628), ('cloudship_am', 1.886), ('airship_vectored_independent', 1.659), ('airship_vectored_independent_am', 2.297)):
    fam = gz_model.family(m)
    ixx = gz_model.inertia(fam, 0)
    T, amp = roll_period(m, 2.0/2.3865*ixx)
    print('%-34s model %.3f s  gz %.3f s  (amp %.1f deg)' % (m, T, gz, amp))
