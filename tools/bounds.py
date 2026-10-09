import sys, numpy as np
sys.path.insert(0,'/home/claude/sim/work')
import trk, gen
def side_kind(sides,prefix):
    for x in sides:
        if x.startswith(prefix+'0:'):
            mat=x.split('/')[-1]
            if mat.startswith('roadA'): return 'road'
            if mat.startswith('curb'): return 'curb'
            if mat.startswith('barrier'): return 'wall'
            return 'other'
    return 'none'
def make_bounds(T,C,road=5.3,curb=4.5,wall=4.0,other=4.3,blend=25):
    """Per-metre limits for the car centre (m from the centre line): (right_limit<=0, left_limit>=0)."""
    val=dict(road=road,curb=curb,wall=wall,other=other,none=other)
    segs=T['segs']; N=C['N']; L=T['length']
    lg=np.array([s['lg'] for s in segs]); kl=[side_kind(s['sides'],'L') for s in segs]; kr=[side_kind(s['sides'],'R') for s in segs]
    idx=np.searchsorted(lg,C['s'],side='right')-1
    hi=np.array([val[kl[i]] for i in idx]); lo=np.array([val[kr[i]] for i in idx])
    # be conservative near changes: running minimum over +-blend metres, then smooth
    def soften(a):
        n=len(a); ext=np.concatenate([a[-blend:],a,a[:blend]])
        mn=np.array([ext[i:i+2*blend+1].min() for i in range(n)])
        return gen.smooth(mn,blend)
    return -soften(lo), soften(hi), kl, kr, idx
if __name__=='__main__':
    T=trk.load(); C=gen.resample(T)
    lo,hi,kl,kr,idx=make_bounds(T,C)
    from collections import Counter
    print('left kinds (m):',Counter(kl[i] for i in idx)); print('right kinds (m):',Counter(kr[i] for i in idx))
    print('hi range',hi.min(),hi.max(),'mean',hi.mean(),' lo range',lo.min(),lo.max(),'mean',lo.mean())
    # where are walls
    w=[int(C['s'][i]) for i in range(C['N']) if kl[idx[i]]=='wall' or kr[idx[i]]=='wall']
    print('wall metres:',len(w),'from',w[0] if w else None,'to',w[-1] if w else None)
    np.save('/home/claude/sim/build/lo_b.npy',lo); np.save('/home/claude/sim/build/hi_b.npy',hi)
