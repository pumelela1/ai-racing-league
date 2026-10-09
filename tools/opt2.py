import sys, numpy as np, random, time, json, os
sys.path.insert(0,'/home/claude/sim/work')
from ev import *
from multiprocessing import Pool
SPEC=dict(uy=(0.90,1.15), ub=(0.58,0.80), ca_lat=(0.0034,0.0048),
 la0=(0.5,6.0), la1=(0.0,0.20), la_min=(3.5,8.0), ke=(0.0,0.20), ke_v=(10,60),
 tl=(0.0,0.12), kpb=(1.0,5.0), kpa=(0.3,1.5), ax_k=(0.9,1.20), uy_c=(0.85,1.15),
 tcs_slip=(1.0,3.5), tcs_k=(0.1,0.5), rpm_up=(17500,18500), rpm_dn=(16000,17600), abs_slip=(0.10,0.25))
SCEN=[dict(),dict(rt=LOW,cfg='config/raceman/quickrace2.xml',port=3002),dict(delay=1),dict(noise=(7,0.01),cfg='config/raceman/quickrace2.xml',port=3002)]
def one(args):
    o,x,lo,hi,kw=args
    try: return run1(o,x,lo,hi,max_time=120.0,**kw)
    except BaseException as e: return dict(lap=None,dmg=9999,tp=9,err=9,edge=9,slip=9,top=0,plan=0)
def score(rs):
    if any(r['lap'] is None for r in rs): return 1e4
    J=rs[0]['lap']+0.25*(max(r['lap'] for r in rs)-rs[0]['lap'])
    for r in rs:
        J+=1000*(r['dmg']>0)+60*max(0,r['edge']-0.30)+3*max(0,r['err']-0.9)+20*max(0,r['slip']-0.25)
    return J
def evaluate(pool,o,x,lo,hi):
    a=pool.map(one,[(o,x,lo,hi,SCEN[0]),(o,x,lo,hi,SCEN[1])]); b=pool.map(one,[(o,x,lo,hi,SCEN[2]),(o,x,lo,hi,SCEN[3])])
    rs=a+b; return score(rs),rs
if __name__=='__main__':
    line=sys.argv[1]; iters=int(sys.argv[2]); seed=int(sys.argv[3]); out=sys.argv[4]; start=sys.argv[5]
    o=np.load(line); random.seed(seed)
    lo=np.load(sys.argv[6]) if len(sys.argv)>6 else -4.3*np.ones(N); hi=np.load(sys.argv[7]) if len(sys.argv)>7 else 4.3*np.ones(N)
    x=json.load(open(start))['x']
    for k,(a,b) in SPEC.items(): x[k]=min(b,max(a,x[k]))
    pool=Pool(2)
    J,rs=evaluate(pool,o,x,lo,hi); best=(J,dict(x),rs)
    print('start J=%.3f '%J+' | '.join('%.2f e%.2f s%.2f'%(r['lap'] or -1,r['edge'],r['slip']) for r in rs)); sys.stdout.flush()
    t0=time.time(); sig=0.10
    for it in range(iters):
        y=dict(best[1]); ks=random.sample(list(SPEC),random.choice([1,2,2,3]))
        for k in ks:
            a,b=SPEC[k]; y[k]=min(b,max(a,y[k]+random.gauss(0,sig)*(b-a)))
        J,rs=evaluate(pool,o,y,lo,hi)
        if J<best[0]:
            best=(J,y,rs)
            print('it %d (%.0fs) J=%.3f '%(it,time.time()-t0,J)+' | '.join('%.2f e%.2f r%.2f s%.2f'%(r['lap'],r['edge'],r['err'],r['slip']) for r in rs)+' '+str(ks)); sys.stdout.flush()
            json.dump(dict(J=J,x=y,rs=rs),open(out,'w'))
        if it%50==49: sig=max(0.04,sig*0.8)
    print('done',best[0],json.dumps(best[1]))
