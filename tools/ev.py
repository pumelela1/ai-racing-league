import sys, numpy as np, json
sys.path.insert(0,'/home/claude/sim/work')
from exp import *
import harness, racer
LOW='/home/claude/sim/rt_low'
SPECP=('uy','ub','ca_lat','ca','cd','ksm','zsm','kz_w','bank_w')
def split(x):
    prm=dict(BASE); P={}
    for k,v in x.items(): (prm if k in SPECP else P).__setitem__(k,v)
    return prm,P
def metrics(tr,r,lo,hi):
    a=np.array(tr); m=a[:,0]>4; s=a[:,1]; y=a[:,3]*6.0; i=(s/C['ds']).astype(int)%N
    edge=np.maximum(y-hi[i],lo[i]-y)
    return dict(lap=r['lap'],dmg=r['damage'],tp=float(np.abs(a[m,3]).max()),err=float(np.abs(a[m,13]).max()),edge=float(edge[m].max()),top=r['top'],
                slip=float(np.abs(a[m,10]/np.maximum(a[m,2],20)).max()))
def run1(o,x,lo,hi,**kw):
    prm,P=split(x); D,v=make_data(o,prm); tr=[]
    r=harness.run(lambda: racer.Racer(D,P),trace=tr,**kw)
    mt=metrics(tr,r,lo,hi); mt['plan']=gen.laptime(C,o,v); return mt
def fmt(m): return 'lap=%-7s dmg=%-3.0f tp=%.2f err=%.2f edge=%+.2f slip=%.2f top=%.0f'%(('%.3f'%m['lap']) if m['lap'] else 'DNF',m['dmg'],m['tp'],m['err'],m['edge'],m['slip'],m['top'])
if __name__=='__main__':
    x=json.load(open('/home/claude/sim/build/opt_base.json'))['x']
    lo43=-4.3*np.ones(N); hi43=4.3*np.ones(N)
    los=np.load('/home/claude/sim/build/lo_b.npy'); his=np.load('/home/claude/sim/build/hi_b.npy')
    for name,f,lo,hi in (('base(4.3 full)','o_base',lo43,hi43),('fast43','o_fast43',lo43,hi43),('side-aware','o_side',los,his)):
        o=np.load('/home/claude/sim/build/%s.npy'%f)
        print('%-15s nominal   '%name+fmt(run1(o,x,lo,hi))); sys.stdout.flush()
        print('%-15s frict -5%% '%''+fmt(run1(o,x,lo,hi,rt=LOW)))
        print('%-15s delay 1   '%''+fmt(run1(o,x,lo,hi,delay=1))); sys.stdout.flush()
