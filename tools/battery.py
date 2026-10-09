import sys, json, numpy as np
sys.path.insert(0,'/home/claude/sim/work')
from ev import *
L8='/home/claude/sim/rt_low8'; H5='/home/claude/sim/rt_hi5'
def where(o,x,lo,hi,**kw):
    prm,P=split(x); D,v=make_data(o,prm); tr=[]
    r=harness.run(lambda: racer.Racer(D,P),trace=tr,**kw); a=np.array(tr); m=metrics(tr,r,lo,hi)
    t,s,v_,tp,le=a[:,0],a[:,1],a[:,2]/3.6,a[:,3],a[:,13]; ok=t>4
    bad=np.where(ok&(np.abs(le)>0.8))[0]; grp=np.split(bad,np.where(np.diff(bad)>15)[0]+1) if len(bad) else []
    return m,['s=%d-%d v=%.0f err=%+.2f tp=%+.2f'%(int(s[g[0]]),int(s[g[-1]]),v_[g].mean()*3.6,le[g][np.argmax(np.abs(le[g]))],tp[g][np.argmax(np.abs(tp[g]))]) for g in grp]
TESTS=(('nominal',{}),('friction -5%',dict(rt=LOW)),('friction -8%',dict(rt=L8)),('friction +5%',dict(rt=H5)),('delay 1',dict(delay=1)),('noise 11',dict(noise=(11,0.015))),('noise 12',dict(noise=(12,0.015))),('delay+noise',dict(delay=1,noise=(3,0.01))))
def battery(o,x,lo,hi,tests=TESTS):
    out=[]
    for name,kw in tests:
        m,w=where(o,x,lo,hi,**kw); out.append(m); print('%-13s '%name+fmt(m)+'  '+' ; '.join(w)); sys.stdout.flush()
    return out
if __name__=='__main__':
    x=json.load(open(sys.argv[2]))['x']; o=np.load('/home/claude/sim/build/o_%s.npy'%sys.argv[1]); B=float(sys.argv[3])
    for a in sys.argv[4:]:
        k,v=a.split('='); x[k]=float(v)
    battery(o,x,-B*np.ones(N),B*np.ones(N))
