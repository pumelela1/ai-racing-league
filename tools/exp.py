import sys, time, numpy as np
sys.path.insert(0,'/home/claude/sim/work')
import trk, gen, racer, harness
T=trk.load(); C=gen.resample(T); N=C['N']
C['bank']=np.load('/home/claude/sim/build/bank.npy')
BASE=dict(uy=0.80,ub=0.65,ca=0.0048,cd=0.0013,a_trac0=11.5,a_trac2=0.0006,p_m=800.0)
def make_data(o,prm):
    v,kap,kz,vlat,ax=gen.speed_profile(C,o,prm)
    return dict(ds=C['ds'],half=6.0,cx=C['cx'].tolist(),cy=C['cy'].tolist(),hd=C['hd'].tolist(),off=o.tolist(),v=v.tolist(),k=kap.tolist(),ax=ax.tolist()), v
def trial(o,prm,P=None,trace=None,**kw):
    D,v=make_data(o,prm)
    r=harness.run(lambda: racer.Racer(D,P),trace=trace,**kw)
    d=r['drv']
    return dict(lap=r['lap'],t=r['t_end'],dmg=r['damage'],top=r['top'],tp=d.max_tp,lat=d.max_lat_err,off=d.offtrack_steps,plan=gen.laptime(C,o,v))
def show(tag,x): print('%-28s lap=%s t_end=%.1f dmg=%.0f top=%.0f maxTP=%.2f maxLatErr=%.2f offSteps=%d plan=%.1f'%(tag,('%.3f'%x['lap']) if x['lap'] else None,x['t'],x['dmg'],x['top'],x['tp'],x['lat'],x['off'],x['plan'])); sys.stdout.flush()
if __name__=='__main__':
    o=np.load('/home/claude/sim/build/o_base.npy')
    for my,mb in ((1.30,1.20),(1.45,1.35)):
        p=dict(BASE,mu_y=my,mu_b=mb); t=time.time(); x=trial(o,p); show('mu_y=%.2f mu_b=%.2f'%(my,mb),x); print('  (%.1fs)'%(time.time()-t))
